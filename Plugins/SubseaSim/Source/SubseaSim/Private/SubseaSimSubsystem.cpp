#include "SubseaSimSubsystem.h"

#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "Math/SubseaFrames.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/App.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Net/SubseaStreamServer.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SubseaSimModule.h"
#include "UnrealClient.h"


bool USubseaSimSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void USubseaSimSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	FParse::Value(FCommandLine::Get(), TEXT("SubseaPort="), StreamPort);
}

void USubseaSimSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	Server = MakeShared<FSubseaStreamServer, ESPMode::ThreadSafe>();
	// Loopback by default; -SubseaBindAll exposes the stream to containers and other machines.
	if (!Server->Start(StreamPort, FParse::Param(FCommandLine::Get(), TEXT("SubseaBindAll"))))
	{
		Server.Reset();
	}
}

void USubseaSimSubsystem::Deinitialize()
{
	if (Server.IsValid())
	{
		Server->Shutdown();
		Server.Reset();
	}
	Super::Deinitialize();
}

TStatId USubseaSimSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(USubseaSimSubsystem, STATGROUP_SubseaSim);
}

double USubseaSimSubsystem::SimTime() const
{
	const UWorld* World = GetWorld();
	return World ? World->GetTimeSeconds() : 0.0;
}

FVector USubseaSimSubsystem::SampleCurrent(const FVector& WorldPositionCm) const
{
	// A steady set plus a slow, spatially coherent meander so station keeping has something to reject.
	const double T = SimTime();
	const double Phase = WorldPositionCm.X * 0.0004 + WorldPositionCm.Y * 0.0003;
	const FVector Gust(
		FMath::Sin(0.11 * T + Phase),
		FMath::Sin(0.07 * T + 1.7 + Phase),
		0.25 * FMath::Sin(0.05 * T + 0.6));
	return CurrentMps + CurrentGustMps * Gust;
}

void USubseaSimSubsystem::PublishTransform(const FAnsiStringView FrameId, const FAnsiStringView ParentFrame, const FTransform& RelativeTransform,
	const FVector& LinearVelocityBodyCmS, const FVector& AngularVelocityBodyRadS)
{
	if (!Server.IsValid() || Server->NumClients() == 0)
	{
		return;
	}
	Server->Enqueue(Subsea::Protocol::MakeTransformFrame(
		Server->NextSeq(Subsea::Protocol::EMessageType::Transform), SimTime(), FrameId, ParentFrame,
		Subsea::Frames::UnrealToRosVector(RelativeTransform.GetLocation()),
		Subsea::Frames::UnrealToRosQuat(RelativeTransform.GetRotation()),
		Subsea::Frames::UnrealToRosVector(LinearVelocityBodyCmS),
		Subsea::Frames::UnrealToRosAngular(AngularVelocityBodyRadS)), false);
}

void USubseaSimSubsystem::PublishTelemetry(const TSharedRef<FJsonObject>& Json)
{
	if (!Server.IsValid() || Server->NumClients() == 0)
	{
		return;
	}
	FString Text;
	const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Text);
	FJsonSerializer::Serialize(Json, Writer);

	const FTCHARToUTF8 Utf8(*Text);
	Server->Enqueue(Subsea::Protocol::MakeTelemetryFrame(
		Server->NextSeq(Subsea::Protocol::EMessageType::Telemetry), SimTime(), "base_link",
		FUtf8StringView(reinterpret_cast<const UTF8CHAR*>(Utf8.Get()), Utf8.Length())), false);
}

void USubseaSimSubsystem::PublishFrame(Subsea::Protocol::FFramePtr Frame, const bool bDroppable)
{
	if (const TSharedPtr<FSubseaStreamServer, ESPMode::ThreadSafe> Pinned = Server)
	{
		Pinned->Enqueue(MoveTemp(Frame), bDroppable);
	}
}

uint32 USubseaSimSubsystem::NextSeq(const Subsea::Protocol::EMessageType Type)
{
	return Server.IsValid() ? Server->NextSeq(Type) : 0;
}

int32 USubseaSimSubsystem::GetNumStreamClients() const
{
	return Server.IsValid() ? Server->NumClients() : 0;
}

void USubseaSimSubsystem::Tick(const float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (!Server.IsValid())
	{
		return;
	}

	FString CommandJson;
	while (Server->DequeueCommand(CommandJson))
	{
		TSharedPtr<FJsonObject> Object;
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(CommandJson);
		FString Command;
		if (!FJsonSerializer::Deserialize(Reader, Object) || !Object.IsValid() || !Object->TryGetStringField(TEXT("cmd"), Command))
		{
			UE_LOG(LogSubseaSim, Warning, TEXT("Ignoring malformed stream command: %s"), *CommandJson.Left(200));
			continue;
		}

		if (Command == TEXT("set_turbidity"))
		{
			double Value = Turbidity;
			double RampSeconds = 0.0;
			Object->TryGetNumberField(TEXT("ramp_s"), RampSeconds);
			if (Object->TryGetNumberField(TEXT("value"), Value))
			{
				// Optional ramp over simulation time, e.g. a silt cloud rolling in.
				TurbidityTarget = FMath::Clamp(Value, 0.0, 1.0);
				TurbidityRate = RampSeconds > 0.0 ? FMath::Abs(TurbidityTarget - Turbidity) / RampSeconds : 0.0;
				if (TurbidityRate <= 0.0)
				{
					Turbidity = TurbidityTarget;
				}
			}
		}
		else if (Command == TEXT("screenshot"))
		{
			// Lets a headless client capture the pilot view, e.g. for CI artefacts. The name is
			// sanitised and the file always lands in Saved/Screenshots.
			FString Name = TEXT("subsea");
			Object->TryGetStringField(TEXT("name"), Name);
			Name = FPaths::MakeValidFileName(FPaths::GetBaseFilename(Name));
			FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir() / TEXT("Screenshots") / (Name + TEXT(".png")), false, false);
		}
		else if (Command == TEXT("hud"))
		{
			Object->TryGetBoolField(TEXT("visible"), bShowHud);
		}
		else if (Command == TEXT("record"))
		{
			FString Name = TEXT("take");
			int32 Frames = 300;
			Object->TryGetStringField(TEXT("name"), Name);
			Object->TryGetNumberField(TEXT("frames"), Frames);
			StartRecording(Name, Frames);
		}
		else if (Command == TEXT("max_fps"))
		{
			// Lets tests check that behaviour does not depend on frame rate. 0 removes the cap.
			double Value = 0.0;
			Object->TryGetNumberField(TEXT("value"), Value);
			if (IConsoleVariable* MaxFps = IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS")))
			{
				MaxFps->Set(static_cast<float>(FMath::Clamp(Value, 0.0, 1000.0)), ECVF_SetByCode);
			}
		}
		else if (Command == TEXT("pace"))
		{
			// With -UseFixedTimeStep the engine runs as fast as it can; pacing holds it to real time
			// between recordings so sensor data isn't produced faster than clients can read it.
			Object->TryGetBoolField(TEXT("realtime"), bPaceRealtime);
		}
		else if (Command == TEXT("quit"))
		{
			// Clean shutdown for scripted runs.
			UE_LOG(LogSubseaSim, Log, TEXT("Quit requested over the sensor stream"));
			FPlatformMisc::RequestExit(false, TEXT("SubseaSim quit command"));
		}
		OnCommand.Broadcast(Command, Object);
	}

	if (RecordFramesLeft > 0)
	{
		// One screenshot per frame. Run with -UseFixedTimeStep -FPS=N so each frame advances the
		// simulation by exactly 1/N s however long the capture takes, giving judder-free footage.
		FScreenshotRequest::RequestScreenshot(RecordDirectory / FString::Printf(TEXT("frame_%05d.png"), RecordFrameIndex++), false, false);
		if (--RecordFramesLeft == 0)
		{
			UE_LOG(LogSubseaSim, Log, TEXT("Recording finished: %d frames in %s"), RecordFrameIndex, *RecordDirectory);
		}
	}

	if (TurbidityRate > 0.0)
	{
		Turbidity = FMath::FInterpConstantTo(Turbidity, TurbidityTarget, DeltaTime, TurbidityRate);
		if (FMath::IsNearlyEqual(Turbidity, TurbidityTarget))
		{
			TurbidityRate = 0.0;
		}
	}

	if (bPaceRealtime && FApp::UseFixedTimeStep() && RecordFramesLeft == 0)
	{
		const double Now = FPlatformTime::Seconds();
		const double Remaining = FApp::GetFixedDeltaTime() - (Now - LastPaceSeconds);
		if (Remaining > 0.0 && Remaining < 0.5)
		{
			FPlatformProcess::Sleep(static_cast<float>(Remaining));
		}
		LastPaceSeconds = FPlatformTime::Seconds();
	}

	RateWindowSeconds += DeltaTime;	if (RateWindowSeconds >= 1.0)
	{
		const uint64 Bytes = Server->BytesSent();
		StreamMbps = static_cast<double>(Bytes - LastBytesSent) * 8.0 / 1.0e6 / RateWindowSeconds;
		LastBytesSent = Bytes;
		RateWindowSeconds = 0.0;
	}
}

void USubseaSimSubsystem::StartRecording(const FString& Name, const int32 Frames)
{
	RecordDirectory = FPaths::ProjectSavedDir() / TEXT("Recordings") / FPaths::MakeValidFileName(FPaths::GetBaseFilename(Name));
	IFileManager::Get().MakeDirectory(*RecordDirectory, true);
	RecordFramesLeft = FMath::Clamp(Frames, 1, 100000);
	RecordFrameIndex = 0;

	// Sidecar so recorded sensor data can be lined up with the frames afterwards.
	const FString Meta = FString::Printf(TEXT("{\"name\": \"%s\", \"frames\": %d, \"start_sim_time\": %.6f, \"fixed_dt\": %.6f}\n"),
		*FPaths::GetBaseFilename(RecordDirectory), RecordFramesLeft, SimTime(), FApp::UseFixedTimeStep() ? FApp::GetFixedDeltaTime() : 0.0);
	FFileHelper::SaveStringToFile(Meta, *(RecordDirectory / TEXT("meta.json")));
	UE_LOG(LogSubseaSim, Log, TEXT("Recording %d frames to %s"), RecordFramesLeft, *RecordDirectory);
}