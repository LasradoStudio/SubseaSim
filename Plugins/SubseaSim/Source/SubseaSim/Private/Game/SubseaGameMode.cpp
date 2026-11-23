#include "Game/SubseaGameMode.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Physics/SubseaThrusterComponent.h"
#include "Sensors/SubseaDepthCameraComponent.h"
#include "Sensors/SubseaSonarComponent.h"
#include "Engine/Texture2D.h"
#include "SubseaSimSubsystem.h"
#include "Vehicle/SubseaROV.h"
#include "World/SubseaEnvironment.h"

ASubseaGameMode::ASubseaGameMode()
{
	DefaultPawnClass = nullptr;
	HUDClass = ASubseaHUD::StaticClass();
}

void ASubseaGameMode::StartPlay()
{
	UWorld* World = GetWorld();

	ASubseaEnvironment* Environment = nullptr;
	for (TActorIterator<ASubseaEnvironment> It(World); It; ++It)
	{
		Environment = *It;
		break;
	}
	if (!Environment)
	{
		Environment = World->SpawnActor<ASubseaEnvironment>(FVector::ZeroVector, FRotator::ZeroRotator);
	}

	ASubseaROV* Vehicle = nullptr;
	for (TActorIterator<ASubseaROV> It(World); It; ++It)
	{
		Vehicle = *It;
		break;
	}
	if (!Vehicle && Environment)
	{
		const FTransform Start = Environment->GetVehicleStart();
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Vehicle = World->SpawnActor<ASubseaROV>(Start.GetLocation(), Start.Rotator(), Params);
	}

	Super::StartPlay();

	if (Vehicle)
	{
		if (APlayerController* PlayerController = World->GetFirstPlayerController())
		{
			PlayerController->Possess(Vehicle);
		}
	}
}

void ASubseaHUD::DrawHUD()
{
	Super::DrawHUD();

	const ASubseaROV* Vehicle = Cast<ASubseaROV>(GetOwningPawn());
	const USubseaSimSubsystem* Sim = GetWorld()->GetSubsystem<USubseaSimSubsystem>();
	if (!Vehicle || !Sim || !Canvas || !Sim->bShowHud)
	{
		return;
	}

	UFont* Font = GEngine->GetMediumFont();
	const FLinearColor Text(0.75f, 1.0f, 0.95f);
	const FLinearColor Dim(0.45f, 0.7f, 0.7f);
	const float Scale = FMath::Max(1.0f, Canvas->SizeY / 900.0f);
	const float LineHeight = 20.0f * Scale;
	float X = 28.0f * Scale;
	float Y = 28.0f * Scale;

	const auto Line = [&](const FString& String, const FLinearColor& Colour)
	{
		DrawText(String, Colour, X, Y, Font, Scale);
		Y += LineHeight;
	};

	Line(TEXT("SUBSEA SIM  |  ROV inspection digital twin"), Text);
	Y += LineHeight * 0.4f;
	Line(FString::Printf(TEXT("MODE      %s"), *FString(ASubseaROV::ModeName(Vehicle->Mode)).ToUpper()), Text);
	Line(FString::Printf(TEXT("DEPTH     %6.2f m"), Vehicle->GetDepthM()), Text);
	Line(FString::Printf(TEXT("ALTITUDE  %6.2f m"), Vehicle->GetAltitudeM()), Text);
	Line(FString::Printf(TEXT("HEADING   %6.1f deg"), Vehicle->GetHeadingDeg()), Text);
	Line(FString::Printf(TEXT("SPEED     %6.2f m/s"), Vehicle->GetSpeedMps()), Text);
	Line(FString::Printf(TEXT("XTRACK    %+6.2f m"), Vehicle->GetCrossTrackErrorM()), Text);
	Line(FString::Printf(TEXT("TURBIDITY %6.2f"), Sim->Turbidity), Text);
	Y += LineHeight * 0.4f;

	// Thruster load bars: centre-zero, cyan forward, amber reverse, red when failed.
	const float BarWidth = 120.0f * Scale;
	const float BarHeight = 9.0f * Scale;
	const TArray<TObjectPtr<USubseaThrusterComponent>>& Thrusters = Vehicle->GetThrusters();
	for (int32 Index = 0; Index < Thrusters.Num(); ++Index)
	{
		const USubseaThrusterComponent* Thruster = Thrusters[Index];
		const float Load = static_cast<float>(FMath::Clamp(Thruster->GetNormalisedThrust(), -1.0, 1.0));
		DrawText(FString::Printf(TEXT("T%d"), Index + 1), Thruster->bFailed ? FLinearColor::Red : Dim, X, Y - 4.0f * Scale, Font, Scale);
		const float BarX = X + 34.0f * Scale;
		DrawRect(FLinearColor(0.0f, 0.12f, 0.14f, 0.7f), BarX, Y, BarWidth, BarHeight);
		const float Half = BarWidth * 0.5f;
		const FLinearColor Colour = Thruster->bFailed ? FLinearColor::Red : (Load >= 0.0f ? FLinearColor(0.1f, 0.9f, 0.9f) : FLinearColor(1.0f, 0.65f, 0.1f));
		if (Load >= 0.0f)
		{
			DrawRect(Colour, BarX + Half, Y, Half * Load, BarHeight);
		}
		else
		{
			DrawRect(Colour, BarX + Half + Half * Load, Y, -Half * Load, BarHeight);
		}
		Y += BarHeight + 7.0f * Scale;
	}
	Y += LineHeight * 0.4f;

	if (const USubseaDepthCameraComponent* Camera = Vehicle->GetDepthCamera())
	{
		const FSubseaSensorStats& Stats = Camera->GetStats();
		Line(FString::Printf(TEXT("DEPTH CAM %dx%d @ %.0f Hz"), Camera->Width, Camera->Height, Camera->RateHz), Text);
		Line(FString::Printf(TEXT("  points  %d"), Stats.LastNumPoints.load()), Dim);
		// Latencies are wall-clock; while frames are being captured each frame takes far longer than
		// real time, so they would mislead in recorded footage and are left out.
		if (Sim->IsRecording())
		{
			Line(FString::Printf(TEXT("  cpu     %.2f ms"), Stats.LastProcessMs.load()), Dim);
		}
		else
		{
			Line(FString::Printf(TEXT("  cpu     %.2f ms   latency %.0f ms"), Stats.LastProcessMs.load(), Stats.LastLatencyMs.load()), Dim);
		}
	}
	if (const USubseaSonarComponent* Sonar = Vehicle->GetSonar())
	{
		Line(FString::Printf(TEXT("SONAR     %d beams x %d bins @ %.0f Hz"), Sonar->NumBeams, Sonar->NumBins, Sonar->RateHz), Text);
		if (Sim->IsRecording())
		{
			Line(FString::Printf(TEXT("  rays    %d per ping"), Sonar->GetRaysPerPing()), Dim);
		}
		else
		{
			Line(FString::Printf(TEXT("  rays    %d per ping   ping %.1f ms"), Sonar->GetRaysPerPing(), Sonar->GetLastPingMs()), Dim);
		}

		// Live fan display in the bottom-right corner, as on a pilot's sonar screen.
		if (UTexture2D* Fan = Sonar->GetFanTexture())
		{
			const float FanW = 420.0f * Scale;
			const float FanH = FanW * Fan->GetSizeY() / FMath::Max(1, Fan->GetSizeX());
			const float FanX = Canvas->SizeX - FanW - 28.0f * Scale;
			const float FanY = Canvas->SizeY - FanH - 70.0f * Scale;
			DrawRect(FLinearColor(0.0f, 0.02f, 0.03f, 0.55f), FanX - 8.0f * Scale, FanY - 26.0f * Scale, FanW + 16.0f * Scale, FanH + 34.0f * Scale);
			DrawTexture(Fan, FanX, FanY, FanW, FanH, 0.0f, 0.0f, 1.0f, 1.0f, FLinearColor::White, BLEND_Translucent);
			DrawText(FString::Printf(TEXT("IMAGING SONAR  %.0f m  %.0f deg"), Sonar->MaxRangeM, Sonar->HorizontalFovDeg), Text, FanX, FanY - 22.0f * Scale, Font, Scale);
		}
	}
	if (Sim->IsRecording())
	{
		Line(FString::Printf(TEXT("STREAM    tcp:%d  clients %d"), Sim->StreamPort, Sim->GetNumStreamClients()), Text);
	}
	else
	{
		Line(FString::Printf(TEXT("STREAM    tcp:%d  clients %d  %.1f Mbit/s"), Sim->StreamPort, Sim->GetNumStreamClients(), Sim->GetStreamMegabitsPerSecond()), Text);
	}

	// Fault banner: makes a thruster failure obvious on screen and in recordings.
	int32 Failed = -1;
	for (int32 Index = 0; Index < Thrusters.Num(); ++Index)
	{
		if (Thrusters[Index]->bFailed)
		{
			Failed = Index;
			break;
		}
	}
	if (Failed >= 0)
	{
		const FString Banner = FString::Printf(TEXT("THRUSTER %d FAILURE  -  ALLOCATION RE-SOLVED ON 7 THRUSTERS"), Failed + 1);
		float BannerW = 0.0f, BannerH = 0.0f;
		GetTextSize(Banner, BannerW, BannerH, Font, Scale * 1.15f);
		const float BannerX = (Canvas->SizeX - BannerW) * 0.5f;
		DrawRect(FLinearColor(0.55f, 0.03f, 0.02f, 0.75f), BannerX - 18.0f * Scale, 26.0f * Scale, BannerW + 36.0f * Scale, BannerH + 16.0f * Scale);
		DrawText(Banner, FLinearColor(1.0f, 0.92f, 0.88f), BannerX, 34.0f * Scale, Font, Scale * 1.15f);
	}

	Y = Canvas->SizeY - LineHeight * 2.4f;
	Line(TEXT("W/S surge  A/D sway  Q/E yaw  Space/Shift heave   1 manual  2 station-keep  3 pipeline"), Dim);
	Line(TEXT("F fail thruster 1   T/G turbidity   L lights   C camera"), Dim);
}
