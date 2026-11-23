#include "Sensors/SubseaDepthCameraComponent.h"

#include "Async/ParallelFor.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"
#include "Misc/FileHelper.h"
#include "Misc/ScopeLock.h"
#include "Net/SubseaStreamServer.h"
#include "RHICommandList.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "SubseaSimModule.h"
#include "SubseaSimSubsystem.h"
#include "Tasks/Task.h"
#include "TextureResource.h"

namespace Proto = Subsea::Protocol;

DECLARE_CYCLE_STAT(TEXT("DepthCamera Unproject"), STAT_SubseaUnproject, STATGROUP_SubseaSim);

namespace
{
	/** At most this many captures may be waiting on the GPU; beyond that the sensor skips frames. */
	constexpr int32 MaxInFlight = 3;

	struct FPendingReadback
	{
		TUniquePtr<FRHIGPUTextureReadback> Readback;
		int32 Width = 0;
		int32 Height = 0;
		float HorizontalFovDeg = 90.0f;
		float Exposure = 1.0f;
		bool bAutoExposure = true;
		double Stamp = 0.0;
		double Turbidity = 0.0;
		double RequestSeconds = 0.0;
		uint32 Seed = 0;
		FSubseaRangeNoiseModel Noise;
		FAnsiString FrameId;
	};

	FORCEINLINE uint8 ToneMapToByte(const float Linear, const float Exposure)
	{
		// Reinhard then gamma 2.2: cheap, monotonic and good enough for colouring points.
		const float Exposed = FMath::Max(Linear * Exposure, 0.0f);
		const float Mapped = Exposed / (1.0f + Exposed);
		return static_cast<uint8>(FMath::Clamp(FMath::Pow(Mapped, 1.0f / 2.2f) * 255.0f + 0.5f, 0.0f, 255.0f));
	}

	/** Standard normal sample via Box-Muller. */
	FORCEINLINE float GaussianSample(FRandomStream& Random)
	{
		const float U1 = FMath::Max(Random.GetFraction(), 1e-7f);
		const float U2 = Random.GetFraction();
		return FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(UE_TWO_PI * U2);
	}
}

struct USubseaDepthCameraComponent::FSharedState
{
	/** Render thread only. */
	TArray<FPendingReadback> Pending;
	std::atomic<int32> InFlight{ 0 };

	FCriticalSection LatestCloudLock;
	TArray<Proto::FPackedPoint> LatestCloud;

	TWeakPtr<FSubseaStreamServer, ESPMode::ThreadSafe> Server;
};

USubseaDepthCameraComponent::USubseaDepthCameraComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void USubseaDepthCameraComponent::BeginPlay()
{
	Super::BeginPlay();

	Sim = GetWorld()->GetSubsystem<USubseaSimSubsystem>();
	State = MakeShared<FSharedState, ESPMode::ThreadSafe>();
	Stats = MakeShared<FSubseaSensorStats, ESPMode::ThreadSafe>();
	CreateCaptureResources();
}

void USubseaDepthCameraComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (State.IsValid())
	{
		// Readbacks must be released on the render thread.
		ENQUEUE_RENDER_COMMAND(SubseaReleaseReadbacks)([KeepAlive = State](FRHICommandListImmediate&)
		{
			KeepAlive->Pending.Empty();
		});
	}
	Super::EndPlay(EndPlayReason);
}

void USubseaDepthCameraComponent::CreateCaptureResources()
{
	RenderTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("DepthCameraTarget"));
	RenderTarget->ClearColor = FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);
	RenderTarget->bAutoGenerateMips = false;
	// 32-bit float RGBA: HDR scene colour in RGB and scene depth (cm) in A, at full depth precision.
	RenderTarget->InitCustomFormat(Width, Height, PF_A32B32G32R32F, true);
	RenderTarget->UpdateResourceImmediate(true);

	if (!Capture)
	{
		Capture = NewObject<USceneCaptureComponent2D>(this, TEXT("DepthCameraCapture"));
		Capture->SetupAttachment(this);
		Capture->RegisterComponent();
	}
	Capture->TextureTarget = RenderTarget;
	Capture->CaptureSource = ESceneCaptureSource::SCS_SceneColorSceneDepth;
	Capture->ProjectionType = ECameraProjectionMode::Perspective;
	Capture->FOVAngle = HorizontalFovDeg;
	Capture->bCaptureEveryFrame = false;
	Capture->bCaptureOnMovement = false;
	Capture->bAlwaysPersistRenderingState = true;
	Capture->ShowFlags.SetMotionBlur(false);
	Capture->ShowFlags.SetBloom(false);
	Capture->ShowFlags.SetLensFlares(false);
	// The vehicle must not see its own hull.
	Capture->HiddenActors.AddUnique(GetOwner());
}

void USubseaDepthCameraComponent::SetResolution(const int32 NewWidth, const int32 NewHeight)
{
	Width = FMath::Clamp(NewWidth, 16, 4096);
	Height = FMath::Clamp(NewHeight, 16, 4096);
	if (HasBegunPlay())
	{
		CreateCaptureResources();
	}
}

void USubseaDepthCameraComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!State.IsValid() || !Sim)
	{
		return;
	}

	PollReadbacks();

	TimeUntilCapture -= DeltaTime;
	if (bEnabled && TimeUntilCapture <= 0.0)
	{
		// Keep phase instead of resetting to a full period so the average rate is exact.
		const double Period = 1.0 / FMath::Max(RateHz, 0.1f);
		TimeUntilCapture = FMath::Max(TimeUntilCapture + Period, 0.0);

		// If the GPU is behind, skip this frame rather than queue without bound.
		if (State->InFlight.load(std::memory_order_relaxed) >= MaxInFlight)
		{
			Stats->FramesSkipped.fetch_add(1, std::memory_order_relaxed);
		}
		else
		{
			RequestCapture();
		}
	}
}

void USubseaDepthCameraComponent::RequestCapture()
{
	if (!Capture || !RenderTarget)
	{
		return;
	}
	FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return;
	}

	Capture->FOVAngle = HorizontalFovDeg;
	// Queues the scene render; the readback command below is ordered after it on the render thread.
	Capture->CaptureScene();

	FPendingReadback Request;
	Request.Width = Width;
	Request.Height = Height;
	Request.HorizontalFovDeg = HorizontalFovDeg;
	Request.Exposure = ColourExposure;
	Request.bAutoExposure = bAutoExposure;
	Request.Stamp = Sim->SimTime();
	Request.Turbidity = Sim->Turbidity;
	Request.RequestSeconds = FPlatformTime::Seconds();
	Request.Seed = HashCombineFast(static_cast<uint32>(NoiseSeed), CaptureIndex++);
	Request.Noise = Noise;
	Request.FrameId = FAnsiString(TCHAR_TO_ANSI(*FrameId));

	State->Server = Sim->GetServer();
	State->InFlight.fetch_add(1, std::memory_order_relaxed);

	ENQUEUE_RENDER_COMMAND(SubseaDepthReadback)(
		[SharedState = State, Resource, Request = MoveTemp(Request)](FRHICommandListImmediate& RHICmdList) mutable
		{
			FRHITexture* Texture = Resource->GetRenderTargetTexture();
			if (!Texture)
			{
				SharedState->InFlight.fetch_sub(1, std::memory_order_relaxed);
				return;
			}
			Request.Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("SubseaDepthCamera"));
			Request.Readback->EnqueueCopy(RHICmdList, Texture);
			SharedState->Pending.Add(MoveTemp(Request));
		});
}

void USubseaDepthCameraComponent::PollReadbacks()
{
	if (State->InFlight.load(std::memory_order_relaxed) == 0)
	{
		return;
	}

	ENQUEUE_RENDER_COMMAND(SubseaDepthPoll)(
		[SharedState = State, SharedStats = Stats](FRHICommandListImmediate&)
		{
			// Readbacks complete in submission order, so only the head needs checking.
			while (SharedState->Pending.Num() > 0 && SharedState->Pending[0].Readback->IsReady())
			{
				FPendingReadback Done = MoveTemp(SharedState->Pending[0]);
				SharedState->Pending.RemoveAt(0);

				TArray<FLinearColor> Pixels;
				Pixels.SetNumUninitialized(Done.Width * Done.Height);

				int32 RowPitchPixels = 0;
				int32 BufferHeight = 0;
				const void* Mapped = Done.Readback->Lock(RowPitchPixels, &BufferHeight);
				if (Mapped && RowPitchPixels >= Done.Width && BufferHeight >= Done.Height)
				{
					// The mapped surface may be padded per row, so copy row by row.
					const FLinearColor* Source = static_cast<const FLinearColor*>(Mapped);
					for (int32 Row = 0; Row < Done.Height; ++Row)
					{
						FMemory::Memcpy(&Pixels[Row * Done.Width], Source + Row * RowPitchPixels, Done.Width * sizeof(FLinearColor));
					}
				}
				else
				{
					Pixels.Reset();
				}
				Done.Readback->Unlock();
				Done.Readback.Reset();

				if (Pixels.IsEmpty())
				{
					SharedState->InFlight.fetch_sub(1, std::memory_order_relaxed);
					continue;
				}

				// Everything from here on runs on a worker; the render thread is released immediately.
				UE::Tasks::Launch(UE_SOURCE_LOCATION,
					[SharedState, SharedStats, Pixels = MoveTemp(Pixels), Done = MoveTemp(Done)]() mutable
					{
						const double Start = FPlatformTime::Seconds();

						// Camera-style auto exposure: scale so the mean luminance of in-range pixels
						// lands mid-grey. Scene colour is scene-referred HDR, so without this the
						// point colours would clip to white under the vehicle's lights.
						float Exposure = Done.Exposure;
						if (Done.bAutoExposure)
						{
							const float MaxDepthCm = static_cast<float>(Done.Noise.MaxRangeM * 100.0);
							double LuminanceSum = 0.0;
							int32 Samples = 0;
							for (int32 Index = 0; Index < Pixels.Num(); Index += 7)
							{
								const FLinearColor& Pixel = Pixels[Index];
								if (Pixel.A > 0.0f && Pixel.A < MaxDepthCm)
								{
									LuminanceSum += 0.2126 * Pixel.R + 0.7152 * Pixel.G + 0.0722 * Pixel.B;
									++Samples;
								}
							}
							if (Samples > 0 && LuminanceSum > 1e-9)
							{
								Exposure *= static_cast<float>(0.35 / (LuminanceSum / Samples));
							}
						}

						TArray<Proto::FPackedPoint> Points;
						USubseaDepthCameraComponent::Unproject(Pixels, Done.Width, Done.Height, Done.HorizontalFovDeg,
							Done.Noise, Done.Turbidity, Exposure, Done.Seed, Points);

						if (const TSharedPtr<FSubseaStreamServer, ESPMode::ThreadSafe> Server = SharedState->Server.Pin())
						{
							if (Server->NumClients() > 0)
							{
								TSharedPtr<Proto::FWireFrame, ESPMode::ThreadSafe> Frame = MakeShared<Proto::FWireFrame, ESPMode::ThreadSafe>();
								const int32 PayloadBytes = 8 + Points.Num() * Proto::PointSize;
								Frame->Reserve(Proto::HeaderSize + PayloadBytes);
								Proto::AppendHeader(*Frame, Proto::EMessageType::PointCloud, Server->NextSeq(Proto::EMessageType::PointCloud),
									Done.Stamp, Done.FrameId, static_cast<uint32>(PayloadBytes));
								Proto::AppendPod(*Frame, static_cast<uint32>(Points.Num()));
								Proto::AppendPod(*Frame, static_cast<uint32>(Proto::HasRgb | Proto::HasIntensity));
								Frame->Append(reinterpret_cast<const uint8*>(Points.GetData()), Points.Num() * Proto::PointSize);
								Server->Enqueue(Frame, true);
							}
						}

						const double End = FPlatformTime::Seconds();
						SharedStats->LastNumPoints.store(Points.Num(), std::memory_order_relaxed);
						SharedStats->LastProcessMs.store(static_cast<float>((End - Start) * 1000.0), std::memory_order_relaxed);
						SharedStats->LastLatencyMs.store(static_cast<float>((End - Done.RequestSeconds) * 1000.0), std::memory_order_relaxed);
						SharedStats->FramesPublished.fetch_add(1, std::memory_order_relaxed);

						{
							FScopeLock Lock(&SharedState->LatestCloudLock);
							SharedState->LatestCloud = MoveTemp(Points);
						}
						SharedState->InFlight.fetch_sub(1, std::memory_order_relaxed);
					});
			}
		});
}

void USubseaDepthCameraComponent::Unproject(const TConstArrayView<FLinearColor> Pixels, const int32 InWidth, const int32 InHeight,
	const float InHorizontalFovDeg, const FSubseaRangeNoiseModel& InNoise, const double Turbidity, const float Exposure, const uint32 Seed,
	TArray<Proto::FPackedPoint>& OutPoints)
{
	SCOPE_CYCLE_COUNTER(STAT_SubseaUnproject);
	OutPoints.Reset();
	if (Pixels.Num() != InWidth * InHeight || InWidth <= 0 || InHeight <= 0)
	{
		return;
	}

	// Pinhole model. Scene depth is distance along the optical axis, so a pixel at normalised
	// image coordinates (nx, ny) un-projects to depth * (1, nx * tan(hfov/2), ny * tan(vfov/2)).
	const float TanHalfH = FMath::Tan(FMath::DegreesToRadians(InHorizontalFovDeg) * 0.5f);
	const float TanHalfV = TanHalfH * static_cast<float>(InHeight) / static_cast<float>(InWidth);
	const float Attenuation = static_cast<float>(FMath::Lerp(InNoise.AttenuationClear, InNoise.AttenuationTurbid, FMath::Clamp(Turbidity, 0.0, 1.0)));
	const float MinRange = static_cast<float>(InNoise.MinRangeM);
	const float MaxRange = static_cast<float>(InNoise.MaxRangeM);
	const float SigmaBase = static_cast<float>(InNoise.SigmaBaseM);
	const float SigmaQuad = static_cast<float>(InNoise.SigmaQuadratic);
	const float InvFloor = 1.0f / static_cast<float>(FMath::Max(InNoise.DetectionFloor, 1e-4));

	// Rows are split into bands processed in parallel. Each band owns its output array and its own
	// seeded random stream, so there is no sharing between workers and the result is deterministic
	// regardless of how many threads run or in what order.
	constexpr int32 RowsPerBand = 16;
	const int32 NumBands = FMath::DivideAndRoundUp(InHeight, RowsPerBand);
	TArray<TArray<Proto::FPackedPoint>> Bands;
	Bands.SetNum(NumBands);

	ParallelFor(NumBands, [&](const int32 Band)
	{
		const int32 RowBegin = Band * RowsPerBand;
		const int32 RowEnd = FMath::Min(RowBegin + RowsPerBand, InHeight);
		TArray<Proto::FPackedPoint>& Out = Bands[Band];
		Out.Reserve((RowEnd - RowBegin) * InWidth);
		FRandomStream Random(static_cast<int32>(HashCombineFast(Seed, static_cast<uint32>(Band))));

		for (int32 Row = RowBegin; Row < RowEnd; ++Row)
		{
			const float Ny = 1.0f - 2.0f * (static_cast<float>(Row) + 0.5f) / static_cast<float>(InHeight);
			const FLinearColor* Line = &Pixels[Row * InWidth];

			for (int32 Col = 0; Col < InWidth; ++Col)
			{
				const FLinearColor& Pixel = Line[Col];
				const float DepthM = Pixel.A * 0.01f;
				if (!FMath::IsFinite(DepthM) || DepthM <= 0.0f)
				{
					continue;
				}

				const float Nx = 2.0f * (static_cast<float>(Col) + 0.5f) / static_cast<float>(InWidth) - 1.0f;
				// Unreal camera axes: X forward, Y right, Z up.
				const float RayY = Nx * TanHalfH;
				const float RayZ = Ny * TanHalfV;
				const float RayLength = FMath::Sqrt(1.0f + RayY * RayY + RayZ * RayZ);
				const float Range = DepthM * RayLength;
				if (Range < MinRange || Range > MaxRange)
				{
					continue;
				}

				// Beer-Lambert two-way attenuation; weak returns are lost with rising probability.
				const float Signal = FMath::Exp(-2.0f * Attenuation * Range);
				const float DetectionProbability = FMath::Min(Signal * InvFloor, 1.0f);
				if (Random.GetFraction() > DetectionProbability)
				{
					continue;
				}

				// Range error acts along the ray, not along the optical axis.
				const float Sigma = SigmaBase + SigmaQuad * Range * Range;
				const float NoisyDepth = DepthM * (1.0f + GaussianSample(Random) * Sigma / Range);

				Proto::FPackedPoint& Point = Out[Out.AddUninitialized()];
				// Convert to ROS axes: Y flips from right to left.
				Point.X = NoisyDepth;
				Point.Y = -NoisyDepth * RayY;
				Point.Z = NoisyDepth * RayZ;
				Point.R = ToneMapToByte(Pixel.R, Exposure);
				Point.G = ToneMapToByte(Pixel.G, Exposure);
				Point.B = ToneMapToByte(Pixel.B, Exposure);
				Point.Intensity = static_cast<uint8>(FMath::Clamp(Signal * 255.0f, 0.0f, 255.0f));
			}
		}
	});

	// Concatenate the bands in order.
	int32 Total = 0;
	for (const TArray<Proto::FPackedPoint>& Band : Bands)
	{
		Total += Band.Num();
	}
	OutPoints.Reserve(Total);
	for (const TArray<Proto::FPackedPoint>& Band : Bands)
	{
		OutPoints.Append(Band);
	}
}

void USubseaDepthCameraComponent::GetLatestCloud(TArray<Proto::FPackedPoint>& OutPoints) const
{
	OutPoints.Reset();
	if (State.IsValid())
	{
		FScopeLock Lock(&State->LatestCloudLock);
		OutPoints = State->LatestCloud;
	}
}

bool USubseaDepthCameraComponent::ExportLatestCloudPly(const FString& FilePath) const
{
	TArray<Proto::FPackedPoint> Points;
	GetLatestCloud(Points);
	if (Points.IsEmpty())
	{
		return false;
	}

	const FString Header = FString::Printf(
		TEXT("ply\nformat binary_little_endian 1.0\ncomment SubseaSim depth camera, sensor frame, ROS axes, metres\n")
		TEXT("element vertex %d\nproperty float x\nproperty float y\nproperty float z\n")
		TEXT("property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar intensity\nend_header\n"),
		Points.Num());

	TArray<uint8> Bytes;
	const FTCHARToUTF8 Utf8(*Header);
	Bytes.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
	Bytes.Append(reinterpret_cast<const uint8*>(Points.GetData()), Points.Num() * Proto::PointSize);
	return FFileHelper::SaveArrayToFile(Bytes, *FilePath);
}
