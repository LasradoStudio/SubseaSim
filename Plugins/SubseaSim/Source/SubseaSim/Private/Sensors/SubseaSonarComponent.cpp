#include "Sensors/SubseaSonarComponent.h"

#include "Async/Async.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"
#include "Net/SubseaStreamServer.h"
#include "SubseaSimModule.h"
#include "SubseaSimSubsystem.h"
#include "Tasks/Task.h"
#include "TextureResource.h"

namespace Proto = Subsea::Protocol;

DECLARE_CYCLE_STAT(TEXT("Sonar image formation"), STAT_SubseaSonarForm, STATGROUP_SubseaSim);

namespace
{
	constexpr int32 FanWidth = 320;
	constexpr int32 FanHeight = 200;
	constexpr int32 SonarPrefixBytes = 20;

	/** Copper/amber map typical of imaging-sonar displays. T in 0..1. */
	FColor CopperMap(const float T)
	{
		const float C = FMath::Clamp(T, 0.0f, 1.0f);
		return FColor(
			static_cast<uint8>(FMath::Clamp(FMath::Pow(C, 0.75f) * 255.0f, 0.0f, 255.0f)),
			static_cast<uint8>(FMath::Clamp(FMath::Pow(C, 1.35f) * 215.0f, 0.0f, 255.0f)),
			static_cast<uint8>(FMath::Clamp(FMath::Pow(C, 2.6f) * 150.0f + 6.0f * (1.0f - C), 0.0f, 255.0f)),
			255);
	}
}

USubseaSonarComponent::USubseaSonarComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void USubseaSonarComponent::BeginPlay()
{
	Super::BeginPlay();
	Sim = GetWorld()->GetSubsystem<USubseaSimSubsystem>();
	TraceDelegate.BindUObject(this, &USubseaSonarComponent::OnTraceDone);

	FanTexture = UTexture2D::CreateTransient(FanWidth, FanHeight, PF_B8G8R8A8, TEXT("SonarFan"));
	FanTexture->SRGB = true;
	FanTexture->Filter = TF_Bilinear;
	FanTexture->UpdateResource();
}

float USubseaSonarComponent::EchoEnergy(const float RangeM, const float CosIncidence, const float BeamWeight, const float InAbsorptionDbPerM)
{
	// Backscatter = diffuse Lambert term (cos^2, weak: sediment scatters about -20 dB) plus a
	// near-specular lobe (cos^20) so surfaces facing the head, like the flank of a pipe, light up,
	// plus a tiny roughness floor. Then two-way absorption, and spreading loss mostly compensated by
	// time-varied gain, leaving a gentle fall-off with range.
	const float Cos = FMath::Max(CosIncidence, 0.0f);
	const float Cos2 = Cos * Cos;
	const float Backscatter = 0.0005f + 0.1f * Cos2 + 0.9f * FMath::Pow(Cos2, 10.0f);
	const float Absorption = FMath::Pow(10.0f, -2.0f * InAbsorptionDbPerM * RangeM / 10.0f);
	const float ResidualSpreading = 1.0f / (1.0f + 0.04f * RangeM);
	return BeamWeight * Backscatter * Absorption * ResidualSpreading;
}

void USubseaSonarComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	TimeUntilPing -= DeltaTime;
	if (bEnabled && TimeUntilPing <= 0.0)
	{
		TimeUntilPing = FMath::Max(TimeUntilPing + 1.0 / FMath::Max(RateHz, 0.1f), 0.0);
		// One ping at a time, like real hardware: the next one waits for the last echo.
		if (!Ping.IsValid())
		{
			StartPing();
		}
	}
}

void USubseaSonarComponent::StartPing()
{
	UWorld* World = GetWorld();
	const FTransform Mount = GetComponentTransform();
	const FVector Start = Mount.GetLocation();
	const float HalfFov = FMath::DegreesToRadians(HorizontalFovDeg) * 0.5f;
	const float Aperture = FMath::DegreesToRadians(VerticalApertureDeg);
	// Gaussian beam pattern whose full width at half maximum equals the aperture.
	const float Sigma = Aperture / 2.3548f;
	const float RangeCm = MaxRangeM * 100.0f;

	Ping = MakeUnique<FPing>();
	Ping->Stamp = Sim ? Sim->SimTime() : World->GetTimeSeconds();
	Ping->StartSeconds = FPlatformTime::Seconds();
	Ping->Seed = HashCombineFast(static_cast<uint32>(NoiseSeed), PingIndex++);
	const int32 NumRays = NumBeams * RaysPerBeam;
	Ping->Echoes.Reserve(NumRays);
	Ping->RayBeam.SetNumUninitialized(NumRays);
	Ping->RayWeight.SetNumUninitialized(NumRays);

	// Each ray stands for a slice of the vertical aperture; its footprint on a surface is what
	// spreads its echo across range bins in FormImage.
	RaySliceRad = Aperture * 1.6f / RaysPerBeam;

	FCollisionQueryParams Params(SCENE_QUERY_STAT(SubseaSonar), false, GetOwner());
	FRandomStream Jitter(static_cast<int32>(Ping->Seed));

	Ping->Outstanding = NumRays;
	for (int32 Beam = 0; Beam < NumBeams; ++Beam)
	{
		// Beam 0 is the port-most beam (negative Unreal yaw is to the left).
		const float Azimuth = -HalfFov + (Beam + 0.5f) * (2.0f * HalfFov / NumBeams);
		for (int32 V = 0; V < RaysPerBeam; ++V)
		{
			// Rays span 1.6x the half-power width to include the beam's skirts, jittered to avoid banding.
			const float Elevation = ((V + Jitter.FRandRange(0.2f, 0.8f)) / RaysPerBeam - 0.5f) * Aperture * 1.6f;
			const FVector Local(FMath::Cos(Elevation) * FMath::Cos(Azimuth), FMath::Cos(Elevation) * FMath::Sin(Azimuth), FMath::Sin(Elevation));
			const FVector End = Start + Mount.TransformVectorNoScale(Local) * RangeCm;

			const int32 Ray = Beam * RaysPerBeam + V;
			Ping->RayBeam[Ray] = Beam;
			// Peak-normalised: a bin is covered by roughly one ray's footprint at a time.
			Ping->RayWeight[Ray] = FMath::Exp(-0.5f * FMath::Square(Elevation / Sigma));

			World->AsyncLineTraceByChannel(EAsyncTraceType::Single, Start, End, ECC_Visibility, Params,
				FCollisionResponseParams::DefaultResponseParam, &TraceDelegate, static_cast<uint32>(Ray));
		}
	}
}

void USubseaSonarComponent::OnTraceDone(const FTraceHandle& Handle, FTraceDatum& Data)
{
	if (!Ping.IsValid())
	{
		return;
	}

	const int32 Ray = static_cast<int32>(Data.UserData);
	if (Ping->RayBeam.IsValidIndex(Ray) && Data.OutHits.Num() > 0 && Data.OutHits[0].bBlockingHit)
	{
		const FHitResult& Hit = Data.OutHits[0];
		const float RangeM = Hit.Distance * 0.01f;
		if (RangeM >= MinRangeM)
		{
			const FVector Direction = (Data.End - Data.Start).GetSafeNormal();
			const float CosIncidence = static_cast<float>(FVector::DotProduct(-Direction, Hit.ImpactNormal));
			FEcho& Echo = Ping->Echoes.AddDefaulted_GetRef();
			Echo.Beam = Ping->RayBeam[Ray];
			Echo.RangeM = RangeM;
			Echo.Energy = EchoEnergy(RangeM, CosIncidence, Ping->RayWeight[Ray], AbsorptionDbPerM);
			// Footprint length along range: r * slice angle * tan(incidence). Long for the grazing
			// seabed, short for a pipe flank facing the head.
			const float Cos = FMath::Clamp(FMath::Abs(CosIncidence), 0.05f, 1.0f);
			const float TanIncidence = FMath::Sqrt(1.0f - Cos * Cos) / Cos;
			Echo.SpreadM = FMath::Min(RangeM * RaySliceRad * TanIncidence, 6.0f);
		}
	}

	if (--Ping->Outstanding == 0)
	{
		FinishPing();
	}
}

void USubseaSonarComponent::FinishPing()
{
	TUniquePtr<FPing> Done = MoveTemp(Ping);

	FSubseaSonarImage Image;
	Image.NumBeams = NumBeams;
	Image.NumBins = NumBins;
	Image.HorizontalFovRad = FMath::DegreesToRadians(HorizontalFovDeg);
	Image.MinRangeM = MinRangeM;
	Image.MaxRangeM = MaxRangeM;
	Image.Stamp = Done->Stamp;

	const float InSpeckle = Speckle;
	const float InNoiseFloor = NoiseFloor;
	const float InDisplayRangeDb = DisplayRangeDb;
	const float ApertureRad = FMath::DegreesToRadians(VerticalApertureDeg);
	const FAnsiString InFrameId(TCHAR_TO_ANSI(*FrameId));
	TWeakPtr<FSubseaStreamServer, ESPMode::ThreadSafe> WeakServer = Sim ? Sim->GetServer() : nullptr;
	TWeakObjectPtr<USubseaSonarComponent> WeakThis(this);

	// Image formation, fan rendering and packing run on a worker; only the texture upload comes back.
	UE::Tasks::Launch(UE_SOURCE_LOCATION,
		[Done = MoveTemp(Done), Image = MoveTemp(Image), InSpeckle, InNoiseFloor, InDisplayRangeDb, ApertureRad, InFrameId, WeakServer, WeakThis]() mutable
		{
			{
				SCOPE_CYCLE_COUNTER(STAT_SubseaSonarForm);
				FormImage(Done->Echoes, Image.NumBeams, Image.NumBins, Image.MinRangeM, Image.MaxRangeM,
					InSpeckle, InNoiseFloor, InDisplayRangeDb, Done->Seed, Image.Intensity);
			}

			if (const TSharedPtr<FSubseaStreamServer, ESPMode::ThreadSafe> Server = WeakServer.Pin())
			{
				if (Server->NumClients() > 0)
				{
					TSharedPtr<Proto::FWireFrame, ESPMode::ThreadSafe> Frame = MakeShared<Proto::FWireFrame, ESPMode::ThreadSafe>();
					const int32 PayloadBytes = SonarPrefixBytes + Image.Intensity.Num();
					Frame->Reserve(Proto::HeaderSize + PayloadBytes);
					Proto::AppendHeader(*Frame, Proto::EMessageType::SonarImage, Server->NextSeq(Proto::EMessageType::SonarImage),
						Image.Stamp, InFrameId, static_cast<uint32>(PayloadBytes));
					Proto::AppendPod(*Frame, static_cast<uint16>(Image.NumBeams));
					Proto::AppendPod(*Frame, static_cast<uint16>(Image.NumBins));
					Proto::AppendPod(*Frame, Image.HorizontalFovRad);
					Proto::AppendPod(*Frame, Image.MinRangeM);
					Proto::AppendPod(*Frame, Image.MaxRangeM);
					Proto::AppendPod(*Frame, ApertureRad);
					Frame->Append(Image.Intensity);
					Server->Enqueue(Frame, true);
				}
			}

			TArray<FColor> Pixels;
			RenderFan(Image, FanWidth, Pixels);
			const float ElapsedMs = static_cast<float>((FPlatformTime::Seconds() - Done->StartSeconds) * 1000.0);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, Pixels = MoveTemp(Pixels), ElapsedMs]() mutable
			{
				USubseaSonarComponent* Self = WeakThis.Get();
				if (!Self || !Self->FanTexture)
				{
					return;
				}
				Self->LastPingMs = ElapsedMs;
				// The render thread reads the buffer later, so both it and the region live on the heap
				// until the upload's cleanup callback runs.
				TArray<FColor>* Data = new TArray<FColor>(MoveTemp(Pixels));
				FUpdateTextureRegion2D* Region = new FUpdateTextureRegion2D(0, 0, 0, 0, FanWidth, FanHeight);
				Self->FanTexture->UpdateTextureRegions(0, 1, Region, FanWidth * sizeof(FColor), sizeof(FColor),
					reinterpret_cast<uint8*>(Data->GetData()),
					[Data, Region](uint8*, const FUpdateTextureRegion2D*)
					{
						delete Data;
						delete Region;
					});
			});
		});
}

void USubseaSonarComponent::FormImage(const TConstArrayView<FEcho> Echoes, const int32 InNumBeams, const int32 InNumBins,
	const float InMinRangeM, const float InMaxRangeM, const float InSpeckle, const float InNoiseFloor, const float InDisplayRangeDb,
	const uint32 Seed, TArray<uint8>& OutIntensity)
{
	const int32 Cells = InNumBeams * InNumBins;
	TArray<float> Energy;
	Energy.SetNumZeroed(Cells);
	const float BinsPerMetre = static_cast<float>(InNumBins) / FMath::Max(InMaxRangeM - InMinRangeM, 1e-3f);

	for (const FEcho& Echo : Echoes)
	{
		if (Echo.Beam < 0 || Echo.Beam >= InNumBeams)
		{
			continue;
		}
		const float SpreadBins = Echo.SpreadM * BinsPerMetre;
		if (SpreadBins > 1.0f)
		{
			// Footprint covering several bins, centred on the hit: deposit by overlap fraction.
			const float Begin = (Echo.RangeM - InMinRangeM) * BinsPerMetre - SpreadBins * 0.5f;
			const float End = Begin + SpreadBins;
			for (int32 Bin = FMath::Max(0, FMath::FloorToInt32(Begin)); Bin < FMath::Min(InNumBins, FMath::CeilToInt32(End)); ++Bin)
			{
				const float Overlap = FMath::Min(End, Bin + 1.0f) - FMath::Max(Begin, static_cast<float>(Bin));
				Energy[Bin * InNumBeams + Echo.Beam] += Echo.Energy * FMath::Clamp(Overlap, 0.0f, 1.0f);
			}
			continue;
		}

		// Point echo: split between the two nearest bins, standing in for the finite pulse length.
		const float Position = (Echo.RangeM - InMinRangeM) * BinsPerMetre - 0.5f;
		const int32 Low = FMath::FloorToInt32(Position);
		const float Frac = Position - Low;
		if (Low >= 0 && Low < InNumBins)
		{
			Energy[Low * InNumBeams + Echo.Beam] += Echo.Energy * (1.0f - Frac);
		}
		if (Low + 1 >= 0 && Low + 1 < InNumBins)
		{
			Energy[(Low + 1) * InNumBeams + Echo.Beam] += Echo.Energy * Frac;
		}
	}

	// Neighbouring beams overlap (each beam's horizontal pattern spills into the next), modelled as
	// a [1 2 1]/4 blur across beams.
	TArray<float> Blurred;
	Blurred.SetNumUninitialized(Cells);
	for (int32 Bin = 0; Bin < InNumBins; ++Bin)
	{
		const float* Row = &Energy[Bin * InNumBeams];
		for (int32 Beam = 0; Beam < InNumBeams; ++Beam)
		{
			const float Left = Row[FMath::Max(Beam - 1, 0)];
			const float Right = Row[FMath::Min(Beam + 1, InNumBeams - 1)];
			Blurred[Bin * InNumBeams + Beam] = 0.25f * Left + 0.5f * Row[Beam] + 0.25f * Right;
		}
	}
	Energy = MoveTemp(Blurred);

	// Speckle: coherent echoes from many scatterers give exponentially distributed intensity.
	FRandomStream Random(static_cast<int32>(Seed));
	OutIntensity.SetNumUninitialized(Cells);
	const float InvRange = 1.0f / FMath::Max(InDisplayRangeDb, 1.0f);
	for (int32 Cell = 0; Cell < Cells; ++Cell)
	{
		const float Exponential = -FMath::Loge(FMath::Max(Random.GetFraction(), 1e-6f));
		const float Noise = InNoiseFloor * -FMath::Loge(FMath::Max(Random.GetFraction(), 1e-6f));
		const float Value = Energy[Cell] * ((1.0f - InSpeckle) + InSpeckle * Exponential) + Noise;
		const float Db = 10.0f * FMath::LogX(10.0f, FMath::Max(Value, 1e-9f));
		OutIntensity[Cell] = static_cast<uint8>(FMath::Clamp((Db * InvRange + 1.0f) * 255.0f, 0.0f, 255.0f));
	}
}

void USubseaSonarComponent::RenderFan(const FSubseaSonarImage& Image, const int32 Width, TArray<FColor>& OutPixels)
{
	const int32 Height = FMath::Max(1, Width * FanHeight / FanWidth);
	OutPixels.Init(FColor(0, 0, 0, 0), Width * Height);
	if (Image.NumBeams <= 0 || Image.NumBins <= 0 || Image.Intensity.Num() != Image.NumBeams * Image.NumBins)
	{
		return;
	}

	const float HalfFov = Image.HorizontalFovRad * 0.5f;
	constexpr float Pad = 4.0f;
	const float Radius = FMath::Min(Height - 2.0f * Pad, (Width * 0.5f - Pad) / FMath::Max(FMath::Sin(HalfFov), 0.2f));
	const float ApexX = Width * 0.5f;
	const float ApexY = Height - Pad;
	const float Span = Image.MaxRangeM - Image.MinRangeM;
	const float RingSpacingPx = Radius * 5.0f / Image.MaxRangeM;

	for (int32 Y = 0; Y < Height; ++Y)
	{
		for (int32 X = 0; X < Width; ++X)
		{
			const float Dx = X + 0.5f - ApexX;
			const float Dy = ApexY - (Y + 0.5f);
			const float DistancePx = FMath::Sqrt(Dx * Dx + Dy * Dy);
			const float Angle = FMath::Atan2(Dx, Dy);
			if (FMath::Abs(Angle) > HalfFov || DistancePx > Radius)
			{
				continue;
			}
			const float RangeM = DistancePx / Radius * Image.MaxRangeM;
			FColor& Out = OutPixels[Y * Width + X];
			if (RangeM < Image.MinRangeM)
			{
				Out = FColor(8, 10, 12, 230);
				continue;
			}

			// Bilinear sample in (beam, bin) space.
			const float BeamPos = (Angle + HalfFov) / Image.HorizontalFovRad * Image.NumBeams - 0.5f;
			const float BinPos = (RangeM - Image.MinRangeM) / Span * Image.NumBins - 0.5f;
			const int32 B0 = FMath::Clamp(FMath::FloorToInt32(BeamPos), 0, Image.NumBeams - 1);
			const int32 R0 = FMath::Clamp(FMath::FloorToInt32(BinPos), 0, Image.NumBins - 1);
			const int32 B1 = FMath::Min(B0 + 1, Image.NumBeams - 1);
			const int32 R1 = FMath::Min(R0 + 1, Image.NumBins - 1);
			const float Fb = FMath::Clamp(BeamPos - B0, 0.0f, 1.0f);
			const float Fr = FMath::Clamp(BinPos - R0, 0.0f, 1.0f);
			const auto At = [&Image](const int32 Bin, const int32 Beam) { return static_cast<float>(Image.Intensity[Bin * Image.NumBeams + Beam]); };
			const float Value = FMath::Lerp(FMath::Lerp(At(R0, B0), At(R0, B1), Fb), FMath::Lerp(At(R1, B0), At(R1, B1), Fb), Fr) / 255.0f;

			Out = CopperMap(Value);
			Out.A = 235;

			// Faint range rings every 5 m and a bright fan edge.
			const float RingPhase = FMath::Fmod(DistancePx, RingSpacingPx);
			const bool bRing = RingPhase < 0.9f && DistancePx > RingSpacingPx * 0.5f;
			const bool bEdge = (HalfFov - FMath::Abs(Angle)) * DistancePx < 1.0f || Radius - DistancePx < 1.0f;
			if (bRing || bEdge)
			{
				Out = FColor(FMath::Max<uint8>(Out.R, 70), FMath::Max<uint8>(Out.G, 120), FMath::Max<uint8>(Out.B, 120), 255);
			}
		}
	}
}
