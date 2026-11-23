#include "World/SubseaEnvironment.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PostProcessComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Math/RandomStream.h"
#include "ProceduralMeshComponent.h"
#include "SubseaSimSubsystem.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	constexpr double MToCm = 100.0;

	UInstancedStaticMeshComponent* MakeInstances(AActor* Owner, USceneComponent* Parent, const TCHAR* Name)
	{
		UInstancedStaticMeshComponent* Component = Owner->CreateDefaultSubobject<UInstancedStaticMeshComponent>(Name);
		Component->SetupAttachment(Parent);
		Component->SetMobility(EComponentMobility::Static);
		Component->SetCollisionProfileName(TEXT("BlockAll"));
		return Component;
	}
}

ASubseaEnvironment::ASubseaEnvironment()
{
	PrimaryActorTick.bCanEverTick = true;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Static);
	SetRootComponent(Root);

	Seabed = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Seabed"));
	Seabed->SetupAttachment(Root);
	Seabed->SetMobility(EComponentMobility::Static);
	Seabed->bUseAsyncCooking = false;
	Seabed->SetCollisionProfileName(TEXT("BlockAll"));

	PipeSegments = MakeInstances(this, Root, TEXT("PipeSegments"));
	Flanges = MakeInstances(this, Root, TEXT("Flanges"));
	Boulders = MakeInstances(this, Root, TEXT("Boulders"));
	Structure = MakeInstances(this, Root, TEXT("Structure"));
	Debris = MakeInstances(this, Root, TEXT("Debris"));

	MarineSnow = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("MarineSnow"));
	MarineSnow->SetupAttachment(Root);
	MarineSnow->SetMobility(EComponentMobility::Movable);
	MarineSnow->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	MarineSnow->SetCastShadow(false);
	// Visual only: sensors must not see it, or every cloud would carry floating specks.
	MarineSnow->bHiddenInSceneCapture = true;

	Fog = CreateDefaultSubobject<UExponentialHeightFogComponent>(TEXT("WaterFog"));
	Fog->SetupAttachment(Root);

	PostProcess = CreateDefaultSubobject<UPostProcessComponent>(TEXT("WaterPostProcess"));
	PostProcess->SetupAttachment(Root);
	PostProcess->bUnbound = true;

	SurfaceLight = CreateDefaultSubobject<UDirectionalLightComponent>(TEXT("SurfaceLight"));
	SurfaceLight->SetupAttachment(Root);
	SurfaceLight->SetMobility(EComponentMobility::Movable);
	SurfaceLight->SetRelativeRotation(FRotator(-78.0, 25.0, 0.0));

	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Surface(TEXT("/SubseaSim/Materials/M_SubseaSurface.M_SubseaSurface"));
	CubeMesh = Cube.Object;
	CylinderMesh = Cylinder.Object;
	SphereMesh = Sphere.Object;
	// The generated surface material when present (Scripts/create_materials.py), else the engine's.
	BaseMaterial = Surface.Succeeded() ? Surface.Object : Material.Object;

	PipeSegments->SetStaticMesh(CylinderMesh);
	Flanges->SetStaticMesh(CylinderMesh);
	Boulders->SetStaticMesh(SphereMesh);
	Structure->SetStaticMesh(CubeMesh);
	Debris->SetStaticMesh(CubeMesh);
	MarineSnow->SetStaticMesh(SphereMesh);
}

void ASubseaEnvironment::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	Rebuild();
}

void ASubseaEnvironment::BeginPlay()
{
	Super::BeginPlay();
	Sim = GetWorld()->GetSubsystem<USubseaSimSubsystem>();
	if (PipelinePath.IsEmpty())
	{
		Rebuild();
	}
}

void ASubseaEnvironment::Tick(const float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (Sim && !FMath::IsNearlyEqual(Sim->Turbidity, AppliedTurbidity, 1e-3))
	{
		ApplyWaterLook(Sim->Turbidity);
	}
	UpdateMarineSnow(DeltaSeconds);
}

UMaterialInstanceDynamic* ASubseaEnvironment::MakeColour(const FLinearColor& Colour, const float Roughness, const float DetailStrength, const float Metallic)
{
	UMaterialInstanceDynamic* Instance = UMaterialInstanceDynamic::Create(BaseMaterial, this);
	Instance->SetVectorParameterValue(TEXT("Color"), Colour);
	Instance->SetScalarParameterValue(TEXT("Roughness"), Roughness);
	Instance->SetScalarParameterValue(TEXT("DetailStrength"), DetailStrength);
	Instance->SetScalarParameterValue(TEXT("Metallic"), Metallic);
	Materials.Add(Instance);
	return Instance;
}

void ASubseaEnvironment::Rebuild()
{
	Materials.Reset();
	BuildSeabed();
	BuildPipeline();
	BuildBoulders();
	BuildStructure();
	BuildMarineSnow();
	ApplyWaterLook(Sim ? Sim->Turbidity : 0.15);
}

double ASubseaEnvironment::SeabedHeightCm(const double WorldXCm, const double WorldYCm) const
{
	// Fractal Brownian motion: long dunes, medium undulation and fine ripples, plus a broad trough
	// across the pipeline route so parts of the pipe end up free-spanning.
	const double X = WorldXCm / MToCm;
	const double Y = WorldYCm / MToCm;
	const double Offset = static_cast<double>(Seed) * 17.31;

	double Height = 0.0;
	double Amplitude = 1.6;
	double Frequency = 0.018;
	for (int32 Octave = 0; Octave < 5; ++Octave)
	{
		Height += Amplitude * FMath::PerlinNoise2D(FVector2D(X * Frequency + Offset, Y * Frequency - Offset));
		Amplitude *= 0.45;
		Frequency *= 2.1;
	}
	Height += 0.035 * FMath::Sin(X * 5.1 + 0.6 * FMath::Sin(Y * 0.7));
	Height -= 0.9 * FMath::Exp(-FMath::Square((X - 18.0) / 5.0));

	return GetActorLocation().Z + (-WaterDepthM + Height) * MToCm;
}

void ASubseaEnvironment::BuildSeabed()
{
	const int32 NumX = FMath::Max(2, FMath::RoundToInt32(LengthM / CellM) + 1);
	const int32 NumY = FMath::Max(2, FMath::RoundToInt32(WidthM / CellM) + 1);
	const FVector Origin = GetActorLocation();

	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Triangles;
	TArray<FColor> Colours;
	Colours.SetNumUninitialized(NumX * NumY);
	Vertices.SetNumUninitialized(NumX * NumY);
	Normals.SetNumUninitialized(NumX * NumY);
	UVs.SetNumUninitialized(NumX * NumY);
	Triangles.Reserve((NumX - 1) * (NumY - 1) * 6);

	const double StepCm = CellM * MToCm;
	const double HalfLengthCm = 0.5 * LengthM * MToCm;
	const double HalfWidthCm = 0.5 * WidthM * MToCm;

	for (int32 Iy = 0; Iy < NumY; ++Iy)
	{
		for (int32 Ix = 0; Ix < NumX; ++Ix)
		{
			const double X = -HalfLengthCm + Ix * StepCm;
			const double Y = -HalfWidthCm + Iy * StepCm;
			const double Z = SeabedHeightCm(Origin.X + X, Origin.Y + Y) - Origin.Z;
			const int32 Index = Iy * NumX + Ix;
			Vertices[Index] = FVector(X, Y, Z);
			UVs[Index] = FVector2D(X / 400.0, Y / 400.0);

			// Normal from central differences of the analytic height field.
			const double Dx = SeabedHeightCm(Origin.X + X + StepCm, Origin.Y + Y) - SeabedHeightCm(Origin.X + X - StepCm, Origin.Y + Y);
			const double Dy = SeabedHeightCm(Origin.X + X, Origin.Y + Y + StepCm) - SeabedHeightCm(Origin.X + X, Origin.Y + Y - StepCm);
			Normals[Index] = FVector(-Dx, -Dy, 2.0 * StepCm).GetSafeNormal();

			// Vertex colour multiplies the sand colour: silt collects in hollows (darker, greener),
			// crests are scoured paler, and broad patches vary the tone. Stored linear.
			const double Relief = (Z + Origin.Z - (GetActorLocation().Z - WaterDepthM * MToCm)) / 100.0;
			const double Patch = FMath::PerlinNoise2D(FVector2D((Origin.X + X) * 0.0011 + Seed, (Origin.Y + Y) * 0.0011 - Seed));
			const double Stain = FMath::PerlinNoise2D(FVector2D((Origin.X + X) * 0.006 - Seed, (Origin.Y + Y) * 0.006 + Seed));
			const double Tone = FMath::Clamp(0.86 + 0.12 * Relief + 0.14 * Patch + 0.06 * Stain, 0.55, 1.08);
			const double Green = FMath::Clamp(1.0 - 0.08 * Relief, 0.9, 1.1);
			Colours[Index] = FColor(
				static_cast<uint8>(FMath::Clamp(Tone * 0.93 * 255.0, 0.0, 255.0)),
				static_cast<uint8>(FMath::Clamp(Tone * Green * 255.0 * 0.96, 0.0, 255.0)),
				static_cast<uint8>(FMath::Clamp(Tone * 0.9 * 255.0, 0.0, 255.0)), 255);
		}
	}

	for (int32 Iy = 0; Iy < NumY - 1; ++Iy)
	{
		for (int32 Ix = 0; Ix < NumX - 1; ++Ix)
		{
			const int32 A = Iy * NumX + Ix;
			const int32 B = A + 1;
			const int32 C = A + NumX;
			const int32 D = C + 1;
			// Wound so the face normal points up (+Z) in Unreal's left-handed frame.
			Triangles.Append({ A, C, B, B, C, D });
		}
	}

	Seabed->ClearAllMeshSections();
	Seabed->CreateMeshSection(0, Vertices, Triangles, Normals, UVs, Colours, TArray<FProcMeshTangent>(), true);
	Seabed->SetMaterial(0, MakeColour(FLinearColor(0.42f, 0.36f, 0.26f), 0.95f, 0.85f));
}

void ASubseaEnvironment::BuildPipeline()
{
	const FVector Origin = GetActorLocation();
	const double RadiusCm = PipeRadiusM * MToCm;
	const double StartX = -0.5 * LengthM * MToCm + 1200.0;
	const double EndX = 0.5 * LengthM * MToCm - 2200.0;
	constexpr double StepCm = 150.0;
	const int32 NumPoints = FMath::Max(2, FMath::FloorToInt32((EndX - StartX) / StepCm) + 1);

	// Lay the pipe on the seabed, then smooth its elevation: a steel pipe cannot follow every
	// ripple, so it bridges hollows (free spans) exactly where the seabed drops away.
	TArray<FVector> Raw;
	Raw.SetNumUninitialized(NumPoints);
	for (int32 I = 0; I < NumPoints; ++I)
	{
		const double X = StartX + I * StepCm;
		const double Y = 420.0 * FMath::Sin(X * 0.00045) + 160.0 * FMath::Sin(X * 0.0011 + 1.3);
		Raw[I] = FVector(Origin.X + X, Origin.Y + Y, SeabedHeightCm(Origin.X + X, Origin.Y + Y) + RadiusCm * 0.7);
	}

	PipelinePath.SetNumUninitialized(NumPoints);
	constexpr int32 Window = 6;
	for (int32 I = 0; I < NumPoints; ++I)
	{
		double Highest = -UE_BIG_NUMBER;
		double Sum = 0.0;
		int32 Count = 0;
		for (int32 J = FMath::Max(0, I - Window); J <= FMath::Min(NumPoints - 1, I + Window); ++J)
		{
			Sum += Raw[J].Z;
			Highest = FMath::Max(Highest, Raw[J].Z);
			++Count;
		}
		// Bias towards the local high points the pipe would actually rest on.
		PipelinePath[I] = FVector(Raw[I].X, Raw[I].Y, FMath::Lerp(Sum / Count, Highest, 0.6));
	}

	PipeSegments->ClearInstances();
	Flanges->ClearInstances();
	Debris->ClearInstances();

	FRandomStream Random(Seed * 31 + 5);
	for (int32 I = 0; I < NumPoints - 1; ++I)
	{
		const FVector A = PipelinePath[I];
		const FVector B = PipelinePath[I + 1];
		const FVector Delta = B - A;
		const double Length = Delta.Size();
		const FQuat Orientation = FRotationMatrix::MakeFromZ(Delta).ToQuat();
		// The engine cylinder is 100 cm tall along Z with a 50 cm radius. Slight overlap hides joints.
		const FVector Scale(RadiusCm / 50.0, RadiusCm / 50.0, (Length + 2.0) / 100.0);
		PipeSegments->AddInstance(FTransform(Orientation, (A + B) * 0.5, Scale), true);

		if (I % 8 == 4)
		{
			Flanges->AddInstance(FTransform(Orientation, A, FVector(RadiusCm * 1.45 / 50.0, RadiusCm * 1.45 / 50.0, 0.16)), true);
		}
		if (I % 23 == 11)
		{
			// Dropped objects and debris: the anomalies an inspection is meant to find.
			const FVector Side = FVector::CrossProduct(Delta.GetSafeNormal(), FVector::UpVector);
			const FVector Position = A + Side * Random.FRandRange(-90.0f, 90.0f) + FVector(0.0, 0.0, Random.FRandRange(-10.0f, 25.0f));
			const FRotator Rotation(Random.FRandRange(-25.0f, 25.0f), Random.FRandRange(0.0f, 360.0f), Random.FRandRange(-25.0f, 25.0f));
			const FVector Size(Random.FRandRange(0.25f, 0.7f), Random.FRandRange(0.2f, 0.45f), Random.FRandRange(0.12f, 0.3f));
			Debris->AddInstance(FTransform(Rotation, Position, Size), true);
		}
	}

	// Weight-coated pipe with marine growth, bare steel flanges, rusting debris.
	PipeSegments->SetMaterial(0, MakeColour(FLinearColor(0.5f, 0.4f, 0.12f), 0.7f, 0.55f));
	Flanges->SetMaterial(0, MakeColour(FLinearColor(0.35f, 0.36f, 0.37f), 0.45f, 0.35f, 0.9f));
	Debris->SetMaterial(0, MakeColour(FLinearColor(0.42f, 0.16f, 0.06f), 0.85f, 0.8f, 0.3f));
}

void ASubseaEnvironment::BuildBoulders()
{
	Boulders->ClearInstances();
	const FVector Origin = GetActorLocation();
	FRandomStream Random(Seed * 131 + 9);

	for (int32 I = 0; I < NumBoulders; ++I)
	{
		const double X = Random.FRandRange(-0.48f, 0.48f) * LengthM * MToCm;
		const double Y = Random.FRandRange(-0.48f, 0.48f) * WidthM * MToCm;
		// Power-law sizes: many cobbles, a few large boulders.
		const double Size = 0.15 + 1.6 * FMath::Pow(Random.GetFraction(), 3.0f);
		const FVector Scale(Size * Random.FRandRange(0.7f, 1.3f), Size * Random.FRandRange(0.7f, 1.3f), Size * Random.FRandRange(0.45f, 0.9f));
		const FVector Position(Origin.X + X, Origin.Y + Y, SeabedHeightCm(Origin.X + X, Origin.Y + Y) + Scale.Z * 15.0);
		const FRotator Rotation(Random.FRandRange(-20.0f, 20.0f), Random.FRandRange(0.0f, 360.0f), Random.FRandRange(-20.0f, 20.0f));
		Boulders->AddInstance(FTransform(Rotation, Position, Scale), true);
	}
	Boulders->SetMaterial(0, MakeColour(FLinearColor(0.24f, 0.24f, 0.22f), 0.9f, 0.9f));
}

void ASubseaEnvironment::BuildStructure()
{
	Structure->ClearInstances();
	if (PipelinePath.IsEmpty())
	{
		return;
	}

	// A manifold frame at the end of the pipeline: four legs, a top ring of beams and a valve block.
	const FVector End = PipelinePath.Last();
	const FVector Base(End.X + 420.0, End.Y, SeabedHeightCm(End.X + 420.0, End.Y));
	constexpr double HalfX = 300.0;
	constexpr double HalfY = 260.0;
	constexpr double Height = 340.0;
	constexpr double Beam = 0.22;

	for (const double Sx : { -1.0, 1.0 })
	{
		for (const double Sy : { -1.0, 1.0 })
		{
			Structure->AddInstance(FTransform(FRotator::ZeroRotator,
				Base + FVector(Sx * HalfX, Sy * HalfY, Height * 0.5 - 30.0), FVector(Beam, Beam, (Height + 60.0) / 100.0)), true);
		}
		Structure->AddInstance(FTransform(FRotator::ZeroRotator,
			Base + FVector(Sx * HalfX, 0.0, Height), FVector(Beam, 2.0 * HalfY / 100.0 + Beam, Beam)), true);
		Structure->AddInstance(FTransform(FRotator::ZeroRotator,
			Base + FVector(Sx * HalfX, 0.0, Height * 0.45), FVector(Beam * 0.7, 2.0 * HalfY / 100.0, Beam * 0.7)), true);
	}
	for (const double Sy : { -1.0, 1.0 })
	{
		Structure->AddInstance(FTransform(FRotator::ZeroRotator,
			Base + FVector(0.0, Sy * HalfY, Height), FVector(2.0 * HalfX / 100.0 + Beam, Beam, Beam)), true);
		// Diagonal brace.
		Structure->AddInstance(FTransform(FRotator(FMath::RadiansToDegrees(FMath::Atan2(Height, 2.0 * HalfX)), 0.0, 0.0),
			Base + FVector(0.0, Sy * HalfY, Height * 0.5), FVector(FMath::Sqrt(FMath::Square(2.0 * HalfX) + FMath::Square(Height)) / 100.0, Beam * 0.6, Beam * 0.6)), true);
	}
	// Valve block and tie-in spool.
	Structure->AddInstance(FTransform(FRotator::ZeroRotator, Base + FVector(0.0, 0.0, 110.0), FVector(2.4, 1.7, 1.5)), true);
	Structure->AddInstance(FTransform(FRotator::ZeroRotator, Base + FVector(-40.0, 0.0, 220.0), FVector(0.7, 0.7, 0.9)), true);

	Structure->SetMaterial(0, MakeColour(FLinearColor(0.72f, 0.52f, 0.05f), 0.55f, 0.45f));
}

void ASubseaEnvironment::ApplyWaterLook(const double Turbidity)
{
	AppliedTurbidity = Turbidity;
	const float T = static_cast<float>(FMath::Clamp(Turbidity, 0.0, 1.0));

	// Uniform fog stands in for scattering; the sensor model uses the same turbidity value, so
	// what the pilot sees and what the depth camera returns degrade together.
	Fog->SetFogDensity(FMath::Lerp(0.7f, 8.0f, T));
	Fog->SetFogHeightFalloff(0.001f);
	Fog->SetFogMaxOpacity(1.0f);
	Fog->SetStartDistance(0.0f);
	Fog->SetFogInscatteringColor(FMath::Lerp(FLinearColor(0.004f, 0.03f, 0.05f), FLinearColor(0.02f, 0.045f, 0.035f), T));
	Fog->SetVolumetricFog(true);
	Fog->SetVolumetricFogScatteringDistribution(0.75f);
	Fog->SetVolumetricFogExtinctionScale(FMath::Lerp(0.8f, 6.0f, T));
	Fog->SetVolumetricFogAlbedo(FColor(120, 190, 200));
	Fog->SetVolumetricFogDistance(3000.0f);

	SurfaceLight->SetIntensity(FMath::Lerp(3.2f, 0.8f, T));
	SurfaceLight->SetLightColor(FLinearColor(0.25f, 0.6f, 0.8f));

	FPostProcessSettings& Settings = PostProcess->Settings;
	Settings.bOverride_AutoExposureMethod = true;
	Settings.AutoExposureMethod = AEM_Manual;
	Settings.bOverride_AutoExposureBias = true;
	Settings.AutoExposureBias = 9.5f;
	Settings.bOverride_ColorGain = true;
	// Water absorbs red first: pull the whole image towards blue-green.
	Settings.ColorGain = FVector4(0.72, 1.0, 1.08, 1.0);
	Settings.bOverride_VignetteIntensity = true;
	Settings.VignetteIntensity = 0.55f;
	Settings.bOverride_BloomIntensity = true;
	Settings.BloomIntensity = 0.9f;
}

FTransform ASubseaEnvironment::GetVehicleStart() const
{
	if (PipelinePath.Num() < 2)
	{
		return FTransform(GetActorLocation() + FVector(0.0, 0.0, -WaterDepthM * MToCm + 300.0));
	}
	const FVector Direction = (PipelinePath[1] - PipelinePath[0]).GetSafeNormal2D();
	const FVector Position = PipelinePath[0] - Direction * 250.0 + FVector(0.0, 0.0, 190.0);
	return FTransform(Direction.Rotation(), Position);
}

void ASubseaEnvironment::BuildMarineSnow()
{
	MarineSnow->ClearInstances();
	SnowTransforms.Reset();
	SnowWobble.Reset();
	FRandomStream Random(Seed * 977 + 3);
	const double Half = MarineSnowBoxM * 50.0;
	for (int32 I = 0; I < NumMarineSnow; ++I)
	{
		// Mostly fine specks with the occasional larger flock of detritus.
		const double Size = 0.004 + 0.012 * FMath::Pow(Random.GetFraction(), 4.0f);
		const FVector Position(Random.FRandRange(-Half, Half), Random.FRandRange(-Half, Half), Random.FRandRange(-Half, Half));
		SnowTransforms.Emplace(FQuat::Identity, GetActorLocation() + Position, FVector(Size, Size, Size * 0.7));
		SnowWobble.Emplace(Random.FRandRange(0.0f, UE_TWO_PI), Random.FRandRange(0.3f, 1.2f), Random.FRandRange(-1.0f, 1.0f));
	}
	MarineSnow->AddInstances(SnowTransforms, false, true);
	MarineSnow->SetMaterial(0, MakeColour(FLinearColor(0.85f, 0.88f, 0.82f), 1.0f));
}

void ASubseaEnvironment::UpdateMarineSnow(const double DeltaSeconds)
{
	if (SnowTransforms.IsEmpty() || !GetWorld())
	{
		return;
	}
	const APlayerController* Player = GetWorld()->GetFirstPlayerController();
	if (!Player || !Player->PlayerCameraManager)
	{
		return;
	}

	// Particles drift with the current and sink slowly; the box is re-centred on the viewer each
	// frame and particles wrap at its faces, so the density around the camera never changes.
	const FVector Centre = Player->PlayerCameraManager->GetCameraLocation();
	const FVector Current = (Sim ? Sim->SampleCurrent(Centre) : FVector::ZeroVector) * 100.0 * 0.8;
	const double Half = MarineSnowBoxM * 50.0;
	const double Box = Half * 2.0;
	const double Time = GetWorld()->GetTimeSeconds();
	for (int32 I = 0; I < SnowTransforms.Num(); ++I)
	{
		const FVector& Wobble = SnowWobble[I];
		const FVector Drift = Current + FVector(
			4.0 * FMath::Sin(Time * Wobble.Y + Wobble.X),
			4.0 * FMath::Cos(Time * Wobble.Y * 0.8 + Wobble.X),
			-1.2 + 1.5 * Wobble.Z);
		FVector Relative = SnowTransforms[I].GetLocation() + Drift * DeltaSeconds - Centre;
		Relative.X -= Box * FMath::Floor((Relative.X + Half) / Box);
		Relative.Y -= Box * FMath::Floor((Relative.Y + Half) / Box);
		Relative.Z -= Box * FMath::Floor((Relative.Z + Half) / Box);
		SnowTransforms[I].SetLocation(Centre + Relative);
	}
	MarineSnow->BatchUpdateInstancesTransforms(0, SnowTransforms, true, true, true);
}