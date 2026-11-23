#include "Vehicle/SubseaROV.h"

#include "Camera/CameraComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputCoreTypes.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Physics/SubseaHydrodynamicsComponent.h"
#include "Physics/SubseaThrusterComponent.h"
#include "Sensors/SubseaDepthCameraComponent.h"
#include "Sensors/SubseaSonarComponent.h"
#include "SubseaSimModule.h"
#include "SubseaSimSubsystem.h"
#include "UObject/ConstructorHelpers.h"
#include "World/SubseaEnvironment.h"

DECLARE_CYCLE_STAT(TEXT("ROV Control"), STAT_SubseaControl, STATGROUP_SubseaSim);

namespace
{
	struct FThrusterLayout
	{
		const TCHAR* Name;
		FVector PositionCm;
		FVector Direction;
	};

	// BlueROV2 Heavy layout in Unreal body axes (X forward, Y right, Z up). The four horizontal
	// thrusters are canted 45 degrees so together they span surge, sway and yaw; the four vertical
	// thrusters span heave, roll and pitch.
	const double C = UE_DOUBLE_INV_SQRT_2;
	const FThrusterLayout Layout[] = {
		{ TEXT("Thruster_FrontRight"), FVector(15.6, 11.1, 0.0), FVector(C, -C, 0.0) },
		{ TEXT("Thruster_FrontLeft"), FVector(15.6, -11.1, 0.0), FVector(C, C, 0.0) },
		{ TEXT("Thruster_RearRight"), FVector(-15.6, 11.1, 0.0), FVector(C, C, 0.0) },
		{ TEXT("Thruster_RearLeft"), FVector(-15.6, -11.1, 0.0), FVector(C, -C, 0.0) },
		{ TEXT("Thruster_VertFrontRight"), FVector(12.0, 21.8, 0.0), FVector(0.0, 0.0, 1.0) },
		{ TEXT("Thruster_VertFrontLeft"), FVector(12.0, -21.8, 0.0), FVector(0.0, 0.0, 1.0) },
		{ TEXT("Thruster_VertRearRight"), FVector(-12.0, 21.8, 0.0), FVector(0.0, 0.0, 1.0) },
		{ TEXT("Thruster_VertRearLeft"), FVector(-12.0, -21.8, 0.0), FVector(0.0, 0.0, 1.0) },
	};

	constexpr double PilotForceN = 70.0;
	constexpr double PilotYawNm = 6.0;
	constexpr double TransformRateHz = 30.0;
	constexpr double TelemetryRateHz = 10.0;
}

const TCHAR* ASubseaROV::ModeName(const ESubseaControlMode InMode)
{
	switch (InMode)
	{
	case ESubseaControlMode::Manual: return TEXT("manual");
	case ESubseaControlMode::StationKeep: return TEXT("station_keep");
	case ESubseaControlMode::PipelineFollow: return TEXT("pipeline_follow");
	default: return TEXT("unknown");
	}
}

ASubseaROV::ASubseaROV()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

	// The hull's collision box is also the rigid body: 457 x 575 x 254 mm.
	Hull = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Hull"));
	Hull->SetStaticMesh(Cube.Object);
	Hull->SetRelativeScale3D(FVector(0.457, 0.575, 0.254));
	Hull->SetSimulatePhysics(true);
	Hull->SetCollisionProfileName(TEXT("PhysicsActor"));
	Hull->SetNotifyRigidBodyCollision(false);
	Hull->BodyInstance.bUseCCD = true;
	SetRootComponent(Hull);

	// Scale is inherited from the hull, so child offsets below are divided through by it.
	const FVector InvScale(1.0 / 0.457, 1.0 / 0.575, 1.0 / 0.254);

	// The collision box itself is hidden; the visible vehicle is assembled from primitives so the
	// project needs no imported art: open frame, buoyancy foam, electronics and battery tubes.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	Hull->SetVisibility(false, false);

	const auto AddPart = [this, &InvScale](const TCHAR* Name, UStaticMesh* Mesh, const FVector& LocationCm, const FRotator& Rotation, const FVector& SizeCm, const FLinearColor& Colour)
	{
		UStaticMeshComponent* Part = CreateDefaultSubobject<UStaticMeshComponent>(Name);
		Part->SetStaticMesh(Mesh);
		Part->SetupAttachment(Hull);
		Part->SetUsingAbsoluteScale(true);
		Part->SetRelativeLocation(LocationCm * InvScale);
		Part->SetRelativeRotation(Rotation);
		Part->SetRelativeScale3D(SizeCm / 100.0);
		Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		VisualParts.Add(Part);
		VisualColours.Add(Colour);
	};

	const FLinearColor FrameColour(0.07f, 0.075f, 0.08f);
	const FLinearColor FoamColour(0.02f, 0.3f, 0.85f);
	const FLinearColor TubeColour(0.55f, 0.58f, 0.6f);
	for (const double Side : { -1.0, 1.0 })
	{
		const TCHAR* Suffix = Side < 0.0 ? TEXT("L") : TEXT("R");
		AddPart(*FString::Printf(TEXT("SidePlate%s"), Suffix), Cube.Object, FVector(0.0, Side * 27.5, -1.0), FRotator::ZeroRotator, FVector(45.0, 1.2, 22.0), FrameColour);
		AddPart(*FString::Printf(TEXT("Skid%s"), Suffix), Cube.Object, FVector(0.0, Side * 27.5, -12.2), FRotator::ZeroRotator, FVector(48.0, 3.0, 2.0), FrameColour);
		AddPart(*FString::Printf(TEXT("Foam%s"), Suffix), Cube.Object, FVector(0.0, Side * 15.0, 9.8), FRotator::ZeroRotator, FVector(42.0, 20.0, 6.0), FoamColour);
	}
	AddPart(TEXT("CrossBarFront"), Cube.Object, FVector(20.0, 0.0, -11.0), FRotator::ZeroRotator, FVector(2.0, 55.0, 2.0), FrameColour);
	AddPart(TEXT("CrossBarRear"), Cube.Object, FVector(-20.0, 0.0, -11.0), FRotator::ZeroRotator, FVector(2.0, 55.0, 2.0), FrameColour);
	// Cylinders are 100 cm long on Z; pitch 90 degrees lays them along X.
	AddPart(TEXT("ElectronicsTube"), Cylinder.Object, FVector(1.0, 0.0, 2.0), FRotator(90.0, 0.0, 0.0), FVector(11.0, 11.0, 32.0), TubeColour);
	AddPart(TEXT("BatteryTube"), Cylinder.Object, FVector(0.0, 0.0, -7.5), FRotator(90.0, 0.0, 0.0), FVector(8.0, 8.0, 26.0), TubeColour);
	AddPart(TEXT("CameraDome"), Sphere.Object, FVector(17.5, 0.0, 2.0), FRotator::ZeroRotator, FVector(10.5, 10.5, 10.5), FLinearColor(0.01f, 0.01f, 0.012f));

	for (const FThrusterLayout& Entry : Layout)
	{
		USubseaThrusterComponent* Thruster = CreateDefaultSubobject<USubseaThrusterComponent>(Entry.Name);
		Thruster->SetupAttachment(Hull);
		Thruster->SetUsingAbsoluteScale(true);
		Thruster->SetRelativeLocation(Entry.PositionCm * InvScale);
		Thruster->SetRelativeRotation(Entry.Direction.Rotation());
		Thrusters.Add(Thruster);

		UStaticMeshComponent* Nacelle = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("%s_Mesh"), Entry.Name));
		Nacelle->SetStaticMesh(Cylinder.Object);
		Nacelle->SetupAttachment(Thruster);
		Nacelle->SetUsingAbsoluteScale(true);
		// Cylinder axis is Z; lay it along the thrust axis.
		Nacelle->SetRelativeRotation(FRotator(90.0, 0.0, 0.0));
		Nacelle->SetRelativeScale3D(FVector(0.1, 0.1, 0.11));
		Nacelle->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		VisualParts.Add(Nacelle);
		VisualColours.Add(FLinearColor(0.015f, 0.015f, 0.02f));
	}

	Hydrodynamics = CreateDefaultSubobject<USubseaHydrodynamicsComponent>(TEXT("Hydrodynamics"));

	DepthCamera = CreateDefaultSubobject<USubseaDepthCameraComponent>(TEXT("DepthCamera"));
	DepthCamera->SetupAttachment(Hull);
	DepthCamera->SetUsingAbsoluteScale(true);
	DepthCamera->SetRelativeLocation(FVector(26.0, 0.0, -4.0) * InvScale);
	DepthCamera->SetRelativeRotation(FRotator(-32.0, 0.0, 0.0));

	// Forward-looking imaging sonar under the camera, tilted down so the fan grazes the seabed ahead.
	Sonar = CreateDefaultSubobject<USubseaSonarComponent>(TEXT("Sonar"));
	Sonar->SetupAttachment(Hull);
	Sonar->SetUsingAbsoluteScale(true);
	Sonar->SetRelativeLocation(FVector(22.0, 0.0, -10.0) * InvScale);
	Sonar->SetRelativeRotation(FRotator(-20.0, 0.0, 0.0));

	FpvCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FpvCamera"));
	FpvCamera->SetupAttachment(Hull);
	FpvCamera->SetUsingAbsoluteScale(true);
	FpvCamera->SetRelativeLocation(FVector(23.0, 0.0, 2.0) * InvScale);
	FpvCamera->SetRelativeRotation(FRotator(-14.0, 0.0, 0.0));
	FpvCamera->SetFieldOfView(95.0f);
	FpvCamera->SetAutoActivate(false);

	FlybyCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FlybyCamera"));
	FlybyCamera->SetupAttachment(Hull);
	FlybyCamera->SetUsingAbsoluteLocation(true);
	FlybyCamera->SetUsingAbsoluteRotation(true);
	FlybyCamera->SetUsingAbsoluteScale(true);
	FlybyCamera->SetAutoActivate(false);
	const auto MakeLight = [this, &InvScale](const TCHAR* Name, const double Y)
	{
		USpotLightComponent* Light = CreateDefaultSubobject<USpotLightComponent>(Name);
		Light->SetupAttachment(Hull);
		Light->SetUsingAbsoluteScale(true);
		Light->SetRelativeLocation(FVector(24.0, Y, 6.0) * InvScale);
		Light->SetRelativeRotation(FRotator(-24.0, Y > 0.0 ? 5.0 : -5.0, 0.0));
		Light->SetIntensityUnits(ELightUnits::Lumens);
		Light->SetIntensity(260.0f);
		Light->SetOuterConeAngle(46.0f);
		Light->SetInnerConeAngle(24.0f);
		Light->SetAttenuationRadius(1600.0f);
		Light->SetLightColor(FLinearColor(1.0f, 0.97f, 0.9f));
		Light->SetVolumetricScatteringIntensity(0.7f);
		Light->SetCastShadows(true);
		return Light;
	};
	LightLeft = MakeLight(TEXT("LightLeft"), -17.0);
	LightRight = MakeLight(TEXT("LightRight"), 17.0);

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(Hull);
	CameraBoom->SetUsingAbsoluteScale(true);
	CameraBoom->TargetArmLength = 330.0f;
	CameraBoom->SetRelativeRotation(FRotator(-22.0, 0.0, 0.0));
	CameraBoom->bDoCollisionTest = false;
	CameraBoom->bInheritPitch = false;
	CameraBoom->bInheritRoll = false;
	CameraBoom->bEnableCameraLag = true;
	CameraBoom->bEnableCameraRotationLag = true;
	CameraBoom->CameraLagSpeed = 3.0f;
	CameraBoom->CameraRotationLagSpeed = 3.0f;

	ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
	ChaseCamera->SetupAttachment(CameraBoom);
	ChaseCamera->SetFieldOfView(80.0f);

	// Presentation only: a soft light riding with the external cameras, like a diver's video light,
	// so the vehicle reads in footage. Off in the onboard view; casts no shadows.
	CameraFill = CreateDefaultSubobject<UPointLightComponent>(TEXT("CameraFill"));
	CameraFill->SetupAttachment(ChaseCamera);
	CameraFill->SetRelativeLocation(FVector(40.0, 0.0, 60.0));
	CameraFill->SetIntensityUnits(ELightUnits::Lumens);
	CameraFill->SetIntensity(180.0f);
	CameraFill->SetAttenuationRadius(900.0f);
	CameraFill->SetSourceRadius(60.0f);
	CameraFill->SetLightColor(FLinearColor(0.75f, 0.9f, 1.0f));
	CameraFill->SetCastShadows(false);
	CameraFill->SetVolumetricScatteringIntensity(0.0f);
	AutoPossessPlayer = EAutoReceiveInput::Disabled;
}

void ASubseaROV::BeginPlay()
{
	Super::BeginPlay();

	Sim = GetWorld()->GetSubsystem<USubseaSimSubsystem>();
	for (TActorIterator<ASubseaEnvironment> It(GetWorld()); It; ++It)
	{
		Environment = *It;
		break;
	}

	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/SubseaSim/Materials/M_SubseaSurface.M_SubseaSurface"));
	if (!Base)
	{
		Base = Hull->GetMaterial(0);
	}
	if (Base)
	{
		for (int32 Index = 0; Index < VisualParts.Num(); ++Index)
		{
			// Anodised aluminium housings read as metal; frame, foam and nacelles stay dielectric.
			const FString PartName = VisualParts[Index]->GetName();
			const bool bMetal = PartName.Contains(TEXT("Tube"));
			UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, this);
			Material->SetVectorParameterValue(TEXT("Color"), VisualColours[Index]);
			Material->SetScalarParameterValue(TEXT("Roughness"), bMetal ? 0.28f : 0.5f);
			Material->SetScalarParameterValue(TEXT("Metallic"), bMetal ? 1.0f : 0.0f);
			VisualParts[Index]->SetMaterial(0, Material);
		}
	}

	ConfigureAllocator();
	SetMode(Mode);

	if (Sim)
	{
		Sim->OnCommand.AddWeakLambda(this, [this](const FString& Command, const TSharedPtr<FJsonObject>& Arguments)
		{
			if (Command == TEXT("set_mode"))
			{
				const FString Requested = Arguments->GetStringField(TEXT("mode"));
				for (const ESubseaControlMode Candidate : { ESubseaControlMode::Manual, ESubseaControlMode::StationKeep, ESubseaControlMode::PipelineFollow })
				{
					if (Requested == ModeName(Candidate))
					{
						SetMode(Candidate);
					}
				}
			}
			else if (Command == TEXT("camera"))
			{
				const FString Requested = Arguments->GetStringField(TEXT("shot"));
				double Orbit = -1.0;
				Arguments->TryGetNumberField(TEXT("orbit_dps"), Orbit);
				double YawOffset = 0.0;
				Arguments->TryGetNumberField(TEXT("yaw_offset"), YawOffset);
				for (uint8 Index = 0; Index <= static_cast<uint8>(ESubseaCameraShot::Flyby); ++Index)
				{
					const ESubseaCameraShot Candidate = static_cast<ESubseaCameraShot>(Index);
					if (Requested == ShotName(Candidate))
					{
						SetCameraShot(Candidate, static_cast<float>(Orbit));
						// Optional starting angle for the circling shots, degrees.
						OrbitYaw = static_cast<float>(YawOffset);
					}
				}
			}
			else if (Command == TEXT("lights"))
			{
				bool bOn = true;
				Arguments->TryGetBoolField(TEXT("on"), bOn);
				LightLeft->SetVisibility(bOn);
				LightRight->SetVisibility(bOn);
			}
			else if (Command == TEXT("fail_thruster"))
			{
				int32 Index = 0;
				bool bFailed = true;
				Arguments->TryGetNumberField(TEXT("index"), Index);
				Arguments->TryGetBoolField(TEXT("failed"), bFailed);
				SetThrusterFailed(Index, bFailed);
			}
		});
	}
}

void ASubseaROV::ConfigureAllocator()
{
	// Geometry is taken from the live components relative to the centre of mass, so the allocation
	// stays correct if a thruster is moved in the editor. A failed thruster contributes a zero
	// column: the pseudo-inverse then assigns it no force and redistributes its share.
	const FTransform BodyToWorld = Hull->GetComponentTransform();
	const FVector CentreOfMass = Hull->IsSimulatingPhysics() ? Hull->GetCenterOfMass() : BodyToWorld.GetLocation();

	TArray<Subsea::FThrusterGeometry> Geometry;
	Geometry.Reserve(Thrusters.Num());
	for (const USubseaThrusterComponent* Thruster : Thrusters)
	{
		Subsea::FThrusterGeometry& Entry = Geometry.AddDefaulted_GetRef();
		Entry.PositionM = BodyToWorld.GetRotation().UnrotateVector(Thruster->GetComponentLocation() - CentreOfMass) * 0.01;
		Entry.Direction = Thruster->bFailed
			? FVector::ZeroVector
			: BodyToWorld.GetRotation().UnrotateVector(Thruster->GetForwardVector());
	}

	// GetSafeNormal of a zero vector is zero, which is exactly the zero column wanted for a failure.
	if (!Allocator.Configure(Geometry, 0.02))
	{
		UE_LOG(LogSubseaSim, Error, TEXT("%s: thrust allocation is singular"), *GetName());
	}
	ThrusterForces.SetNumZeroed(Thrusters.Num());
}

void ASubseaROV::SetThrusterFailed(const int32 Index, const bool bFailed)
{
	if (Thrusters.IsValidIndex(Index) && Thrusters[Index]->bFailed != bFailed)
	{
		Thrusters[Index]->bFailed = bFailed;
		ConfigureAllocator();
		UE_LOG(LogSubseaSim, Log, TEXT("Thruster %d %s; allocation re-solved"), Index, bFailed ? TEXT("FAILED") : TEXT("restored"));
	}
}

void ASubseaROV::SetMode(const ESubseaControlMode NewMode)
{
	Mode = NewMode;
	HoldPositionCm = GetActorLocation();
	HoldZCm = HoldPositionCm.Z;
	HoldHeading = GetActorForwardVector().GetSafeNormal2D();
	SurgePid.Reset();
	SwayPid.Reset();
	DepthPid.Reset();
}

bool ASubseaROV::KeyDown(const FKey& Key) const
{
	const APlayerController* PlayerController = Cast<APlayerController>(GetController());
	return PlayerController && PlayerController->IsInputKeyDown(Key);
}

bool ASubseaROV::KeyPressed(const FKey& Key) const
{
	const APlayerController* PlayerController = Cast<APlayerController>(GetController());
	return PlayerController && PlayerController->WasInputKeyJustPressed(Key);
}

void ASubseaROV::ReadPilotInput(const double DeltaSeconds)
{
	PilotTranslate = FVector(
		(KeyDown(EKeys::W) ? 1.0 : 0.0) - (KeyDown(EKeys::S) ? 1.0 : 0.0),
		(KeyDown(EKeys::D) ? 1.0 : 0.0) - (KeyDown(EKeys::A) ? 1.0 : 0.0),
		(KeyDown(EKeys::SpaceBar) ? 1.0 : 0.0) - (KeyDown(EKeys::LeftShift) ? 1.0 : 0.0));
	PilotYaw = (KeyDown(EKeys::E) ? 1.0 : 0.0) - (KeyDown(EKeys::Q) ? 1.0 : 0.0);

	if (KeyPressed(EKeys::One)) { SetMode(ESubseaControlMode::Manual); }
	if (KeyPressed(EKeys::Two)) { SetMode(ESubseaControlMode::StationKeep); }
	if (KeyPressed(EKeys::Three)) { SetMode(ESubseaControlMode::PipelineFollow); }
	if (KeyPressed(EKeys::F)) { SetThrusterFailed(0, !Thrusters[0]->bFailed); }
	if (KeyPressed(EKeys::C))
	{
		SetCameraShot(static_cast<ESubseaCameraShot>((static_cast<uint8>(CameraShot) + 1) % (static_cast<uint8>(ESubseaCameraShot::Flyby) + 1)));
	}
	if (KeyPressed(EKeys::L))
	{
		LightLeft->ToggleVisibility();
		LightRight->ToggleVisibility();
	}
	if (Sim)
	{
		const double Change = (KeyDown(EKeys::T) ? 1.0 : 0.0) - (KeyDown(EKeys::G) ? 1.0 : 0.0);
		Sim->Turbidity = FMath::Clamp(Sim->Turbidity + Change * 0.25 * DeltaSeconds, 0.0, 1.0);
	}

	// Any stick input while on autopilot hands control back to the pilot.
	if (Mode != ESubseaControlMode::Manual && (!PilotTranslate.IsNearlyZero() || !FMath::IsNearlyZero(PilotYaw)))
	{
		SetMode(ESubseaControlMode::Manual);
	}
}

ASubseaROV::FGuidance ASubseaROV::GuidePipeline()
{
	FGuidance Guidance;
	const TArray<FVector>& Path = Environment->GetPipelinePath();
	const FVector Position = GetActorLocation();

	// Advance to the closest path point, searching only a short window ahead so the vehicle cannot
	// jump to a different part of a path that doubles back.
	const int32 Last = Path.Num() - 1;
	for (int32 Step = 0; Step < 8; ++Step)
	{
		const int32 Next = PathIndex + PathDirection;
		if (Next < 0 || Next > Last)
		{
			break;
		}
		if (FVector::DistSquared2D(Path[Next], Position) < FVector::DistSquared2D(Path[PathIndex], Position))
		{
			PathIndex = Next;
		}
		else
		{
			break;
		}
	}

	// Turn round at either end so an unattended demo runs indefinitely.
	if ((PathDirection > 0 && PathIndex >= Last - 1) || (PathDirection < 0 && PathIndex <= 1))
	{
		PathDirection = -PathDirection;
	}

	const int32 Neighbour = FMath::Clamp(PathIndex + PathDirection, 0, Last);
	const FVector Tangent = (Path[Neighbour] - Path[PathIndex]).GetSafeNormal2D();

	// Signed cross-track error: positive when the vehicle is to the right of the direction of travel.
	const FVector Offset = Position - Path[PathIndex];
	const FVector Right = FVector::CrossProduct(FVector::UpVector, Tangent);
	CrossTrackM = FVector::DotProduct(Offset, Right) * 0.01;

	// Line-of-sight guidance: fly along the tangent while converging on the line, with the
	// correction speed bounded so a large initial offset does not produce a violent dash.
	const double Correction = FMath::Clamp(-CrossTrackM * 0.6, -0.35, 0.35);
	Guidance.VelocityWorldMps = Tangent * SurveySpeedMps + Right * Correction;

	const double LookaheadCm = LookaheadM * 100.0;
	int32 Ahead = PathIndex;
	while (FVector::Dist2D(Path[Ahead], Path[PathIndex]) < LookaheadCm)
	{
		const int32 Next = Ahead + PathDirection;
		if (Next < 0 || Next > Last)
		{
			break;
		}
		Ahead = Next;
	}
	const FVector ToAhead = (Path[Ahead] - Position).GetSafeNormal2D();
	Guidance.HeadingDirection = ToAhead.IsNearlyZero() ? Tangent : ToAhead;
	Guidance.TargetZCm = Path[PathIndex].Z + StandOffM * 100.0;
	return Guidance;
}

ASubseaROV::FGuidance ASubseaROV::ComputeGuidance(const double DeltaSeconds)
{
	FGuidance Guidance;
	const FQuat Rotation = GetActorQuat();

	switch (Mode)
	{
	case ESubseaControlMode::PipelineFollow:
		if (Environment && Environment->GetPipelinePath().Num() >= 2)
		{
			return GuidePipeline();
		}
		[[fallthrough]];

	case ESubseaControlMode::StationKeep:
	{
		// Proportional position loop feeding the velocity loops, with the approach speed capped.
		const FVector ErrorM = (HoldPositionCm - GetActorLocation()) * 0.01;
		Guidance.VelocityWorldMps = (FVector(ErrorM.X, ErrorM.Y, 0.0) * 0.8).GetClampedToMaxSize(0.5);
		Guidance.HeadingDirection = HoldHeading;
		Guidance.TargetZCm = HoldZCm;
		CrossTrackM = FVector(ErrorM.X, ErrorM.Y, 0.0).Size();
		return Guidance;
	}

	case ESubseaControlMode::Manual:
	default:
	{
		// Sticks command body-frame velocity, which is what a pilot expects in a current.
		const FVector Forward = Rotation.GetForwardVector().GetSafeNormal2D();
		const FVector Right = FVector::CrossProduct(FVector::UpVector, Forward);
		Guidance.VelocityWorldMps = Forward * (PilotTranslate.X * 1.0) + Right * (PilotTranslate.Y * 0.7);

		// Depth hold engages whenever the heave stick is centred.
		if (!FMath::IsNearlyZero(PilotTranslate.Z))
		{
			Guidance.bHoldDepth = false;
			Guidance.ManualHeaveN = PilotTranslate.Z * PilotForceN;
			HoldZCm = GetActorLocation().Z;
		}
		Guidance.TargetZCm = HoldZCm;

		// Same idea for heading.
		if (!FMath::IsNearlyZero(PilotYaw))
		{
			Guidance.bManualYaw = true;
			Guidance.ManualYawNm = PilotYaw * PilotYawNm;
			HoldHeading = Forward;
		}
		Guidance.HeadingDirection = HoldHeading;
		CrossTrackM = 0.0;
		return Guidance;
	}
	}
}

Subsea::FWrench ASubseaROV::ComputeWrench(const FGuidance& Guidance, const double DeltaSeconds)
{
	const FQuat Rotation = GetActorQuat();
	const FVector VelocityWorldMps = Hull->GetPhysicsLinearVelocity() * 0.01;
	const FVector AngularBody = Rotation.UnrotateVector(Hull->GetPhysicsAngularVelocityInRadians());

	// Horizontal velocity loops run in a heading-aligned frame so roll and pitch do not couple in.
	const FVector Forward = Rotation.GetForwardVector().GetSafeNormal2D();
	const FVector Right = FVector::CrossProduct(FVector::UpVector, Forward);
	const double SurgeNow = FVector::DotProduct(VelocityWorldMps, Forward);
	const double SwayNow = FVector::DotProduct(VelocityWorldMps, Right);
	const double SurgeWanted = FVector::DotProduct(Guidance.VelocityWorldMps, Forward);
	const double SwayWanted = FVector::DotProduct(Guidance.VelocityWorldMps, Right);

	const double SurgeN = SurgePid.Step(SurgeWanted - SurgeNow, SurgeNow, DeltaSeconds);
	const double SwayN = SwayPid.Step(SwayWanted - SwayNow, SwayNow, DeltaSeconds);

	double HeaveN = Guidance.ManualHeaveN;
	if (Guidance.bHoldDepth)
	{
		const double ZNowM = GetActorLocation().Z * 0.01;
		HeaveN = DepthPid.Step(Guidance.TargetZCm * 0.01 - ZNowM, ZNowM, DeltaSeconds);
	}

	const FVector ForceWorld = Forward * SurgeN + Right * SwayN + FVector::UpVector * HeaveN;

	// Attitude is controlled with vector errors rather than Euler angles, which avoids both gimbal
	// lock and the sign conventions of FRotator. Rotating a unit vector a towards b needs angular
	// velocity along a x b, so the torque demand is proportional to that cross product.
	FVector TorqueWorld = FVector::ZeroVector;

	// Level: bring the body's up axis onto world up.
	const FVector Up = Rotation.GetUpVector();
	TorqueWorld += FVector::CrossProduct(Up, FVector::UpVector) * LevelGains.X;

	// Heading: bring the horizontal forward axis onto the wanted direction.
	if (Guidance.bManualYaw)
	{
		TorqueWorld.Z += Guidance.ManualYawNm;
	}
	else
	{
		const FVector Wanted = Guidance.HeadingDirection.GetSafeNormal2D();
		const double Sine = FVector::CrossProduct(Forward, Wanted).Z;
		const double Cosine = FVector::DotProduct(Forward, Wanted);
		TorqueWorld.Z += FMath::Atan2(Sine, Cosine) * HeadingGains.X;
	}

	Subsea::FWrench Wrench;
	Wrench.Force = Rotation.UnrotateVector(ForceWorld);
	Wrench.Torque = Rotation.UnrotateVector(TorqueWorld);
	// Rate damping in the body frame.
	Wrench.Torque.X -= AngularBody.X * LevelGains.Y;
	Wrench.Torque.Y -= AngularBody.Y * LevelGains.Y;
	Wrench.Torque.Z -= AngularBody.Z * HeadingGains.Y;
	return Wrench;
}

void ASubseaROV::UpdateState()
{
	const FVector Position = GetActorLocation();
	DepthM = Sim ? Sim->DepthAt(Position) : -Position.Z * 0.01;
	SpeedMps = Hull->GetPhysicsLinearVelocity().Size() * 0.01;

	// Compass heading: 0 = +X ("north" of the site), increasing clockwise seen from above.
	const FVector Forward = GetActorForwardVector();
	HeadingDeg = FMath::Fmod(FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X)) + 360.0, 360.0);

	// Altimeter: single beam straight down.
	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(SubseaAltimeter), false, this);
	const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, Position, Position - FVector(0.0, 0.0, 10000.0), ECC_Visibility, Params);
	AltitudeM = bHit ? Hit.Distance * 0.01 : -1.0;
}

void ASubseaROV::Tick(const float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	SCOPE_CYCLE_COUNTER(STAT_SubseaControl);

	if (!Allocator.IsConfigured() || DeltaSeconds <= 0.0f)
	{
		return;
	}

	ReadPilotInput(DeltaSeconds);
	UpdateState();

	const FGuidance Guidance = ComputeGuidance(DeltaSeconds);
	const Subsea::FWrench Wrench = ComputeWrench(Guidance, DeltaSeconds);

	const USubseaThrusterComponent* Reference = Thrusters[0];
	Allocator.AllocateSaturated(Wrench, Reference->MaxForwardN, Reference->MaxReverseN, ThrusterForces);
	for (int32 Index = 0; Index < Thrusters.Num(); ++Index)
	{
		Thrusters[Index]->SetCommandN(ThrusterForces[Index]);
	}

	UpdateCamera(DeltaSeconds);
	Publish(DeltaSeconds);
}

void ASubseaROV::Publish(const double DeltaSeconds)
{
	if (!Sim || Sim->GetNumStreamClients() == 0)
	{
		return;
	}

	const FQuat Rotation = GetActorQuat();

	TransformTimer -= DeltaSeconds;
	if (TransformTimer <= 0.0)
	{
		TransformTimer += 1.0 / TransformRateHz;
		TransformTimer = FMath::Max(TransformTimer, 0.0);
		Sim->PublishTransform("base_link", "map", FTransform(Rotation, GetActorLocation()),
			Rotation.UnrotateVector(Hull->GetPhysicsLinearVelocity()),
			Rotation.UnrotateVector(Hull->GetPhysicsAngularVelocityInRadians()));
	}

	// The sensor mount is static, but late joiners need it, so it is repeated at a low rate.
	StaticTransformTimer -= DeltaSeconds;
	if (StaticTransformTimer <= 0.0)
	{
		StaticTransformTimer = 0.5;
		// Built from world poses rather than GetRelativeTransform, which divides by the hull's
		// non-uniform scale.
		const auto MountOf = [this, &Rotation](const USceneComponent* Sensor)
		{
			return FTransform(Rotation.Inverse() * Sensor->GetComponentQuat(), Rotation.UnrotateVector(Sensor->GetComponentLocation() - GetActorLocation()));
		};
		Sim->PublishTransform(TCHAR_TO_ANSI(*DepthCamera->FrameId), "base_link", MountOf(DepthCamera), FVector::ZeroVector, FVector::ZeroVector);
		Sim->PublishTransform(TCHAR_TO_ANSI(*Sonar->FrameId), "base_link", MountOf(Sonar), FVector::ZeroVector, FVector::ZeroVector);
	}

	TelemetryTimer -= DeltaSeconds;
	if (TelemetryTimer <= 0.0)
	{
		TelemetryTimer = 1.0 / TelemetryRateHz;

		const TSharedRef<FJsonObject> Json = MakeShared<FJsonObject>();
		Json->SetNumberField(TEXT("depth_m"), DepthM);
		Json->SetNumberField(TEXT("altitude_m"), AltitudeM);
		Json->SetNumberField(TEXT("heading_deg"), HeadingDeg);
		Json->SetNumberField(TEXT("speed_mps"), SpeedMps);
		Json->SetNumberField(TEXT("cross_track_m"), CrossTrackM);
		Json->SetStringField(TEXT("mode"), ModeName(Mode));
		Json->SetNumberField(TEXT("waypoint"), PathIndex);
		Json->SetNumberField(TEXT("turbidity"), Sim->Turbidity);

		TArray<TSharedPtr<FJsonValue>> ThrusterValues;
		for (const USubseaThrusterComponent* Thruster : Thrusters)
		{
			ThrusterValues.Add(MakeShared<FJsonValueNumber>(Thruster->GetNormalisedThrust()));
		}
		Json->SetArrayField(TEXT("thrusters"), ThrusterValues);

		const FSubseaSensorStats& SensorStats = DepthCamera->GetStats();
		Json->SetNumberField(TEXT("sensor_ms"), SensorStats.LastProcessMs.load());
		Json->SetNumberField(TEXT("sensor_latency_ms"), SensorStats.LastLatencyMs.load());
		Json->SetNumberField(TEXT("points"), SensorStats.LastNumPoints.load());
		Json->SetNumberField(TEXT("fps"), DeltaSeconds > 0.0 ? 1.0 / DeltaSeconds : 0.0);
		Json->SetNumberField(TEXT("sonar_ms"), Sonar->GetLastPingMs());
		Json->SetNumberField(TEXT("sonar_rays"), Sonar->GetRaysPerPing());
		Json->SetStringField(TEXT("camera"), ShotName(CameraShot));
		Sim->PublishTelemetry(Json);
	}
}

const TCHAR* ASubseaROV::ShotName(const ESubseaCameraShot Shot)
{
	switch (Shot)
	{
	case ESubseaCameraShot::Chase: return TEXT("chase");
	case ESubseaCameraShot::Orbit: return TEXT("orbit");
	case ESubseaCameraShot::Wide: return TEXT("wide");
	case ESubseaCameraShot::Side: return TEXT("side");
	case ESubseaCameraShot::Top: return TEXT("top");
	case ESubseaCameraShot::Fpv: return TEXT("fpv");
	case ESubseaCameraShot::Flyby: return TEXT("flyby");
	default: return TEXT("unknown");
	}
}

void ASubseaROV::SetCameraShot(const ESubseaCameraShot Shot, const float InOrbitDegPerSec)
{
	CameraShot = Shot;
	OrbitYaw = 0.0f;
	switch (Shot)
	{
	case ESubseaCameraShot::Orbit: OrbitDegPerSec = 14.0f; break;
	case ESubseaCameraShot::Wide: OrbitDegPerSec = 5.0f; break;
	default: OrbitDegPerSec = 0.0f; break;
	}
	if (InOrbitDegPerSec >= 0.0f)
	{
		OrbitDegPerSec = InOrbitDegPerSec;
	}

	const bool bBoomShot = Shot != ESubseaCameraShot::Fpv && Shot != ESubseaCameraShot::Flyby;
	ChaseCamera->SetActive(bBoomShot);
	// The fill rides with whichever external camera is live; the onboard view has none.
	const bool bFlyby = Shot == ESubseaCameraShot::Flyby;
	CameraFill->AttachToComponent(bFlyby ? FlybyCamera.Get() : ChaseCamera.Get(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	CameraFill->SetRelativeLocation(FVector(40.0, 0.0, 60.0));
	CameraFill->SetIntensity(bFlyby ? 900.0f : 180.0f);
	CameraFill->SetAttenuationRadius(bFlyby ? 1800.0f : 900.0f);
	CameraFill->SetVisibility(Shot != ESubseaCameraShot::Fpv);
	FpvCamera->SetActive(Shot == ESubseaCameraShot::Fpv);
	FlybyCamera->SetActive(Shot == ESubseaCameraShot::Flyby);

	// Only the pilot's chase view lags; every other shot is rigid so recorded cuts are clean.
	const bool bChase = Shot == ESubseaCameraShot::Chase;
	CameraBoom->bEnableCameraLag = bChase;
	CameraBoom->bEnableCameraRotationLag = bChase;

	if (Shot == ESubseaCameraShot::Flyby)
	{
		// Plant the camera ahead of the vehicle and off to one side, at about its height.
		const FVector Forward = GetActorForwardVector().GetSafeNormal2D();
		const FVector Right = FVector::CrossProduct(FVector::UpVector, Forward);
		// Above and to the side, looking down on the vehicle and the lit seabed, out of the lights' throw.
		FlybyPoint = GetActorLocation() + Forward * 520.0 + Right * 430.0 + FVector(0.0, 0.0, 170.0);
	}
	UpdateCamera(0.0);
}

void ASubseaROV::UpdateCamera(const double DeltaSeconds)
{
	OrbitYaw = FMath::Fmod(OrbitYaw + OrbitDegPerSec * static_cast<float>(DeltaSeconds), 360.0f);

	float Arm = 330.0f;
	FRotator Rotation(-22.0, 0.0, 0.0);
	switch (CameraShot)
	{
	// The boom places the camera opposite its yaw: 0 is directly behind. Start rear-left and sweep
	// round to the side, never in front where the vehicle's lights would shine into the lens.
	case ESubseaCameraShot::Orbit: Arm = 260.0f; Rotation = FRotator(-16.0, -40.0 + OrbitYaw, 0.0); break;
	case ESubseaCameraShot::Wide: Arm = 1500.0f; Rotation = FRotator(-24.0, 200.0 + OrbitYaw, 0.0); break;
	case ESubseaCameraShot::Side: Arm = 290.0f; Rotation = FRotator(-4.0, 95.0 + OrbitYaw, 0.0); break;
	case ESubseaCameraShot::Top: Arm = 1000.0f; Rotation = FRotator(-84.0, OrbitYaw, 0.0); break;
	default: break;
	}
	CameraBoom->TargetArmLength = Arm;
	CameraBoom->SetRelativeRotation(Rotation);

	if (CameraShot == ESubseaCameraShot::Flyby)
	{
		const FVector Target = GetActorLocation();
		const double Distance = FVector::Dist(FlybyPoint, Target);
		// Once the vehicle is well past, leapfrog ahead for the next pass.
		if (Distance > 1600.0 && FVector::DotProduct(Target - FlybyPoint, GetActorForwardVector()) > 0.0)
		{
			SetCameraShot(ESubseaCameraShot::Flyby);
			return;
		}
		FlybyCamera->SetWorldLocationAndRotation(FlybyPoint, (Target - FlybyPoint).Rotation());
		// Zoom to keep the vehicle a similar size in frame as it approaches and passes.
		const float Fov = FMath::RadiansToDegrees(2.0f * FMath::Atan(150.0f / FMath::Max(static_cast<float>(Distance), 1.0f)));
		FlybyCamera->SetFieldOfView(FMath::Clamp(Fov, 22.0f, 75.0f));
	}
}