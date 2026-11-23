#include "Physics/SubseaHydrodynamicsComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "SubseaSimModule.h"
#include "SubseaSimSubsystem.h"

DECLARE_CYCLE_STAT(TEXT("Hydrodynamics"), STAT_SubseaHydro, STATGROUP_SubseaSim);

namespace
{
	constexpr double GravityMps2 = 9.80665;
	/** Unreal force is kg cm/s^2 and torque kg cm^2/s^2. */
	constexpr double NewtonToUnreal = 100.0;
	constexpr double NewtonMetreToUnreal = 10000.0;
}

USubseaHydrodynamicsComponent::USubseaHydrodynamicsComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

double USubseaHydrodynamicsComponent::GetEffectiveMassKg() const
{
	return DryMassKg + (AddedMassKg.X + AddedMassKg.Y + AddedMassKg.Z) / 3.0;
}

double USubseaHydrodynamicsComponent::GetNetBuoyancyN() const
{
	const double Density = Sim ? Sim->WaterDensity : 1025.0;
	return (Density * DisplacedVolumeM3 - DryMassKg) * GravityMps2;
}

FVector USubseaHydrodynamicsComponent::ComputeDamping(const FVector& Velocity, const FVector& Linear, const FVector& Quadratic)
{
	// F = -(Dl + Dq |v|) v, evaluated per axis (the decoupled diagonal form of Fossen's D(v)).
	return FVector(
		-(Linear.X + Quadratic.X * FMath::Abs(Velocity.X)) * Velocity.X,
		-(Linear.Y + Quadratic.Y * FMath::Abs(Velocity.Y)) * Velocity.Y,
		-(Linear.Z + Quadratic.Z * FMath::Abs(Velocity.Z)) * Velocity.Z);
}

void USubseaHydrodynamicsComponent::BeginPlay()
{
	Super::BeginPlay();

	Body = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
	Sim = GetWorld()->GetSubsystem<USubseaSimSubsystem>();

	if (!Body || !Body->IsSimulatingPhysics())
	{
		UE_LOG(LogSubseaSim, Warning, TEXT("%s: hydrodynamics needs a physics-simulating root primitive"), *GetOwner()->GetName());
		SetComponentTickEnabled(false);
		return;
	}

	Body->SetMassOverrideInKg(NAME_None, static_cast<float>(GetEffectiveMassKg()), true);
	// All damping comes from the model below; engine damping would double count it.
	Body->SetLinearDamping(0.0f);
	Body->SetAngularDamping(0.0f);
	Body->SetEnableGravity(true);
}

void USubseaHydrodynamicsComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	SCOPE_CYCLE_COUNTER(STAT_SubseaHydro);

	if (!Body || !Sim)
	{
		return;
	}

	const FTransform BodyToWorld = Body->GetComponentTransform();
	const FQuat Rotation = BodyToWorld.GetRotation();
	const FVector CentreOfMass = Body->GetCenterOfMass();

	// --- Restoring forces ------------------------------------------------------------------
	// Chaos applies weight for the effective mass. Cancel the share that belongs to added mass
	// (water has no weight in water) and add buoyancy at the centre of buoyancy.
	const double AddedMassMean = GetEffectiveMassKg() - DryMassKg;
	const double BuoyancyN = Sim->WaterDensity * DisplacedVolumeM3 * GravityMps2;

	Body->AddForce(FVector(0.0, 0.0, AddedMassMean * GravityMps2 * NewtonToUnreal));

	const FVector CentreOfBuoyancyWorld = CentreOfMass + Rotation.RotateVector(CentreOfBuoyancyCm);
	Body->AddForceAtLocation(FVector(0.0, 0.0, BuoyancyN * NewtonToUnreal), CentreOfBuoyancyWorld);

	// --- Damping on velocity relative to the water --------------------------------------------
	const FVector VelocityWorldMps = Body->GetPhysicsLinearVelocity() * 0.01;
	const FVector RelativeWorld = VelocityWorldMps - Sim->SampleCurrent(CentreOfMass);
	RelativeVelocityBody = Rotation.UnrotateVector(RelativeWorld);

	const FVector DragBodyN = ComputeDamping(RelativeVelocityBody, LinearDrag, QuadraticDrag);
	Body->AddForce(Rotation.RotateVector(DragBodyN) * NewtonToUnreal);

	const FVector AngularBody = Rotation.UnrotateVector(Body->GetPhysicsAngularVelocityInRadians());
	const FVector DragTorqueBodyNm = ComputeDamping(AngularBody, AngularLinearDrag, AngularQuadraticDrag);
	Body->AddTorqueInRadians(Rotation.RotateVector(DragTorqueBodyNm) * NewtonMetreToUnreal);
}
