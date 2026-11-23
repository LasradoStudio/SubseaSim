#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "SubseaHydrodynamicsComponent.generated.h"

class UPrimitiveComponent;
class USubseaSimSubsystem;

/**
 * 6-DOF hydrodynamics for a fully submerged rigid body, after Fossen's marine craft model:
 *
 *     (M_rb + M_a) dv/dt + D(v_r) v_r + g(eta) = tau
 *
 *  - Restoring forces g(eta): weight acts at the centre of gravity, buoyancy (rho g V) at the centre
 *    of buoyancy. Placing the CoB above the CoG gives the passive roll/pitch righting moment real
 *    ROVs rely on.
 *  - Damping D(v_r) v_r: linear skin friction plus quadratic form drag, per body axis, evaluated on
 *    the velocity relative to the water so an ocean current pushes the vehicle correctly.
 *  - Added mass M_a: the water the hull has to accelerate. Chaos bodies have a scalar mass, so the
 *    isotropic mean of the added mass is folded into the body's mass and the spurious extra weight
 *    is cancelled, keeping the solve implicit and stable where an explicit -M_a * a force is not.
 *
 * Forces are applied to the owning actor's simulating root primitive every physics tick.
 */
UCLASS(ClassGroup = (Subsea), meta = (BlueprintSpawnableComponent))
class SUBSEASIM_API USubseaHydrodynamicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USubseaHydrodynamicsComponent();

	/** Dry mass, kg. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics", meta = (ClampMin = "0.1"))
	double DryMassKg = 11.5;

	/** Displaced volume, m^3. Buoyancy is WaterDensity * g * Volume. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics", meta = (ClampMin = "0"))
	double DisplacedVolumeM3 = 0.01142;

	/** Centre of buoyancy relative to the centre of mass, body frame, cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics")
	FVector CentreOfBuoyancyCm = FVector(0.0, 0.0, 2.0);

	/** Added mass along body X/Y/Z, kg. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics")
	FVector AddedMassKg = FVector(5.5, 12.7, 14.57);

	/** Linear damping, N per m/s, body X/Y/Z. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics")
	FVector LinearDrag = FVector(4.03, 6.22, 5.18);

	/** Quadratic damping, N per (m/s)^2, body X/Y/Z. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics")
	FVector QuadraticDrag = FVector(18.18, 21.66, 36.99);

	/** Linear angular damping, N m per rad/s, about body X/Y/Z. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics")
	FVector AngularLinearDrag = FVector(0.07, 0.07, 0.07);

	/** Quadratic angular damping, N m per (rad/s)^2, about body X/Y/Z. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Hydrodynamics")
	FVector AngularQuadraticDrag = FVector(1.55, 1.55, 1.55);

	/** Velocity relative to the water in the body frame, m/s. Updated each tick. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Hydrodynamics")
	FVector GetRelativeVelocityBodyMps() const { return RelativeVelocityBody; }

	/** Net buoyancy minus weight, N. Positive floats. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Hydrodynamics")
	double GetNetBuoyancyN() const;

	/** Mass the rigid body is simulated with (dry + mean added mass), kg. */
	[[nodiscard]] double GetEffectiveMassKg() const;

	/** Drag force in the body frame for a body-frame relative velocity. Pure; exposed for tests. */
	[[nodiscard]] static FVector ComputeDamping(const FVector& Velocity, const FVector& Linear, const FVector& Quadratic);

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> Body;

	UPROPERTY(Transient)
	TObjectPtr<USubseaSimSubsystem> Sim;

	FVector RelativeVelocityBody = FVector::ZeroVector;
};
