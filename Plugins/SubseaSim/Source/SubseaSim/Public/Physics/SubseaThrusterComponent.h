#pragma once

#include "Components/SceneComponent.h"
#include "CoreMinimal.h"
#include "SubseaThrusterComponent.generated.h"

class UPrimitiveComponent;

/**
 * One fixed thruster. Thrust acts along the component's +X axis at its location.
 *
 * The motor and propeller cannot change thrust instantly, so the commanded force passes through a
 * first-order lag; forward and reverse limits differ because propellers are asymmetric.
 * Defaults approximate a Blue Robotics T200 at 16 V.
 */
UCLASS(ClassGroup = (Subsea), meta = (BlueprintSpawnableComponent))
class SUBSEASIM_API USubseaThrusterComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	USubseaThrusterComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Thruster", meta = (ClampMin = "0"))
	double MaxForwardN = 51.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Thruster", meta = (ClampMin = "0"))
	double MaxReverseN = 40.2;

	/** Time constant of the thrust response, seconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Thruster", meta = (ClampMin = "0.001"))
	double TimeConstant = 0.12;

	/** Simulates a failed thruster for fault-injection testing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Thruster")
	bool bFailed = false;

	/** Requested force in newtons; clamped to the thruster's limits. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Thruster")
	void SetCommandN(double ForceN);

	UFUNCTION(BlueprintCallable, Category = "Subsea|Thruster")
	double GetThrustN() const { return ThrustN; }

	/** Current thrust as a fraction of the limit in its direction, -1..1. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Thruster")
	double GetNormalisedThrust() const;

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	UPROPERTY(Transient)
	TObjectPtr<UPrimitiveComponent> Body;

	double CommandN = 0.0;
	double ThrustN = 0.0;
};
