#pragma once

#include "Control/SubseaPid.h"
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Math/SubseaThrustAllocation.h"
#include "SubseaROV.generated.h"

class UStaticMeshComponent;
class USpringArmComponent;
class UCameraComponent;
class USpotLightComponent;
class UPointLightComponent;
class USubseaHydrodynamicsComponent;
class USubseaThrusterComponent;
class USubseaDepthCameraComponent;
class USubseaSonarComponent;
class USubseaSimSubsystem;
class ASubseaEnvironment;
class FJsonObject;

UENUM(BlueprintType)
enum class ESubseaControlMode : uint8
{
	/** Pilot commands surge/sway/heave/yaw; depth and attitude are still held automatically. */
	Manual,
	/** Hold the position and heading captured when the mode was entered, rejecting the current. */
	StationKeep,
	/** Fly along the pipeline at a fixed stand-off, turning round at each end. */
	PipelineFollow,
};

/** Camera shots for piloting and for recording footage. */
UENUM(BlueprintType)
enum class ESubseaCameraShot : uint8
{
	/** Behind and above, lagging smoothly: the pilot's default. */
	Chase,
	/** Close, slowly circling the vehicle. */
	Orbit,
	/** Pulled far back and circling, to show the site. */
	Wide,
	/** Abeam, level with the vehicle. */
	Side,
	/** Looking straight down from above. */
	Top,
	/** The vehicle's own forward camera. */
	Fpv,
	/** Fixed point ahead on the route that tracks the vehicle as it passes. */
	Flyby,
};

/**
 * Inspection-class ROV modelled on the BlueROV2 Heavy: eight thrusters (four vectored horizontal,
 * four vertical) giving full 6-DOF authority.
 *
 * Control is layered the way a real vehicle's is:
 *   guidance    mode logic produces a desired world velocity, depth and heading
 *   control     PID loops turn the errors into a desired body wrench
 *   allocation  a damped pseudo-inverse of the thruster configuration matrix turns the wrench into
 *               per-thruster forces, and is rebuilt on the fly when a thruster fails
 *   actuation   each thruster applies its lagged force to the rigid body
 */
UCLASS()
class SUBSEASIM_API ASubseaROV : public APawn
{
	GENERATED_BODY()

public:
	ASubseaROV();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control")
	ESubseaControlMode Mode = ESubseaControlMode::PipelineFollow;

	/** Survey speed along the pipeline, m/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control", meta = (ClampMin = "0"))
	double SurveySpeedMps = 0.45;

	/** Height held above the pipeline centreline, m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control", meta = (ClampMin = "0.3"))
	double StandOffM = 1.7;

	/** Distance ahead on the path that the vehicle steers towards, m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control", meta = (ClampMin = "0.5"))
	double LookaheadM = 2.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control")
	FSubseaPid SurgePid = FSubseaPid(70.0, 25.0, 0.0, 90.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control")
	FSubseaPid SwayPid = FSubseaPid(70.0, 25.0, 0.0, 90.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control")
	FSubseaPid DepthPid = FSubseaPid(90.0, 14.0, 70.0, 120.0);

	/** Heading loop: N m per rad, and N m per rad/s of yaw-rate feedback. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control")
	FVector2D HeadingGains = FVector2D(7.0, 3.0);

	/** Roll/pitch levelling: N m per rad, and N m per rad/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Control")
	FVector2D LevelGains = FVector2D(9.0, 1.6);

	UFUNCTION(BlueprintCallable, Category = "Subsea|Control")
	void SetMode(ESubseaControlMode NewMode);

	/** Fails or restores a thruster and re-solves the allocation around it. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Control")
	void SetThrusterFailed(int32 Index, bool bFailed);

	UFUNCTION(BlueprintCallable, Category = "Subsea|State")
	double GetDepthM() const { return DepthM; }

	UFUNCTION(BlueprintCallable, Category = "Subsea|State")
	double GetAltitudeM() const { return AltitudeM; }

	UFUNCTION(BlueprintCallable, Category = "Subsea|State")
	double GetHeadingDeg() const { return HeadingDeg; }

	UFUNCTION(BlueprintCallable, Category = "Subsea|State")
	double GetSpeedMps() const { return SpeedMps; }

	UFUNCTION(BlueprintCallable, Category = "Subsea|State")
	double GetCrossTrackErrorM() const { return CrossTrackM; }

	const TArray<TObjectPtr<USubseaThrusterComponent>>& GetThrusters() const { return Thrusters; }
	USubseaDepthCameraComponent* GetDepthCamera() const { return DepthCamera; }
	USubseaSonarComponent* GetSonar() const { return Sonar; }

	/** Switches camera shot. OrbitDegPerSec overrides the shot's default circling speed when non-negative. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Camera")
	void SetCameraShot(ESubseaCameraShot Shot, float OrbitDegPerSec = -1.0f);

	UFUNCTION(BlueprintCallable, Category = "Subsea|Camera")
	ESubseaCameraShot GetCameraShot() const { return CameraShot; }

	static const TCHAR* ShotName(ESubseaCameraShot Shot);
	static const TCHAR* ModeName(ESubseaControlMode InMode);

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	struct FGuidance
	{
		FVector VelocityWorldMps = FVector::ZeroVector;
		FVector HeadingDirection = FVector::ForwardVector;
		double TargetZCm = 0.0;
		bool bHoldDepth = true;
		double ManualHeaveN = 0.0;
		double ManualYawNm = 0.0;
		bool bManualYaw = false;
	};

	void ConfigureAllocator();
	void ReadPilotInput(double DeltaSeconds);
	FGuidance ComputeGuidance(double DeltaSeconds);
	FGuidance GuidePipeline();
	Subsea::FWrench ComputeWrench(const FGuidance& Guidance, double DeltaSeconds);
	void UpdateState();
	void Publish(double DeltaSeconds);
	bool KeyDown(const FKey& Key) const;
	bool KeyPressed(const FKey& Key) const;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UStaticMeshComponent> Hull;

	/** Non-colliding meshes that make up the visible vehicle. */
	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TArray<TObjectPtr<UStaticMeshComponent>> VisualParts;

	TArray<FLinearColor> VisualColours;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USpringArmComponent> CameraBoom;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UCameraComponent> ChaseCamera;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UPointLightComponent> CameraFill;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USpotLightComponent> LightLeft;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USpotLightComponent> LightRight;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USubseaHydrodynamicsComponent> Hydrodynamics;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USubseaDepthCameraComponent> DepthCamera;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USubseaSonarComponent> Sonar;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UCameraComponent> FpvCamera;

	/** Detached camera used by the flyby shot. */
	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UCameraComponent> FlybyCamera;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TArray<TObjectPtr<USubseaThrusterComponent>> Thrusters;

	UPROPERTY(Transient)
	TObjectPtr<USubseaSimSubsystem> Sim;

	UPROPERTY(Transient)
	TObjectPtr<ASubseaEnvironment> Environment;

	Subsea::FThrustAllocator Allocator;
	TArray<double> ThrusterForces;

	// Pilot input, -1..1.
	FVector PilotTranslate = FVector::ZeroVector;
	double PilotYaw = 0.0;

	// Guidance state.
	FVector HoldPositionCm = FVector::ZeroVector;
	FVector HoldHeading = FVector::ForwardVector;
	double HoldZCm = 0.0;
	int32 PathIndex = 0;
	int32 PathDirection = 1;

	// Derived state for the HUD and telemetry.
	double DepthM = 0.0;
	double AltitudeM = 0.0;
	double HeadingDeg = 0.0;
	double SpeedMps = 0.0;
	double CrossTrackM = 0.0;

	double TransformTimer = 0.0;
	double TelemetryTimer = 0.0;
	double StaticTransformTimer = 0.0;

	void UpdateCamera(double DeltaSeconds);
	ESubseaCameraShot CameraShot = ESubseaCameraShot::Chase;
	float OrbitDegPerSec = 0.0f;
	float OrbitYaw = 0.0f;
	FVector FlybyPoint = FVector::ZeroVector;
};
