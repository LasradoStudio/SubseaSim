#pragma once

#include "CoreMinimal.h"
#include "Net/SubseaProtocol.h"
#include "Subsystems/WorldSubsystem.h"
#include "SubseaSimSubsystem.generated.h"

class FSubseaStreamServer;
class FJsonObject;

DECLARE_MULTICAST_DELEGATE_TwoParams(FOnSubseaCommand, const FString& /*Command*/, const TSharedPtr<FJsonObject>& /*Arguments*/);

/**
 * Per-world hub for the simulation: owns the water state every component samples (density,
 * current, turbidity, surface height) and the sensor stream server.
 *
 * Components depend on this subsystem rather than on each other, so a vehicle, a sensor or an
 * environment can each be dropped into a level on their own.
 */
UCLASS()
class SUBSEASIM_API USubseaSimSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	// --- Water state -------------------------------------------------------------------------

	/** Sea water density, kg/m^3. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water")
	double WaterDensity = 1025.0;

	/** World Z of the sea surface, cm. Depth is measured down from here. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water")
	double SurfaceZCm = 0.0;

	/** Steady ocean current in Unreal world axes, m/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water")
	FVector CurrentMps = FVector(0.0, 0.12, 0.0);

	/** Amplitude of the slowly varying part of the current, m/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water")
	double CurrentGustMps = 0.05;

	/** 0 = gin-clear, 1 = near-zero visibility. Drives fog and sensor degradation together. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water", meta = (ClampMin = "0", ClampMax = "1"))
	double Turbidity = 0.15;

	/** Current at a world position (Unreal axes, m/s), including the time-varying component. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Water")
	FVector SampleCurrent(const FVector& WorldPositionCm) const;

	UFUNCTION(BlueprintCallable, Category = "Subsea|Water")
	double DepthAt(const FVector& WorldPositionCm) const { return (SurfaceZCm - WorldPositionCm.Z) * 0.01; }

	// --- Presentation -----------------------------------------------------------------------

	/** Draw the pilot HUD. Turned off for clean recordings. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Presentation")
	bool bShowHud = true;

	/** True while a frame sequence is being written to Saved/Recordings. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Presentation")
	bool IsRecording() const { return RecordFramesLeft > 0; }

	// --- Sensor stream -----------------------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Stream")
	int32 StreamPort = 9870;

	/** Pose of a frame relative to its parent, both given in Unreal conventions; converted to ROS on the way out. */
	void PublishTransform(FAnsiStringView FrameId, FAnsiStringView ParentFrame, const FTransform& RelativeTransform,
		const FVector& LinearVelocityBodyCmS, const FVector& AngularVelocityBodyRadS);

	void PublishTelemetry(const TSharedRef<FJsonObject>& Json);

	/** Hands over a fully built frame. Safe to call from any thread. */
	void PublishFrame(Subsea::Protocol::FFramePtr Frame, bool bDroppable);

	/** Safe to call from any thread. */
	uint32 NextSeq(Subsea::Protocol::EMessageType Type);

	UFUNCTION(BlueprintCallable, Category = "Subsea|Stream")
	int32 GetNumStreamClients() const;

	UFUNCTION(BlueprintCallable, Category = "Subsea|Stream")
	double GetStreamMegabitsPerSecond() const { return StreamMbps; }

	/** The stream server, for producers that publish from worker threads. Null if the port was unavailable. */
	[[nodiscard]] TSharedPtr<FSubseaStreamServer, ESPMode::ThreadSafe> GetServer() const { return Server; }

	/** Simulation clock used to stamp every outgoing message, seconds. */
	[[nodiscard]] double SimTime() const;

	/** Fired on the game thread for each JSON command a stream client sends. */
	FOnSubseaCommand OnCommand;

	// UWorldSubsystem
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

	// FTickableGameObject
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

private:
	TSharedPtr<FSubseaStreamServer, ESPMode::ThreadSafe> Server;
	void StartRecording(const FString& Name, int32 Frames);

	bool bPaceRealtime = false;
	double LastPaceSeconds = 0.0;
	double TurbidityTarget = 0.0;
	double TurbidityRate = 0.0;
	FString RecordDirectory;
	int32 RecordFramesLeft = 0;
	int32 RecordFrameIndex = 0;
	uint64 LastBytesSent = 0;
	double RateWindowSeconds = 0.0;
	double StreamMbps = 0.0;
};
