#pragma once

#include "Components/SceneComponent.h"
#include "CoreMinimal.h"
#include "Net/SubseaProtocol.h"
#include <atomic>
#include "SubseaDepthCameraComponent.generated.h"

class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class USubseaSimSubsystem;
class FSubseaStreamServer;

/** Parameters of the underwater range-sensor error model. Plain data so workers can copy it. */
USTRUCT(BlueprintType)
struct SUBSEASIM_API FSubseaRangeNoiseModel
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0"))
	double MinRangeM = 0.25;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0"))
	double MaxRangeM = 12.0;

	/** Range noise floor, metres (1 sigma). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0"))
	double SigmaBaseM = 0.003;

	/** Range noise growth, metres per metre^2: stereo/structured-light error grows with range squared. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0"))
	double SigmaQuadratic = 0.0008;

	/** Beam attenuation coefficient in clear water, 1/m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0"))
	double AttenuationClear = 0.05;

	/** Beam attenuation coefficient at turbidity 1, 1/m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0"))
	double AttenuationTurbid = 0.9;

	/** Two-way signal fraction below which returns start to drop out. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0.0001"))
	double DetectionFloor = 0.02;
};

/** Thread-safe counters a sensor exposes for telemetry and benchmarking. */
struct FSubseaSensorStats
{
	std::atomic<int32> LastNumPoints{ 0 };
	std::atomic<float> LastProcessMs{ 0.0f };
	std::atomic<float> LastLatencyMs{ 0.0f };
	std::atomic<uint32> FramesPublished{ 0 };
	std::atomic<uint32> FramesSkipped{ 0 };
};

/**
 * RGB-D camera that turns the rendered scene into a coloured point cloud without ever blocking
 * the game thread or the render thread:
 *
 *   game thread    schedules a scene capture (HDR colour + scene depth in one float target)
 *   render thread  queues a non-blocking GPU readback, and later copies out the finished frame
 *   task graph     un-projects pixels to 3D in parallel, applies the underwater error model,
 *                  packs the wire frame and hands it to the stream server
 *
 * The conventional alternative, ReadPixels, flushes the render thread and stalls the frame for
 * the duration of the GPU round trip; this pipeline keeps frame time flat as resolution grows.
 * Points are emitted in the sensor frame using ROS axes (X forward, Y left, Z up, metres).
 */
UCLASS(ClassGroup = (Subsea), meta = (BlueprintSpawnableComponent))
class SUBSEASIM_API USubseaDepthCameraComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	USubseaDepthCameraComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera", meta = (ClampMin = "16", ClampMax = "4096"))
	int32 Width = 320;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera", meta = (ClampMin = "16", ClampMax = "4096"))
	int32 Height = 240;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera", meta = (ClampMin = "10", ClampMax = "170"))
	float HorizontalFovDeg = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera", meta = (ClampMin = "0.1", ClampMax = "120"))
	float RateHz = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera")
	FSubseaRangeNoiseModel Noise;

	/** Linear multiplier applied to scene colour before tone mapping into 8-bit point colours. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera", meta = (ClampMin = "0"))
	float ColourExposure = 1.0f;

	/** Scales exposure each frame so in-range pixels average mid-grey, like a camera's auto exposure. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera")
	bool bAutoExposure = true;

	/** Seed for the noise model. The same seed and scene give the same cloud, frame for frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera")
	int32 NoiseSeed = 1337;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera")
	FString FrameId = TEXT("depth_cam");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Depth Camera")
	bool bEnabled = true;

	/** Applies a new resolution at runtime (used by the benchmark). */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Depth Camera")
	void SetResolution(int32 NewWidth, int32 NewHeight);

	UFUNCTION(BlueprintCallable, Category = "Subsea|Depth Camera")
	UTextureRenderTarget2D* GetRenderTarget() const { return RenderTarget; }

	[[nodiscard]] const FSubseaSensorStats& GetStats() const { return *Stats; }

	/** Copy of the most recent cloud (sensor frame, ROS axes). Thread-safe. */
	void GetLatestCloud(TArray<Subsea::Protocol::FPackedPoint>& OutPoints) const;

	/** Writes the most recent cloud as a binary PLY. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Depth Camera")
	bool ExportLatestCloudPly(const FString& FilePath) const;

	/**
	 * The CPU half of the sensor: pixels -> noisy, packed points. Pure and thread-safe, so it can
	 * be unit tested and benchmarked without a GPU. Pixels are row-major RGBA float with scene
	 * depth (cm along the optical axis) in A.
	 */
	static void Unproject(TConstArrayView<FLinearColor> Pixels, int32 InWidth, int32 InHeight, float InHorizontalFovDeg,
		const FSubseaRangeNoiseModel& InNoise, double Turbidity, float Exposure, uint32 Seed,
		TArray<Subsea::Protocol::FPackedPoint>& OutPoints);

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	struct FSharedState;

	void CreateCaptureResources();
	void RequestCapture();
	void PollReadbacks();

	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> Capture;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> RenderTarget;

	UPROPERTY(Transient)
	TObjectPtr<USubseaSimSubsystem> Sim;

	/** Shared with the render thread and worker tasks; outlives the component if work is in flight. */
	TSharedPtr<FSharedState, ESPMode::ThreadSafe> State;
	TSharedPtr<FSubseaSensorStats, ESPMode::ThreadSafe> Stats;

	double TimeUntilCapture = 0.0;
	uint32 CaptureIndex = 0;
};
