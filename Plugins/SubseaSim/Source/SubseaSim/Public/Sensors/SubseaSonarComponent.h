#pragma once

#include "Components/SceneComponent.h"
#include "CoreMinimal.h"
#include "WorldCollision.h"
#include "SubseaSonarComponent.generated.h"

class UTexture2D;
class USubseaSimSubsystem;

/** One formed sonar image: intensity per (range bin, beam), bins rows, beams columns. Plain data. */
struct FSubseaSonarImage
{
	int32 NumBeams = 0;
	int32 NumBins = 0;
	float HorizontalFovRad = 0.0f;
	float MinRangeM = 0.0f;
	float MaxRangeM = 0.0f;
	double Stamp = 0.0;
	TArray<uint8> Intensity;
};

/**
 * Forward-looking multibeam imaging sonar, the primary perception sensor on real inspection ROVs
 * because it keeps working in water too turbid for cameras.
 *
 * Each ping fires a fan of beams. Every beam is sampled by several rays spread across the vertical
 * aperture and weighted by a Gaussian beam pattern. Rays run as engine async traces, so they are
 * resolved on worker threads during the frame and the game thread never waits for them. Echo
 * strength follows a simple acoustic model: Lambertian backscatter (strong when the beam meets a
 * surface head-on, weak at grazing angles), two-way absorption, and spreading loss compensated by
 * time-varied gain the way real sonar heads do. Speckle and a noise floor are added, then the image
 * is log-compressed to 8 bits. Acoustic shadows behind objects fall out of the ray model for free.
 */
UCLASS(ClassGroup = (Subsea), meta = (BlueprintSpawnableComponent))
class SUBSEASIM_API USubseaSonarComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	USubseaSonarComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "8", ClampMax = "1024"))
	int32 NumBeams = 256;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "16", ClampMax = "2048"))
	int32 NumBins = 256;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "10", ClampMax = "170"))
	float HorizontalFovDeg = 120.0f;

	/** Full vertical opening of each beam, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "1", ClampMax = "60"))
	float VerticalApertureDeg = 20.0f;

	/** Rays per beam across the vertical aperture. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "1", ClampMax = "64"))
	int32 RaysPerBeam = 16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "0"))
	float MinRangeM = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "1"))
	float MaxRangeM = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "0.1", ClampMax = "60"))
	float RateHz = 10.0f;

	/** Acoustic absorption, dB per metre (one way). About 0.1 dB/m near 1 MHz. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "0"))
	float AbsorptionDbPerM = 0.1f;

	/** Strength of multiplicative speckle, 0 = none. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "0", ClampMax = "1"))
	float Speckle = 0.6f;

	/** Additive noise floor relative to a perpendicular return at 1 m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "0"))
	float NoiseFloor = 0.0001f;

	/** Dynamic range mapped onto 0..255, dB. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar", meta = (ClampMin = "6"))
	float DisplayRangeDb = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar")
	int32 NoiseSeed = 4242;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar")
	FString FrameId = TEXT("sonar");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Sonar")
	bool bEnabled = true;

	/** Fan-shaped display image (what a pilot sees), refreshed after every ping. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Sonar")
	UTexture2D* GetFanTexture() const { return FanTexture; }

	[[nodiscard]] float GetLastPingMs() const { return LastPingMs; }
	[[nodiscard]] int32 GetRaysPerPing() const { return NumBeams * RaysPerBeam; }

	/** One ray's echo before image formation. Exposed so image formation can be unit tested. */
	struct FEcho
	{
		int32 Beam = 0;
		float RangeM = 0.0f;
		/** Linear echo energy after beam pattern, backscatter, absorption and gain. */
		float Energy = 0.0f;
		/** Range extent of the ray's footprint on the surface, metres. 0 = a point echo. */
		float SpreadM = 0.0f;
	};

	/**
	 * Forms an 8-bit sonar image from echoes. Pure and thread-safe. Each echo deposits its energy in
	 * every range bin its footprint covers (weighted by overlap), so a surface met at a grazing angle
	 * fills a continuous band as it would on real hardware. Speckle and the noise floor are applied,
	 * then the result is log-compressed.
	 */
	static void FormImage(TConstArrayView<FEcho> Echoes, int32 InNumBeams, int32 InNumBins, float InMinRangeM, float InMaxRangeM,
		float InSpeckle, float InNoiseFloor, float InDisplayRangeDb, uint32 Seed, TArray<uint8>& OutIntensity);

	/** Linear echo energy for one ray. Pure; exposed for tests. */
	static float EchoEnergy(float RangeM, float CosIncidence, float BeamWeight, float AbsorptionDbPerM);

	/** Renders the polar image as a fan (apex at bottom centre) into a Width x (Width*5/8) buffer, copper colour map. */
	static void RenderFan(const FSubseaSonarImage& Image, int32 Width, TArray<FColor>& OutPixels);

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	struct FPing
	{
		double Stamp = 0.0;
		double StartSeconds = 0.0;
		int32 Outstanding = 0;
		uint32 Seed = 0;
		TArray<FEcho> Echoes;
		/** Per ray: which beam it belongs to and its beam-pattern weight. */
		TArray<int32> RayBeam;
		TArray<float> RayWeight;
	};

	void StartPing();
	void OnTraceDone(const FTraceHandle& Handle, FTraceDatum& Data);
	void FinishPing();

	UPROPERTY(Transient)
	TObjectPtr<USubseaSimSubsystem> Sim;

	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> FanTexture;

	FTraceDelegate TraceDelegate;
	TUniquePtr<FPing> Ping;
	double TimeUntilPing = 0.0;
	uint32 PingIndex = 0;
	float LastPingMs = 0.0f;
	float RaySliceRad = 0.0f;
	int32 FanSize = 256;
};
