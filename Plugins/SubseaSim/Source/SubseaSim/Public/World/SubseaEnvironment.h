#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SubseaEnvironment.generated.h"

class UProceduralMeshComponent;
class UInstancedStaticMeshComponent;
class UExponentialHeightFogComponent;
class UPostProcessComponent;
class UDirectionalLightComponent;
class UStaticMesh;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class USubseaSimSubsystem;

/**
 * Procedurally generated inspection site: an undulating seabed, a pipeline with flanges, free
 * spans and debris, a manifold structure, boulders, and the water column's look (fog, absorption,
 * dim surface light). Everything derives from a seed, so a scenario is reproducible from a single
 * integer and needs no authored assets.
 */
UCLASS()
class SUBSEASIM_API ASubseaEnvironment : public AActor
{
	GENERATED_BODY()

public:
	ASubseaEnvironment();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site")
	int32 Seed = 7;

	/** Extent along the pipeline (X), metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site", meta = (ClampMin = "20"))
	double LengthM = 160.0;

	/** Extent across the pipeline (Y), metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site", meta = (ClampMin = "10"))
	double WidthM = 70.0;

	/** Seabed grid spacing, metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site", meta = (ClampMin = "0.1"))
	double CellM = 0.5;

	/** Mean water depth at the site, metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site", meta = (ClampMin = "2"))
	double WaterDepthM = 30.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site", meta = (ClampMin = "0.05"))
	double PipeRadiusM = 0.25;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Site", meta = (ClampMin = "0"))
	int32 NumBoulders = 220;

	/** Suspended particles drifting around the viewer, purely visual (hidden from sensors). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water", meta = (ClampMin = "0", ClampMax = "20000"))
	int32 NumMarineSnow = 2600;

	/** Side of the box of particles kept around the viewer, metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsea|Water", meta = (ClampMin = "2"))
	double MarineSnowBoxM = 12.0;

	/** Rebuilds all geometry from the current parameters. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Subsea|Site")
	void Rebuild();

	/** Seabed elevation (world Z, cm) at a world XY position (cm). Analytic, so it is valid off-mesh too. */
	UFUNCTION(BlueprintCallable, Category = "Subsea|Site")
	double SeabedHeightCm(double WorldXCm, double WorldYCm) const;

	/** Pipeline centreline, world space, cm, ordered along +X. */
	const TArray<FVector>& GetPipelinePath() const { return PipelinePath; }

	/** A sensible spawn pose for a vehicle: above the start of the pipeline, facing along it. */
	FTransform GetVehicleStart() const;

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildSeabed();
	void BuildPipeline();
	void BuildBoulders();
	void BuildStructure();
	void ApplyWaterLook(double Turbidity);
	UMaterialInstanceDynamic* MakeColour(const FLinearColor& Colour, float Roughness, float DetailStrength = 0.0f, float Metallic = 0.0f);
	void BuildMarineSnow();
	void UpdateMarineSnow(double DeltaSeconds);

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UProceduralMeshComponent> Seabed;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UInstancedStaticMeshComponent> PipeSegments;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UInstancedStaticMeshComponent> Flanges;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UInstancedStaticMeshComponent> Boulders;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UInstancedStaticMeshComponent> Structure;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UInstancedStaticMeshComponent> Debris;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UInstancedStaticMeshComponent> MarineSnow;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UExponentialHeightFogComponent> Fog;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UPostProcessComponent> PostProcess;

	UPROPERTY(VisibleAnywhere, Category = "Subsea")
	TObjectPtr<UDirectionalLightComponent> SurfaceLight;

	UPROPERTY()
	TObjectPtr<UStaticMesh> CubeMesh;

	UPROPERTY()
	TObjectPtr<UStaticMesh> CylinderMesh;

	UPROPERTY()
	TObjectPtr<UStaticMesh> SphereMesh;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> BaseMaterial;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;

	UPROPERTY(Transient)
	TObjectPtr<USubseaSimSubsystem> Sim;

	TArray<FVector> PipelinePath;
	TArray<FTransform> SnowTransforms;
	TArray<FVector> SnowWobble;
	double AppliedTurbidity = -1.0;
};
