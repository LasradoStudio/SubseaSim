#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/HUD.h"
#include "SubseaGameMode.generated.h"

class ASubseaROV;

/**
 * Makes any map a subsea scenario: if the level has no environment or vehicle, they are spawned,
 * so the demo runs from an empty map and from the command line with no authored content.
 */
UCLASS()
class SUBSEASIM_API ASubseaGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ASubseaGameMode();
	virtual void StartPlay() override;
};

/** On-screen pilot display: navigation state, thruster loads, sensor and stream health. */
UCLASS()
class SUBSEASIM_API ASubseaHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;
};
