#include "Physics/SubseaThrusterComponent.h"

#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"

USubseaThrusterComponent::USubseaThrusterComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void USubseaThrusterComponent::BeginPlay()
{
	Super::BeginPlay();
	Body = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
}

void USubseaThrusterComponent::SetCommandN(const double ForceN)
{
	CommandN = FMath::Clamp(ForceN, -MaxReverseN, MaxForwardN);
}

double USubseaThrusterComponent::GetNormalisedThrust() const
{
	const double Limit = ThrustN >= 0.0 ? MaxForwardN : MaxReverseN;
	return Limit > 0.0 ? ThrustN / Limit : 0.0;
}

void USubseaThrusterComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Exact discretisation of dT/dt = (cmd - T) / tau, stable for any step size.
	const double Target = bFailed ? 0.0 : CommandN;
	const double Alpha = 1.0 - FMath::Exp(-static_cast<double>(DeltaTime) / TimeConstant);
	ThrustN += (Target - ThrustN) * Alpha;

	if (Body && Body->IsSimulatingPhysics() && !FMath::IsNearlyZero(ThrustN, 1e-4))
	{
		constexpr double NewtonToUnreal = 100.0;
		Body->AddForceAtLocation(GetForwardVector() * (ThrustN * NewtonToUnreal), GetComponentLocation());
	}
}
