#pragma once

#include "CoreMinimal.h"
#include "SubseaPid.generated.h"

/**
 * PID with derivative-on-measurement (no setpoint kick), a first-order derivative filter and
 * conditional-integration anti-windup against the output limit.
 */
USTRUCT(BlueprintType)
struct SUBSEASIM_API FSubseaPid
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	double Kp = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	double Ki = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	double Kd = 0.0;

	/** Symmetric output clamp. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID", meta = (ClampMin = "0"))
	double OutputLimit = 100.0;

	/** Derivative low-pass time constant in seconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID", meta = (ClampMin = "0"))
	double DerivativeTau = 0.05;

	FSubseaPid() = default;
	FSubseaPid(const double InKp, const double InKi, const double InKd, const double InLimit)
		: Kp(InKp), Ki(InKi), Kd(InKd), OutputLimit(InLimit)
	{
	}

	void Reset()
	{
		Integral = 0.0;
		FilteredDerivative = 0.0;
		bHasPrevious = false;
	}

	/** Error is (setpoint - measurement); Measurement is passed separately for the derivative term. */
	double Step(const double Error, const double Measurement, const double Dt)
	{
		if (Dt <= UE_DOUBLE_SMALL_NUMBER)
		{
			return FMath::Clamp(Kp * Error + Integral, -OutputLimit, OutputLimit);
		}

		double RawDerivative = 0.0;
		if (bHasPrevious)
		{
			RawDerivative = -(Measurement - PreviousMeasurement) / Dt;
		}
		PreviousMeasurement = Measurement;
		bHasPrevious = true;

		const double Alpha = Dt / (DerivativeTau + Dt);
		FilteredDerivative += Alpha * (RawDerivative - FilteredDerivative);

		const double Unclamped = Kp * Error + Integral + Kd * FilteredDerivative;
		const double Output = FMath::Clamp(Unclamped, -OutputLimit, OutputLimit);

		// Only integrate when not saturated, or when the error would pull the output back in range.
		const bool bSaturated = Unclamped != Output;
		if (!bSaturated || (Error * Unclamped) < 0.0)
		{
			Integral = FMath::Clamp(Integral + Ki * Error * Dt, -OutputLimit, OutputLimit);
		}
		return Output;
	}

private:
	double Integral = 0.0;
	double FilteredDerivative = 0.0;
	double PreviousMeasurement = 0.0;
	bool bHasPrevious = false;
};
