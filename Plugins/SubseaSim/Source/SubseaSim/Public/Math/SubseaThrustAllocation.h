#pragma once

#include "CoreMinimal.h"

/**
 * Thrust allocation for an over-actuated vehicle.
 *
 * Each thruster i sits at body position r_i (m) and pushes along unit direction d_i. Its unit
 * wrench is the column b_i = [d_i ; r_i x d_i], so the 6xN configuration matrix B maps thruster
 * forces f (N) to a body wrench tau = B f (force in N, torque in N m).
 *
 * Allocation inverts that map with a damped least-squares pseudo-inverse,
 *     f = B^T (B B^T + lambda^2 I)^-1 tau,
 * which is the minimum-norm solution when B has full row rank and degrades gracefully (rather
 * than blowing up) near singular configurations or when a thruster is disabled.
 */
namespace Subsea
{
	struct FThrusterGeometry
	{
		FVector3d PositionM = FVector3d::ZeroVector;
		FVector3d Direction = FVector3d::ForwardVector;
	};

	/** Body wrench: force (N) then torque (N m), both in the body frame. */
	struct FWrench
	{
		FVector3d Force = FVector3d::ZeroVector;
		FVector3d Torque = FVector3d::ZeroVector;

		[[nodiscard]] double operator[](int32 Index) const
		{
			return Index < 3 ? Force[Index] : Torque[Index - 3];
		}
	};

	class SUBSEASIM_API FThrustAllocator
	{
	public:
		/** Builds B and caches the damped pseudo-inverse. Returns false if the system is unsolvable. */
		bool Configure(TConstArrayView<FThrusterGeometry> Thrusters, double Damping = 1e-3);

		/** Thruster forces (N) that best realise the wrench in the least-squares sense. */
		void Allocate(const FWrench& Desired, TArray<double>& OutForces) const;

		/**
		 * Allocates, then uniformly scales the result so no thruster exceeds its limit. Uniform
		 * scaling preserves the direction of the achieved wrench, which matters more for control
		 * than hitting the magnitude: clipping individual thrusters would rotate it.
		 */
		void AllocateSaturated(const FWrench& Desired, double MaxForwardN, double MaxReverseN, TArray<double>& OutForces) const;

		/** Wrench actually produced by a set of thruster forces (tau = B f). */
		[[nodiscard]] FWrench Resolve(TConstArrayView<double> Forces) const;

		[[nodiscard]] int32 NumThrusters() const { return NumThrusters_; }
		[[nodiscard]] bool IsConfigured() const { return NumThrusters_ > 0; }

	private:
		static constexpr int32 Dof = 6;

		int32 NumThrusters_ = 0;
		/** Row-major 6xN. */
		TArray<double> B_;
		/** Row-major Nx6 pseudo-inverse. */
		TArray<double> PInv_;
	};

	/** Solves A X = Rhs in place for a small dense system with partial pivoting. A is NxN row-major, Rhs is NxM. */
	SUBSEASIM_API bool SolveDense(TArray<double>& A, TArray<double>& Rhs, int32 N, int32 M);
}
