#include "Math/SubseaThrustAllocation.h"

namespace Subsea
{
	bool SolveDense(TArray<double>& A, TArray<double>& Rhs, const int32 N, const int32 M)
	{
		check(A.Num() == N * N && Rhs.Num() == N * M);

		for (int32 Col = 0; Col < N; ++Col)
		{
			// Partial pivoting: pick the largest remaining entry in this column.
			int32 Pivot = Col;
			double Best = FMath::Abs(A[Col * N + Col]);
			for (int32 Row = Col + 1; Row < N; ++Row)
			{
				const double Candidate = FMath::Abs(A[Row * N + Col]);
				if (Candidate > Best)
				{
					Best = Candidate;
					Pivot = Row;
				}
			}
			if (Best < 1e-12)
			{
				return false;
			}
			if (Pivot != Col)
			{
				for (int32 K = 0; K < N; ++K)
				{
					Swap(A[Col * N + K], A[Pivot * N + K]);
				}
				for (int32 K = 0; K < M; ++K)
				{
					Swap(Rhs[Col * M + K], Rhs[Pivot * M + K]);
				}
			}

			const double Inv = 1.0 / A[Col * N + Col];
			for (int32 Row = 0; Row < N; ++Row)
			{
				if (Row == Col)
				{
					continue;
				}
				const double Factor = A[Row * N + Col] * Inv;
				if (Factor == 0.0)
				{
					continue;
				}
				for (int32 K = Col; K < N; ++K)
				{
					A[Row * N + K] -= Factor * A[Col * N + K];
				}
				for (int32 K = 0; K < M; ++K)
				{
					Rhs[Row * M + K] -= Factor * Rhs[Col * M + K];
				}
			}
		}

		for (int32 Row = 0; Row < N; ++Row)
		{
			const double Inv = 1.0 / A[Row * N + Row];
			for (int32 K = 0; K < M; ++K)
			{
				Rhs[Row * M + K] *= Inv;
			}
		}
		return true;
	}

	bool FThrustAllocator::Configure(const TConstArrayView<FThrusterGeometry> Thrusters, const double Damping)
	{
		NumThrusters_ = 0;
		const int32 N = Thrusters.Num();
		if (N == 0)
		{
			return false;
		}

		B_.SetNumZeroed(Dof * N);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector3d Dir = Thrusters[I].Direction.GetSafeNormal();
			const FVector3d Moment = FVector3d::CrossProduct(Thrusters[I].PositionM, Dir);
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				B_[Axis * N + I] = Dir[Axis];
				B_[(Axis + 3) * N + I] = Moment[Axis];
			}
		}

		// G = B B^T + lambda^2 I  (6x6, symmetric positive definite for lambda > 0)
		TArray<double> G;
		G.SetNumZeroed(Dof * Dof);
		for (int32 R = 0; R < Dof; ++R)
		{
			for (int32 C = 0; C < Dof; ++C)
			{
				double Sum = 0.0;
				for (int32 I = 0; I < N; ++I)
				{
					Sum += B_[R * N + I] * B_[C * N + I];
				}
				G[R * Dof + C] = Sum + (R == C ? Damping * Damping : 0.0);
			}
		}

		// Solve G X = I for X = G^-1, then PInv = B^T G^-1.
		TArray<double> GInv;
		GInv.SetNumZeroed(Dof * Dof);
		for (int32 I = 0; I < Dof; ++I)
		{
			GInv[I * Dof + I] = 1.0;
		}
		if (!SolveDense(G, GInv, Dof, Dof))
		{
			return false;
		}

		PInv_.SetNumZeroed(N * Dof);
		for (int32 I = 0; I < N; ++I)
		{
			for (int32 C = 0; C < Dof; ++C)
			{
				double Sum = 0.0;
				for (int32 R = 0; R < Dof; ++R)
				{
					Sum += B_[R * N + I] * GInv[R * Dof + C];
				}
				PInv_[I * Dof + C] = Sum;
			}
		}

		NumThrusters_ = N;
		return true;
	}

	void FThrustAllocator::Allocate(const FWrench& Desired, TArray<double>& OutForces) const
	{
		OutForces.SetNumUninitialized(NumThrusters_);
		for (int32 I = 0; I < NumThrusters_; ++I)
		{
			double Sum = 0.0;
			for (int32 C = 0; C < Dof; ++C)
			{
				Sum += PInv_[I * Dof + C] * Desired[C];
			}
			OutForces[I] = Sum;
		}
	}

	void FThrustAllocator::AllocateSaturated(const FWrench& Desired, const double MaxForwardN, const double MaxReverseN, TArray<double>& OutForces) const
	{
		Allocate(Desired, OutForces);

		double Scale = 1.0;
		for (const double Force : OutForces)
		{
			const double Limit = Force >= 0.0 ? MaxForwardN : MaxReverseN;
			const double Magnitude = FMath::Abs(Force);
			if (Magnitude > Limit && Magnitude > UE_DOUBLE_SMALL_NUMBER)
			{
				Scale = FMath::Min(Scale, Limit / Magnitude);
			}
		}
		if (Scale < 1.0)
		{
			for (double& Force : OutForces)
			{
				Force *= Scale;
			}
		}
	}

	FWrench FThrustAllocator::Resolve(const TConstArrayView<double> Forces) const
	{
		FWrench Out;
		const int32 N = FMath::Min(Forces.Num(), NumThrusters_);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			double Force = 0.0;
			double Torque = 0.0;
			for (int32 I = 0; I < N; ++I)
			{
				Force += B_[Axis * NumThrusters_ + I] * Forces[I];
				Torque += B_[(Axis + 3) * NumThrusters_ + I] * Forces[I];
			}
			Out.Force[Axis] = Force;
			Out.Torque[Axis] = Torque;
		}
		return Out;
	}
}
