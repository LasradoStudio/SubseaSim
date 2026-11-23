#pragma once

#include "CoreMinimal.h"

/**
 * Conversions between Unreal's frame (left-handed, X forward, Y right, Z up, centimetres) and the
 * ROS REP-103 frame (right-handed, X forward, Y left, Z up, metres).
 *
 * The two frames differ by a mirror in Y plus a uniform scale. A mirror has determinant -1, so
 * true vectors and pseudovectors transform differently:
 *   vector        v' =  M v          -> ( x, -y,  z)
 *   pseudovector  w' = det(M) M w    -> (-x,  y, -z)   (angular velocity, torque, quaternion axis)
 */
namespace Subsea::Frames
{
	inline constexpr double CmToM = 0.01;
	inline constexpr double MToCm = 100.0;

	/** Position / linear velocity / force direction: Unreal cm -> ROS m. */
	[[nodiscard]] inline FVector3d UnrealToRosVector(const FVector3d& V)
	{
		return FVector3d(V.X * CmToM, -V.Y * CmToM, V.Z * CmToM);
	}

	[[nodiscard]] inline FVector3d RosToUnrealVector(const FVector3d& V)
	{
		return FVector3d(V.X * MToCm, -V.Y * MToCm, V.Z * MToCm);
	}

	/** Angular velocity (rad/s) is a pseudovector and carries no length unit. */
	[[nodiscard]] inline FVector3d UnrealToRosAngular(const FVector3d& W)
	{
		return FVector3d(-W.X, W.Y, -W.Z);
	}

	[[nodiscard]] inline FVector3d RosToUnrealAngular(const FVector3d& W)
	{
		return FVector3d(-W.X, W.Y, -W.Z);
	}

	/** A unit quaternion's vector part is a pseudovector scaled by sin(theta/2); w is unchanged. */
	[[nodiscard]] inline FQuat4d UnrealToRosQuat(const FQuat4d& Q)
	{
		return FQuat4d(-Q.X, Q.Y, -Q.Z, Q.W);
	}

	[[nodiscard]] inline FQuat4d RosToUnrealQuat(const FQuat4d& Q)
	{
		return FQuat4d(-Q.X, Q.Y, -Q.Z, Q.W);
	}
}
