#include "Control/SubseaPid.h"
#include "Math/SubseaFrames.h"
#include "Math/SubseaThrustAllocation.h"
#include "Misc/AutomationTest.h"
#include "Net/SubseaProtocol.h"
#include "Physics/SubseaHydrodynamicsComponent.h"
#include "Sensors/SubseaDepthCameraComponent.h"
#include "Sensors/SubseaSonarComponent.h"
#include "SubseaSimModule.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter;

	TArray<Subsea::FThrusterGeometry> MakeHeavyLayout()
	{
		const double C = UE_DOUBLE_INV_SQRT_2;
		return {
			{ FVector3d(0.156, 0.111, 0.0), FVector3d(C, -C, 0.0) },
			{ FVector3d(0.156, -0.111, 0.0), FVector3d(C, C, 0.0) },
			{ FVector3d(-0.156, 0.111, 0.0), FVector3d(C, C, 0.0) },
			{ FVector3d(-0.156, -0.111, 0.0), FVector3d(C, -C, 0.0) },
			{ FVector3d(0.12, 0.218, 0.0), FVector3d(0.0, 0.0, 1.0) },
			{ FVector3d(0.12, -0.218, 0.0), FVector3d(0.0, 0.0, 1.0) },
			{ FVector3d(-0.12, 0.218, 0.0), FVector3d(0.0, 0.0, 1.0) },
			{ FVector3d(-0.12, -0.218, 0.0), FVector3d(0.0, 0.0, 1.0) },
		};
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaFramesTest, "SubseaSim.Math.FrameConversion", TestFlags)
bool FSubseaFramesTest::RunTest(const FString&)
{
	using namespace Subsea::Frames;

	const FVector3d Point(120.0, 45.0, -300.0);
	const FVector3d Ros = UnrealToRosVector(Point);
	TestTrue(TEXT("cm to m and Y flipped"), Ros.Equals(FVector3d(1.2, -0.45, -3.0), 1e-9));
	TestTrue(TEXT("vector round trip"), RosToUnrealVector(Ros).Equals(Point, 1e-9));

	// The defining property: rotating then converting must equal converting then rotating.
	const FQuat4d Rotation = FQuat4d(FVector3d(0.3, -0.5, 0.8).GetSafeNormal(), 1.1);
	const FVector3d RotatedThenConverted = UnrealToRosVector(Rotation.RotateVector(Point));
	const FVector3d ConvertedThenRotated = UnrealToRosQuat(Rotation).RotateVector(Ros);
	TestTrue(TEXT("quaternion conversion commutes with rotation"), RotatedThenConverted.Equals(ConvertedThenRotated, 1e-9));

	// Unreal yaw is clockwise seen from above; ROS yaw is counter-clockwise. A +90 degree Unreal
	// yaw must therefore turn ROS +X towards ROS -Y.
	const FQuat4d YawRight = FQuat4d(FVector3d::UpVector, UE_DOUBLE_HALF_PI);
	const FVector3d Turned = UnrealToRosQuat(YawRight).RotateVector(FVector3d(1.0, 0.0, 0.0));
	TestTrue(TEXT("yaw handedness"), Turned.Equals(FVector3d(0.0, -1.0, 0.0), 1e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaAllocationTest, "SubseaSim.Math.ThrustAllocation", TestFlags)
bool FSubseaAllocationTest::RunTest(const FString&)
{
	Subsea::FThrustAllocator Allocator;
	TArray<Subsea::FThrusterGeometry> Layout = MakeHeavyLayout();
	TestTrue(TEXT("configures"), Allocator.Configure(Layout, 1e-4));

	// Every wrench in a 6-DOF-capable layout must be reproduced by the allocated forces.
	const Subsea::FWrench Cases[] = {
		{ FVector3d(30.0, 0.0, 0.0), FVector3d::ZeroVector },
		{ FVector3d(0.0, -22.0, 0.0), FVector3d::ZeroVector },
		{ FVector3d(0.0, 0.0, 40.0), FVector3d::ZeroVector },
		{ FVector3d::ZeroVector, FVector3d(0.0, 0.0, 4.0) },
		{ FVector3d::ZeroVector, FVector3d(2.0, -1.5, 0.0) },
		{ FVector3d(12.0, -7.0, 20.0), FVector3d(1.0, 0.5, -2.0) },
	};
	TArray<double> Forces;
	for (const Subsea::FWrench& Wanted : Cases)
	{
		Allocator.Allocate(Wanted, Forces);
		const Subsea::FWrench Got = Allocator.Resolve(Forces);
		TestTrue(TEXT("force reproduced"), Got.Force.Equals(Wanted.Force, 1e-3));
		TestTrue(TEXT("torque reproduced"), Got.Torque.Equals(Wanted.Torque, 1e-3));
	}

	// Pure surge must load the four horizontal thrusters equally and leave the verticals idle.
	Allocator.Allocate({ FVector3d(40.0, 0.0, 0.0), FVector3d::ZeroVector }, Forces);
	TestTrue(TEXT("surge shared equally"), FMath::IsNearlyEqual(Forces[0], Forces[1], 1e-6) && FMath::IsNearlyEqual(Forces[2], Forces[3], 1e-6));
	TestTrue(TEXT("verticals idle in surge"), FMath::Abs(Forces[4]) < 1e-6 && FMath::Abs(Forces[7]) < 1e-6);

	// Saturation scales uniformly, so the achieved wrench keeps its direction.
	const Subsea::FWrench Large{ FVector3d(400.0, 150.0, 0.0), FVector3d::ZeroVector };
	Allocator.AllocateSaturated(Large, 50.0, 40.0, Forces);
	for (const double Force : Forces)
	{
		TestTrue(TEXT("within limits"), Force <= 50.0 + 1e-9 && Force >= -40.0 - 1e-9);
	}
	const Subsea::FWrench Achieved = Allocator.Resolve(Forces);
	TestTrue(TEXT("direction preserved under saturation"),
		FVector3d::DotProduct(Achieved.Force.GetSafeNormal(), Large.Force.GetSafeNormal()) > 0.9999);

	// Fault tolerance: with one horizontal thruster dead the remaining three still span
	// surge/sway/yaw, so the wrench is still met and the dead thruster is given nothing.
	Layout[0].Direction = FVector3d::ZeroVector;
	TestTrue(TEXT("reconfigures after failure"), Allocator.Configure(Layout, 1e-4));
	const Subsea::FWrench Wanted{ FVector3d(15.0, 5.0, 10.0), FVector3d(0.0, 0.0, 1.0) };
	Allocator.Allocate(Wanted, Forces);
	TestTrue(TEXT("failed thruster unused"), FMath::Abs(Forces[0]) < 1e-9);
	const Subsea::FWrench Degraded = Allocator.Resolve(Forces);
	TestTrue(TEXT("wrench met with a failed thruster"), Degraded.Force.Equals(Wanted.Force, 1e-2) && Degraded.Torque.Equals(Wanted.Torque, 1e-2));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaPidTest, "SubseaSim.Control.Pid", TestFlags)
bool FSubseaPidTest::RunTest(const FString&)
{
	// Drive a first-order plant (a mass with linear drag) to a velocity setpoint.
	FSubseaPid Pid(70.0, 25.0, 0.0, 90.0);
	double Velocity = 0.0;
	constexpr double Mass = 22.0;
	constexpr double Drag = 10.0;
	constexpr double Dt = 1.0 / 60.0;
	for (int32 Step = 0; Step < 60 * 20; ++Step)
	{
		const double Force = Pid.Step(0.5 - Velocity, Velocity, Dt);
		Velocity += (Force - Drag * Velocity) / Mass * Dt;
	}
	TestTrue(TEXT("converges with zero steady-state error"), FMath::IsNearlyEqual(Velocity, 0.5, 1e-3));

	// Anti-windup: after a long saturated stretch the output must leave the limit promptly.
	FSubseaPid Saturating(10.0, 50.0, 0.0, 5.0);
	for (int32 Step = 0; Step < 600; ++Step)
	{
		Saturating.Step(100.0, 0.0, Dt);
	}
	double Output = 0.0;
	for (int32 Step = 0; Step < 30; ++Step)
	{
		Output = Saturating.Step(-1.0, 0.0, Dt);
	}
	TestTrue(TEXT("integrator did not wind up"), Output < 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaDampingTest, "SubseaSim.Physics.Damping", TestFlags)
bool FSubseaDampingTest::RunTest(const FString&)
{
	const FVector Linear(4.0, 6.0, 5.0);
	const FVector Quadratic(18.0, 21.0, 37.0);

	const FVector Forward = USubseaHydrodynamicsComponent::ComputeDamping(FVector(1.0, 0.0, 0.0), Linear, Quadratic);
	TestTrue(TEXT("drag opposes motion"), FMath::IsNearlyEqual(Forward.X, -22.0, 1e-9) && Forward.Y == 0.0 && Forward.Z == 0.0);

	const FVector Backward = USubseaHydrodynamicsComponent::ComputeDamping(FVector(-1.0, 0.0, 0.0), Linear, Quadratic);
	TestTrue(TEXT("drag is odd-symmetric"), FMath::IsNearlyEqual(Backward.X, 22.0, 1e-9));

	// Drag must always dissipate energy: F . v < 0 for any non-zero velocity.
	const FVector Velocity(0.4, -0.7, 0.2);
	TestTrue(TEXT("dissipative"), FVector::DotProduct(USubseaHydrodynamicsComponent::ComputeDamping(Velocity, Linear, Quadratic), Velocity) < 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaProtocolTest, "SubseaSim.Net.Protocol", TestFlags)
bool FSubseaProtocolTest::RunTest(const FString&)
{
	using namespace Subsea::Protocol;

	const FFramePtr Frame = MakeTransformFrame(42, 12.5, "base_link", "map",
		FVector3d(1.0, 2.0, 3.0), FQuat4d::Identity, FVector3d::ZeroVector, FVector3d::ZeroVector);
	TestEqual(TEXT("transform frame size"), Frame->Num(), HeaderSize + TransformPayloadSize);

	FParsedHeader Header;
	TestTrue(TEXT("header parses"), ParseHeader(Frame->GetData(), Header));
	TestEqual(TEXT("type"), static_cast<int32>(Header.Type), static_cast<int32>(EMessageType::Transform));
	TestEqual(TEXT("seq"), Header.Seq, 42u);
	TestEqual(TEXT("stamp"), Header.Stamp, 12.5);
	TestEqual(TEXT("payload size"), Header.PayloadSize, static_cast<uint32>(TransformPayloadSize));
	TestTrue(TEXT("frame id at offset 20"), FMemory::Memcmp(Frame->GetData() + 20, "base_link\0", 10) == 0);
	TestTrue(TEXT("parent frame leads the payload"), FMemory::Memcmp(Frame->GetData() + HeaderSize, "map\0", 4) == 0);

	uint8 Garbage[HeaderSize] = {};
	TestFalse(TEXT("rejects bad magic"), ParseHeader(Garbage, Header));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaUnprojectTest, "SubseaSim.Sensors.Unproject", TestFlags)
bool FSubseaUnprojectTest::RunTest(const FString&)
{
	using Subsea::Protocol::FPackedPoint;

	// A wall 4 m in front of a noise-free sensor.
	constexpr int32 Width = 64;
	constexpr int32 Height = 48;
	TArray<FLinearColor> Pixels;
	Pixels.Init(FLinearColor(0.2f, 0.4f, 0.6f, 400.0f), Width * Height);

	FSubseaRangeNoiseModel Clean;
	Clean.SigmaBaseM = 0.0;
	Clean.SigmaQuadratic = 0.0;
	Clean.AttenuationClear = 0.0;
	Clean.AttenuationTurbid = 0.0;
	Clean.MaxRangeM = 50.0;

	TArray<FPackedPoint> Points;
	USubseaDepthCameraComponent::Unproject(Pixels, Width, Height, 90.0f, Clean, 0.0, 1.0f, 1, Points);
	TestEqual(TEXT("every pixel returns"), Points.Num(), Width * Height);

	bool bPlanar = true;
	float MinY = 0.0f, MaxY = 0.0f, MinZ = 0.0f, MaxZ = 0.0f;
	for (const FPackedPoint& Point : Points)
	{
		bPlanar &= FMath::IsNearlyEqual(Point.X, 4.0f, 1e-4f);
		MinY = FMath::Min(MinY, Point.Y);
		MaxY = FMath::Max(MaxY, Point.Y);
		MinZ = FMath::Min(MinZ, Point.Z);
		MaxZ = FMath::Max(MaxZ, Point.Z);
	}
	TestTrue(TEXT("wall is planar at 4 m along X"), bPlanar);
	// 90 degree HFOV: half-width equals depth, less half a pixel.
	TestTrue(TEXT("horizontal extent matches FOV"), FMath::IsNearlyEqual(MaxY, 4.0f * (1.0f - 1.0f / Width), 1e-3f) && FMath::IsNearlyEqual(MinY, -MaxY, 1e-3f));
	TestTrue(TEXT("vertical extent matches aspect"), FMath::IsNearlyEqual(MaxZ, 3.0f * (1.0f - 1.0f / Height), 1e-3f) && FMath::IsNearlyEqual(MinZ, -MaxZ, 1e-3f));
	// First pixel is top-left: in ROS axes that is +Y (left) and +Z (up).
	TestTrue(TEXT("image origin maps to up-left"), Points[0].Y > 0.0f && Points[0].Z > 0.0f);

	// Determinism: same seed gives the same cloud; a different seed does not.
	FSubseaRangeNoiseModel Noisy;
	TArray<FPackedPoint> A, B, Other;
	USubseaDepthCameraComponent::Unproject(Pixels, Width, Height, 90.0f, Noisy, 0.3, 1.0f, 99, A);
	USubseaDepthCameraComponent::Unproject(Pixels, Width, Height, 90.0f, Noisy, 0.3, 1.0f, 99, B);
	USubseaDepthCameraComponent::Unproject(Pixels, Width, Height, 90.0f, Noisy, 0.3, 1.0f, 100, Other);
	TestTrue(TEXT("deterministic for a given seed"), A.Num() == B.Num() && FMemory::Memcmp(A.GetData(), B.GetData(), A.Num() * sizeof(FPackedPoint)) == 0);
	TestTrue(TEXT("seed changes the noise"), A.Num() != Other.Num() || FMemory::Memcmp(A.GetData(), Other.GetData(), A.Num() * sizeof(FPackedPoint)) != 0);

	// Turbidity must cost returns, and out-of-range pixels must be rejected.
	TArray<FPackedPoint> Clear, Murky, Far;
	TArray<FLinearColor> Distant;
	Distant.Init(FLinearColor(0.2f, 0.4f, 0.6f, 900.0f), Width * Height);
	USubseaDepthCameraComponent::Unproject(Distant, Width, Height, 90.0f, Noisy, 0.0, 1.0f, 5, Clear);
	USubseaDepthCameraComponent::Unproject(Distant, Width, Height, 90.0f, Noisy, 0.8, 1.0f, 5, Murky);
	TestTrue(TEXT("turbid water returns fewer points"), Murky.Num() < Clear.Num());

	Distant.Init(FLinearColor(0.2f, 0.4f, 0.6f, 5000.0f), Width * Height);
	USubseaDepthCameraComponent::Unproject(Distant, Width, Height, 90.0f, Noisy, 0.0, 1.0f, 5, Far);
	TestEqual(TEXT("beyond max range is dropped"), Far.Num(), 0);
	return true;
}

// Not a pass/fail test: runs the benchmark so CI records performance alongside correctness.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaUnprojectBenchmark, "SubseaSim.Perf.UnprojectBenchmark", TestFlags)
bool FSubseaUnprojectBenchmark::RunTest(const FString&)
{
	Subsea::RunUnprojectBenchmark();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubseaSonarTest, "SubseaSim.Sensors.Sonar", TestFlags)
bool FSubseaSonarTest::RunTest(const FString&)
{
	// Acoustic model: head-on beats grazing, absorption costs range, beam pattern scales energy.
	const float HeadOn = USubseaSonarComponent::EchoEnergy(10.0f, 1.0f, 1.0f, 0.1f);
	const float Grazing = USubseaSonarComponent::EchoEnergy(10.0f, 0.2f, 1.0f, 0.1f);
	const float Far = USubseaSonarComponent::EchoEnergy(25.0f, 1.0f, 1.0f, 0.1f);
	TestTrue(TEXT("head-on echo stronger than grazing"), HeadOn > 10.0f * Grazing);
	TestTrue(TEXT("absorption weakens far echoes"), Far < HeadOn);
	TestTrue(TEXT("back-facing surface still returns a little"), USubseaSonarComponent::EchoEnergy(5.0f, -0.5f, 1.0f, 0.1f) > 0.0f);
	TestTrue(TEXT("beam weight scales energy"), FMath::IsNearlyEqual(USubseaSonarComponent::EchoEnergy(5.0f, 1.0f, 0.5f, 0.1f), 0.5f * USubseaSonarComponent::EchoEnergy(5.0f, 1.0f, 1.0f, 0.1f), 1e-6f));

	// Image formation: an echo lands in the right (bin, beam) cell; empty cells sit at the noise floor.
	constexpr int32 Beams = 16;
	constexpr int32 Bins = 100;
	TArray<USubseaSonarComponent::FEcho> Echoes;
	Echoes.Add({ 3, 10.125f, 1.0f }); // exactly on bin 40's centre
	// A grazing echo spread over 2 m must fill a band of bins, not one.
	Echoes.Add({ 9, 5.0f, 0.2f, 2.0f });
	Echoes.Add({ 12, 20.25f, 0.05f });
	TArray<uint8> Image;
	USubseaSonarComponent::FormImage(Echoes, Beams, Bins, 0.0f, 25.0f, 0.0f, 0.0f, 40.0f, 1, Image);
	TestEqual(TEXT("image size"), Image.Num(), Beams * Bins);
	const int32 Bin = FMath::FloorToInt32(10.125f / 25.0f * Bins);
	// Beam overlap leaves half the energy in the centre beam (-3 dB) and a quarter in each neighbour.
	TestTrue(TEXT("strong echo at its range and beam"), Image[Bin * Beams + 3] > 220);
	TestTrue(TEXT("overlap reaches the neighbouring beams"), Image[Bin * Beams + 2] > 150 && Image[Bin * Beams + 4] > 150 && Image[Bin * Beams + 6] == 0);
	TestTrue(TEXT("weaker echo is dimmer"), Image[FMath::FloorToInt32(20.25f / 25.0f * Bins) * Beams + 12] < Image[Bin * Beams + 3]);
	TestEqual(TEXT("empty cell is black without noise"), static_cast<int32>(Image[0]), 0);
	int32 BandBins = 0;
	for (int32 Row = 0; Row < Bins; ++Row)
	{
		BandBins += Image[Row * Beams + 9] > 0 ? 1 : 0;
	}
	TestTrue(TEXT("footprint fills a continuous band of about 2 m"), BandBins >= 7 && BandBins <= 10);

	// Speckle is seeded: same seed same image, different seed different image.
	TArray<uint8> A, B, C;
	USubseaSonarComponent::FormImage(Echoes, Beams, Bins, 0.0f, 25.0f, 0.6f, 0.004f, 40.0f, 7, A);
	USubseaSonarComponent::FormImage(Echoes, Beams, Bins, 0.0f, 25.0f, 0.6f, 0.004f, 40.0f, 7, B);
	USubseaSonarComponent::FormImage(Echoes, Beams, Bins, 0.0f, 25.0f, 0.6f, 0.004f, 40.0f, 8, C);
	TestTrue(TEXT("deterministic speckle"), A == B);
	TestTrue(TEXT("seed changes speckle"), A != C);

	// Fan rendering puts port beams on the left.
	FSubseaSonarImage Polar;
	Polar.NumBeams = Beams;
	Polar.NumBins = Bins;
	Polar.HorizontalFovRad = FMath::DegreesToRadians(120.0f);
	Polar.MaxRangeM = 25.0f;
	Polar.Intensity.Init(0, Beams * Bins);
	for (int32 Row = 0; Row < Bins; ++Row)
	{
		Polar.Intensity[Row * Beams + 0] = 255;
	}
	TArray<FColor> Fan;
	USubseaSonarComponent::RenderFan(Polar, 320, Fan);
	int64 Left = 0, Right = 0;
	for (int32 Y = 0; Y < 200; ++Y)
	{
		for (int32 X = 0; X < 320; ++X)
		{
			(X < 160 ? Left : Right) += Fan[Y * 320 + X].R;
		}
	}
	TestTrue(TEXT("port beam drawn on the left"), Left > 4 * Right);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
