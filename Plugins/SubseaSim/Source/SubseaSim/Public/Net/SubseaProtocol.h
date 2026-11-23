#pragma once

#include "CoreMinimal.h"

/**
 * Wire format shared with the Python bridge. See Docs/PROTOCOL.md; the two must change together.
 * Everything is little-endian, which is also the host byte order on every platform Unreal targets,
 * so fields are written with Memcpy rather than per-byte shifts.
 */
namespace Subsea::Protocol
{
	inline constexpr uint16 Version = 1;
	inline constexpr int32 HeaderSize = 40;
	inline constexpr int32 FrameIdSize = 16;
	inline constexpr int32 PointSize = 16;
	inline constexpr int32 TransformPayloadSize = 120;
	inline constexpr uint8 Magic[4] = { 'S', 'S', 'I', 'M' };

	enum class EMessageType : uint16
	{
		PointCloud = 1,
		Transform = 2,
		Telemetry = 3,
		Command = 4,
		SonarImage = 5,
	};

	enum EPointCloudFlags : uint32
	{
		HasRgb = 1u << 0,
		HasIntensity = 1u << 1,
	};

#pragma pack(push, 1)
	struct FPackedPoint
	{
		float X;
		float Y;
		float Z;
		uint8 R;
		uint8 G;
		uint8 B;
		uint8 Intensity;
	};
#pragma pack(pop)
	static_assert(sizeof(FPackedPoint) == PointSize, "FPackedPoint must match PROTOCOL.md");

	using FWireFrame = TArray<uint8>;
	using FFramePtr = TSharedPtr<const FWireFrame, ESPMode::ThreadSafe>;

	/** Appends a fixed-width, NUL-padded ASCII identifier. */
	SUBSEASIM_API void AppendFrameId(FWireFrame& Out, FAnsiStringView Id);

	/** Writes the 40-byte header. PayloadSize may be patched later with PatchPayloadSize. */
	SUBSEASIM_API void AppendHeader(FWireFrame& Out, EMessageType Type, uint32 Seq, double Stamp, FAnsiStringView FrameId, uint32 PayloadSize);

	/** Sets payload_size from the buffer's current length. */
	SUBSEASIM_API void PatchPayloadSize(FWireFrame& Frame);

	template <typename T>
	void AppendPod(FWireFrame& Out, const T& Value)
	{
		static_assert(std::is_trivially_copyable_v<T>);
		const int32 Offset = Out.AddUninitialized(sizeof(T));
		FMemory::Memcpy(Out.GetData() + Offset, &Value, sizeof(T));
	}

	/** Transform of FrameId in ParentFrame. All arguments are already in ROS conventions (m, rad). */
	SUBSEASIM_API FFramePtr MakeTransformFrame(uint32 Seq, double Stamp, FAnsiStringView FrameId, FAnsiStringView ParentFrame,
		const FVector3d& Position, const FQuat4d& Orientation, const FVector3d& LinearVelocity, const FVector3d& AngularVelocity);

	SUBSEASIM_API FFramePtr MakeTelemetryFrame(uint32 Seq, double Stamp, FAnsiStringView FrameId, FUtf8StringView Json);

	struct FParsedHeader
	{
		EMessageType Type = EMessageType::Telemetry;
		uint32 Seq = 0;
		double Stamp = 0.0;
		uint32 PayloadSize = 0;
	};

	/** Returns false if Data does not start with a valid header. Data must hold at least HeaderSize bytes. */
	SUBSEASIM_API bool ParseHeader(const uint8* Data, FParsedHeader& Out);
}
