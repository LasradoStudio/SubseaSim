#include "Net/SubseaProtocol.h"

namespace Subsea::Protocol
{
	namespace
	{
		constexpr int32 PayloadSizeOffset = 36;
	}

	void AppendFrameId(FWireFrame& Out, const FAnsiStringView Id)
	{
		const int32 Offset = Out.AddZeroed(FrameIdSize);
		// Leave at least one trailing NUL so clients can treat it as a C string.
		const int32 Count = FMath::Min(Id.Len(), FrameIdSize - 1);
		FMemory::Memcpy(Out.GetData() + Offset, Id.GetData(), Count);
	}

	void AppendHeader(FWireFrame& Out, const EMessageType Type, const uint32 Seq, const double Stamp, const FAnsiStringView FrameId, const uint32 PayloadSize)
	{
		Out.Append(Magic, 4);
		AppendPod(Out, Version);
		AppendPod(Out, static_cast<uint16>(Type));
		AppendPod(Out, Seq);
		AppendPod(Out, Stamp);
		AppendFrameId(Out, FrameId);
		AppendPod(Out, PayloadSize);
	}

	void PatchPayloadSize(FWireFrame& Frame)
	{
		check(Frame.Num() >= HeaderSize);
		const uint32 PayloadSize = static_cast<uint32>(Frame.Num() - HeaderSize);
		FMemory::Memcpy(Frame.GetData() + PayloadSizeOffset, &PayloadSize, sizeof(PayloadSize));
	}

	FFramePtr MakeTransformFrame(const uint32 Seq, const double Stamp, const FAnsiStringView FrameId, const FAnsiStringView ParentFrame,
		const FVector3d& Position, const FQuat4d& Orientation, const FVector3d& LinearVelocity, const FVector3d& AngularVelocity)
	{
		TSharedPtr<FWireFrame, ESPMode::ThreadSafe> Frame = MakeShared<FWireFrame, ESPMode::ThreadSafe>();
		Frame->Reserve(HeaderSize + TransformPayloadSize);
		AppendHeader(*Frame, EMessageType::Transform, Seq, Stamp, FrameId, TransformPayloadSize);
		AppendFrameId(*Frame, ParentFrame);

		const double Values[13] = {
			Position.X, Position.Y, Position.Z,
			Orientation.X, Orientation.Y, Orientation.Z, Orientation.W,
			LinearVelocity.X, LinearVelocity.Y, LinearVelocity.Z,
			AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z,
		};
		Frame->Append(reinterpret_cast<const uint8*>(Values), sizeof(Values));
		check(Frame->Num() == HeaderSize + TransformPayloadSize);
		return Frame;
	}

	FFramePtr MakeTelemetryFrame(const uint32 Seq, const double Stamp, const FAnsiStringView FrameId, const FUtf8StringView Json)
	{
		TSharedPtr<FWireFrame, ESPMode::ThreadSafe> Frame = MakeShared<FWireFrame, ESPMode::ThreadSafe>();
		Frame->Reserve(HeaderSize + Json.Len());
		AppendHeader(*Frame, EMessageType::Telemetry, Seq, Stamp, FrameId, static_cast<uint32>(Json.Len()));
		Frame->Append(reinterpret_cast<const uint8*>(Json.GetData()), Json.Len());
		return Frame;
	}

	bool ParseHeader(const uint8* Data, FParsedHeader& Out)
	{
		if (FMemory::Memcmp(Data, Magic, 4) != 0)
		{
			return false;
		}
		uint16 WireVersion = 0;
		uint16 WireType = 0;
		FMemory::Memcpy(&WireVersion, Data + 4, 2);
		FMemory::Memcpy(&WireType, Data + 6, 2);
		if (WireVersion != Version)
		{
			return false;
		}
		Out.Type = static_cast<EMessageType>(WireType);
		FMemory::Memcpy(&Out.Seq, Data + 8, 4);
		FMemory::Memcpy(&Out.Stamp, Data + 12, 8);
		FMemory::Memcpy(&Out.PayloadSize, Data + PayloadSizeOffset, 4);
		return true;
	}
}
