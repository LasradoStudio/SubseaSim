#include "Net/SubseaStreamServer.h"

#include "Common/TcpSocketBuilder.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "HAL/RunnableThread.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "SubseaSimModule.h"

namespace Proto = Subsea::Protocol;

FSubseaStreamServer::FSubseaStreamServer() = default;

FSubseaStreamServer::~FSubseaStreamServer()
{
	Shutdown();
}

bool FSubseaStreamServer::Start(const int32 Port, const bool bAllInterfaces)
{
	if (Thread)
	{
		return true;
	}

	ListenSocket = FTcpSocketBuilder(TEXT("SubseaStreamListen"))
		.AsReusable()
		.AsNonBlocking()
		.BoundToEndpoint(FIPv4Endpoint(bAllInterfaces ? FIPv4Address::Any : FIPv4Address::InternalLoopback, static_cast<uint16>(Port)))
		.Listening(8)
		.Build();

	if (!ListenSocket)
	{
		UE_LOG(LogSubseaSim, Error, TEXT("Sensor stream: could not listen on TCP port %d (already in use?)"), Port);
		return false;
	}

	Port_ = Port;
	bStopRequested = false;
	WakeEvent = FPlatformProcess::GetSynchEventFromPool(false);
	Thread = FRunnableThread::Create(this, TEXT("SubseaStreamServer"), 0, TPri_AboveNormal);
	UE_LOG(LogSubseaSim, Log, TEXT("Sensor stream listening on tcp://%s:%d"), bAllInterfaces ? TEXT("0.0.0.0") : TEXT("127.0.0.1"), Port);
	return Thread != nullptr;
}

void FSubseaStreamServer::Shutdown()
{
	if (Thread)
	{
		Stop();
		Thread->WaitForCompletion();
		delete Thread;
		Thread = nullptr;
	}
	if (WakeEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
		WakeEvent = nullptr;
	}

	ISocketSubsystem* Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	for (FClient& Client : Clients)
	{
		if (Client.Socket)
		{
			Client.Socket->Close();
			Sockets->DestroySocket(Client.Socket);
		}
	}
	Clients.Reset();
	NumClients_ = 0;

	if (ListenSocket)
	{
		ListenSocket->Close();
		Sockets->DestroySocket(ListenSocket);
		ListenSocket = nullptr;
	}

	Proto::FFramePtr Discard;
	while (Outgoing.Dequeue(Discard))
	{
	}
	Backlog = 0;
}

void FSubseaStreamServer::Stop()
{
	bStopRequested = true;
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}
}

void FSubseaStreamServer::Enqueue(Proto::FFramePtr Frame, const bool bDroppable)
{
	if (!Frame.IsValid() || !Thread)
	{
		return;
	}
	if (NumClients() == 0)
	{
		return;
	}
	if (bDroppable && Backlog.load(std::memory_order_relaxed) >= MaxBacklog)
	{
		FramesDropped_.fetch_add(1, std::memory_order_relaxed);
		return;
	}

	Backlog.fetch_add(1, std::memory_order_relaxed);
	Outgoing.Enqueue(MoveTemp(Frame));
	WakeEvent->Trigger();
}

uint32 FSubseaStreamServer::NextSeq(const Proto::EMessageType Type)
{
	const uint32 Index = FMath::Min<uint32>(static_cast<uint32>(Type), UE_ARRAY_COUNT(Sequences) - 1);
	return Sequences[Index].fetch_add(1, std::memory_order_relaxed);
}

bool FSubseaStreamServer::DequeueCommand(FString& OutJson)
{
	return Commands.Dequeue(OutJson);
}

uint32 FSubseaStreamServer::Run()
{
	while (!bStopRequested)
	{
		AcceptPending();
		DrainOutgoing();
		PollIncoming();
		// Woken immediately by Enqueue; the timeout only bounds accept/command latency.
		WakeEvent->Wait(FTimespan::FromMilliseconds(5));
	}
	return 0;
}

void FSubseaStreamServer::AcceptPending()
{
	bool bPending = false;
	while (ListenSocket->HasPendingConnection(bPending) && bPending)
	{
		ISocketSubsystem* Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		TSharedRef<FInternetAddr> Remote = Sockets->CreateInternetAddr();
		FSocket* Socket = ListenSocket->Accept(*Remote, TEXT("SubseaStreamClient"));
		if (!Socket)
		{
			break;
		}

		Socket->SetNonBlocking(true);
		Socket->SetNoDelay(true);
		int32 Granted = 0;
		Socket->SetSendBufferSize(4 * 1024 * 1024, Granted);

		FClient& Client = Clients.AddDefaulted_GetRef();
		Client.Socket = Socket;
		Client.Description = Remote->ToString(true);
		NumClients_ = Clients.Num();
		UE_LOG(LogSubseaSim, Log, TEXT("Sensor stream: client connected (%s), %d total"), *Client.Description, Clients.Num());
	}
}

void FSubseaStreamServer::DrainOutgoing()
{
	Proto::FFramePtr Frame;
	while (Outgoing.Dequeue(Frame))
	{
		Backlog.fetch_sub(1, std::memory_order_relaxed);
		for (int32 Index = Clients.Num() - 1; Index >= 0; --Index)
		{
			if (!SendAll(Clients[Index], Frame->GetData(), Frame->Num()))
			{
				CloseClient(Index);
			}
		}
		if (bStopRequested)
		{
			return;
		}
	}
}

bool FSubseaStreamServer::SendAll(FClient& Client, const uint8* Data, const int32 Count)
{
	int32 Offset = 0;
	int32 Stalls = 0;
	while (Offset < Count)
	{
		int32 Sent = 0;
		if (Client.Socket->Send(Data + Offset, Count - Offset, Sent) && Sent > 0)
		{
			Offset += Sent;
			Stalls = 0;
			continue;
		}

		const ESocketErrors Error = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode();
		if (Error != SE_EWOULDBLOCK && Error != SE_NO_ERROR)
		{
			return false;
		}
		// Kernel buffer is full: give the client a bounded amount of time to catch up.
		if (++Stalls > 10 || !Client.Socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromMilliseconds(50)))
		{
			return false;
		}
	}
	BytesSent_.fetch_add(static_cast<uint64>(Count), std::memory_order_relaxed);
	return true;
}

void FSubseaStreamServer::PollIncoming()
{
	for (int32 Index = Clients.Num() - 1; Index >= 0; --Index)
	{
		FClient& Client = Clients[Index];

		if (Client.Socket->GetConnectionState() != SCS_Connected)
		{
			CloseClient(Index);
			continue;
		}

		uint32 Pending = 0;
		while (Client.Socket->HasPendingData(Pending) && Pending > 0)
		{
			const int32 Offset = Client.RxBuffer.AddUninitialized(static_cast<int32>(Pending));
			int32 Read = 0;
			if (!Client.Socket->Recv(Client.RxBuffer.GetData() + Offset, static_cast<int32>(Pending), Read) || Read <= 0)
			{
				Client.RxBuffer.SetNum(Offset, EAllowShrinking::No);
				break;
			}
			Client.RxBuffer.SetNum(Offset + Read, EAllowShrinking::No);
		}

		// Extract complete command frames.
		int32 Consumed = 0;
		while (Client.RxBuffer.Num() - Consumed >= Proto::HeaderSize)
		{
			Proto::FParsedHeader Header;
			if (!Proto::ParseHeader(Client.RxBuffer.GetData() + Consumed, Header) || Header.PayloadSize > MaxCommandBytes)
			{
				// Not our protocol; discard everything rather than guess at a resync point.
				Consumed = Client.RxBuffer.Num();
				break;
			}
			const int32 Total = Proto::HeaderSize + static_cast<int32>(Header.PayloadSize);
			if (Client.RxBuffer.Num() - Consumed < Total)
			{
				break;
			}
			if (Header.Type == Proto::EMessageType::Command)
			{
				const FUtf8StringView Json(reinterpret_cast<const UTF8CHAR*>(Client.RxBuffer.GetData() + Consumed + Proto::HeaderSize), static_cast<int32>(Header.PayloadSize));
				Commands.Enqueue(FString(Json));
			}
			Consumed += Total;
		}
		if (Consumed > 0)
		{
			Client.RxBuffer.RemoveAt(0, Consumed, EAllowShrinking::No);
		}
	}
}

void FSubseaStreamServer::CloseClient(const int32 Index)
{
	FClient& Client = Clients[Index];
	UE_LOG(LogSubseaSim, Log, TEXT("Sensor stream: client disconnected (%s)"), *Client.Description);
	Client.Socket->Close();
	ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Client.Socket);
	Clients.RemoveAtSwap(Index);
	NumClients_ = Clients.Num();
}
