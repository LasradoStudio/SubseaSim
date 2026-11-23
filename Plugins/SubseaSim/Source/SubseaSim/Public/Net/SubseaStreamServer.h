#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "HAL/Runnable.h"
#include "Net/SubseaProtocol.h"
#include <atomic>

class FSocket;
class FRunnableThread;
class FEvent;

/**
 * Fan-out TCP server for sensor frames, running entirely on its own thread.
 *
 * Producers (game thread, task-graph workers) hand over immutable, reference-counted frames through
 * a lock-free MPSC queue, so publishing never blocks the simulation and a frame is shared, not
 * copied, between clients. A slow or stalled client cannot stall the sim: bulky frames are dropped
 * at the door once the backlog passes a threshold, and a client whose socket stays unwritable is
 * disconnected.
 */
class SUBSEASIM_API FSubseaStreamServer final : public FRunnable
{
public:
	FSubseaStreamServer();
	virtual ~FSubseaStreamServer() override;

	/** Listens on loopback only unless bAllInterfaces is set (needed for Docker or another machine). */
	bool Start(int32 Port, bool bAllInterfaces = false);
	void Shutdown();

	/**
	 * Queues a frame for every connected client. Droppable frames (point clouds) are discarded when
	 * nobody is listening or the backlog is deep; small state frames are always queued.
	 * Safe to call from any thread.
	 */
	void Enqueue(Subsea::Protocol::FFramePtr Frame, bool bDroppable);

	/** Per-message-type sequence counter. Safe to call from any thread. */
	uint32 NextSeq(Subsea::Protocol::EMessageType Type);

	/** Pops one JSON command received from a client. Game thread. */
	bool DequeueCommand(FString& OutJson);

	[[nodiscard]] int32 NumClients() const { return NumClients_.load(std::memory_order_relaxed); }
	[[nodiscard]] int32 Port() const { return Port_; }
	[[nodiscard]] uint64 BytesSent() const { return BytesSent_.load(std::memory_order_relaxed); }
	[[nodiscard]] uint64 FramesDropped() const { return FramesDropped_.load(std::memory_order_relaxed); }
	[[nodiscard]] bool IsRunning() const { return Thread != nullptr; }

	// FRunnable
	virtual uint32 Run() override;
	virtual void Stop() override;

private:
	struct FClient
	{
		FSocket* Socket = nullptr;
		TArray<uint8> RxBuffer;
		FString Description;
	};

	void AcceptPending();
	void DrainOutgoing();
	void PollIncoming();
	bool SendAll(FClient& Client, const uint8* Data, int32 Count);
	void CloseClient(int32 Index);

	static constexpr int32 MaxBacklog = 6;
	static constexpr int32 MaxCommandBytes = 64 * 1024;

	FSocket* ListenSocket = nullptr;
	FRunnableThread* Thread = nullptr;
	FEvent* WakeEvent = nullptr;
	int32 Port_ = 0;

	TArray<FClient> Clients;
	TQueue<Subsea::Protocol::FFramePtr, EQueueMode::Mpsc> Outgoing;
	TQueue<FString, EQueueMode::Spsc> Commands;

	std::atomic<bool> bStopRequested{ false };
	std::atomic<int32> NumClients_{ 0 };
	std::atomic<int32> Backlog{ 0 };
	std::atomic<uint64> BytesSent_{ 0 };
	std::atomic<uint64> FramesDropped_{ 0 };
	std::atomic<uint32> Sequences[8] = {};
};
