#ifndef MOTHERBOARD_HPP
#define MOTHERBOARD_HPP

#include "RequestHandler.hpp"

struct Motherboard {
private:
	MotherboardContext Ctx;
	RequestHandler Handler;

	BufferType DummyBuf;
private:
	static void BodyStrobeCallback(MotherboardContext &ctx) {
		struct FrameQueues::BodyResponce responce;
		auto request = BodyMsgs::Requests::GetAllPos { };

		auto &txData = request.Data;
		auto &txSize = request.RequestSize;
		auto rxData = responce.Data.data();
		auto &rxSize = request.ResponceSize;

		BodyClient::Status status;
		status = ctx.Body.Synchronize(txData, txSize, rxData, rxSize);

		responce.Status = status;
		if (status == BodyClient::Status::Success)
			responce.Size = request.ResponceSize;
		else
			responce.Size = 0;

		ctx.FQueues.BodyPos.Add(responce);
	}

	static void ImuStrobeCallback(MotherboardContext &ctx) {
		ctx.FQueues.IMUFrames.Add(ctx.IMU.GetFrame());
	}

	void RegisterObservers() {
		Ctx.SObservers.RegisterObserver(0, Motherboard::ImuStrobeCallback);
		Ctx.SObservers.RegisterObserver(0, Motherboard::BodyStrobeCallback);
	}

	void UpdateTimers() {
		Ctx.Head.Output.CheckTimer();
		Ctx.IMU.CheckTimer();
	}

	void HandleRequests() {
		if (!Ctx.Head.Input.HasRequest())
			return;

		auto request = Ctx.Head.Input.GetRequest();
		auto responce = Handler.Handle(Ctx, request);

		Ctx.Head.Output.AddResponce(responce);
	}

	void UpdateBodyQueue() {
		if (!Ctx.BQueue.IsReady())
			return;
		auto request = Ctx.BQueue.GetRequest();

		auto txBuf = request.Data.data();
		auto txSize = request.TxSize;
		auto rxBuf = DummyBuf.data();
		auto rxSize = request.RxSize;

		Ctx.Body.Synchronize(txBuf, txSize, rxBuf, rxSize);
	}

	void UpdateStrobes() {
		if (Ctx.SFilter.HasStrobe())
			Ctx.SObservers.AddStrobe(Ctx.SFilter.GetStrobe());

		Ctx.SObservers.TryProcess(Ctx, Ctx.IMU.GetSeq());
	}
public:
	void Init() {
		Ctx.Head.Input.Init();
		Ctx.IMU.Init();
	}

	void Run() {
		while (1) {
			UpdateTimers();
			HandleRequests();
			UpdateBodyQueue();
			UpdateStrobes();
		}
	}

	Motherboard(const MotherboardConfig &config) :
			Ctx { config } {
		RegisterObservers();
	}

public: // Callbacks
	void HeadRxCallback() {
		Ctx.Head.Input.RxCpltCallback();
	}

	void HeadTxCallback() {
		Ctx.Head.Output.TxCpltCallback();
	}

	void BQueueTimCallback() {
		Ctx.BQueue.TimCallback();
	}

	void IMUTimCallback() {
		Ctx.IMU.TimCallback();
	}

	void SFCompCallback() {
		Ctx.SFilter.CompCallback();
	}
};

#endif
