/*
 * MotherBoardMain.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: tndrd
 */

#include "MotherboardMain.hpp"
#include "Motherboard.hpp"

struct MotherboardContext {
	HeadInterface HeadService;
	HeadInterface HeadStream;

	AcknowledgeHandler AckHandler {0, 0};

	QueueSender Body;
	IMUFrameContainer FrameContainer;
	BHYWrapper IMU;
	IMURequestHandler IMUHandler;

	bool UpdateIMU = false;

	MotherboardContext(MotherboardConfig conf) :
			HeadService { conf.HeadServiceUart, conf.HeadTimeout }, HeadStream {
					conf.HeadStreamUart, conf.HeadTimeout }, Body {
					conf.BodyUart, conf.BodyTimeout }, FrameContainer { }, IMU {
					conf.IMUSpi }, IMUHandler { }, AckHandler {
					conf.VersionMajor, conf.VersionMinor } {
	}

	MotherboardContext() = default;
};

static MotherboardContext mbctx;

int MotherboardInit(MotherboardConfig conf) {
	mbctx = MotherboardContext { conf };

	mbctx.HeadService.ResetReadState();
	mbctx.HeadStream.ResetReadState();
	return mbctx.IMU.Init(800, 0);
}

int MotherboardTick() {
	if (mbctx.UpdateIMU)
		mbctx.UpdateIMU = !mbctx.IMU.Poll();

	if (mbctx.HeadService.HasRequest()) {
		auto request = mbctx.HeadService.GetRequest();

		switch (request.PeripheryID) {
		case Periphery::Ack:
			mbctx.HeadService.Send(mbctx.AckHandler.Handle(request));
			break;

		case Periphery::Body:
			mbctx.Body.AddRequest(std::move(request));
			break;
		case Periphery::Imu:
			mbctx.HeadService.Send(
					mbctx.IMUHandler.Handle(request, mbctx.FrameContainer,
							mbctx.IMU));
			break;
		}
	}

	if (mbctx.Body.HasResponce()) {
		mbctx.HeadService.Send(mbctx.Body.GetResponce());
	}

	mbctx.Body.ProcessPriorityRequest();
	mbctx.Body.ProcessRequests();

	return 0;
}

void MotherboardOnStrobe() {
	mbctx.FrameContainer.Add(mbctx.IMU.GetFrame());
}

void MotherboardOnBodyRecieveComplete() {
	mbctx.Body.ProcessResponces();
}

void MotherboardOnHeadServiceRecieveComplete() {
	mbctx.HeadService.ProcessRecievedData();
}

void MotherboardOnHeadStreamRecieveComplete() {
	mbctx.HeadStream.ProcessRecievedData();
}

void MotherboardOnHeadServiceTransmitComplete() {
	mbctx.HeadService.FinishTransmit();
}

void MotherboardOnHeadStreamTransmitComplete() {
	mbctx.HeadStream.FinishTransmit();
}

void MotherboardOnBodyTransmitComplete() {
	mbctx.Body.FinishTransmit();
}
void MotherboardOnBodyTimerTick() {
	mbctx.Body.TickTimer();
}

void MotherboardOnImuTimerTick() {
	mbctx.UpdateIMU = true;
}
