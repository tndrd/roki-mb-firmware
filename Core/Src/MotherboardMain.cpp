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

	AcknowledgeHandler AckHandler { 0, 0 };

	QueueSender Body;
	IMUFrameContainer FrameContainer;
	IMUFrameMemo FrameMemo;
	BHYWrapper IMU;
	IMURequestHandler IMUHandler;

	StrobeDurationFilter StrobeFilter;
	size_t StrobeOffset;

	bool UpdateIMU = false;

	MotherboardContext(MotherboardConfig conf) :
			HeadService { conf.HeadServiceUart, conf.HeadTimeout }, HeadStream {
					conf.HeadStreamUart, conf.HeadTimeout }, Body {
					conf.BodyUart, conf.BodyTimeout, conf.BodyPeriod }, FrameContainer { }, IMU {
					conf.IMUSpi }, IMUHandler { }, AckHandler {
					conf.VersionMajor, conf.VersionMinor }, StrobeOffset {
					conf.StrobeOffset } {
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
	if (mbctx.StrobeFilter.HasStrobe()) {
		size_t targetSeq = mbctx.StrobeFilter.GetStrobe() + mbctx.StrobeOffset;
		if (mbctx.FrameMemo.Has(targetSeq)) {
			mbctx.FrameContainer.Add(mbctx.FrameMemo.Get(targetSeq));
			mbctx.StrobeFilter.PopStrobe();
		}
	}

	if (mbctx.UpdateIMU && mbctx.IMU.Poll()) {
		mbctx.FrameMemo.Add(mbctx.IMU.GetFrame(), mbctx.IMU.GetSeq());
		mbctx.UpdateIMU = false;
	}

	if (mbctx.HeadService.HasRequest()) {
		Request request = mbctx.HeadService.GetRequest();

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
							mbctx.IMU, mbctx.StrobeOffset, mbctx.StrobeFilter));
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
	static bool firstEntry = true;
	if (firstEntry) {
		firstEntry = false;
		return;
	}

	mbctx.StrobeFilter.ProcessStrobe(mbctx.IMU);
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
