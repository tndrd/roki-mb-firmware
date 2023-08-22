/*
 * MotherBoardMain.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: tndrd
 */

#include "MotherboardMain.hpp"
#include "Motherboard.hpp"

struct MotherboardContext {
	HeadInterface Head;
	QueueSender Body;
	IMUFrameContainer FrameContainer;
	BHYWrapper IMU;
	IMURequestHandler IMUHandler;

	bool UpdateIMU = false;

	MotherboardContext(MotherboardConfig conf) :
		Head{conf.HeadUart, conf.HeadTimeout},
		Body{conf.BodyUart, conf.BodyTimeout},
		FrameContainer{},
		IMU{conf.IMUSpi},
		IMUHandler{} {}

	MotherboardContext() = default;
};

static MotherboardContext mbctx;

int MotherboardInit(MotherboardConfig conf) {
	mbctx = MotherboardContext { conf };

	mbctx.Head.ResetReadState();
	return mbctx.IMU.Init(800, 0);
}

int MotherboardTick() {
	if (mbctx.UpdateIMU)
		mbctx.UpdateIMU = !mbctx.IMU.Poll();

	if (mbctx.Head.HasRequest()) {
		auto request = mbctx.Head.GetRequest();

		switch (request.PeripheryID) {
		case Periphery::Body:
			mbctx.Body.AddRequest(std::move(request));
			break;
		case Periphery::Imu:
			mbctx.Head.Send(
					mbctx.IMUHandler.Handle(request,
							mbctx.FrameContainer));
			break;
		}
	}

	if (mbctx.Body.HasResponce()) {
		mbctx.Head.Send(mbctx.Body.GetResponce());
	}

	return 0;
}

void MotherboardOnStrobe() {
	mbctx.FrameContainer.Add(mbctx.IMU.GetFrame());
}

void MotherboardOnBodyTransmitComplete() {
	mbctx.Body.ProcessResponces();
}

void MotherboardOnHeadRecieveComplete() {
	mbctx.Head.ProcessRecievedData();
}

void MotherboardOnHeadTransmitComplete() {
	mbctx.Head.FinishTransmit();
}

void MotherboardOnBodyTimerTick() {
	mbctx.Body.ProcessRequests();
}

void MotherboardOnImuTimerTick() {
	mbctx.UpdateIMU = true;
}
