/*
 * MotherBoardMain.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: tndrd
 */

#include "MotherboardMain.h"
#include "Motherboard.hpp"

static MotherboardContext mbCtx;
static MotherboardRequestHandler mbHandler;

int MotherboardInit(MotherboardConfig conf) {
	mbCtx = MotherboardContext { conf };

	mbCtx.Head.ResetReadState();
	return mbCtx.IMU.Init(800, 0);
}

int MotherboardTick() {
	if (mbCtx.StrobeFilter.HasStrobe()) {
		mbCtx.IMUStrobes.push(
				mbCtx.StrobeFilter.GetStrobe() + mbCtx.IMUStrobeOffset);
		mbCtx.BodyStrobes.push(
				mbCtx.StrobeFilter.GetStrobe() + mbCtx.BodyStrobeOffset);
		mbCtx.StrobeFilter.PopStrobe();
	}

	if (!mbCtx.IMUStrobes.empty()) {
		size_t seq = mbCtx.IMUStrobes.front();
		if (mbCtx.FrameMemo.Has(seq)) {
			mbCtx.IMUFrameContainer.Add(mbCtx.FrameMemo.Get(seq));
		}
	}

	if (!mbCtx.BodyStrobes.empty()) {
		size_t seq = mbCtx.BodyStrobes.front();
		if (mbCtx.FrameMemo.Has(seq)) {
			mbCtx.BodyFrameContainer.Add(mbCtx.Body.GetServoData());
		}
	}

	if (mbCtx.UpdateIMU && mbCtx.IMU.Poll()) {
		mbCtx.FrameMemo.Add(mbCtx.IMU.GetFrame(), mbCtx.IMU.GetSeq());
		mbCtx.UpdateIMU = false;
	}

	if (mbCtx.Head.HasRequest()) {
		Request request = mbCtx.Head.GetRequest();

		switch (request.PeripheryID) {
		case Periphery::Body:
			mbCtx.Body.AddRequest(request);
			break;
		case Periphery::Motherboard:
			mbCtx.Head.Send(mbHandler.Handle(request, mbCtx));
			break;
		}
	}

	if (mbCtx.Body.HasResponce()) {
		mbCtx.Head.Send(mbCtx.Body.GetResponce());
	}

	mbCtx.Body.ProcessPriorityRequest();
	mbCtx.Body.ProcessRequests();

	return 0;
}

void MotherboardOnStrobe() {
	static bool firstEntry = true;
	if (firstEntry) {
		firstEntry = false;
		return;
	}

	mbCtx.StrobeFilter.ProcessStrobe(mbCtx.IMU);
}

void MotherboardOnBodyRecieveComplete() {
	mbCtx.Body.ProcessResponces();
}

void MotherboardOnHeadRecieveComplete() {
	mbCtx.Head.ProcessRecievedData();
}

void MotherboardOnHeadTransmitComplete() {
	mbCtx.Head.FinishTransmit();
}

void MotherboardOnBodyTransmitComplete() {
	mbCtx.Body.FinishTransmit();
}
void MotherboardOnBodyTimerTick() {
	mbCtx.Body.TickTimer();
}

void MotherboardOnImuTimerTick() {
	mbCtx.UpdateIMU = true;
}
