#ifndef MOTHERBOARD_CONTEXT
#define MOTHERBOARD_CONTEXT

#include "BodyClient.hpp"
#include "FrameQueue.hpp"
#include "IMUDevice.hpp"
#include "HeadIO.hpp"
#include "BodyQueue.hpp"
#include "StrobeFilter.hpp"
#include "StrobeObservers.hpp"
#include "MotherboardConfig.h"

struct FrameQueues {
public:
	struct BodyResponce {
		BodyClient::Status Status;
		BufferType Data;
		uint8_t Size;
	};
public:
	static constexpr size_t MaxSize = FrameQueueMaxSize;

	FrameQueue<BodyResponce, MaxSize> BodyPos;
	FrameQueue<IMUDevice::Frame, MaxSize> IMUFrames;

	void Clear() {
		BodyPos.Clear();
		IMUFrames.Clear();
	}
};

struct MotherboardContext {
	struct VersionT {
		uint8_t Major;
		uint8_t Minor;
	};

	VersionT Version { 0, 4 };

	HeadIO Head;
	BodyClient Body;
	BodyQueue BQueue;
	IMUDevice IMU;
	StrobeFilter SFilter;
	StrobeObservers SObservers;
	FrameQueues FQueues;

	MotherboardContext(MotherboardConfig c) :
			Head { c.HeadIO.Uart }, Body { c.BodyClient.Uart,
					c.BodyClient.TimeoutMs, c.BodyClient.NAttempts }, BQueue {
					c.BodyQueue.Period }, IMU { c.IMUDevice.Spi,
					c.IMUDevice.SampleRate, c.IMUDevice.ReportLatency }, SFilter {
					c.StrobeFilter.TargetDuration,
					c.StrobeFilter.DurationThreshold, &IMU } {
	}
};

#endif
