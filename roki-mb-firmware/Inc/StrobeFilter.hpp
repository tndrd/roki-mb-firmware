#ifndef STROBE_FILTER
#define STROBE_FILTER

#include "IMUDevice.hpp"
#include "stm32h7xx_hal.h"
#include <queue>

struct StrobeFilter {
private:
	enum class PulseState {
		Up, Down
	};

private:
	/* Callback-local */
	uint32_t FallTime = 0;
	uint32_t RiseTime = 0;
	PulseState State = PulseState::Down;
	size_t CurrentSeq = 0;
	const IMUDevice *IMU;

	/* Concurrent */
	uint32_t DurationThreshold;
	uint32_t TargetDuration;
	float StrobeDuration = 0;

	std::queue<size_t> StrobeQueue;

private:
	void Callback() {
		uint32_t currentTime = HAL_GetTick();

		switch (State) {
		case PulseState::Down: {
			State = PulseState::Up;

			uint32_t startTime = RiseTime;
			RiseTime = currentTime;

			if (!FallTime || !RiseTime)
				return;

			uint32_t duration = currentTime - startTime;

			StrobeDuration += duration;
			StrobeDuration /= 2;

			if (std::abs(long(duration - TargetDuration))
					< long(DurationThreshold))
				StrobeQueue.push(CurrentSeq);

			CurrentSeq = IMU->GetSeq();
			break;
		}
		case PulseState::Up:
			State = PulseState::Down;
			FallTime = currentTime;
			break;
		}
	}

public:
	StrobeFilter(uint8_t targetDuration, uint8_t durationThreshold,
			const IMUDevice *imu) :
			IMU { imu } {
		assert(imu);
		Configure(targetDuration, durationThreshold);
	}

	void Configure(uint8_t targetDuration, uint8_t durationThreshold) {
		__disable_irq();
		TargetDuration = targetDuration;
		DurationThreshold = durationThreshold;
		__enable_irq();
	}

	float GetStrobeDuration() const {
		return StrobeDuration;
	}

	bool HasStrobe() const {
		return !StrobeQueue.empty();
	}

	size_t GetStrobe() {
		assert(HasStrobe());
		size_t strobe = StrobeQueue.front();
		StrobeQueue.pop();
		return strobe;
	}

	void CompCallback() {
		Callback();
	}
};

#endif
