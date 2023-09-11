/*
 * BHYWrapper.cpp
 *
 *  Created on: Aug 18, 2023
 *      Author: tndrd
 */

#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_parse.h"
#include "bhy2_defs.h"
#include <stdarg.h>
#include <stdio.h>

#include "Bosch_SHUTTLE_BHI260_2.fw.h"

#include "IMU_funcs.h"

#include <string.h>
#include <array>
#include <vector>

class BHYWrapper {
public:
	struct BHYQuaternion {
		int16_t X;
		int16_t Y;
		int16_t Z;
		int16_t W;

		//float Accuracy;
	};

	struct BHYTimestamp {
		uint32_t TimeS;
		uint32_t TimeNS;
	};

	struct BHYFrame {
		BHYQuaternion Orientation;
		BHYTimestamp Timestamp;

		uint8_t SensorId;


		static constexpr size_t Size = 4 * sizeof(int16_t) + /* sizeof(float) + */ 2 * sizeof(uint32_t)
		+ sizeof(uint8_t);

		void SerializeTo(uint8_t* dest, uint8_t* size);
	};

	BHYWrapper(SPI_HandleTypeDef *spiHandle);
	BHYWrapper() = default;

	int Init(float sampleRate, uint32_t reportLatency);
	bool Poll();
	BHYFrame GetFrame() const;
	size_t GetSeq() const;

private:
	bhy2_dev bhy2;

	SPI_HandleTypeDef *SPIHandle;

	static constexpr size_t WorkBufferSize = 2048;
	std::array<uint8_t, WorkBufferSize> WorkBuffer;

	BHYFrame CurrentFrame;
	static const size_t CallbackDataSize = 11;

	static void ParseFrame(const bhy2_fifo_parse_data_info *cbInfo,
			void *cbRef);

	size_t frameSeq = 0;
};
