#ifndef IMU_DEVICE
#define IMU_DEVICE

#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_parse.h"
#include "bhy2_defs.h"
#include <stdarg.h>
#include <stdio.h>

#include "Bosch_SHUTTLE_BHI260_2.fw.h"
#include "IMU_funcs.h"

#include "MbMessages.hpp"

class IMUDevice {
public:
	using Frame = Roki::Messages::IMUFrameMsg;

private:
	static constexpr size_t WorkBufferSize = 2048;
	std::array<uint8_t, WorkBufferSize> WorkBuffer;
	static const size_t CallbackDataSize = 11;

	bhy2_dev bhy2;
	SPI_HandleTypeDef *Spi;

	float SampleRate;
	uint32_t ReportLatency;
	Frame CurrentFrame;
	size_t CurrentSeq = 0;

	bool DoUpdate = false;

private:
	void Callback() {
		DoUpdate = true;
	}

	void Update() {
		if (!DoUpdate)
			return;

		uint8_t interruptStatus = 0;
		bhy2_get_interrupt_status(&interruptStatus, &bhy2);

		if (!interruptStatus)
			return;

		int status;
		uint8_t *wbData = WorkBuffer.data();
		size_t wbSize = WorkBuffer.size();

		status = bhy2_get_and_process_fifo(wbData, wbSize, &bhy2);
		assert(status == BHY2_OK);

		CurrentSeq++;
		DoUpdate = false;
	}

	static void ParseFrame(const bhy2_fifo_parse_data_info *cbInfo,
			void *cbRef) {
		Frame *frame = reinterpret_cast<Frame*>(cbRef);
		bhy2_data_quaternion qtData;

		assert(frame);
		auto &timestamp = frame->Timestamp;
		auto &quaternion = frame->Orientation;

		if (cbInfo->data_size != CallbackDataSize)
			return;

		bhy2_parse_quaternion(cbInfo->data_ptr, &qtData);

		frame->SensorID = cbInfo->sensor_id;

		uint64_t timeData = *cbInfo->time_stamp * 15625; /* Store the last timestamp */

		timestamp.TimeS = (timeData / UINT64_C(1000000000));
		timestamp.TimeNS =
				(timeData - (timestamp.TimeS * UINT64_C(1000000000)));
		quaternion.X = qtData.x;
		quaternion.Y = qtData.y;
		quaternion.Z = qtData.z;
		quaternion.W = qtData.w;
		/*
		 quaternion.Accuracy = ((qtData.accuracy * 180.0f) / 16384.0f)
		 / 3.141592653589793f; */
	}

	int InitInternal() {
		uint8_t product_id = 0;
		uint16_t bhy2KernelVersion;

		uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO
				| BHY2_ICTL_DISABLE_DEBUG;

		uint8_t hif_ctrl = 0;
		uint8_t boot_status;
		uint8_t sensor_error;

		spi_init(Spi);

		if (bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write,
				bhy2_delay_us, 64, NULL, &bhy2))
			return 1;

		if (bhy2_soft_reset(&bhy2))
			return 2;

		if (bhy2_get_product_id(&product_id, &bhy2))
			return 3;

		if (product_id != BHY2_PRODUCT_ID)
			return 4;

		if (bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2))
			return 5;

		if (bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2))
			return 6;

		if (bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2))
			return 7;

		if (bhy2_get_boot_status(&boot_status, &bhy2))
			return 8;

		if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY))
			return 9;

		if (bhy2_upload_firmware_to_ram(bhy2_firmware_image,
				sizeof(bhy2_firmware_image), &bhy2))
			return 9;

		if (bhy2_get_error_value(&sensor_error, &bhy2))
			return 10;

		if (sensor_error)
			return 11;

		if (bhy2_boot_from_ram(&bhy2))
			return 12;

		if (bhy2_get_error_value(&sensor_error, &bhy2))
			return 13;

		if (sensor_error)
			return 14;

		if (bhy2_get_kernel_version(&bhy2KernelVersion, &bhy2))
			return 15;

		if (bhy2KernelVersion == 0)
			return 16;

		if (bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAMERV,
				IMUDevice::ParseFrame, &CurrentFrame, &bhy2))
			return 17;

		if (bhy2_get_and_process_fifo(WorkBuffer.data(), WorkBuffer.size(),
				&bhy2))
			return 18;

		if (bhy2_update_virtual_sensor_list(&bhy2))
			return 16;

		if (bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GAMERV, SampleRate,
				ReportLatency, &bhy2))
			return 17;

		return 0;
	}

public:
	IMUDevice(SPI_HandleTypeDef *spi, float sampleRate, uint32_t reportLatency) :
			Spi { spi }, SampleRate { sampleRate }, ReportLatency {
					reportLatency } {
		assert(spi);
	}

	IMUDevice() = default;

	void Init() {
		assert(InitInternal() == 0);
	}

	void CheckTimer() {
		Update();
	}

	Frame GetFrame() const {
		return CurrentFrame;
	}

	size_t GetSeq() const {
		return CurrentSeq;
	}

	void TimCallback() {
		Callback();
	}
};

#endif
