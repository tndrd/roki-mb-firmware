/*
 * BHYWrapper.cpp
 *
 *  Created on: Aug 18, 2023
 *      Author: tndrd
 */

#include "BHYWrapper.hpp"

BHYWrapper::BHYWrapper(SPI_HandleTypeDef *spiHandle) :
		SPIHandle { spiHandle } {
	assert(spiHandle);
}

int BHYWrapper::Init(float sampleRate, uint32_t reportLatency) {
	uint8_t product_id = 0;
	uint16_t bhy2KernelVersion;

	uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;

	uint8_t hif_ctrl = 0;
	uint8_t boot_status;
	uint8_t sensor_error;

	spi_init(SPIHandle);

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
			BHYWrapper::ParseFrame, &CurrentFrame, &bhy2))
		return 17;

	if (bhy2_get_and_process_fifo(WorkBuffer.data(), WorkBuffer.size(), &bhy2))
		return 18;

	if (bhy2_update_virtual_sensor_list(&bhy2))
		return 16;

	if (bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GAMERV, sampleRate,
			reportLatency, &bhy2))
		return 17;

	return 0;
}

bool BHYWrapper::Poll() {
	uint8_t interruptStatus = 0;
	bhy2_get_interrupt_status(&interruptStatus, &bhy2);

	if (interruptStatus) {
		assert(
				bhy2_get_and_process_fifo(WorkBuffer.data(), WorkBuffer.size(), &bhy2) == BHY2_OK);
		return true;
	}

	return false;
}

BHYWrapper::BHYFrame BHYWrapper::GetFrame() const {
	return CurrentFrame;
}

void BHYWrapper::ParseFrame(const bhy2_fifo_parse_data_info *cbInfo,
		void *cbRef) {
	BHYFrame *frame = reinterpret_cast<BHYFrame*>(cbRef);
	bhy2_data_quaternion qtData;

	assert(frame);
	auto &timestamp = frame->Timestamp;
	auto &quaternion = frame->Orientation;

	if (cbInfo->data_size != CallbackDataSize)
		return;

	bhy2_parse_quaternion(cbInfo->data_ptr, &qtData);

	frame->SensorId = cbInfo->sensor_id;

	uint64_t timeData = *cbInfo->time_stamp * 15625; /* Store the last timestamp */

	timestamp.TimeS = (timeData / UINT64_C(1000000000));
	timestamp.TimeNS = (timeData - (timestamp.TimeS * UINT64_C(1000000000)));
	quaternion.X = qtData.x;
	quaternion.Y = qtData.y;
	quaternion.Z = qtData.z;
	quaternion.W = qtData.w;
	/*
	 quaternion.Accuracy = ((qtData.accuracy * 180.0f) / 16384.0f)
	 / 3.141592653589793f; */
}

void BHYWrapper::BHYFrame::SerializeTo(uint8_t *dest, uint8_t *size) {
	assert(dest);
	assert(size);

	uint8_t *ptr = dest;

	*reinterpret_cast<int16_t*>(ptr) = Orientation.X;
	ptr += sizeof(int16_t);

	*reinterpret_cast<int16_t*>(ptr) = Orientation.Y;
	ptr += sizeof(int16_t);

	*reinterpret_cast<int16_t*>(ptr) = Orientation.Z;
	ptr += sizeof(int16_t);

	*reinterpret_cast<int16_t*>(ptr) = Orientation.W;
	ptr += sizeof(int16_t);

	 /*
	 *reinterpret_cast<float*>(ptr) = Orientation.Accuracy;
	 ptr += sizeof(float);
	 */

	*reinterpret_cast<uint32_t*>(ptr) = Timestamp.TimeS;
	ptr += sizeof(uint32_t);

	*reinterpret_cast<uint32_t*>(ptr) = Timestamp.TimeNS;
	ptr += sizeof(uint32_t);

	*reinterpret_cast<uint8_t*>(ptr) = SensorId;
	ptr += sizeof(uint8_t);

	*size = Size;
}
