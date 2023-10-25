#ifndef MOTHERBOARD_CONFIG
#define MOTHERBOARD_CONFIG

#include "usart.h"
#include <stdint.h>

struct MotherboardConfig {
	struct {
		UART_HandleTypeDef* Uart;
	} HeadIO;

	struct {
		UART_HandleTypeDef* Uart;
		size_t TimeoutMs;
		size_t NAttempts;
	} BodyClient;

	struct {
		uint8_t Period;
	} BodyQueue;

	struct {
		SPI_HandleTypeDef* Spi;
		float SampleRate;
		uint32_t ReportLatency;
	} IMUDevice;

	struct {
		uint8_t TargetDuration;
		uint8_t DurationThreshold;
	} StrobeFilter;
};

#endif
