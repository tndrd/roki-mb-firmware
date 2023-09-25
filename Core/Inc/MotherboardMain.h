/*
 * MotherBoardMain.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: tndrd
 */

#include "usart.h"
#include "spi.h"

typedef struct {
	UART_HandleTypeDef *HeadServiceUart;
	UART_HandleTypeDef *HeadStreamUart;
	size_t HeadTimeout;

	UART_HandleTypeDef *BodyUart;
	size_t BodyTimeout;
	uint8_t BodyPeriod;

	SPI_HandleTypeDef *IMUSpi;

	size_t BodyStrobeOffset;
	size_t IMUStrobeOffset;

	uint16_t FrameContainerCapacity;

	uint8_t VersionMajor;
	uint8_t VersionMinor;
} MotherboardConfig;

#ifdef __cplusplus
extern "C" {
#endif

int MotherboardInit(MotherboardConfig conf);
int MotherboardTick();
void MotherboardOnStrobe();
void MotherboardOnBodyRecieveComplete();
void MotherboardOnBodyTransmitComplete();
void MotherboardOnHeadRecieveComplete();
void MotherboardOnHeadTransmitComplete();
void MotherboardOnBodyTimerTick();
void MotherboardOnImuTimerTick();

#ifdef __cplusplus
}
#endif

