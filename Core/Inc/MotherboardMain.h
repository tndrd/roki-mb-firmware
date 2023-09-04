/*
 * MotherBoardMain.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: tndrd
 */

#include "usart.h"
#include "spi.h"

struct MotherboardConfig {
	UART_HandleTypeDef *HeadServiceUart;
	UART_HandleTypeDef *HeadStreamUart;
	size_t HeadTimeout;

	UART_HandleTypeDef *BodyUart;
	size_t BodyTimeout;

	SPI_HandleTypeDef *IMUSpi;

	uint8_t VersionMajor;
	uint8_t VersionMinor;
};

int MotherboardInit(struct MotherboardConfig conf);
int MotherboardTick();
void MotherboardOnStrobe();
void MotherboardOnBodyRecieveComplete();
void MotherboardOnHeadServiceRecieveComplete();
void MotherboardOnHeadStreamRecieveComplete();
void MotherboardOnHeadServiceTransmitComplete();
void MotherboardOnHeadStreamTransmitComplete();
void MotherboardOnBodyTimerTick();
void MotherboardOnImuTimerTick();

