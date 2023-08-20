/*
 * MotherBoardMain.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: tndrd
 */

#include "usart.h"
#include "spi.h"

struct MotherboardConfig {
	UART_HandleTypeDef *HeadUart;
	size_t HeadTimeout;

	UART_HandleTypeDef *BodyUart;
	size_t BodyTimeout;

	SPI_HandleTypeDef *IMUSpi;
};

int MotherboardInit(struct MotherboardConfig conf);
int MotherboardTick();
void MotherboardOnStrobe();
void MotherboardOnBodyTransmitComplete();
void MotherboardOnBodyTimerTick();
void MotherboardOnImuTimerTick();

