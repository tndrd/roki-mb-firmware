#ifndef BODY_CLIENT
#define BODY_CLIENT

#include <usart.h>
#include "BodyMsgs.hpp"
#include <string.h>

struct BodyClient {
private:
	BodyMsgs::Responces::KondoNACK NACK;
public:
	enum class Status {
		Success, Nack, ACKTimeout, Unknown, EOMTimeout
	};

private:
	UART_HandleTypeDef *Uart;
	size_t Timeout;
	size_t NAttempts;

	const uint8_t *TxBuf;
	uint8_t TxSize;

	uint8_t *RxBuf;
	uint8_t RxSize;

private:
	Status TrySynchronize() {
		HAL_UART_Transmit(Uart, TxBuf, TxSize, Timeout);

		auto status = HAL_UART_Receive(Uart, RxBuf, NACK.Size, Timeout);
		if (status == HAL_TIMEOUT)
			return Status::ACKTimeout;
		if (status != HAL_OK)
			return Status::Unknown;
		if (RxIsNack())
			return Status::Nack;

		uint8_t *rxPtr = RxBuf + NACK.Size;
		uint8_t rxRem = RxSize - NACK.Size;

		status = HAL_UART_Receive(Uart, rxPtr, rxRem, Timeout);

		if (status == HAL_TIMEOUT)
			return Status::EOMTimeout;
		if (status != HAL_OK)
			return Status::Unknown;

		return Status::Success;
	}

	bool RxIsNack() const {
		return memcmp(RxBuf, NACK.Data, NACK.Size) == 0;
	}

public:
	explicit BodyClient(UART_HandleTypeDef *uart, size_t timeoutMs,
			size_t nAttempts) :
			Uart { uart }, Timeout { timeoutMs }, NAttempts { nAttempts } {
		assert(uart);
	}

	Status Synchronize(const uint8_t *txBuf, uint8_t txSize, uint8_t *rxBuf,
			uint8_t rxSize) {
		assert(txBuf && rxBuf);
		TxBuf = txBuf;
		TxSize = txSize;
		RxBuf = rxBuf;
		RxSize = rxSize;

		Status status;

		for (size_t i = 0; i < NAttempts; ++i)
			if ((status = TrySynchronize()) == Status::Success)
				break;

		return status;
	}
};

#endif
