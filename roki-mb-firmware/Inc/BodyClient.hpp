#ifndef BODY_CLIENT
#define BODY_CLIENT

#include <usart.h>
#include <string.h>
#include "roki-mb-service/MbService.hpp"

struct BodyClient {
public:
	enum class Status {
		Success, Nack, ACKTimeout, Unknown, EOMTimeout
	};

private:
	UART_HandleTypeDef *Uart;
	size_t Timeout;

	bool ARQEnabled = false;
	uint8_t NACKBuf[256] = { };
	uint8_t NACKSize = 0;
	size_t AttemptC = 1;

	const uint8_t *TxBuf;
	uint8_t TxSize;

	uint8_t *RxBuf;
	uint8_t RxSize;

	bool Locked = false;

private:
	static HAL_StatusTypeDef DebugReceive(UART_HandleTypeDef *uart,
			uint8_t *rxBuf, size_t rxSize, size_t timeout) {
		assert(uart);
		assert(rxBuf);

		size_t i = 0;
		HAL_StatusTypeDef status = HAL_OK;
		for (; i < rxSize; ++i) {
			status = HAL_UART_Receive(uart, rxBuf + i, 1, timeout);
			if (status != HAL_OK)
				break;
		}

		if (i > 0)
			return HAL_OK;
		return status;
	}

	Status TrySynchronize() {
		HAL_UART_Transmit(Uart, TxBuf, TxSize, Timeout);

		if (!ARQEnabled) {
			//auto status = HAL_UART_Receive(Uart, RxBuf, RxSize, Timeout);
			auto status = DebugReceive(Uart, RxBuf, RxSize, Timeout);

			if (status == HAL_TIMEOUT)
				return Status::EOMTimeout;

			return Status::Success;
		}

		auto status = HAL_UART_Receive(Uart, RxBuf, NACKSize, Timeout);
		if (status == HAL_TIMEOUT)
			return Status::ACKTimeout;
		if (status != HAL_OK)
			return Status::Unknown;
		if (RxIsNack()) {
			HAL_UART_Receive(Uart, RxBuf, RxSize, Timeout);
			return Status::Nack;
		}

		uint8_t *rxPtr = RxBuf + NACKSize;
		uint8_t rxRem = RxSize - NACKSize;

		if (rxRem == 0)
			return Status::Success;

		status = HAL_UART_Receive(Uart, rxPtr, rxRem, Timeout);

		if (status == HAL_TIMEOUT)
			return Status::EOMTimeout;
		if (status != HAL_OK)
			return Status::Unknown;

		return Status::Success;
	}

	bool RxIsNack() const {
		return memcmp(RxBuf, NACKBuf, NACKSize) == 0;
	}

public:
	explicit BodyClient(UART_HandleTypeDef *uart, size_t timeoutMs) :
			Uart { uart }, Timeout { timeoutMs } {
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

		for (size_t i = 0; i < AttemptC; ++i)
			if ((status = TrySynchronize()) == Status::Success)
				break;

		return status;
	}

	void EnableARQ(const uint8_t *nackBuf, uint8_t nackSz, uint8_t attemptC) {
		assert(nackBuf);

		__disable_irq();
		ARQEnabled = true;
		NACKSize = nackSz;
		AttemptC = attemptC;
		memcpy(NACKBuf, nackBuf, NACKSize);
		__enable_irq();
	}

	void DisableARQ() {
		__disable_irq();
		ARQEnabled = false;
		AttemptC = 1;
		__enable_irq();
	}

	void SetTimeout(uint8_t timeoutMs) {
		__disable_irq();
		Timeout = timeoutMs;
		__enable_irq();
	}

	uint8_t ReconfigureUart(
			const MbInterface::Messages::BodyUARTConfig &config) {
		size_t ByteSize;
		size_t StopBits;
		size_t Parity;

		const auto badReq = MbInterface::MbService::ErrorCodes::BadRequest;
		const auto halErr = MbInterface::MbService::ErrorCodes::BodyUnknownError;

		switch (config.ByteSize.Value) {
		case 7:
			ByteSize = UART_WORDLENGTH_7B;
			break;
		case 8:
			ByteSize = UART_WORDLENGTH_8B;
			break;
		case 9:
			ByteSize = UART_WORDLENGTH_9B;
			break;
		default:
			return badReq;
		}

		switch (config.StopBits.Value) {
		case 1:
			StopBits = UART_STOPBITS_1;
			break;
		case 2:
			StopBits = UART_STOPBITS_2;
			break;
		default:
			return badReq;
		}

		using Par = MbInterface::Messages::BodyUARTConfig::ParityVal;

		switch (config.Parity.Value) {
		case Par::None:
			Parity = UART_PARITY_NONE;
			break;
		case Par::Even:
			Parity = UART_PARITY_EVEN;
			break;
		case Par::Odd:
			Parity = UART_PARITY_ODD;
			break;
		default:
			return badReq;
		}

		if (HAL_UART_DeInit(Uart) != HAL_OK)
			return halErr;

		Uart->Init.BaudRate = config.Baudrate.Value;
		Uart->Init.WordLength = ByteSize;
		Uart->Init.StopBits = StopBits;
		Uart->Init.Parity = Parity;

		Timeout = config.TimeoutMs.Value;

		if (HAL_UART_Init(Uart) != HAL_OK)
			return halErr;
		if (HAL_UARTEx_SetTxFifoThreshold(Uart, UART_TXFIFO_THRESHOLD_8_8)
				!= HAL_OK)
			return halErr;
		if (HAL_UARTEx_SetRxFifoThreshold(Uart, UART_RXFIFO_THRESHOLD_8_8)
				!= HAL_OK)
			return halErr;
		if (HAL_UARTEx_EnableFifoMode(Uart) != HAL_OK)
			return halErr;

		return MbInterface::MbService::ErrorCodes::Success;
	}
};

#endif
