#ifndef BODY_CLIENT
#define BODY_CLIENT

#include <usart.h>
#include <string.h>

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

private:
	Status TrySynchronize() {
		HAL_UART_Transmit(Uart, TxBuf, TxSize, Timeout);

		if (!ARQEnabled) {
			auto status = HAL_UART_Receive(Uart, RxBuf, RxSize, Timeout);

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

	void EnableARQ(const uint8_t* nackBuf, uint8_t nackSz, uint8_t attemptC) {
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
};

#endif
