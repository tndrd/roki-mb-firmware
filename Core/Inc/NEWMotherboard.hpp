/*
 * NEWMotherboard.hpp
 *
 *  Created on: Oct 23, 2023
 *      Author: tndrd
 */

#ifndef INC_NEWMOTHERBOARD_HPP_
#define INC_NEWMOTHERBOARD_HPP_

#include "MbService.hpp"
#include "usart.h"
#include <string.h>
#include <array>
#include <queue>

template<typename T, size_t Capacity>
class FixedQueue {
private:
	size_t Size = 0;
	size_t Head = 0;
	size_t Tail = 0;

	std::array<T, Capacity> Buffer;
public:
	void Push(const T &rhs) {
		assert(Size != Capacity);

		Buffer[Head] = rhs;
		Head = (Head + 1) % Capacity;
		Size++;
	}

	void Pop() {
		assert(Size != 0);
		Tail = (Tail + 1) % Capacity;
		Size--;
	}

	void Clear() {
		Size = 0;
		Head = 0;
		Tail = 0;
		Buffer = { };
	}

	const T& Front() const {
		assert(Size);
		return Buffer[Tail];
	}

	bool Empty() const {
		return Size == 0;
	}

	bool Full() const {
		return Size == Capacity;
	}

	size_t GetSize() const {
		return Size;
	}

};

static constexpr size_t BufferSize = 256;
using BufferType = std::array<uint8_t, BufferSize>;

static constexpr size_t HeadRequestQueueSize = 4;
static constexpr size_t HeadResponceQueueSize = 4;
static constexpr size_t BodyQueueMaxSize = 500;

struct HeadIO {
private:
	static const uint8_t SOM1Val = 0xFF;
	static const uint8_t SOM2Val = 0xAA;
	static const uint8_t SOM3Val = 0xAF;
	static const uint8_t NACKVal = 0xFF;

public:
	struct Request {
		uint8_t ProcID;

		BufferType Data;
		uint8_t Size;

		bool IsBad() const {
			return ProcID == NACKVal;
		}
	};

	struct Responce {
		uint8_t Size;
		uint8_t Error;

		BufferType Data;
	};

public:
	struct Listener {
	private:
		enum class PackageState {
			SOM1, SOM2, PROC_ID, SIZE, DATA, SOM3
		};
		using PS = PackageState;

	private:
		FixedQueue<Request, HeadRequestQueueSize> Requests;

		UART_HandleTypeDef *Uart;
		uint8_t CurrentValue;
		Request CurrentRequest;
		PackageState State;

	private:
		void Receive1() {
			HAL_UART_Receive_IT(Uart, &CurrentValue, 1);
		}

		void ReceiveData() {
			HAL_UART_Receive_IT(Uart, CurrentRequest.Data.data(),
					CurrentRequest.Size);
		}

		void Reset() {
			State = PS::SOM1;
			Receive1();
		}

		uint8_t Get1() {
			return CurrentValue;
		}

		void HandleBadRequest() {
			Request badRequest;
			badRequest.ProcID = NACKVal;

			Requests.Push(badRequest);
			Reset();
		}

		void Callback() {
			switch (State) {
			case PS::SOM1: {
				if (Get1() == SOM1Val) {
					State = PS::SOM2;
					Receive1();
				} else
					HandleBadRequest();
				break;
			}
			case PS::SOM2: {
				if (Get1() == SOM2Val) {
					State = PS::PROC_ID;
					Receive1();
				} else
					HandleBadRequest();
				break;
			}
			case PS::PROC_ID: {
				CurrentRequest.ProcID = Get1();
				State = PS::SIZE;
				Receive1();
				break;
			}
			case PS::SIZE: {
				CurrentRequest.Size = Get1();
				State = PS::DATA;
				ReceiveData();
				break;
			}
			case PS::DATA: {
				State = PS::SOM3;
				Receive1();
				break;
			}
			case PS::SOM3: {
				if (Get1() == SOM3Val) {
					Requests.Push(CurrentRequest);
					Reset();
				} else
					HandleBadRequest();
				break;
			}
			default:
				HandleBadRequest();
			}
		}
	public:
		explicit Listener(UART_HandleTypeDef *uart) :
				Uart { uart } {
			assert(uart);
		}

		void Init() {
			Reset();
		}

		bool HasRequest() {
			return !Requests.Empty();
		}

		Request GetRequest() {
			assert(HasRequest());
			Request request = Requests.Front();
			Requests.Pop();
			return request;
		}
	};

	struct Sender {
	private:
		FixedQueue<Responce, HeadResponceQueueSize> Responces;
		UART_HandleTypeDef *Uart;
		BufferType Buffer;

		bool Ready = true;
	private:
		void Send(const Responce &responce) {
			assert(Ready);
			Ready = false;

			size_t bufSize = responce.Size + 5;

			Buffer[0] = SOM1Val;
			Buffer[1] = SOM2Val;
			Buffer[2] = responce.Error;
			Buffer[3] = responce.Size;

			memcpy(&Buffer[4], responce.Data.data(), responce.Size);

			Buffer[bufSize - 1] = SOM3Val;

			HAL_UART_Transmit_IT(Uart, Buffer.data(), Buffer.size());
		}

		void Callback() {
			Ready = true;
		}

	public:
		explicit Sender(UART_HandleTypeDef *uart) :
				Uart(uart) {
			assert(uart);
		}

		void AddResponce(const Responce &responce) {
			Responces.Push(responce);
		}

		void Tick() {
			if (Responces.Empty())
				return;
			if (!Ready)
				return;

			Send(Responces.Front());
			Responces.Pop();
		}
	};
public:
	Listener Input;
	Sender Output;

	explicit HeadIO(UART_HandleTypeDef *uart) :
			Input { uart }, Output { uart } {
	}
};

struct BodyClient {
public:
	enum class Status {
		Success, Nack, ACKTimeout, Unknown, EOMTimeout
	};
private:
	static constexpr uint8_t NACKVal[] = { 0x4, 0xFE, 0x15, 0x17 };
	static constexpr size_t NACKSize = sizeof(NACKVal) / sizeof(NACKVal[0]);

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

		auto status = HAL_UART_Receive(Uart, RxBuf, NACKSize, Timeout);
		if (status == HAL_TIMEOUT)
			return Status::ACKTimeout;
		if (status != HAL_OK)
			return Status::Unknown;
		if (RxIsNack())
			return Status::Nack;

		uint8_t *rxPtr = RxBuf + NACKSize;
		uint8_t rxRem = RxSize - NACKSize;

		status = HAL_UART_Receive(Uart, rxPtr, rxRem, Timeout);

		if (status == HAL_TIMEOUT)
			return Status::EOMTimeout;
		if (status != HAL_OK)
			return Status::Unknown;

		return Status::Success;
	}

	bool RxIsNack() const {
		return memcmp(RxBuf, NACKVal, NACKSize) == 0;
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

		for (int i = 0; i < NAttempts; ++i)
			if ((status = TrySynchronize()) == Status::Success)
				break;

		return status;
	}
};

struct BodyQueue {
public:
	struct BodyRequest {
		BufferType Data;
		uint8_t TxSize;
		uint8_t RxSize;
	};

private:
	FixedQueue<BodyRequest, BodyQueueMaxSize> Requests;
	size_t Period;
	size_t Counter = 0;

	bool Ready = true;
private:
	void Callback() {
		Counter = (Counter + 1) % Period;
		if (Counter == 0)
			Ready = true;
	}

public:
	BodyQueue(size_t period) :
			Period { period } {
		assert(period > 0);
	}

	bool IsReady() const {
		return !Requests.Empty() && Ready;
	}

	BodyRequest GetRequest() {
		assert(IsReady());
		BodyRequest request = Requests.Front();
		Requests.Pop();
		Ready = false;
		return request;
	}

	bool IsFull() const {
		return Requests.Full();
	}

	void AddRequest(const BodyRequest &request) {
		assert(!Requests.Full());
		Requests.Push(request);
	}

	void SetPeriod(size_t periodMs) {
		assert(periodMs > 0);
		Period = periodMs;
	}

	void GetSize() const {
		return Requests.GetSize();
	}
};

#include "IMU_funcs.h"

class IMUDevice {
public:
	struct QuaternionT {
		int16_t X;
		int16_t Y;
		int16_t Z;
		int16_t W;
	};

	struct TimestampT {
		uint32_t TimeS;
		uint32_t TimeNS;
	};

	struct IMUFrame {
		QuaternionT Orientation;
		TimestampT Timestamp;

		uint8_t SensorId;
	};

private:
	static constexpr size_t WorkBufferSize = 2048;
	std::array<uint8_t, WorkBufferSize> WorkBuffer;
	static const size_t CallbackDataSize = 11;

	bhy2_dev bhy2;
	float SampleRate;
	uint32_t ReportLatency;

	SPI_HandleTypeDef *Spi;
	IMUFrame CurrentFrame;
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
			void *cbRef);

	int InitInternal() {
		uint8_t product_id = 0;
		uint16_t bhy2KernelVersion;

		uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO
				| BHY2_ICTL_DISABLE_DEBUG;

		uint8_t hif_ctrl = 0;
		uint8_t boot_status;
		uint8_t sensor_error;

		spi_init (SPIHandle);

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

	void Tick() {
		Update();
	}

	IMUFrame GetFrame() const {
		return CurrentFrame;
	}

	size_t GetSeq() const {
		return CurrentSeq;
	}
};

struct StrobeFilter {
private:
	enum class PulseState {
		Up, Down
	};

private:
	uint32_t FallTime = 0;
	uint32_t RiseTime = 0;
	PulseState State = PulseState::Down;

	uint32_t DurationThreshold;
	uint32_t TargetDuration;

	float StrobeDuration = 0;

	std::queue<size_t> StrobeQueue;
	size_t CurrentSeq = 0;

	const IMUDevice *IMU;

private:
	void Callback() {
		uint32_t currentTime = HAL_GetTick();

		switch (State) {
		case PulseState::Down: {
			State = PulseState::Up;

			uint32_t startTime = RiseTime;
			RiseTime = currentTime;

			if (!FallTime || !RiseTime)
				return;

			uint32_t duration = currentTime - startTime;

			StrobeDuration += duration;
			StrobeDuration /= 2;

			if (std::abs(long(duration - TargetDuration))
					< long(DurationThreshold))
				StrobeQueue.push(CurrentSeq);

			CurrentSeq = IMU->GetSeq();
			break;
		}
		case PulseState::Up:
			State = PulseState::Down;
			FallTime = currentTime;
			break;
		}
	}

public:
	StrobeFilter(uint8_t targetDuration, uint8_t durationThreshold,
			const IMUDevice *imu) :
			IMU { imu } {
		assert(imu);
		Configure(targetDuration, durationThreshold);
	}

	void Configure(uint8_t targetDuration, uint8_t durationThreshold) {
		TargetDuration = targetDuration;
		DurationThreshold = durationThreshold;
	}

	float GetStrobeDuration() const {
		return StrobeDuration;
	}

	bool HasStrobe() const {
		return !StrobeQueue.empty();
	}

	size_t GetStrobe() {
		assert(HasStrobe());
		size_t strobe = StrobeQueue.front();
		StrobeQueue.pop();
		return strobe;
	}
};

struct StrobeContainer {
private:
	std::queue<size_t> Strobes;
	uint8_t Offset;
public:
	explicit StrobeContainer(uint8_t offset) :
			Offset { offset } {
	}

	void SetOffset(uint8_t offset) {
		Offset = offset;
	}

	void AddStrobe(size_t strobe) {
		Strobes.push(strobe + Offset);
	}

	void HasStrobe() const {
		return !Strobes.empty();
	}

	size_t GetStrobe() const {
		assert(HasStrobe());
		return Strobes.front();
	}

	bool CheckSeq(size_t seq) {
		if (!HasStrobe()) return false;
		if (seq < GetStrobe()) return false;
		Strobes.pop();
		return true;
	}
};

struct StrobeCollector {
public:
	struct CollectTargets {
		bool IMU;
		bool Body;
	};

private:
	StrobeContainer IMU;
	StrobeContainer Body;

public:
	StrobeCollector(size_t imuOffset, size_t bodyOffset) :
			IMU { imuOffset }, Body { bodyOffset } {
	}

	void SetIMUOffset(uint8_t offset) {
		IMU.SetOffset(offset);
	}

	void SetBodyOffset(uint8_t offset) {
		Body.SetOffset(offset);
	}

	void AddStrobe(size_t strobe) {
		IMU.AddStrobe(strobe);
		Body.AddStrobe(strobe);
	}

	CollectTargets CheckIfNeedsToCollect(const IMUDevice& imu) const {
		size_t seq = imu.GetSeq();

		if (seq
	}

};

#endif /* INC_NEWMOTHERBOARD_HPP_ */
