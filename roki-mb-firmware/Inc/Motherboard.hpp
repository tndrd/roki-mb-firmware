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

	bool HasElt(size_t i) const {
		return i < GetSize();
	}

	const T& Peek(size_t i) const {
		assert(HasElt(i));

		return Buffer[(Tail + i) % Capacity];
	}

};

struct Motherboard;

static constexpr size_t BufferSize = 256;
using BufferType = std::array<uint8_t, BufferSize>;

static constexpr size_t HeadRequestQueueSize = 4;
static constexpr size_t HeadResponceQueueSize = 4;
static constexpr size_t BodyQueueMaxSize = 500;
static constexpr size_t FrameQueueMaxSize = 300;

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

		void RxCpltCallback() {
			Callback();
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

		void CheckTimer() {
			if (Responces.Empty())
				return;
			if (!Ready)
				return;

			Send(Responces.Front());
			Responces.Pop();
		}

		void TxCpltCallback() {
			Callback();
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

struct BodyRequests {
	static constexpr struct GetAllPosRequest {
		const uint8_t Data[] = { 0xA, 0, 0x20, 0, 0, 0, 0x70, 0, 0x1E, 0xB8 };
		const size_t RequestSize = sizeof(Data) / sizeof(Data[0]);
		const size_t ResponceSize = 33;
	} GetAllPos;
};

struct BodyQueue {
public:
	struct Request {
		BufferType Data;
		uint8_t TxSize;
		uint8_t RxSize;
	};

private:
	FixedQueue<Request, BodyQueueMaxSize> Requests;
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

	Request GetRequest() {
		assert(IsReady());
		Request request = Requests.Front();
		Requests.Pop();
		Ready = false;
		return request;
	}

	bool IsFull() const {
		return Requests.Full();
	}

	void AddRequest(const Request &request) {
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

	void TimCallback() {
		Callback();
	}
};

#include "IMU_funcs.h"

class IMUDevice {
public:
	using Frame = Roki::Messages::IMUFrameMsg;

private:
	static constexpr size_t WorkBufferSize = 2048;
	std::array<uint8_t, WorkBufferSize> WorkBuffer;
	static const size_t CallbackDataSize = 11;

	bhy2_dev bhy2;
	float SampleRate;
	uint32_t ReportLatency;

	SPI_HandleTypeDef *Spi;
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

	void CompCallback() {
		Callback();
	}
};

// Saves desired strobes and calls callbacks when needed
struct StrobeObserver {
public:
	using CallbackT = void (*)(Motherboard*);
private:
	std::queue<size_t> Strobes;
	uint8_t Offset;

	const CallbackT Callback;
public:
	explicit StrobeObserver(uint8_t offset, CallbackT callback) :
			Offset { offset }, Callback { callback } {
		assert(callback);
	}

	void SetOffset(uint8_t offset) {
		Offset = offset;
	}

	void AddStrobe(size_t strobe) {
		Strobes.push(strobe + Offset);
	}

	void TryProcess(size_t currentSeq, Motherboard *mb) {
		if (Strobes.empty())
			return;
		if (currentSeq < Strobes.front())
			return;

		Callback(mb);

		Strobes.pop();
	}
};

struct StrobeObservers {
private:
	std::vector<StrobeObserver> Observers;

public:
	StrobeObservers() = default;

	void SetOffset(size_t i, uint8_t offset) {
		Observers.at(i).SetOffset(offset);
	}

	void RegisterObserver(uint8_t offset, StrobeObserver::CallbackT callback) {
		Observers.emplace_back(offset, callback);
	}

	void AddStrobe(size_t strobe) {
		for (auto &observer : Observers)
			observer.AddStrobe(strobe);
	}

	void TryProcess(size_t currentSeq) {
		for (auto &observer : Observers)
			observer.Tick(currentSeq);
	}
};

template<typename T, size_t Capacity>
struct FrameQueue {
private:
	FixedQueue<T, Capacity> Queue;
	size_t FirstSeq = 0;
public:
	FrameQueue() = default;

	void Add(const T &rhs) {
		if (Queue.Full()) {
			FirstSeq++;
			Queue.Pop();
		}
		Queue.Push(rhs);
	}

	bool HasSeq(size_t seq) const {
		if (Queue.Empty())
			return false;

		bool c1 = seq >= FirstSeq;
		bool c2 = seq < FirstSeq + Queue.GetSize();

		return c1 && c2;
	}

	T GetSeq(size_t seq) {
		assert(HasSeq(seq));

		return Queue.Peek(seq - FirstSeq);
	}

	void Clear() {
		FirstSeq = 0;
		Queue.Clear();
	}

public:
	Roki::Messages::FrameContainerInfo GetInfo() const {
		return {FirstSeq, Queue.GetSize(), Capacity};
	}
};

struct FrameQueues {
public:
	struct BodyResponce {
		BodyClient::Status Status;
		BufferType Data;
		uint8_t Size;
	};
public:
	static constexpr size_t MaxSize = FrameQueueMaxSize;

	FrameQueue<BodyResponce, MaxSize> BodyPos;
	FrameQueue<IMUDevice::Frame, MaxSize> IMUFrames;

	void Clear() {
		BodyPos.Clear();
		IMUFrames.Clear();
	}
};

struct MotherboardConfig {
	struct {
		UART_HandleTypeDef Uart;
	} HeadIO;

	struct {
		UART_HandleTypeDef Uart;
		size_t TimeoutMs;
		size_t NAttempts;
	} BodyClient;

	struct {
		uint8_t Period;
	} BodyQueue;

	struct {
		SPI_HandleTypeDef Spi;
		float SampleRate;
		uint32_t ReportLatency;
	} IMUDevice;

	struct {
		uint8_t TargetDuration;
		uint8_t DurationThreshold;
	} StrobeFilter;
};

struct MotherboardContext {
	static constexpr struct VersionT {
		uint8_t Major = 0;
		uint8_t Minor = 4;
	} Version;

	HeadIO Head;
	BodyClient Body;
	BodyQueue BQueue;
	IMUDevice IMU;
	StrobeFilter SFilter;
	StrobeObservers SObservers;
	FrameQueues FQueues;

	BufferType DummyBuf;

	MotherboardContext(MotherboardConfig c) :
			Head { c.HeadIO.Uart }, Body { c.BodyClient.Uart,
					c.BodyClient.TimeoutMs, c.BodyClient.NAttempts }, BQueue {
					c.BodyQueue.Period }, IMU { c.IMUDevice.Spi,
					c.IMUDevice.SampleRate, c.IMUDevice.ReportLatency }, SFilter {
					c.StrobeFilter.TargetDuration,
					c.StrobeFilter.DurationThreshold, &IMU } {
	}
};

struct RequestHandler {
private:
	using Request = HeadIO::Request;
	using Responce = HeadIO::Responce;
	using Service = Roki::MbService;
	using Msgs = Roki::Messages;
	using Proc = Service::Procedures;
	using Errors = Service::ErrorCodes;
	using PID = Service::ProcedureID;

private:
	BufferType Buffer;

private:
	Errors::Type ConvertBodyError(BodyClient::Status status) {
		using S = BodyClient::Status;

		switch (status) {
		case S::Success:
			return Errors::Success;
		case S::ACKTimeout:
			return Errors::BodyTimeout;
		case S::EOMTimeout:
			return Errors::BodyTimeout;
		case S::Nack:
			return Errors::BodyNACK;
		case S::Unknown:
			return Errors::BodyUnknownError;
		default:
			assert(0);
		}
	}

private:
	Responce CreateError(Errors::Type error) {
		Responce responce;
		responce.Error = Errors::Serialize(error);
		responce.Size = 0;
	}

	template<typename Procedure>
	Responce GenericHandler(MotherboardContext &ctx, const Request &request) {
		Procedure::RequestType reqMsg;
		Procedure::ResponceType rspMsg;

		reqMsg = Procedure::RequestType::Deserialize(request.Data.data());
		if (reqMsg.GetPackedSize() != request.Size)
			return CreateError(Errors::BadRequest);

		Errors::Type error = Handler<typename Procedure>(ctx, reqMsg, rspMsg);

		if (error != Errors::Success)
			return CreateError(error);

		Responce responce;
		responce.Error = Errors::Success;
		responce.Size = rspMsg.GetPackedSize();

		rspMsg.Serialize(responce.Data.data());

		return responce;
	}

	template<>
	Errors::Type Handler<Proc::GetIMUFrame>(MotherboardContext &ctx,
			const Msgs::FrameNumber &frNum, Msgs::IMUFrameMsg &result) {
		auto &fQueue = ctx.FQueues.IMUFrames;
		int16_t seq = frNum.Seq;

		if (!fQueue.HasSeq(seq))
			return Errors::FrameUnavailable;

		result = fQueue.GetSeq(seq);

		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::GetBodyFrame>(MotherboardContext &ctx,
			const Msgs::FrameNumber &frNum, Msgs::BodyResponce &result) {
		auto &fQueue = ctx.FQueues.BodyPos;
		int16_t seq = frNum.Seq;

		if (!fQueue.HasSeq(seq))
			return Errors::FrameUnavailable;

		auto resp = fQueue.GetSeq(seq);
		auto error = ConvertBodyError(resp.Status);

		if (error == Errors::Success) {
			memcpy(Buffer.data(), resp.Data.data(), resp.Size);
			result.Data = Buffer.data();
			result.ResponceSize = resp.Size;
		}

		return error;
	}

	template<>
	Errors::Type Handler<Proc::GetIMUContainerInfo>(Motherboard &ctx,
			const Msgs::Empty&, Msgs::FrameContainerInfo &result) {
		result = ctx.FQueues.IMUFrames.GetInfo();

		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::GetBodyContainerInfo>(Motherboard &ctx,
			const Msgs::Empty&, Msgs::FrameContainerInfo &result) {
		result = ctx.FQueues.BodyPos.GetInfo();
	}

	template<>
	Errors::Type Handler<Proc::GetIMULatest>(Motherboard &ctx,
			const Msgs::Empty&, Msgs::IMUFrameMsg &result) {
		result = ctx.IMU.GetFrame();

		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::ResetStrobeContainers>(Motherboard &ctx,
			const Msgs::Empty&, Msgs::Empty&) {
		ctx.FQueues.Clear();

		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::SetIMUStrobeOffset>(Motherboard &ctx,
			const Msgs::Byte &offset, Msgs::Empty&) {
		ctx.SObservers.SetOffset(0, offset.Value);
		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::SetBodyStrobeOffset>(Motherboard &ctx,
			const Msgs::Byte &offset, Msgs::Empty&) {
		ctx.SObservers.SetOffset(1, offset.Value);
		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::GetStrobeWidth>(Motherboard &ctx,
			const Msgs::Empty&, Msgs::Byte &result) {
		result.Value = uint8_t(ctx.SFilter.GetStrobeDuration());
		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::ConfigureStrobeFilter>(Motherboard &ctx,
			const Msgs::StrobeFilterConfig &config, Msgs::Empty&) {
		ctx.SFilter.Configure(config.TargetDuration, config.DurationThreshold);
		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::GetVersion>(Motherboard &ctx, const Msgs::Empty&,
			Msgs::Version &version) {
		version.Major = ctx.Version.Major;
		version.Minor = ctx.Version.Minor;
		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::BodySendForward>(Motherboard &ctx,
			const Msgs::BodyRequest &request,
			const Msgs::BodyResponce &responce) {

		auto &txBuf = request.Data;
		auto &txSize = request.RequestSize;
		auto &rxBuf = Buffer.data();
		auto &rxSize = request.ResponceSize;

		auto status = ctx.Body.Synchronize(txBuf, txSize, rxBuf, rxSize);
		auto error = ConvertBodyError(status);

		if (error == Errors::Success) {
			responce.Data = Buffer.data();
			responce.ResponceSize = request.ResponceSize;
		}

		return error;
	}

	template<>
	Errors::Type Handler<Proc::BodySendQueue>(Motherboard &ctx,
			const Msgs::BodyRequest &request, Msgs::Empty&) {
		if (ctx.BQueue.IsFull())
			return Errors::BodyQueueFull;

		BodyQueue::Request newReq;

		memcpy(newReq.Data.data(), request.Data, request.RequestSize);
		newReq.TxSize = request.RequestSize;
		newReq.RxSize = request.ResponceSize;

		ctx.BQueue.AddRequest(newReq);

		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::GetBodyQueueInfo>(Motherboard &ctx,
			const Msgs::Empty&, Msgs::BodyQueueInfo &info) {
		info.NumRequests = ctx.BQueue.GetSize();
		info.NumResponces = 0; // Placeholder
		return Errors::Success;
	}

	template<>
	Errors::Type Handler<Proc::SetBodyQueuePeriod>(Motherboard &ctx,
			const Msgs::PeriodMs &period, Msgs::Empty&) {
		if (period.Ms == 0)
			return Errors::BadBodyQueuePeriod;
		ctx.BQueue.SetPeriod(period.Ms);
		return Errors::Success;
	}

public:
	Responce Handle(MotherboardContext &ctx, const Request &request) {
		if (request.IsBad())
			return CreateError(Errors::NACK);

		auto procID = Service::ProcedureID::Deserialize(request.ProcID);

		switch (procID) {
		case PID::BodySendForward:
			return GenericHandler<Proc::BodySendForward>(ctx, request);
		case PID::BodySendQueue:
			return GenericHandler<Proc::BodySendQueue>(ctx, request);
		case PID::ConfigureStrobeFilter:
			return GenericHandler<Proc::ConfigureStrobeFilter>(ctx, request);
		case PID::GetBodyContainerInfo:
			return GenericHandler<Proc::GetBodyContainerInfo>(ctx, request);
		case PID::GetBodyFrame:
			return GenericHandler<Proc::GetBodyFrame>(ctx, request);
		case PID::GetBodyQueueInfo:
			return GenericHandler<Proc::GetBodyQueueInfo>(ctx, request);
		case PID::GetIMUContainerInfo:
			return GenericHandler<Proc::GetIMUContainerInfo>(ctx, request);
		case PID::GetIMUFrame:
			return GenericHandler<Proc::GetIMUFrame>(ctx, request);
		case PID::GetIMULatest:
			return GenericHandler<Proc::GetIMULatest>(ctx, request);
		case PID::GetStrobeWidth:
			return GenericHandler<Proc::GetStrobeWidth>(ctx, request);
		case PID::GetVersion:
			return GenericHandler<Proc::GetVersion>(ctx, request);
		case PID::ResetStrobeContainers:
			return GenericHandler<Proc::ResetStrobeContainers>(ctx, request);
		case PID::SetBodyQueuePeriod:
			return GenericHandler<Proc::SetBodyQueuePeriod>(ctx, request);
		case PID::SetBodyStrobeOffset:
			return GenericHandler<Proc::SetBodyStrobeOffset>(ctx, request);
		case PID::SetIMUStrobeOffset:
			return GenericHandler<Proc::SetIMUStrobeOffset>(ctx, request);

		default:
			return CreateError(Errors::UnknownProcedure);
		}
	}
}
};

struct Motherboard: public MotherboardContext {
private:
void BodyStrobeCallback() {
	struct FrameQueues::BodyResponce responce;
	auto &request = BodyRequests::GetAllPos;

	auto &txData = request.Data;
	auto &txSize = request.RequestSize;
	auto &rxData = responce.Data.data();
	auto &rxSize = request.ResponceSize;

	BodyClient::Status status;
	status = Body.Synchronize(txData, txSize, rxData, rxSize);

	responce.Status = status;
	if (status == BodyClient::Status::Success)
		responce.Size = request.ResponceSize;
	else
		responce.Size = 0;

	FQueues.BodyPos.Add(responce);
}

void ImuStrobeCallback() {
	FQueues.IMUFrames.Add(IMU.GetFrame());
}

void RegisterObservers() {
	SObservers.RegisterObserver(0, Motherboard::ImuStrobeCallback);
	SObservers.RegisterObserver(0, Motherboard::BodyStrobeCallback);
}

void UpdateTimers() {
	Head.Output.CheckTimer();
	IMU.CheckTimer();
}

void HandleRequests() {
	if (!Head.Input.HasRequest())
		return;

	auto request = Head.Input.GetRequest();
	auto responce = Handle(request);

	Head.Output.AddResponce(responce);
}

void UpdateBodyQueue() {
	if (!BQueue.IsReady())
		return;
	auto request = BQueue.GetRequest();

	auto &txBuf = request.Data.data();
	auto &txSize = request.TxSize;
	auto &rxBuf = DummyBuf.data();
	auto &rxSize = request.RxSize;

	Body.Synchronize(txBuf, txSize, rxBuf, rxSize);
}

void UpdateStrobes() {
	if (SFilter.HasStrobe())
		SObservers.AddStrobe(SFilter.GetStrobe());

	SObservers.TryProcess(IMU.GetSeq());
}
public:
void Init() {
	Head.Input.Init();
	IMU.Init();
}

void Run() {
	while (1) {
		UpdateTimers();
		HandleRequests();
		UpdateBodyQueue();
		UpdateStrobes();
	}
}

Motherboard(const MotherboardConfig &config) :
		MotherboardContext { config } {
	RegisterObservers();
}

public: // Callbacks
void HeadRxCallback() {
	Head.Input.RxCpltCallback();
}

void HeadTxCallback() {
	Head.Output.TxCpltCallback();
}

void BQueueTimCallback() {
	BQueue.TimCallback();
}

void IMUTimCallback() {
	IMU.TimCallback();
}

void SFCompCallback() {
	SFilter.CompCallback();
}
};

#endif /* INC_NEWMOTHERBOARD_HPP_ */
