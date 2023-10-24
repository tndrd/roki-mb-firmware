/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include <cassert>

#include <queue>
#include <deque>
#include <vector>
#include <BHYWrapper.hpp>
#include <cmath>
#include <memory.h>
#include <string.h>

#include "Requests.hpp"

#define DEFAULT_CONTAINER_CAPACITY 300

struct Periphery {
	static const uint8_t Body = 0;
	static const uint8_t Motherboard = 1;
};

struct Request {
	using BufferT = std::array<uint8_t, 256>;

	BufferT Data;
	size_t RequestSize;
	size_t ResponceSize;
	uint8_t MetaInfo;
	uint8_t PeripheryID;
};

struct Responce {
	using BufferT = std::array<uint8_t, 256>;

	BufferT Data;
	size_t ResponceSize;
	uint8_t Error;
};

template<typename T, size_t Capacity>
class FixedQueue {
private:
	size_t Size = 0;
	size_t Head = 0;
	size_t Tail = 0;

	std::array<T, Capacity> Buffer;
public:
	bool Push(const T &rhs) {
		assert(Size != Capacity);

		Buffer[Head] = rhs;
		Head = (Head + 1) % Capacity;
		Size++;

		return true;
	}

	bool Pop() {
		assert(Size != 0);
		Tail = (Tail + 1) % Capacity;
		Size--;
		return true;
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

struct QueueSender {
private:
	struct MessageMode final {
		using Type = uint8_t;
		static constexpr Type Sync = 0;
		static constexpr Type Async = 1;
		static constexpr Type Info = 2;
		static constexpr Type SetPeriod = 3;

		static uint8_t Serialize(Type mode) {
			return mode;
		}
		static Type Deserialize(uint8_t val) {
			return val;
		}
	};

	struct ErrorCode final {
		using Type = uint8_t;
		static constexpr Type Success = 0;
		static constexpr Type Timeout = 1;
		static constexpr Type NACK = 2;
		static constexpr Type Unknown = 3;
		static constexpr Type BadPeriod = 4;
		static constexpr Type QueueFull = 5;

		static uint8_t Serialize(Type error) {
			return error;
		}
		static Type Deserialize(uint8_t val) {
			return val;
		}
	};

private:
	FixedQueue<Request, 500> Requests;
	FixedQueue<Responce, 10> Responces;

	Request PriorityRequest;
	bool HasPriorityRequest = false;

	bool WaitResponce = false;
	Responce::BufferT CurrentResponceBuffer;

	UART_HandleTypeDef *UartHandle;

	size_t TimeoutS;
	uint8_t SendPeriod = 1;

	uint8_t SendTick = 0;

	bool TransmitComplete = true;
	bool TimerReady = false;

public:
	struct Info {
		uint16_t NumRequests;
		uint16_t NumResponces;

		static constexpr size_t Size = 2 * sizeof(uint16_t);

		void SerializeTo(uint8_t **ptr) {
			assert(ptr);
			assert(*ptr);

			*reinterpret_cast<uint16_t*>(*ptr) = NumRequests;
			*ptr += sizeof(uint16_t);

			*reinterpret_cast<uint16_t*>(*ptr) = NumResponces;
			*ptr += sizeof(uint16_t);
		}
	};

public:
	QueueSender() = default;

	QueueSender(UART_HandleTypeDef *uart, size_t timeoutS, uint8_t sendPeriod) :
			UartHandle { uart }, TimeoutS { timeoutS }, SendPeriod { sendPeriod } {
		assert(uart != NULL);
	}

	void AddRequest(const Request &request) {
		switch (MessageMode::Deserialize(request.MetaInfo)) {
		case MessageMode::Async:
			Responces.Push(CreateAsyncResponce(request));
			break;
		case MessageMode::Sync:
			if (HasPriorityRequest)
				break;

			PriorityRequest = request;
			HasPriorityRequest = true;
			break;

		case MessageMode::Info:
			Responces.Push(CreateInfoResponce());
			break;
		case MessageMode::SetPeriod:
			Responces.Push(ProcessSetPeriodRequest(request));
			break;
		}
	}

	bool HasResponce() const {
		return !Responces.Empty();
	}

	void TickTimer() {
		SendTick = (SendTick + 1) % SendPeriod;
		if (SendTick != 0)
			return;

		TimerReady = true;
	}

	void SetSendPeriod(uint8_t periodMs) {
		assert(periodMs);
		SendPeriod = periodMs;
	}

	Responce GetResponce() {
		assert(HasResponce());
		Responce responce = Responces.Front();
		Responces.Pop();
		return responce;
	}

	void ProcessPriorityRequest() {
		__disable_irq();
		if (HasPriorityRequest && !WaitResponce && TransmitComplete) {
			HasPriorityRequest = false;
			WaitResponce = true;
			__enable_irq();

			auto &request = PriorityRequest;
			auto &data = request.Data;

			assert(
					MessageMode::Deserialize(request.MetaInfo)
							== MessageMode::Sync);
			__disable_irq();
			TransmitComplete = false;
			__enable_irq();

			assert(
					HAL_UART_Transmit_IT(UartHandle, data.data(),
							request.RequestSize) == HAL_OK);

			while (!TransmitComplete)
				;

			ErrorCode::Type error = Receive(request.ResponceSize);

			if (error == ErrorCode::NACK) {
				uint32_t delayMS = 5;
				HAL_Delay(delayMS);

				uint8_t dummy;
				while (HAL_UART_Receive(UartHandle, &dummy, 1, 0) == HAL_OK)
					;
			}

			Responces.Push(
					CreateResponce(CurrentResponceBuffer, request.ResponceSize,
							MessageMode::Sync, error));

			WaitResponce = false;
		} else {
			__enable_irq();
		}
	}

	Responce SendRequest(const Request &request) {
		while (WaitResponce)
			;
		WaitResponce = true;

		auto &data = request.Data;

		assert(TransmitComplete);

		__disable_irq();
		TransmitComplete = false;
		__enable_irq();

		uint8_t testBuf[64];
		memcpy(testBuf, data.data(), request.RequestSize);

		assert(
				HAL_UART_Transmit_IT(UartHandle, data.data(),
						request.RequestSize) == HAL_OK);

		while (!TransmitComplete)
			;

		ErrorCode::Type error = Receive(request.ResponceSize);

		if (error == ErrorCode::NACK) {
			uint32_t delayMS = 5;
			HAL_Delay(delayMS);

			uint8_t dummy;
			while (HAL_UART_Receive(UartHandle, &dummy, 1, 0) == HAL_OK)
				;
		}

		auto newResponce = CreateResponce(CurrentResponceBuffer,
				request.ResponceSize, MessageMode::Sync, error);

		WaitResponce = false;

		return newResponce;
	}

	Request ServoDataRequest() {
		Request request;

		memcpy(request.Data.data(), BodyMessages::ServoPosRequest,
				sizeof(BodyMessages::ServoPosRequest));
		request.RequestSize = sizeof(BodyMessages::ServoPosRequest);
		request.ResponceSize = BodyMessages::ServoPosResponceSize;

		request.MetaInfo = 0;
		request.PeripheryID = Periphery::Body;

		return request;
	}

	Responce GetServoData() {
		Request request = ServoDataRequest();
		return SendRequest(request);
	}

	void ProcessRequests() {
		__disable_irq();
		if (TimerReady && !Requests.Empty() && !WaitResponce
				&& TransmitComplete) {

			WaitResponce = true;
			__enable_irq();

			auto &request = Requests.Front();
			auto &data = request.Data;

			assert(
					MessageMode::Deserialize(request.MetaInfo)
							== MessageMode::Async);

			const size_t nAttempts = 5;
			const uint32_t delayMS = 5;
			size_t i = 0;

			while (i++ < nAttempts) {

				TransmitComplete = false;

				assert(
						HAL_UART_Transmit_IT(UartHandle, data.data(),
								request.RequestSize) == HAL_OK);

				while (!TransmitComplete)
					;

				ErrorCode::Type error = Receive(request.ResponceSize);
				if (error == ErrorCode::Success)
					break;
				HAL_Delay(delayMS);
				if (error == ErrorCode::NACK) {
					uint8_t dummy;
					while (HAL_UART_Receive(UartHandle, &dummy, 1, 0) == HAL_OK)
						;
				}
			}

			Requests.Pop();
			WaitResponce = false;
			TimerReady = false;
		} else {
			__enable_irq();
		}
	}

	ErrorCode::Type Receive(uint8_t size) {
		assert(size >= 4);

		auto ret = HAL_UART_Receive(UartHandle, CurrentResponceBuffer.data(), 4,
				TimeoutS);

		if (ret == HAL_TIMEOUT)
			return ErrorCode::Timeout;

		if (IsNack(CurrentResponceBuffer.data()))
			return ErrorCode::NACK;

		if (ret != HAL_OK)
			return ErrorCode::Unknown;

		if (size == 4)
			return ErrorCode::Success;

		ret = HAL_UART_Receive(UartHandle, CurrentResponceBuffer.data() + 4,
				size - 4, TimeoutS);

		if (ret == HAL_TIMEOUT)
			return ErrorCode::Timeout;

		if (ret != HAL_OK)
			return ErrorCode::Unknown;

		return ErrorCode::Success;
	}

	bool IsNack(const uint8_t *data) {
		assert(data);

		for (size_t i = 0; i < sizeof(BodyMessages::KondoNack); ++i)
			if (data[i] != BodyMessages::KondoNack[i])
				return false;

		return true;
	}

	Responce CreateResponce(const Responce::BufferT &data, size_t ResponceSize,
			MessageMode::Type messageMode, ErrorCode::Type error) const {
		Responce responce;
		responce.Data = data;
		responce.ResponceSize = ResponceSize;
		responce.Error = ErrorCode::Serialize(error);
		return responce;
	}

	Responce CreateInfoResponce() const {
		Responce::BufferT data;
		uint8_t *ptr = data.data();
		GetInfo().SerializeTo(&ptr);

		return CreateResponce(data, Info::Size, MessageMode::Info,
				ErrorCode::Success);
	}

	Responce ProcessSetPeriodRequest(const Request &request) {
		assert(
				MessageMode::Deserialize(request.MetaInfo)
						== MessageMode::SetPeriod);
		Responce::BufferT data;

		uint8_t newPeriod = request.Data[0];

		ErrorCode::Type error = ErrorCode::Success;

		if (newPeriod == 0)
			error = ErrorCode::BadPeriod;
		else
			SetSendPeriod(newPeriod);

		return CreateResponce(data, 1, MessageMode::SetPeriod, error);
	}

	Responce CreateAsyncResponce(const Request &request) {
		assert(
				MessageMode::Deserialize(request.MetaInfo)
						== MessageMode::Async);
		ErrorCode::Type error = ErrorCode::Success;

		if (Requests.Full())
			error = ErrorCode::QueueFull;
		else
			Requests.Push(request);

		return CreateResponce( { 0 }, 1, MessageMode::Async, error);
	}

	void ProcessResponces() {
		//WaitResponce = false;
	}

	void FinishTransmit() {
		TransmitComplete = true;
	}

	Info GetInfo() const {
		uint16_t numRequests = Requests.GetSize();
		uint16_t numResponces = Responces.GetSize();
		return {numRequests, numResponces};
	}
};

struct HeadInterface {
private:
	static const uint8_t SOM1Val = 0xFF;
	static const uint8_t SOM2Val = 0xAA;
	static const uint8_t SOM3Val = 0xAF;

	enum class ReadState {
		SOM1, SOM2, PERIPHERY_ID, REQUEST_SIZE, RESPONCE_SIZE, META, DATA, SOM3
	};

private:
	FixedQueue<Request, 10> Requests;
	UART_HandleTypeDef *UartHandle;
	size_t TimeoutS;

	uint8_t CurrentValue;
	ReadState CurrentState;
	Request CurrentRequest;
	Responce::BufferT CurrentResponceBuffer;

	bool TransmitComplete = true;
public:
	HeadInterface() = default;

	HeadInterface(UART_HandleTypeDef *uart, size_t timeoutS) :
			UartHandle { uart }, TimeoutS { timeoutS } {
		assert(uart != NULL);
	}

	void ResetReadState() {
		CurrentState = ReadState::SOM1;
		assert(HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1) == HAL_OK);
	}

	bool HasRequest() const {
		return !Requests.Empty();
	}

	void Send(const Responce &responce) {

		while (!TransmitComplete) {
		}

		uint8_t *ptr = CurrentResponceBuffer.data();

		*(ptr++) = SOM1Val;
		*(ptr++) = SOM2Val;
		*(ptr++) = responce.Error;

		memcpy(ptr, responce.Data.data(), responce.ResponceSize);

		ptr += responce.ResponceSize;
		*ptr = SOM3Val;

		//ResetReadState();

		TransmitComplete = false;

		size_t sz = responce.ResponceSize + 3 + 1;
		uint8_t testBuf[64];
		memcpy(testBuf, CurrentResponceBuffer.data(), sz);

		assert(
				HAL_UART_Transmit_IT(UartHandle, CurrentResponceBuffer.data(),
						sz) == HAL_OK);
	}

	Request GetRequest() {
		assert(HasRequest());
		Request request = Requests.Front();
		Requests.Pop();
		return request;
	}

	void FinishTransmit() {
		TransmitComplete = true;
	}

	void ProcessRecievedData() {
		static size_t nRequests = 0;

		switch (CurrentState) {
		case ReadState::SOM1: {
			if (CurrentValue == SOM1Val) {
				CurrentState = ReadState::SOM2;
				HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			} else
				ResetReadState();
			break;
		}
		case ReadState::SOM2: {
			if (CurrentValue == SOM2Val) {
				CurrentState = ReadState::PERIPHERY_ID;
				HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			} else
				ResetReadState();
			break;
		}
		case ReadState::PERIPHERY_ID: {
			CurrentRequest.PeripheryID = CurrentValue;
			CurrentState = ReadState::REQUEST_SIZE;
			HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			break;
		}
		case ReadState::REQUEST_SIZE: {
			CurrentRequest.RequestSize = CurrentValue;
			CurrentState = ReadState::RESPONCE_SIZE;
			HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			break;
		}
		case ReadState::RESPONCE_SIZE: {
			CurrentRequest.ResponceSize = CurrentValue;
			CurrentState = ReadState::META;
			HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			break;
		}
		case ReadState::META: {
			CurrentRequest.MetaInfo = CurrentValue;
			CurrentState = ReadState::DATA;
			HAL_UART_Receive_IT(UartHandle, CurrentRequest.Data.data(),
					CurrentRequest.RequestSize);
			break;
		}
		case ReadState::DATA: {
			CurrentState = ReadState::SOM3;
			HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			break;
		}
		case ReadState::SOM3: {
			if (CurrentValue == SOM3Val) {
				Requests.Push(CurrentRequest);
				nRequests++;
			}
			ResetReadState();
			break;
		}
		default:
			ResetReadState();
		}
	}
};

struct ServoDataFrame {
	Responce ServoData;
	uint8_t Error;

	static constexpr size_t Size = BodyMessages::ServoPosResponceSize;

	ServoDataFrame(const Responce &servoData) :
			ServoData { servoData }, Error { servoData.Error } {
		assert(ServoData.ResponceSize == BodyMessages::ServoPosResponceSize);
	}

	ServoDataFrame() = default;

	void SerializeTo(uint8_t **ptr) {
		assert(ptr);
		assert(*ptr);

		memcpy(*ptr, ServoData.Data.data(), ServoData.ResponceSize);

		*ptr += ServoData.ResponceSize;
	}
};

struct IMUFrame {
	BHYWrapper::BHYFrame Frame;
	uint8_t Error = 0;

	static constexpr size_t Size = 4 * sizeof(int16_t)
			+ /* sizeof(float) + */2 * sizeof(uint32_t) + sizeof(uint8_t);

	IMUFrame(const BHYWrapper::BHYFrame &frame) :
			Frame { frame } {
	}

	IMUFrame() = default;

	void SerializeTo(uint8_t **ptr) {
		assert(ptr);
		assert(*ptr);

		*reinterpret_cast<int16_t*>(*ptr) = Frame.Orientation.X;
		*ptr += sizeof(int16_t);

		*reinterpret_cast<int16_t*>(*ptr) = Frame.Orientation.Y;
		*ptr += sizeof(int16_t);

		*reinterpret_cast<int16_t*>(*ptr) = Frame.Orientation.Z;
		*ptr += sizeof(int16_t);

		*reinterpret_cast<int16_t*>(*ptr) = Frame.Orientation.W;
		*ptr += sizeof(int16_t);

		/*
		 *reinterpret_cast<float*>(*ptr) = Orientation.Accuracy;
		 *ptr += sizeof(float);
		 */

		*reinterpret_cast<uint32_t*>(*ptr) = Frame.Timestamp.TimeS;
		*ptr += sizeof(uint32_t);

		*reinterpret_cast<uint32_t*>(*ptr) = Frame.Timestamp.TimeNS;
		*ptr += sizeof(uint32_t);

		*reinterpret_cast<uint8_t*>(*ptr) = Frame.SensorId;
		*ptr += sizeof(uint8_t);
	}
};

template<typename T>
class FrameContainer {
	std::deque<T> FrameQueue;
	uint16_t FirstSeq = 0;
	uint16_t MaxFrames;

public:
	struct Info {
		uint16_t First;
		uint16_t NumAv;
		uint16_t MaxFrames;

		static constexpr size_t Size = 3 * sizeof(uint16_t);

		void SerializeTo(uint8_t **ptr) {
			assert(ptr);
			assert(*ptr);

			*reinterpret_cast<uint16_t*>(*ptr) = First;
			*ptr += sizeof(uint16_t);

			*reinterpret_cast<uint16_t*>(*ptr) = NumAv;
			*ptr += sizeof(uint16_t);

			*reinterpret_cast<uint16_t*>(*ptr) = MaxFrames;
			*ptr += sizeof(uint16_t);
		}
	};

public:

	FrameContainer(uint16_t maxFrames) :
			MaxFrames { maxFrames } {
	}

	void Reset() {
		FrameQueue = { };
		FirstSeq = 0;
	}

	void Add(const T &frame) {
		FrameQueue.push_front(frame);
		if (FrameQueue.size() > MaxFrames)
			Remove();
	}

	void Remove() {
		FrameQueue.pop_back();
		FirstSeq++;
	}

	bool Get(size_t seq, T &frame) const {
		if (FrameQueue.empty())
			return false;

		if (seq < FirstSeq || seq > FrameQueue.size() + FirstSeq - 1)
			return false;

		frame = FrameQueue[(FrameQueue.size() - 1) - (seq - FirstSeq)];
		return true;
	}

	Info GetInfo() const {
		uint16_t size = FrameQueue.size();
		return {FirstSeq, size, MaxFrames};
	}
};

class StrobeDurationFilter {
public:
	enum class PulseState {
		Up, Down
	};
private:
	uint32_t FallTime = 0;
	uint32_t RiseTime = 0;
	PulseState State = PulseState::Down;

	uint32_t DurationThreshold = 0;
	uint32_t TargetDuration = 0;

	float StrobeDuration = 1;

	std::queue<size_t> StrobeQueue;
	size_t CurrentSeq = 0;

public:
	void ProcessStrobe(const BHYWrapper &IMU) {
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

			CurrentSeq = IMU.GetSeq();
			break;
		}
		case PulseState::Up:
			State = PulseState::Down;
			FallTime = currentTime;
			break;
		}
	}

	void Configure(uint8_t targetDuration, uint8_t durationThreshold) {
		TargetDuration = targetDuration;
		DurationThreshold = durationThreshold;
	}

	float GetStrobeDuration() const {
		return StrobeDuration;
	}

	void ResetStrobeDuration() {
		StrobeDuration = 1;
	}

	size_t GetStrobe() const {
		assert(HasStrobe());
		return StrobeQueue.front();
	}

	bool HasStrobe() const {
		return !StrobeQueue.empty();
	}

	void PopStrobe() {
		StrobeQueue.pop();
	}
};

class VersionProvider {
public:
	struct Version {
		static constexpr uint8_t Size = 2;

		uint8_t Major;
		uint8_t Minor;

		void SerializeTo(uint8_t **ptr) {
			assert(ptr);
			assert(*ptr);

			**ptr = Major;
			*ptr += sizeof(uint8_t);

			**ptr = Minor;
			*ptr += sizeof(uint8_t);
		}
	};

private:
	Version CurrentVersion;

public:
	VersionProvider(uint8_t versionMaj, uint8_t versionMin) {
		CurrentVersion.Major = versionMaj;
		CurrentVersion.Minor = versionMin;
	}

	Version GetVersion() const {
		return CurrentVersion;
	}
};

class IMUFrameMemo {
	std::deque<BHYWrapper::BHYFrame> Queue;
	size_t MaxSize = 800 / 5;
	size_t FirstSeq = 0;

public:
	void Add(const BHYWrapper::BHYFrame &frame, size_t seq) {
		if (Queue.empty()) {
			FirstSeq = seq;
		}

		Queue.push_front(frame);

		if (Queue.size() > MaxSize) {
			Queue.pop_back();
			FirstSeq++;
		}
	}

	bool Has(size_t seq) const {
		if (Queue.empty())
			return false;

		if (seq < FirstSeq + Queue.size())
			return true;

		return false;
	}

	BHYWrapper::BHYFrame Get(size_t seq) const {
		assert(Has(seq));

		if (seq < FirstSeq) {
			return Queue.front();
		}

		return Queue[(Queue.size() - 1) - (seq - FirstSeq)];
	}
};

struct MotherboardContext {
	HeadInterface Head;
	QueueSender Body;


	FrameContainer<IMUFrame> IMUFrameContainer {DEFAULT_CONTAINER_CAPACITY};
	FrameContainer<ServoDataFrame> BodyFrameContainer {DEFAULT_CONTAINER_CAPACITY};

	BHYWrapper IMU;
	VersionProvider Version { 0, 0 };

	IMUFrameMemo FrameMemo;

	StrobeDurationFilter StrobeFilter;
	size_t IMUStrobeOffset;
	size_t BodyStrobeOffset;

	std::queue<size_t> IMUStrobes;
	std::queue<size_t> BodyStrobes;

	bool UpdateIMU = false;

	MotherboardContext(MotherboardConfig conf) :
			Head { conf.HeadServiceUart, conf.HeadTimeout }, Body {
					conf.BodyUart, conf.BodyTimeout, conf.BodyPeriod }, IMUFrameContainer {
					conf.FrameContainerCapacity }, BodyFrameContainer {
					conf.FrameContainerCapacity }, IMU { conf.IMUSpi }, Version {
					conf.VersionMajor, conf.VersionMinor }, IMUStrobeOffset {
					conf.IMUStrobeOffset }, BodyStrobeOffset {
					conf.BodyStrobeOffset } {
	}

	MotherboardContext() = default;
};

class MotherboardRequestHandler {
public:
	struct RequestMode {
		using Type = uint8_t;
		static constexpr Type IMUFrame = 0;
		static constexpr Type BodyFrame = 1;
		static constexpr Type IMUInfo = 2;
		static constexpr Type BodyInfo = 3;
		static constexpr Type IMULatest = 4;
		static constexpr Type ResetContainers = 5;
		static constexpr Type IMUStrobeOffset = 6;
		static constexpr Type BodyStrobeOffset = 7;
		static constexpr Type GetStrobeWidth = 8;
		static constexpr Type ConfigureStrobeFilter = 9;
		static constexpr Type GetVersion = 10;

		static uint8_t Serialize(Type mode) {
			return mode;
		}
		static Type Deserialize(uint8_t meta) {
			return meta;
		}
	};

	struct ErrorCodes {
		using Type = uint8_t;
		static constexpr Type Success = 0;
		static constexpr Type FrameUnavailable = 1;
		static constexpr Type UnknownMode = 2;
		static constexpr Type BadRequest = 3;
		static constexpr Type BadOffset = 4;
		static constexpr Type TargetError = 5;
	};

private:
	template<typename T>
	Responce GetFrameBySeq(const Request &request,
			const FrameContainer<T> &container) {
		Responce responce;
		responce.ResponceSize = T::Size;

		if (request.RequestSize != 2) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		uint16_t frameSeq =
				*reinterpret_cast<const uint16_t*>(request.Data.data());

		T frame;
		bool ok = container.Get(frameSeq, frame);

		if (!ok) {
			responce.Error = ErrorCodes::FrameUnavailable;
			return responce;
		}

		if (frame.Error) {
			responce.Error = ErrorCodes::TargetError;
			responce.Data[0] = frame.Error;
			return responce;
		}

		uint8_t *ptr = responce.Data.data();
		frame.SerializeTo(&ptr);
		responce.Error = ErrorCodes::Success;

		return responce;
	}

	template<typename T>
	Responce GetInfo(const Request &request,
			const FrameContainer<T> &container) {
		Responce responce;
		responce.ResponceSize = FrameContainer<T>::Info::Size;

		if (request.RequestSize != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		uint8_t *ptr = responce.Data.data();
		auto info = container.GetInfo();

		info.SerializeTo(&ptr);

		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce GetLatestFrame(const Request &request, const BHYWrapper &IMU) {
		Responce responce;
		responce.ResponceSize = IMUFrame::Size;

		if (request.RequestSize != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		IMUFrame frame { IMU.GetFrame() };

		uint8_t *ptr = responce.Data.data();
		frame.SerializeTo(&ptr);

		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce DoReset(const Request &request, FrameContainer<IMUFrame> &imuCont,
			FrameContainer<ServoDataFrame> &bodyCont) {
		Responce responce;
		responce.ResponceSize = 1;

		if (request.RequestSize != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		imuCont.Reset();
		bodyCont.Reset();

		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce SetOffset(const Request &request, size_t &strobeOffset) {
		Responce responce;
		responce.ResponceSize = 1;

		if (request.RequestSize != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		uint8_t newOffset = request.Data[0];

		strobeOffset = newOffset;
		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce ConfigureFilter(const Request &request,
			StrobeDurationFilter &sFilter) {
		Responce responce;
		responce.ResponceSize = 1;

		if (request.RequestSize != 2) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		uint8_t targetDuration = request.Data[0];
		uint8_t durationThreshold = request.Data[1];

		sFilter.Configure(targetDuration, durationThreshold);
		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce StrobeWidth(const Request &request,
			const StrobeDurationFilter &sFilter) {
		Responce responce;
		responce.ResponceSize = 1;

		if (request.RequestSize != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		float strobeWidth = sFilter.GetStrobeDuration();

		if (strobeWidth < 0)
			strobeWidth = 0;
		if (strobeWidth > 255)
			strobeWidth = 255;

		responce.Data[0] = static_cast<uint8_t>(strobeWidth);
		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce UnknownModeResponce(const Request &request) {
		Responce responce;
		responce.ResponceSize = request.ResponceSize;
		responce.Error = ErrorCodes::UnknownMode;
		return responce;
	}

	Responce GetVersion(const Request &request,
			const VersionProvider &versionProvider) {
		Responce responce;
		responce.ResponceSize = VersionProvider::Version::Size;
		responce.Error = ErrorCodes::Success;

		uint8_t *ptr = responce.Data.data();
		versionProvider.GetVersion().SerializeTo(&ptr);

		return responce;
	}

public:
	Responce Handle(const Request &request, MotherboardContext &mbCtx) {
		assert(request.PeripheryID == Periphery::Motherboard);

		using M = RequestMode;

		switch (M::Deserialize(request.MetaInfo)) {
		case M::IMUFrame:
			return GetFrameBySeq(request, mbCtx.IMUFrameContainer);
		case M::BodyFrame:
			return GetFrameBySeq(request, mbCtx.BodyFrameContainer);
		case M::IMUInfo:
			return GetInfo(request, mbCtx.IMUFrameContainer);
		case M::BodyInfo:
			return GetInfo(request, mbCtx.BodyFrameContainer);
		case M::ResetContainers:
			return DoReset(request, mbCtx.IMUFrameContainer,
					mbCtx.BodyFrameContainer);
		case M::IMULatest:
			return GetLatestFrame(request, mbCtx.IMU);
		case M::ConfigureStrobeFilter:
			return ConfigureFilter(request, mbCtx.StrobeFilter);
		case M::IMUStrobeOffset:
			return SetOffset(request, mbCtx.IMUStrobeOffset);
		case M::BodyStrobeOffset:
			return SetOffset(request, mbCtx.BodyStrobeOffset);
		case M::GetStrobeWidth:
			return StrobeWidth(request, mbCtx.StrobeFilter);
		case M::GetVersion:
			return GetVersion(request, mbCtx.Version);

		default:
			return UnknownModeResponce(request);
		}
	}
};

/*
 class SystemStateFactory {
 public:
 Responce GetSystemState(const QueueSender &qs,
 const IMUFrameContainer &imuCont, const BHYWrapper &imu) {
 auto qsInfo = qs.GetInfo();
 auto imuInfo = imuCont.GetInfo();
 auto lastFr = imu.GetFrame();

 Responce responce;

 responce.Error = 0;
 responce.PeripheryID = Periphery::Global;
 responce.MetaInfo = 0;
 responce.Data.resize(
 QueueSender::Info::Size + IMUFrameContainer::Info::Size
 + BHYWrapper::BHYFrame::Size);

 uint8_t *ptr = responce.Data.data();

 qsInfo.SerializeTo(&ptr);
 imuInfo.SerializeTo(&ptr);

 uint8_t sz;
 lastFr.SerializeTo(ptr, &sz);

 return responce;
 }
 };
 */
