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

struct Periphery {
	static const uint8_t Body = 0;
	static const uint8_t Imu = 1;
	static const uint8_t Ack = 2;
};

struct Request {
	std::vector<uint8_t> Data;
	size_t ResponceSize;
	uint8_t MetaInfo;
	uint8_t PeripheryID;
};

struct Responce {
	std::vector<uint8_t> Data;
	uint8_t PeripheryID;
	uint8_t MetaInfo;
	uint8_t Error;
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

		static uint8_t Serialize(Type error) {
			return error;
		}
		static Type Deserialize(uint8_t val) {
			return val;
		}
	};

private:
	std::deque<Request> Requests;
	std::queue<Responce> Responces;

	Request PriorityRequest;
	bool HasPriorityRequest = false;

	bool WaitResponce = false;
	std::vector<uint8_t> CurrentResponceBuffer;

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

	void AddRequest(Request &&request) {
		switch (MessageMode::Deserialize(request.MetaInfo)) {
		case MessageMode::Async:
			Requests.emplace_back(std::move(request));
			break;
		case MessageMode::Sync:
			if (!HasPriorityRequest) {
				PriorityRequest = std::move(request);
				HasPriorityRequest = true;
			}
			break;

		case MessageMode::Info:
			Responces.emplace(CreateInfoResponce());
			break;
		case MessageMode::SetPeriod:
			Responces.emplace(ProcessSetPeriodRequest(request));
			break;
		}
	}

	bool HasResponce() const {
		return !Responces.empty();
	}

	void TickTimer() {
		SendTick = (SendTick + 1) % SendPeriod;
		if (SendTick != 0) return;

		TimerReady = true;
	}

	void SetSendPeriod(uint8_t periodMs) {
		assert(periodMs);
		SendPeriod = periodMs;
	}

	Responce GetResponce() {
		assert(HasResponce());
		Responce responce = std::move(Responces.front());
		Responces.pop();
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
					HAL_UART_Transmit_IT(UartHandle, data.data(), data.size())
							== HAL_OK);

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

			Responces.emplace(
					CreateResponce(CurrentResponceBuffer, MessageMode::Sync,
							error));

			WaitResponce = false;
		} else {
			__enable_irq();
		}
	}

	void ProcessRequests() {
		__disable_irq();
		if (TimerReady && !Requests.empty() && !WaitResponce
				&& TransmitComplete) {

			WaitResponce = true;
			__enable_irq();

			auto &request = Requests.front();
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
								data.size()) == HAL_OK);

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

			auto cap = Requests.max_size();
			auto sz = Requests.size();

			Requests.pop_front();
			WaitResponce = false;
			TimerReady = false;
		} else {
			__enable_irq();
		}
	}

	ErrorCode::Type Receive(uint8_t size) {
		assert(size >= 4);
		CurrentResponceBuffer.resize(size);

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
		uint8_t kondoNACK[4] = { 0x4, 0xFE, 0x15, 0x17 };

		for (int i = 0; i < 4; ++i)
			if (data[i] != kondoNACK[i])
				return false;

		return true;
	}

	Responce CreateResponce(const std::vector<uint8_t> &data,
			MessageMode::Type messageMode, ErrorCode::Type error) const {
		Responce responce;
		responce.Data = data;
		responce.PeripheryID = Periphery::Body;
		responce.Error = ErrorCode::Serialize(error);
		responce.MetaInfo = MessageMode::Serialize(messageMode);
		return responce;
	}

	Responce CreateInfoResponce() const {
		std::vector<uint8_t> data;
		data.resize(Info::Size);

		uint8_t *ptr = data.data();
		GetInfo().SerializeTo(&ptr);

		return CreateResponce(data, MessageMode::Info, ErrorCode::Success);
	}

	Responce ProcessSetPeriodRequest(const Request &request) {
		assert(MessageMode::Deserialize(request.MetaInfo) == MessageMode::SetPeriod);
		std::vector<uint8_t> data = {0};

		uint8_t newPeriod = request.Data[0];

		ErrorCode::Type error = ErrorCode::Success;

		if (newPeriod == 0)
			error = ErrorCode::BadPeriod;
		else
			SetSendPeriod(newPeriod);

		return CreateResponce(data, MessageMode::SetPeriod, error);
	}

	void ProcessResponces() {
		//WaitResponce = false;
	}

	void FinishTransmit() {
		TransmitComplete = true;
	}

	Info GetInfo() const {
		return {Requests.size(), Responces.size()};
	}
};

struct HeadInterface {
	static const uint8_t SOM1Val = 0xFF;
	static const uint8_t SOM2Val = 0xAA;
	static const uint8_t SOM3Val = 0xAF;

	std::queue<Request> Requests;

	uint8_t CurrentValue;
	size_t RequestSize;

	enum class ReadState {
		SOM1, SOM2, PERIPHERY_ID, REQUEST_SIZE, RESPONCE_SIZE, META, DATA, SOM3
	};

	ReadState CurrentState;

	Request CurrentRequest;

	std::vector<uint8_t> CurrentResponceBuffer;

	UART_HandleTypeDef *UartHandle;
	size_t TimeoutS;

	bool TransmitComplete = true;

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
		return !Requests.empty();
	}

	void Send(const Responce &responce) {

		while (!TransmitComplete) {
		}

		size_t size = responce.Data.size() + 3 + 3;
		CurrentResponceBuffer.resize(size);

		uint8_t *ptr = CurrentResponceBuffer.data();

		*(ptr++) = SOM1Val;
		*(ptr++) = SOM2Val;
		*(ptr++) = responce.PeripheryID;
		*(ptr++) = responce.MetaInfo;
		*(ptr++) = responce.Error;

		memcpy(ptr, responce.Data.data(), responce.Data.size());

		ptr += responce.Data.size();

		*ptr = SOM3Val;

		//ResetReadState();

		TransmitComplete = false;

		size_t sz = CurrentResponceBuffer.size();

		uint8_t testBuf[64];
		memcpy(testBuf, CurrentResponceBuffer.data(), sz);

		assert(
				HAL_UART_Transmit_IT(UartHandle, CurrentResponceBuffer.data(),
						sz) == HAL_OK);
	}

	Request GetRequest() {
		assert(HasRequest());
		Request request = std::move(Requests.front());
		//auto cap = Requests.max_size();
		auto sz = Requests.size();

		Requests.pop();
		return request;
	}

	void FinishTransmit() {
		TransmitComplete = true;
	}

	void ProcessRecievedData() {
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
			RequestSize = CurrentValue;
			CurrentRequest.Data.resize(RequestSize);
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
					RequestSize);
			break;
		}
		case ReadState::DATA: {
			CurrentState = ReadState::SOM3;
			HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
			break;
		}
		case ReadState::SOM3: {
			if (CurrentValue == SOM3Val) {
				Requests.emplace(std::move(CurrentRequest));
				CurrentRequest = { };
			}
			ResetReadState();
			break;
		}
		default:
			ResetReadState();
		}
	}
};

using IMUFrame = BHYWrapper::BHYFrame;

class IMUFrameContainer {
	std::deque<IMUFrame> FrameQueue;
	size_t FirstSeq = 0;

	size_t MaxFrames = 300;
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
	void Reset() {
		FrameQueue = { };
		FirstSeq = 0;
	}

	void Add(const BHYWrapper::BHYFrame &frame) {
		FrameQueue.push_front(frame);
		if (FrameQueue.size() > MaxFrames)
			Remove();
	}

	void Remove() {
		FrameQueue.pop_back();
		FirstSeq++;
	}

	bool Get(size_t seq, BHYWrapper::BHYFrame &frame) const {
		if (FrameQueue.empty())
			return false;

		if (seq < FirstSeq || seq > FrameQueue.size() + FirstSeq - 1)
			return false;

		auto imuFrame = FrameQueue[(FrameQueue.size() - 1) - (seq - FirstSeq)];
		//assert(imuFrame.Seq == seq);

		frame = imuFrame;
		return true;
	}

	bool GetLast(BHYWrapper::BHYFrame &frame) const {
		if (FrameQueue.empty())
			return false;
		return Get(FrameQueue.size() - 1, frame);
	}

	Info GetInfo() const {
		return {FirstSeq, FrameQueue.size(), MaxFrames};
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

class IMURequestHandler {
public:
	struct RequestMode {
		using Type = uint8_t;
		static constexpr Type FrameBySeq = 0;
		static constexpr Type Info = 1;
		static constexpr Type LatestFrame = 2;
		static constexpr Type Reset = 3;
		static constexpr Type SetOffset = 4;
		static constexpr Type StrobeWidth = 5;
		static constexpr Type ConfigureFilter = 6;

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
	};

private:
	Responce GetFrameBySeq(const Request &request,
			const IMUFrameContainer &container) {
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::FrameBySeq);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::FrameBySeq);
		responce.Data.resize(BHYWrapper::BHYFrame::Size);

		if (request.Data.size() != 2) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		uint16_t frameSeq =
				*reinterpret_cast<const uint16_t*>(request.Data.data());

		BHYWrapper::BHYFrame imuFrame;
		bool ok = container.Get(frameSeq, imuFrame);

		if (!ok) {
			responce.Error = ErrorCodes::FrameUnavailable;
			return responce;
		}

		uint8_t sz;
		imuFrame.SerializeTo(responce.Data.data(), &sz);
		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce GetInfo(const Request &request,
			const IMUFrameContainer &container) {
		assert(RequestMode::Deserialize(request.MetaInfo) == RequestMode::Info);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::Info);
		responce.Data.resize(IMUFrameContainer::Info::Size);

		if (request.Data.size() != 1) {
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
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::LatestFrame);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::LatestFrame);
		responce.Data.resize(BHYWrapper::BHYFrame::Size);

		if (request.Data.size() != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		BHYWrapper::BHYFrame imuFrame = IMU.GetFrame();

		uint8_t sz;
		imuFrame.SerializeTo(responce.Data.data(), &sz);

		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce DoReset(const Request &request, IMUFrameContainer &container,
			StrobeDurationFilter &sFilter) {
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::Reset);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::Reset);
		responce.Data.resize(1);

		if (request.Data.size() != 1) {
			responce.Error = ErrorCodes::BadRequest;
			return responce;
		}

		container.Reset();
		sFilter.ResetStrobeDuration();

		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce SetOffset(const Request &request, size_t &strobeOffset) {
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::SetOffset);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::SetOffset);
		responce.Data.resize(1);

		if (request.Data.size() != 1) {
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
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::ConfigureFilter);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(
				RequestMode::ConfigureFilter);
		responce.Data.resize(1);

		if (request.Data.size() != 2) {
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
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::StrobeWidth);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::StrobeWidth);
		responce.Data.resize(1);

		if (request.Data.size() != 1) {
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
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(
				RequestMode::Deserialize(request.MetaInfo));
		responce.Data.resize(request.ResponceSize);
		responce.Error = ErrorCodes::UnknownMode;
		return responce;
	}

public:
	Responce Handle(const Request &request, IMUFrameContainer &container,
			const BHYWrapper &IMU, size_t &strobeOffset,
			StrobeDurationFilter &sFilter) {
		assert(request.PeripheryID == Periphery::Imu);

		switch (RequestMode::Deserialize(request.MetaInfo)) {
		case RequestMode::FrameBySeq:
			return GetFrameBySeq(request, container);
		case RequestMode::Info:
			return GetInfo(request, container);
		case RequestMode::LatestFrame:
			return GetLatestFrame(request, IMU);
		case RequestMode::Reset:
			return DoReset(request, container, sFilter);
		case RequestMode::SetOffset:
			return SetOffset(request, strobeOffset);
		case RequestMode::StrobeWidth:
			return StrobeWidth(request, sFilter);
		case RequestMode::ConfigureFilter:
			return ConfigureFilter(request, sFilter);
		default:
			return UnknownModeResponce(request);
		}
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

class AcknowledgeHandler {
private:
	struct Version {
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

	Version CurrentVersion;

public:
	AcknowledgeHandler(uint8_t versionMaj, uint8_t versionMin) {
		CurrentVersion.Major = versionMaj;
		CurrentVersion.Minor = versionMin;
	}

	Responce Handle(const Request &request) {
		assert(request.PeripheryID == Periphery::Ack);

		Responce responce;
		responce.PeripheryID = Periphery::Ack;
		responce.MetaInfo = 0;
		responce.Error = 0;
		responce.Data.resize(2);

		uint8_t *ptr = responce.Data.data();
		CurrentVersion.SerializeTo(&ptr);

		return responce;
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
