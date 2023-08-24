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

struct Periphery {
	static const uint8_t Body = 0;
	static const uint8_t Imu = 1;
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
	std::deque<Request> Requests;
	std::queue<Responce> Responces;

	bool WaitResponce = false;
	std::vector<uint8_t> CurrentResponceBuffer;

	struct MessageMode final {
		static constexpr uint8_t Forward = 0;
		static constexpr uint8_t Ignore = 1;
		static constexpr uint8_t Notify = 2;
		static constexpr uint8_t Priority = 3;
	};

	using MessageModeT = uint8_t;

	MessageModeT CurrentRequestMode;

	UART_HandleTypeDef *UartHandle;

	size_t TimeoutS;

	QueueSender() = default;

	QueueSender(UART_HandleTypeDef *uart, size_t timeoutS) :
			UartHandle { uart }, TimeoutS { timeoutS } {
		assert(uart != NULL);
	}

	MessageModeT GetMessageMode(const Request &request) {
		return request.MetaInfo;
	}

	void SetMessageMode(Responce &responce, MessageModeT mode) {
		responce.MetaInfo = mode;
	}

	void AddRequest(Request &&request) {
		switch (GetMessageMode(request)) {
		case MessageMode::Forward:
		case MessageMode::Ignore:
		case MessageMode::Notify:
			Requests.emplace_back(std::move(request));
			break;
		case MessageMode::Priority:
			Requests.emplace_front(std::move(request));
			break;
		}
	}

	bool HasResponce() const {
		return !Responces.empty();
	}

	Responce GetResponce() {
		assert(HasResponce());
		auto responce = std::move(Responces.front());
		Responces.pop();
		return responce;
	}

	void ProcessRequests() {
		if (!Requests.empty() && !WaitResponce) {
			auto &request = Requests.front();
			auto &data = request.Data;

			if (GetMessageMode(request) == MessageMode::Notify) {
				Responces.emplace(CreateResponce( { }, MessageMode::Notify));
				return;
			}

			CurrentResponceBuffer.resize(request.ResponceSize);
			CurrentRequestMode = GetMessageMode(request);
			WaitResponce = true;

			HAL_UART_Receive_IT(UartHandle, CurrentResponceBuffer.data(),
					CurrentResponceBuffer.size());
			HAL_UART_Transmit(UartHandle, data.data(), data.size(), TimeoutS);

			Requests.pop_front();
		}
	}

	Responce CreateResponce(const std::vector<uint8_t> &data,
			MessageModeT messageMode) {
		Responce responce;
		responce.Data = data;
		responce.PeripheryID = Periphery::Body;
		responce.Error = 0;
		SetMessageMode(responce, messageMode);
		return responce;
	}

	void ProcessResponces() {
		if (CurrentRequestMode != MessageMode::Ignore) {
			Responces.emplace(
					CreateResponce(CurrentResponceBuffer, CurrentRequestMode));
		}
		WaitResponce = false;
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
		HAL_UART_Receive_IT(UartHandle, &CurrentValue, 1);
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

		HAL_UART_Transmit_IT(UartHandle, CurrentResponceBuffer.data(), sz);
	}

	Request GetRequest() {
		assert(HasRequest());
		auto request = std::move(Requests.front());
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
				Requests.push(std::move(CurrentRequest));
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

struct IMUFrame: public BHYWrapper::BHYFrame {
	size_t Seq;

	IMUFrame(const BHYWrapper::BHYFrame &frame, size_t seq) :
			BHYFrame { frame }, Seq { seq } {
	}
};

class IMUFrameContainer {
	std::deque<IMUFrame> FrameQueue;
	size_t FirstSeq = 0;

	size_t TestSeq = 0;
public:
	struct Info {
		uint16_t First;
		uint16_t NumAv;

		static constexpr size_t Size = 2 * sizeof(uint16_t);
	};

public:
	void Reset() {
		FrameQueue = { };
		FirstSeq = 0;
	}

	void Add(const BHYWrapper::BHYFrame &frame) {
		FrameQueue.push_front( { frame, TestSeq++ });
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
		return {FirstSeq, FrameQueue.size()};
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
		}

		uint8_t *ptr = responce.Data.data();
		auto info = container.GetInfo();

		*reinterpret_cast<uint16_t*>(ptr) = info.First;
		ptr += sizeof(uint16_t);

		*reinterpret_cast<uint16_t*>(ptr) = info.NumAv;
		ptr += sizeof(uint16_t);

		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce GetLatestFrame(const Request &request,
			const BHYWrapper &IMU) {
		assert(
				RequestMode::Deserialize(request.MetaInfo)
						== RequestMode::LatestFrame);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::LatestFrame);
		responce.Data.resize(BHYWrapper::BHYFrame::Size);

		if (request.Data.size() != 1) {
			responce.Error = ErrorCodes::BadRequest;
		}

		BHYWrapper::BHYFrame imuFrame = IMU.GetFrame();

		uint8_t sz;
		imuFrame.SerializeTo(responce.Data.data(), &sz);
		responce.Error = ErrorCodes::Success;

		return responce;
	}

	Responce DoReset(const Request &request,
			IMUFrameContainer &container) {
		assert(RequestMode::Deserialize(request.MetaInfo) == RequestMode::Reset);

		Responce responce;
		responce.PeripheryID = Periphery::Imu;
		responce.MetaInfo = RequestMode::Serialize(RequestMode::Reset);
		responce.Data.resize(1);

		if (request.Data.size() != 1) {
			responce.Error = ErrorCodes::BadRequest;
		}

		container.Reset();
		responce.Error = ErrorCodes::Success;

		return responce;
	}

public:
	Responce Handle(const Request &request,
				IMUFrameContainer &container, const BHYWrapper& IMU) {
		assert(request.PeripheryID == Periphery::Imu);

		switch (RequestMode::Deserialize(request.MetaInfo)) {
		case RequestMode::FrameBySeq:
			return GetFrameBySeq(request, container);
		case RequestMode::Info:
			return GetInfo(request, container);
		case RequestMode::LatestFrame:
			return GetLatestFrame(request, IMU);
		case RequestMode::Reset:
					return DoReset(request, container);
		default: assert(0 && "Unknown Mode");
		}
	}
};
