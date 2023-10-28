#ifndef REQUEST_HANDLER
#define REQUEST_HANDLER

#include "MotherboardContext.hpp"
#include "MbService.hpp"

namespace Msgs = Roki::Messages;

struct RequestHandler {
private:
	using Request = HeadIO::Request;
	using Responce = HeadIO::Responce;
	using Service = Roki::MbService;

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
		return responce;
	}

	template<typename Procedure> Errors::Type Handler(MotherboardContext &ctx,
			const typename Procedure::RequestType&,
			typename Procedure::ResponceType&);

	template<typename Procedure>
	Responce GenericHandler(MotherboardContext &ctx, const Request &request) {
		typename Procedure::RequestType reqMsg;
		typename Procedure::ResponceType rspMsg;

		reqMsg = Procedure::RequestType::Deserialize(request.Data.data());
		if (reqMsg.GetPackedSize() != request.Size)
			return CreateError(Errors::BadRequest);

		Errors::Type error = Handler<Procedure>(ctx, reqMsg, rspMsg);

		if (error != Errors::Success)
			return CreateError(error);

		Responce responce;
		responce.Error = Errors::Success;
		responce.Size = rspMsg.GetPackedSize();

		rspMsg.Serialize(responce.Data.data());

		return responce;
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
		case PID::ResetBodyQueue:
			return GenericHandler<Proc::ResetBodyQueue>(ctx, request);

		default:
			return CreateError(Errors::UnknownProcedure);
		}
	}
};

#define RPC(procName) typename RequestHandler::Proc::procName
#define HANDLER(procName) template<> RequestHandler::Errors::Type RequestHandler::Handler<RPC(procName)>(MotherboardContext& ctx, const RPC(procName)::RequestType& request, RPC(procName)::ResponceType& responce)

HANDLER(GetIMUFrame) {
	auto &fQueue = ctx.FQueues.IMUFrames;
	int16_t seq = request.Seq;

	if (!fQueue.HasSeq(seq))
		return Errors::FrameUnavailable;

	responce = fQueue.GetSeq(seq);

	return Errors::Success;
}

HANDLER(GetBodyFrame) {
	auto &fQueue = ctx.FQueues.BodyPos;
	int16_t seq = request.Seq;

	if (!fQueue.HasSeq(seq))
		return Errors::FrameUnavailable;

	auto resp = fQueue.GetSeq(seq);
	auto error = ConvertBodyError(resp.Status);

	if (error == Errors::Success) {
		memcpy(Buffer.data(), resp.Data.data(), resp.Size);
		responce.Data = Buffer.data();
		responce.ResponceSize = resp.Size;
	}

	return error;
}

HANDLER(GetIMUContainerInfo) {
	responce = ctx.FQueues.IMUFrames.GetInfo();
	return Errors::Success;
}

HANDLER(GetBodyContainerInfo) {
	responce = ctx.FQueues.BodyPos.GetInfo();
	return Errors::Success;
}

HANDLER(GetIMULatest) {
	responce = ctx.IMU.GetFrame();
	return Errors::Success;
}

HANDLER(ResetStrobeContainers) {
	ctx.FQueues.Clear();
	return Errors::Success;
}

HANDLER(SetIMUStrobeOffset) {
	ctx.SObservers.SetOffset(0, request.Value);
	return Errors::Success;
}

HANDLER(SetBodyStrobeOffset) {
	ctx.SObservers.SetOffset(1, request.Value);
	return Errors::Success;
}

HANDLER(GetStrobeWidth) {
	responce.Value = uint8_t(ctx.SFilter.GetStrobeDuration());
	return Errors::Success;
}

HANDLER(ConfigureStrobeFilter) {
	ctx.SFilter.Configure(request.TargetDuration, request.DurationThreshold);
	return Errors::Success;
}

HANDLER(GetVersion) {
	responce.Major = ctx.Version.Major;
	responce.Minor = ctx.Version.Minor;
	return Errors::Success;
}

HANDLER(BodySendForward) {
	auto txBuf = request.Data;
	auto txSize = request.RequestSize;
	auto rxBuf = Buffer.data();
	auto rxSize = request.ResponceSize;

	auto status = ctx.Body.Synchronize(txBuf, txSize, rxBuf, rxSize);
	auto error = ConvertBodyError(status);

	if (error == Errors::Success) {
		responce.Data = Buffer.data();
		responce.ResponceSize = request.ResponceSize;
	}

	return error;
}

HANDLER(BodySendQueue) {
	if (ctx.BQueue.IsFull())
		return Errors::BodyQueueFull;

	BodyQueue::Request newReq;

	memcpy(newReq.Data.data(), request.Request.Data, request.Request.RequestSize);
	newReq.TxSize = request.Request.RequestSize;
	newReq.RxSize = request.Request.ResponceSize;
	newReq.Pause = request.Pause;

	ctx.BQueue.AddRequest(newReq);

	return Errors::Success;
}

HANDLER(GetBodyQueueInfo) {
	responce.Size = ctx.BQueue.GetSize();
	responce.Capacity= ctx.BQueue.GetCapacity(); // Placeholder
	return Errors::Success;
}

HANDLER(SetBodyQueuePeriod) {
	if (request.Ms == 0)
		return Errors::BadBodyQueuePeriod;
	ctx.BQueue.SetPeriod(request.Ms);
	return Errors::Success;
}

HANDLER(ResetBodyQueue) {
	ctx.BQueue.Clear();
	return Errors::Success;
}

#undef RPC
#undef HANDLER

#endif
