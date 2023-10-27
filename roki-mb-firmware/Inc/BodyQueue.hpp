#ifndef BODY_QUEUE
#define BODY_QUEUE

#include "Common.hpp"
#include "FixedQueue.hpp"

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
		Counter = 0;
	}

	size_t GetSize() const {
		return Requests.GetSize();
	}

	size_t GetCapacity() const {
		return BodyQueueMaxSize;
	}

	void TimCallback() {
		Callback();
	}
};

#endif
