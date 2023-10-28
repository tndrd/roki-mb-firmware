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
		uint8_t Pause;
	};

private:
	FixedQueue<Request, BodyQueueMaxSize> Requests;

	/* Concurrent data */
	size_t Period;
	size_t Counter = 0;
	size_t Pause = 0;

	/* Condition variable */
	bool Ready = true;

private:
	void Callback() {
		Counter = (Counter + 1) % Period;
		if (Counter != 0) return;

		if (Pause != 0) {
			Pause--;
			return;
		}

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

		__disable_irq();
		Pause = request.Pause;
		__enable_irq();

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

		__disable_irq();
		Period = periodMs;
		Counter = 0;
		Pause = 0;
		__enable_irq();
	}

	size_t GetSize() const {
		return Requests.GetSize();
	}

	size_t GetCapacity() const {
		return BodyQueueMaxSize;
	}

	size_t Clear() {
		Requests.Clear();

		__disable_irq();
		Counter = 0;
		Pause = 0;
		__enable_irq();
	}

	void TimCallback() {
		Callback();
	}
};

#endif
