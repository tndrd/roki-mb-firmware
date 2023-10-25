#ifndef FRAME_QUEUE
#define FRAME_QUEUE

#include "FixedQueue.hpp"
#include "MbMessages.hpp"

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

#endif
