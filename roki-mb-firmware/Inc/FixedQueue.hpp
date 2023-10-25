#ifndef FIXED_QUEUE
#define FIXED_QUEUE

#include <assert.h>
#include <array>
#include <cstdlib>

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

#endif
