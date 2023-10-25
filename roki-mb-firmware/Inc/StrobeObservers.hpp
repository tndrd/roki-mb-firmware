#ifndef STROBE_OBSERVERS
#define STROBE_OBSERVERS

#include <vector>
#include <queue>
#include "Common.hpp"

// Saves desired strobes and calls callbacks when needed
struct StrobeObserver {
public:
	using CallbackT = void (*)(MotherboardContext&);
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

	void TryProcess(MotherboardContext &mbCtx, size_t currentSeq) {
		if (Strobes.empty())
			return;
		if (currentSeq < Strobes.front())
			return;

		Callback(mbCtx);

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

	void TryProcess(MotherboardContext &mbCtx, size_t currentSeq) {
		for (auto &observer : Observers)
			observer.TryProcess(mbCtx, currentSeq);
	}
};

#endif
