#include "MotherboardMain.h"
#include "Motherboard.hpp"

static uint8_t MbCell[sizeof(Motherboard)];
static Motherboard* MbPtr = reinterpret_cast<Motherboard*>(MbCell);
static bool MbInit = false;

extern "C" {

void MotherboardInit(MotherboardConfig config) {
	new (MbPtr) Motherboard {config};
	MbPtr->Init();
	MbInit = true;
}

void MotherboardRun() {
	assert(MbInit);
	MbPtr->Run();
}

void HeadRxCallback() {
	assert(MbInit);
	MbPtr->HeadRxCallback();
}

void HeadTxCallback() {
	assert(MbInit);
	MbPtr->HeadTxCallback();
}

void BQueueTimCallback() {
	assert(MbInit);
	MbPtr->BQueueTimCallback();
}

void IMUTimCallback() {
	assert(MbInit);
	MbPtr->IMUTimCallback();
}

void SFCompCallback(){
	assert(MbInit);
	MbPtr->SFCompCallback();
}

}
