#ifndef MBMAIN
#define MBMAIN

#include "MotherboardConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotherboardInit(struct MotherboardConfig config);
void MotherboardRun();

void HeadRxCallback();
void HeadTxCallback();
void BQueueTimCallback();
void IMUTimCallback();
void SFCompCallback();

#ifdef __cplusplus
}
#endif

#endif
