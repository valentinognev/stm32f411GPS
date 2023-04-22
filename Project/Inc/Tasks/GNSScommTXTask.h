#ifndef GNSSCOMMTXTASK_H_
#define GNSSCOMMTXTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

void setupGNSScommTX();
void vGNSScommTXTask(void *pvParameters);

EXTERNC void DMA_GNSS_TX_ISR(void);

#undef EXTERNC
#endif /* GNSSCOMMTXTASK_H_ */