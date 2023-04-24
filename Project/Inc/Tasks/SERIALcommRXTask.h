#ifndef SERIALRXTASK_H_
#define SERIALRXTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

// TODO: Use dynamic allocation for message strings

extern TaskHandle_t xGPSMessageRXTaskHandle;
extern QueueHandle_t xGPSQueue;

void setupSERIALcommRX();
void vSERIALcommRXTask(void *pvParameters);


EXTERNC void DMA_SERIAL_RX_ISR(void);


#endif /* GPSTASK_H_ */