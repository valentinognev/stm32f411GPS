#ifndef SERIALTXTASK_H_
#define SERIALTXTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

// TODO: Use dynamic allocation for message strings
typedef char *SERIALMessage_t;
// extern TaskHandle_t xUSARTTaskHandle;
// extern QueueHandle_t xUSARTQueue;

void osQueueSERIALMessage(const char *format, ...);
void vSERIALcommTXTask(void *pvParameters);
void setupSERIALcommTX();

EXTERNC void DMA_SERIAL_TX_ISR(void);

#undef EXTERNC

#endif /* UARTTASK_H_ */
