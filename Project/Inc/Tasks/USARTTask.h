#ifndef USARTTASK_H_
#define USARTTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

// TODO: Use dynamic allocation for message strings
typedef char *UARTMessage_t;
// extern TaskHandle_t xUSARTTaskHandle;
// extern QueueHandle_t xUSARTQueue;

void osQueueUSARTMessage(const char *format, ...);
void vUSARTTask(void *pvParameters);
void vSetupUART();

EXTERNC void DMA_USART_TX_ISR(void);

#undef EXTERNC

#endif /* UARTTASK_H_ */
