#ifndef USARTTASK_H_
#define USARTTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// TODO: Use dynamic allocation for message strings
typedef char *UARTMessage_t;
// extern TaskHandle_t xUSARTTaskHandle;
// extern QueueHandle_t xUSARTQueue;

void osQueueUSARTMessage(const char *format, ...);
void vUSARTTask(void *pvParameters);

void DMA_UART_TX_ISR(void);

#endif /* UARTTASK_H_ */
