#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "portable.h"
#include "SWO.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "USARTTask.h"

#define LL_SERIAL_DMA_STREAM_TX __LL_DMA_GET_STREAM(SERIAL_DMA_STREAM_TX)

// #define LL_DMA_IsActive
#define UART_QUEUE_SIZE		85

DMA_HandleTypeDef dma;

QueueHandle_t xUSARTQueue;
TaskHandle_t xUSARTTaskHandle;

/**
 * A task that prints strings via UART through DMA
 */
void vUSARTTask(void *pvParameters) 
{
	UARTMessage_t message = NULL;

	while (1) {
		if (xQueueReceive(xUSARTQueue, &message, portMAX_DELAY)) 
        { // Receive a message from the queue
			LL_USART_EnableDMAReq_TX(SERIAL_USART); // Enable DMA in the USART registers
			LL_DMA_SetDataLength(SERIAL_DMA, LL_SERIAL_DMA_STREAM_TX, strlen(message)); // Set amount of copied bits for DMA
			LL_DMA_ConfigAddresses(SERIAL_DMA, LL_SERIAL_DMA_STREAM_TX, (uint32_t)message,
					LL_USART_DMA_GetRegAddr(SERIAL_USART), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
			LL_DMA_EnableStream(SERIAL_DMA, LL_SERIAL_DMA_STREAM_TX); // Enable the DMA transaction

			xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
			vPortFree(message); // Free up the memory held by the message string
			message = NULL;
		}
	}
}

/**
 * Queue a UART message so that it can be printed later
 */
void osQueueUARTMessage(const char * format, ...) {
	// TODO: Less copying around bits

	va_list arg;
	char buffer[128];

	va_start(arg, format);
	vsnprintf(buffer, 128, format, arg);
	va_end(arg);

	//configASSERT(strlen(message) < 127);
	UARTMessage_t pcUARTMessage = (UARTMessage_t)pvPortMalloc(strlen(buffer) + 1);

	if (pcUARTMessage == NULL) {
		USART_PrintString("ERROR! Not enough memory to store UART string\r\n");
	} else {
		strcpy(pcUARTMessage, buffer);

		// TODO: Show a warning if the queue is full (e.g. replace the last
		// message in the queue)
		if (xQueueSend(xUARTQueue, (void*) (&pcUARTMessage), (TickType_t) 0) == pdFAIL) {
			// Make sure to deallocate the failed message
			vPortFree(pcUARTMessage);
		}
	}
}


void DMA_USART_TX_ISR(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Notify the UART1 TX task that transaction has ended
	vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
