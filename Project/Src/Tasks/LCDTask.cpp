#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "portable.h"
#include "SWO.h"
#include "Lcd_Driver.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>


#define LL_TFT_DMA_STREAM_TX __LL_DMA_GET_STREAM(TFT_DMA_STREAM_TX)

// #define LL_DMA_IsActive
#define UART_QUEUE_SIZE		85

// DMA_HandleTypeDef dma;

QueueHandle_t xSPIQueue;
TaskHandle_t xSPITaskHandle;

/**
 * A task that prints strings via UART through DMA
 */
void vLCDTask(void *pvParameters) 
{
	char *message = NULL;
    Lcd_Init();
    int counter=0;
	while (1) {
        counter++;
		if (xQueueReceive(xSPIQueue, &message, portMAX_DELAY)) 
        { // Receive a message from the queue
			
            TFT_PrintString(counter%7, message);

			xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
			vPortFree(message); // Free up the memory held by the message string
			message = NULL;
		}
	}
}



// void DMA_SPI_TX_ISR(void)
// {
// 	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// 	// Notify the UART1 TX task that transaction has ended
// 	vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
// 	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }
