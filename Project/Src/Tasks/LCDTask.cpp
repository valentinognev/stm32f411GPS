#include "main.h"

#include "SWO.h"

#include "LCDTask.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

extern "C"
{
#include "FreeRTOS.h"
#include "queue.h"
#include "portable.h"   
#include "Lcd_Driver.h" 
}

#define LL_TFT_DMA_STREAM_TX __LL_DMA_GET_STREAM(TFT_DMA_STREAM_TX)

// #define LL_DMA_IsActive
#define LCD_QUEUE_SIZE		5
#define MESSAGE_SIZE        20

 struct LCDMessage
 {
	uint8_t line;
	char ucData[ MESSAGE_SIZE ];
 };
// DMA_HandleTypeDef dma;

QueueHandle_t xSPIQueue;
TaskHandle_t xSPITaskHandle;


/**
 * A task that prints strings via UART through DMA
 */
void vLCDTransmitTask(void *pvParameters)
{
	LCDMessage *lcdMessage = NULL;
    Lcd_Init();
    int counter=0;
	while (1) 
    {
        counter++;
		if (xQueueReceive(xSPIQueue, &lcdMessage, portMAX_DELAY)) 
        { // Receive a message from the queue
			
            TFT_PrintString(lcdMessage->line, lcdMessage->ucData);

			xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
			vPortFree(lcdMessage); // Free up the memory held by the message string
			lcdMessage = NULL;
		}
	}
}

void setupLCD()
{
    xSPIQueue = xQueueCreate(LCD_QUEUE_SIZE, sizeof(LCDMessage));
}

// void DMA_SPI_TX_ISR(void)
// {
// 	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// 	// Notify the UART1 TX task that transaction has ended
// 	vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
// 	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }
