#include "main.h"

#include "SWO.h"

#include "TFTcommTXTask.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

extern "C"
{
#include "FreeRTOS.h"
#include "queue.h"
#include "portable.h"   
#include "TFTcommTXTask.h" 
}

// #define LL_DMA_IsActive
#define TFTcommTX_QUEUE_SIZE      20
#define TFTMESSAGE_SIZE        100

enum TFTmessageType
{
    TFT_COMMAND,
    TFT_DATA,
    TFT_SELECT,
    TFT_UNSELECT,
    TFT_RESET
};

struct LCDMessage
{
    TFTmessageType type;
    uint8_t len;
    char ucData[TFTMESSAGE_SIZE];
 };
// DMA_HandleTypeDef dma;

 QueueHandle_t xTFTcommTXQueue;
 TaskHandle_t xTFTcommTXTaskHandle;

 /**
  * A task that prints strings via UART through DMA
  */
 void vTFTcommTXTask(void *pvParameters)
 {
    LCDMessage lcdMessage;

    LL_DMA_ConfigAddresses(TFT_DMA, TFT_DMA_STREAM_TX, (uint32_t)lcdMessage.ucData,
                           LL_SPI_DMA_GetRegAddr(TFT_SPI), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
    while (1)
    {
        if (xQueueReceive(xTFTcommTXQueue, &lcdMessage, portMAX_DELAY))
        { // Receive a message from the queue
            
            LL_DMA_DisableStream(TFT_DMA, TFT_DMA_STREAM_TX);
            switch (lcdMessage.type)
            {
            case TFT_COMMAND:
                LL_GPIO_ResetOutputPin(TFT_RS_GPIO_Port, TFT_RS_Pin);
                break;
            case TFT_DATA:
                LL_GPIO_SetOutputPin(TFT_RS_GPIO_Port, TFT_RS_Pin);
                break; 
            case TFT_SELECT:
                LL_GPIO_ResetOutputPin(TFT_CS_GPIO_Port, TFT_CS_Pin);
                break;
            case TFT_UNSELECT:
                LL_GPIO_SetOutputPin(TFT_CS_GPIO_Port, TFT_CS_Pin);
                break;
            case TFT_RESET:
                LL_GPIO_ResetOutputPin(TFT_RST_GPIO_Port, TFT_RST_Pin);
                vTaskDelay(portTICK_PERIOD_MS * 5);
                LL_GPIO_SetOutputPin(TFT_RST_GPIO_Port, TFT_RST_Pin);
                break;
            default:
                break;
            }
            if (lcdMessage.len > 0)
            {
                LL_DMA_SetDataLength(TFT_DMA, TFT_DMA_STREAM_TX, lcdMessage.len); // Set amount of copied bits for DMA
                LL_SPI_Enable(TFT_SPI);
                LL_DMA_EnableStream(TFT_DMA, TFT_DMA_STREAM_TX);
                xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
                LL_DMA_DisableStream(TFT_DMA, TFT_DMA_STREAM_TX);
                LL_SPI_Disable(TFT_SPI);
            }
        }
    }
}

void setupTFTcommTX()
{
    LL_GPIO_SetOutputPin(TFT_CS_GPIO_Port, TFT_CS_Pin);
    LL_GPIO_SetOutputPin(TFT_RST_GPIO_Port, TFT_RST_Pin);
    LL_GPIO_SetOutputPin(LCD_BLK_GPIO_Port,LCD_BLK_Pin);

    LL_SPI_EnableDMAReq_TX(TFT_SPI);
    LL_DMA_EnableIT_TC(TFT_DMA, TFT_DMA_STREAM_TX);
    LL_DMA_EnableIT_TE(TFT_DMA, TFT_DMA_STREAM_TX);

    xTFTcommTXQueue = xQueueCreate(TFTcommTX_QUEUE_SIZE, sizeof(LCDMessage));
    xTaskCreate(vTFTcommTXTask, "TFT_TX", STACK_SIZE_WORDS, NULL, tskIDLE_PRIORITY + 3, &xTFTcommTXTaskHandle);
}

/**
 * @brief  Function called from DMA2 IRQ Handler when Tx transfer is completed
 * @param  None
 * @retval None
 */
void TFT_DMA_TransmitComplete_Callback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the UART1 TX task that transaction has ended
    vTaskNotifyGiveFromISR(xTFTcommTXTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  Function called in case of error detected in SPI IT Handler
 * @param  None
 * @retval None
 */
void TFT_DMA_TransferError_Callback(void)
{
    /* Disable DMA2 Tx Channel */
    LL_DMA_DisableStream(TFT_DMA, TFT_DMA_STREAM_TX);
    /* Set LED2 to Blinking mode to indicate error occurs */
}

void TFT_WriteCommand(uint8_t cmd)
{
    LCDMessage lcdMessage = {.type = TFT_COMMAND, .len = 1, .ucData = {cmd}};
    xQueueSend(xTFTcommTXQueue, &lcdMessage, portMAX_DELAY);
}

void TFT_WriteData(uint8_t *buff, size_t buff_size)
{
    LCDMessage lcdMessage = {.type = TFT_DATA, .len = buff_size};
    memcpy(lcdMessage.ucData, buff, buff_size);
    xQueueSend(xTFTcommTXQueue, &lcdMessage, portMAX_DELAY);
}

void TFT_Select()
{
    LCDMessage lcdMessage = {.type = TFT_SELECT, .len = 0};
    xQueueSend(xTFTcommTXQueue, &lcdMessage, portMAX_DELAY);
}

void TFT_Unselect()
{
    LCDMessage lcdMessage = {.type = TFT_UNSELECT, .len = 0};
    xQueueSend(xTFTcommTXQueue, &lcdMessage, portMAX_DELAY);
}

void TFT_Reset()
{
    LCDMessage lcdMessage = {.type = TFT_RESET, .len = 0};
    xQueueSend(xTFTcommTXQueue, &lcdMessage, portMAX_DELAY);
}