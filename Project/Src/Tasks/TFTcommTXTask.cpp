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
#define TFTcommTX_QUEUE_SIZE      5
// #define TFTMESSAGE_SIZE        300

enum TFTmessageType
{
    TFT_COMMAND,
    TFT_DATA,
    TFT_SELECT,
    TFT_UNSELECT,
    TFT_RESET
};

struct TFTMessage
{
    TFTmessageType type;
    uint8_t len;
    uint8_t *ucData = nullptr;//[TFTMESSAGE_SIZE];
 };

bool osQueueTFTMessage(const TFTMessage& TFTMessage_, const uint8_t *buf, const uint16_t len);


// DMA_HandleTypeDef dma;

 QueueHandle_t xTFTcommTXQueue;
 TaskHandle_t xTFTcommTXTaskHandle;

 /**
  * A task that prints strings via UART through DMA
  */
 void vTFTcommTXTask(void *pvParameters)
 {
    TFTMessage TFTMessage;

    while (1)
    {
        if (xQueueReceive(xTFTcommTXQueue, &TFTMessage, portMAX_DELAY))
        { // Receive a message from the queue
            
            LL_DMA_DisableStream(TFT_DMA, TFT_DMA_STREAM_TX);
            switch (TFTMessage.type)
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
                vTaskDelay(pdMS_TO_TICKS(5));
                LL_GPIO_SetOutputPin(TFT_RST_GPIO_Port, TFT_RST_Pin);
                break;
            default:
                break;
            }
            if (TFTMessage.len > 0)
            {
                LL_DMA_SetDataLength(TFT_DMA, TFT_DMA_STREAM_TX, TFTMessage.len); // Set amount of copied bits for DMA
                LL_DMA_ConfigAddresses(TFT_DMA, TFT_DMA_STREAM_TX, (uint32_t)TFTMessage.ucData,
                           LL_SPI_DMA_GetRegAddr(TFT_SPI), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
                LL_SPI_Enable(TFT_SPI);
                LL_DMA_EnableStream(TFT_DMA, TFT_DMA_STREAM_TX);
                xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
                LL_DMA_DisableStream(TFT_DMA, TFT_DMA_STREAM_TX);
                LL_SPI_Disable(TFT_SPI);
                vPortFree(TFTMessage.ucData);
                TFTMessage.ucData=nullptr;
            }
        }
    }
}

void setupTFTcommTX()
{
    LL_GPIO_SetOutputPin(TFT_CS_GPIO_Port, TFT_CS_Pin);
    LL_GPIO_SetOutputPin(TFT_RST_GPIO_Port, TFT_RST_Pin);
    LL_GPIO_SetOutputPin(TFT_BLK_GPIO_Port,TFT_BLK_Pin);

    LL_SPI_EnableDMAReq_TX(TFT_SPI);
    LL_DMA_EnableIT_TC(TFT_DMA, TFT_DMA_STREAM_TX);
    LL_DMA_EnableIT_TE(TFT_DMA, TFT_DMA_STREAM_TX);

    xTFTcommTXQueue = xQueueCreate(TFTcommTX_QUEUE_SIZE, sizeof(TFTMessage));
    xTaskCreate(vTFTcommTXTask, "TFT_TX", STACK_SIZE_WORDS, NULL, TFTcommTXTaskPriority, &xTFTcommTXTaskHandle);
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
    TFTMessage TFTMessage = {.type = TFT_COMMAND, .len = 1, .ucData = nullptr };
    osQueueTFTMessage(TFTMessage, &cmd, 1);
}

void TFT_WriteData(uint8_t *buff, size_t buff_size)
{
    TFTMessage TFTMessage = {.type = TFT_DATA, .len = buff_size, .ucData = nullptr};
    osQueueTFTMessage(TFTMessage, buff, buff_size);
}

void TFT_Select()
{
    TFTMessage TFTMessage = {.type = TFT_SELECT, .len = 0, .ucData = nullptr};
    osQueueTFTMessage(TFTMessage, nullptr, 0);
}

void TFT_Unselect()
{
    TFTMessage TFTMessage = {.type = TFT_UNSELECT, .len = 0, .ucData = nullptr};
    xQueueSend(xTFTcommTXQueue, &TFTMessage, portMAX_DELAY);
    osQueueTFTMessage(TFTMessage, nullptr, 0);
}

void TFT_Reset()
{
    TFTMessage TFTMessage = {.type = TFT_RESET, .len = 0, .ucData = nullptr};
    osQueueTFTMessage(TFTMessage, nullptr, 0);
}

bool osQueueTFTMessage(const TFTMessage& TFTMessage_, const uint8_t *buf, const uint16_t len)
{
    // TODO: Less copying around bits

    // configASSERT(strlen(message) < 127);
    TFTMessage TFTMessage = TFTMessage_;
    if (len==0)      
    {
        xQueueSend(xTFTcommTXQueue, (void *)(&TFTMessage), (TickType_t)0);
        return true;
    }
    
    TFTMessage.ucData = (uint8_t *)pvPortMalloc(len);
     if (TFTMessage.ucData == nullptr)
    {
        if (len==1)
        {
            USART_PrintString("ERROR! Not enough memory to store UART string\r\n");
            return false;
        }   
        osQueueTFTMessage(TFTMessage_, buf, len/2);
        osQueueTFTMessage(TFTMessage_, buf+len/2, len-len/2);
    }
    else
    {
        memcpy(TFTMessage.ucData, buf, len);
        TFTMessage.len = len;

        // TODO: Show a warning if the queue is full (e.g. replace the last
        // message in the queue)
        if (xQueueSend(xTFTcommTXQueue, (void *)(&TFTMessage), (TickType_t)0) == pdFAIL)
        {
            // Make sure to deallocate the failed message
            vPortFree(TFTMessage.ucData);
        }
    }
}
