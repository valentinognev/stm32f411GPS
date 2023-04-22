#include "GNSScommRXTask.h"
#include "main.h"
#include "SWO.h"
#include <string.h>
#include <stdio.h>
#include "GNSSprocessTask.h"

#include "SERIALcommTXTask.h"
extern "C"
{
#include "gnss.h"
}

#define DMA_RX_BUFFER_SIZE          550
#define GNSS_COMMAND_REQUEST_SIZE        100

// Private function prototypes

void prvGNSS_RX_DMA(void);

// Private variables

char cDMA_RX_Buffer[GNSS_COMMAND_REQUEST_SIZE];

// Task handle
TaskHandle_t xGNSScommRXTaskHandle;

void vGNSScommRXTask(void *pvParameters) 
{
    while(1) 
    {
        if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) 
        {// waiting for notification from RX DMA TC interrupt
            prvGNSS_RX_DMA(); // transfer the data to GNSSprocess queue
        }
    }
}

void prvGNSS_RX_DMA(void)
{ // after DMA received the data and placed it to buffer, the function
  // transfers the buffer data to GNSSprocess queue
    size_t xLen = 0;
    size_t xBufDataLen = LL_DMA_GetDataLength(GNSS_DMA, GNSS_DMA_STREAM_RX);
    char *pcTokSstr = NULL;

    if(xBufDataLen == DMA_RX_BUFFER_SIZE)
        xLen = DMA_RX_BUFFER_SIZE;
    else
        xLen = DMA_RX_BUFFER_SIZE - xBufDataLen;
    cDMA_RX_Buffer[xLen] = '\0';  // Append a null terminator

    pcTokSstr = strtok(cDMA_RX_Buffer, "\r\n");
    while(pcTokSstr) 
    {
        osQueueGNSSprocessMessage(pcTokSstr);
        pcTokSstr = strtok(NULL, "\r\n");  // Get the other strings, if any
    }

    // Reset the flags in the DMA to prepare for the next transaction
    LL_DMA_ClearFlag_HT2(GNSS_DMA);
    LL_DMA_ClearFlag_TC2(GNSS_DMA);
    LL_DMA_ClearFlag_TE2(GNSS_DMA);

    LL_DMA_DisableStream(GNSS_DMA, GNSS_DMA_STREAM_RX);
    LL_DMA_SetDataLength(GNSS_DMA, GNSS_DMA_STREAM_RX, (uint32_t)DMA_RX_BUFFER_SIZE);
    LL_DMA_EnableStream(GNSS_DMA, GNSS_DMA_STREAM_RX);
}

// void prvGNSSDMAMessageTX(char *pcTxMessage)
// {
//     LL_DMA_SetDataLength(GNSS_DMA, GNSS_DMA_STREAM_TX, strlen(pcTxMessage));
//     LL_DMA_ConfigAddresses(GNSS_DMA, GNSS_DMA_STREAM_TX,
//                            (uint32_t)pcTxMessage, LL_USART_DMA_GetRegAddr(GNSS_USART),
//                            LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
//     LL_DMA_EnableStream(GNSS_DMA, GNSS_DMA_STREAM_TX);
// }

void DMA_GNSS_RX_ISR(void) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send a notification to FREERTOS for the task to take priority
    vTaskNotifyGiveFromISR(xGNSScommRXTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setupGNSScommRX()
{
    // RX initialization
	LL_DMA_SetDataLength(GNSS_DMA, GNSS_DMA_STREAM_RX,(uint32_t)DMA_RX_BUFFER_SIZE);
	LL_DMA_ConfigAddresses(GNSS_DMA, GNSS_DMA_STREAM_RX,
			LL_USART_DMA_GetRegAddr(GNSS_USART), (uint32_t)cDMA_RX_Buffer,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_USART_EnableDMAReq_RX(GNSS_USART);
    LL_DMA_EnableStream(GNSS_DMA, GNSS_DMA_STREAM_RX);

    LL_DMA_EnableIT_HT(GNSS_DMA, GNSS_DMA_STREAM_RX);
    LL_DMA_EnableIT_TC(GNSS_DMA, GNSS_DMA_STREAM_RX);
    // LL_USART_EnableIT_IDLE(GNSS_USART);

    xTaskCreate(vGNSScommRXTask, "GNSScommRX", STACK_SIZE_WORDS, NULL, tskIDLE_PRIORITY + 3, &xGNSScommRXTaskHandle);
    __NOP();
    //xUSARTQueue = xQueueCreate(USART_QUEUE_SIZE, sizeof(UARTMessage_t *));
}

