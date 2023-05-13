#include "SERIALcommRXTask.h"
#include "main.h"
#include "SWO.h"
#include <string.h>
#include <stdio.h>

#include "SERIALcommTXTask.h"

#define DMA_RX_BUFFER_SIZE          550
#define SERIAL_COMMAND_REQUEST_SIZE        100

// Private function prototypes

void prvSERIAL_RX_DMA(void);

// Private variables

char cDMA_SERIAL_RX_Buffer[SERIAL_COMMAND_REQUEST_SIZE];

// Task handle
TaskHandle_t xSERIALcommRXTaskHandle;

void vSERIALcommRXTask(void *pvParameters) 
{
    while(1) 
    {
        if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) 
        {// waiting for notification from RX DMA TC interrupt
            prvSERIAL_RX_DMA(); // transfer the data to SERIALprocess queue
        }
    }
}

void prvSERIAL_RX_DMA(void)
{ // after DMA received the data and placed it to buffer, the function
  // transfers the buffer data to SERIALprocess queue
    size_t xLen = 0;
    size_t xBufDataLen = LL_DMA_GetDataLength(SERIAL_DMA, SERIAL_DMA_STREAM_RX);
    char *pcTokSstr = NULL;

    if(xBufDataLen == DMA_RX_BUFFER_SIZE)
        xLen = DMA_RX_BUFFER_SIZE;
    else
        xLen = DMA_RX_BUFFER_SIZE - xBufDataLen;
    cDMA_SERIAL_RX_Buffer[xLen] = '\0';  // Append a null terminator

    pcTokSstr = strtok(cDMA_SERIAL_RX_Buffer, "\r\n");
    while(pcTokSstr) 
    {
        osQueueSERIALMessage(pcTokSstr);
        pcTokSstr = strtok(NULL, "\r\n");  // Get the other strings, if any
    }

    // Reset the flags in the DMA to prepare for the next transaction
    LL_DMA_ClearFlag_HT5(SERIAL_DMA);
    LL_DMA_ClearFlag_TC5(SERIAL_DMA);
    LL_DMA_ClearFlag_TE5(SERIAL_DMA);

    LL_DMA_DisableStream(SERIAL_DMA, SERIAL_DMA_STREAM_RX);
    LL_DMA_SetDataLength(SERIAL_DMA, SERIAL_DMA_STREAM_RX, (uint32_t)DMA_RX_BUFFER_SIZE);
    LL_DMA_EnableStream(SERIAL_DMA, SERIAL_DMA_STREAM_RX);
}

// void prvSERIALDMAMessageTX(char *pcTxMessage)
// {
//     LL_DMA_SetDataLength(SERIAL_DMA, SERIAL_DMA_STREAM_TX, strlen(pcTxMessage));
//     LL_DMA_ConfigAddresses(SERIAL_DMA, SERIAL_DMA_STREAM_TX,
//                            (uint32_t)pcTxMessage, LL_USART_DMA_GetRegAddr(SERIAL_USART),
//                            LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
//     LL_DMA_EnableStream(SERIAL_DMA, SERIAL_DMA_STREAM_TX);
// }

void DMA_SERIAL_RX_ISR(void) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send a notification to FREERTOS for the task to take priority
    vTaskNotifyGiveFromISR(xSERIALcommRXTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setupSERIALcommRX()
{
    // RX initialization
	LL_DMA_SetDataLength(SERIAL_DMA, SERIAL_DMA_STREAM_RX,(uint32_t)DMA_RX_BUFFER_SIZE);
	LL_DMA_ConfigAddresses(SERIAL_DMA, SERIAL_DMA_STREAM_RX,
			LL_USART_DMA_GetRegAddr(SERIAL_USART), (uint32_t)cDMA_SERIAL_RX_Buffer,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_USART_EnableDMAReq_RX(SERIAL_USART);
    LL_DMA_EnableStream(SERIAL_DMA, SERIAL_DMA_STREAM_RX);

    LL_DMA_EnableIT_HT(SERIAL_DMA, SERIAL_DMA_STREAM_RX);
    LL_DMA_EnableIT_TC(SERIAL_DMA, SERIAL_DMA_STREAM_RX);
    LL_USART_EnableIT_IDLE(SERIAL_USART);

    xTaskCreate(vSERIALcommRXTask, "SERIALcommRX", STACK_SIZE_WORDS, NULL, SERIALcommRXTaskPriority, &xSERIALcommRXTaskHandle);
    __NOP();
    //xUSARTQueue = xQueueCreate(USART_QUEUE_SIZE, sizeof(UARTMessage_t *));
}

