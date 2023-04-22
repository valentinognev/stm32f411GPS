#include "GNSScommTXTask.h"
#include "main.h"
#include "SWO.h"
#include <string.h>
#include <stdio.h>

#include "SERIALcommTXTask.h"

#define GNSS_QUEUE_SIZE 85
#define DMA_TX_BUFFER_SIZE 100

static QueueHandle_t xGNSScommTXQueue;
TaskHandle_t xGNSScommTXTaskHandle;



void vGNSScommTXTask(void *pvParameters)
{
    GNSScommMessage_t message = NULL;

    while (1)
    {
        if (xQueueReceive(xGNSScommTXQueue, &message, portMAX_DELAY))
        {                                                       // Receive a message from the queue
            LL_USART_DisableDMAReq_TX(GNSS_USART);              // Disable DMA in the USART registers
            LL_DMA_DisableStream(GNSS_DMA, GNSS_DMA_STREAM_TX); // Disable the DMA transaction

            LL_DMA_SetDataLength(GNSS_DMA, GNSS_DMA_STREAM_TX, strlen(message)); // Set amount of copied bits for DMA
            LL_DMA_ConfigAddresses(GNSS_DMA, GNSS_DMA_STREAM_TX, (uint32_t)message,
                                   LL_USART_DMA_GetRegAddr(GNSS_USART), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register

            LL_USART_EnableDMAReq_TX(GNSS_USART);              // Enable DMA in the USART registers
            LL_DMA_EnableStream(GNSS_DMA, GNSS_DMA_STREAM_TX); // Enable the DMA transaction

            xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
            vPortFree(message); // Free up the memory held by the message string
            message = NULL;
        }
    }
}

void setupGNSScommTX()
{
    // TX initialization
    xGNSScommTXQueue = xQueueCreate(GNSS_QUEUE_SIZE, sizeof(GNSScommMessage_t));
    LL_USART_EnableDMAReq_TX(GNSS_USART);

    LL_DMA_EnableIT_TC(GNSS_DMA, GNSS_DMA_STREAM_TX);
    LL_DMA_EnableIT_TE(GNSS_DMA, GNSS_DMA_STREAM_TX);

    LL_DMA_DisableStream(GNSS_DMA, GNSS_DMA_STREAM_TX);

    xTaskCreate(vGNSScommTXTask, "GNSScommTX", STACK_SIZE_WORDS, NULL, tskIDLE_PRIORITY + 3, &xGNSScommTXTaskHandle);
 }

void DMA_GNSS_TX_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the UART1 TX task that transaction has ended
    vTaskNotifyGiveFromISR(xGNSScommTXTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * Queue a GNSS message so that it can be printed later
 */
void osQueueGNSStransmitMessage(const char *gnssmess)
{

    GNSScommMessage_t pcGNSSMessage = (GNSScommMessage_t)pvPortMalloc(strlen(gnssmess));

    if (pcGNSSMessage == NULL)
    {
        osQueueSERIALMessage("ERROR! Not enough memory to store GNSS string\r\n");
        return;
    }

    strcpy(pcGNSSMessage, gnssmess);

    // TODO: Show a warning if the queue is full (e.g. replace the last
    // message in the queue)
    if (xQueueSend(xGNSScommTXQueue, (void *)(&pcGNSSMessage), (TickType_t)0) == pdFAIL)
    {
        // Make sure to deallocate the failed message
        vPortFree(pcGNSSMessage);
    }
   
}