#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "portable.h"
#include "SWO.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "SERIALcommTXTask.h"

// #define LL_DMA_IsActive
#define SERIAL_TX_QUEUE_SIZE 85

static QueueHandle_t xSERIALcommTXQueue;
TaskHandle_t xSERIALcommTXTaskHandle;

/**
 * A task that prints strings via UART through DMA
 */
void vSERIALcommTXTask(void *pvParameters)
{
    SERIALMessage_t message = NULL;

    while (1)
    {
        if (xQueueReceive(xSERIALcommTXQueue, &message, portMAX_DELAY))
        {                                                           // Receive a message from the queue
            LL_USART_DisableDMAReq_TX(SERIAL_USART);                // Disable DMA in the USART registers
            LL_DMA_DisableStream(SERIAL_DMA, SERIAL_DMA_STREAM_TX); // Disable the DMA transaction

            LL_DMA_SetDataLength(SERIAL_DMA, SERIAL_DMA_STREAM_TX, strlen(message)); // Set amount of copied bits for DMA
            LL_DMA_ConfigAddresses(SERIAL_DMA, SERIAL_DMA_STREAM_TX, (uint32_t)message,
                                   LL_USART_DMA_GetRegAddr(SERIAL_USART), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register

            LL_USART_EnableDMAReq_TX(SERIAL_USART);                // Enable DMA in the USART registers
            LL_DMA_EnableStream(SERIAL_DMA, SERIAL_DMA_STREAM_TX); // Enable the DMA transaction

            xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
            vPortFree(message); // Free up the memory held by the message string
            message = NULL;
        }
    }
}

/**
 * Queue a UART message so that it can be printed later
 */
void osQueueSERIALMessage(const char *format, ...)
{
    // TODO: Less copying around bits

    va_list arg;
    char buffer[128];

    va_start(arg, format);
    vsnprintf(buffer, 128, format, arg);
    va_end(arg);

    // configASSERT(strlen(message) < 127);
    SERIALMessage_t pcSERIALMessage = (SERIALMessage_t)pvPortMalloc(strlen(buffer) + 1);

    if (pcSERIALMessage == NULL)
    {
        USART_PrintString("ERROR! Not enough memory to store UART string\r\n");
    }
    else
    {
        strcpy(pcSERIALMessage, buffer);

        // TODO: Show a warning if the queue is full (e.g. replace the last
        // message in the queue)
        if (xQueueSend(xSERIALcommTXQueue, (void *)(&pcSERIALMessage), (TickType_t)0) == pdFAIL)
        {
            // Make sure to deallocate the failed message
            vPortFree(pcSERIALMessage);
        }
    }
}

void DMA_SERIAL_TX_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the UART1 TX task that transaction has ended
    vTaskNotifyGiveFromISR(xSERIALcommTXTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setupSERIALcommTX()
{
    LL_DMA_EnableIT_TC(SERIAL_DMA, SERIAL_DMA_STREAM_TX);
    LL_DMA_EnableIT_TE(SERIAL_DMA, SERIAL_DMA_STREAM_TX);
    xSERIALcommTXQueue = xQueueCreate(SERIAL_TX_QUEUE_SIZE, sizeof(SERIALMessage_t *));
    
    xTaskCreate(vSERIALcommTXTask, "SERIALcommTXTask", STACK_SIZE_WORDS, NULL, tskIDLE_PRIORITY + 3, &xSERIALcommTXTaskHandle);
}