#include "GNSSprocessTask.h"
#include "SERIALcommTXTask.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern "C"
{
    #include "gnss.h"
}

#include <string.h>

#define GNSS_INIT_BUFFER_SIZE 100
#define GNSS_PROCESS_QUEUE_SIZE 30

char gnssBuffer[GNSS_INIT_BUFFER_SIZE];

QueueHandle_t xGNSSprocessQueue;
TaskHandle_t xGNSSprocessTaskHandle;
SemaphoreHandle_t xGNSSprocessSemaphore;

gnss_simple_data_t xGNSSData;

bool xGNSSprocessReadFromQueue(char* message);

void vGNSSprocessTask(void *pvParameters)
{

    vTaskDelay(portTICK_PERIOD_MS * 500);
    vTaskPrioritySet(xGNSSprocessTaskHandle, tskIDLE_PRIORITY + 3);

    gnss_simple_data_t gnssData = {
        .hour = 0,
        .minute = 0,
        .second = 0,
        .year = 0,
        .month = 0,
        .day = 0,
        .fix = 0,
        .satellites = 0,
        .latitude = 0,
        .longitude = 0,
        .altitude = 0,
        .speed = 0,
        .course = 0,
    };
    
    const size_t gnssMessageBufferSizeBytes = 100;

    gnss_run_mode_e mode = GNSS_RUN_HOT;
    uint16_t fixFreq = 100;
    uint32_t timeoutS = 1;

    LL_USART_DisableDMAReq_RX(GNSS_USART);
    LL_USART_DisableIT_IDLE(GNSS_USART);    
    LL_USART_EnableIT_RXNE(GNSS_USART);

    // LL_GPIO_ResetOutputPin(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK, ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
    // LL_mDelay(30); // 10 ms min according to doc
    // LL_GPIO_SetOutputPin(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK, ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
    // char mes[GNSS_INIT_BUFFER_SIZE];
    // if (xQueueReceive(xGNSSprocessQueue, mes, portMAX_DELAY))
    // {
    //     gnss_processString(mes);
    // }

    gnss_setup();
    gnss_ret_e res = gnss_start(mode, fixFreq, timeoutS);

    GNSSprocessMessage_t message;
    while (1)
    {
        if (xQueueReceive(xGNSSprocessQueue, &message, portMAX_DELAY))
        {
            gnss_processString(message);

            gnss_getData(&gnssData);

            gnss_printState();
        }
    }
}

/**
 * Queue a GNSS message so that it can be printed later
 */
void osQueueGNSSprocessMessageFromISR(const char *gnssmess)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const size_t len = strlen(gnssmess) + 1;
    if (xQueueSendFromISR(xGNSSprocessQueue, gnssmess, &xHigherPriorityTaskWoken) == pdFAIL)
    {
        // Make sure to deallocate the failed message
        __NOP();
    }
    if (uxQueueMessagesWaitingFromISR(xGNSSprocessQueue) > 2)
    {
        // Make sure to deallocate the failed message
        __NOP();
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void setupGNSSprocess()
{
    xGNSSprocessSemaphore = xSemaphoreCreateBinary();
    xGNSSprocessQueue = xQueueCreate(GNSS_PROCESS_QUEUE_SIZE, GNSS_INIT_BUFFER_SIZE);
    xTaskCreate(vGNSSprocessTask, "GNSSprocess", STACK_SIZE_WORDS, NULL, tskIDLE_PRIORITY + 2, NULL);
}

void GNSSreceiveData()
{
    static uint8_t flag = 0, i = 0;
    static char buff[GNSS_INIT_BUFFER_SIZE];
    // a data byte is received from the user
    char data_byte = LL_USART_ReceiveData8(USART1);
    if (data_byte == '$')
    {
        flag = 1;
    }
    else if (data_byte == '\r')
    {
        flag = 0;
        if (strlen(buff) > 3)
        {
            osQueueGNSSprocessMessageFromISR(buff);
        }
    }
    // extract the NMEA sentence
    if ((i < NMEA_GPRMC_SENTENCE_SIZE - 1) & (flag == 1))
    {
        buff[i] = (char)data_byte;
        i++;
    }
    else
    {
        i = 0;
        flag = 0;
        memset(buff, 0, GNSS_INIT_BUFFER_SIZE);
    }
}
// void setupGNSScommRX()
// {
//     // RX initialization
//     LL_DMA_SetDataLength(GNSS_DMA, GNSS_DMA_STREAM_RX, (uint32_t)GNSS_INIT_BUFFER_SIZE);
//     LL_DMA_ConfigAddresses(GNSS_DMA, GNSS_DMA_STREAM_RX,
//                            LL_USART_DMA_GetRegAddr(GNSS_USART), (uint32_t)gnssBuffer,
//                            LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//     LL_USART_EnableDMAReq_RX(GNSS_USART);
//     LL_DMA_EnableStream(GNSS_DMA, GNSS_DMA_STREAM_RX);

//     LL_DMA_EnableIT_HT(GNSS_DMA, GNSS_DMA_STREAM_RX);
//     LL_DMA_EnableIT_TC(GNSS_DMA, GNSS_DMA_STREAM_RX);
//     LL_USART_EnableIT_IDLE(GNSS_USART);

//    // xUSARTQueue = xQueueCreate(USART_QUEUE_SIZE, sizeof(UARTMessage_t *));
// }