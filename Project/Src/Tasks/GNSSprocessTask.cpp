#include "GNSSprocessTask.h"
#include "USARTTask.h"
#include "FreeRTOS.h"
#include "gnss.h"

static QueueHandle_t xGNSSprocessQueue;
TaskHandle_t xGNSSprocessTaskHandle;

gnss_simple_data_t xGNSSData;

void vGNSSTask(void *pvParameters)
{
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
    gnssDataQueue = xQueueCreate(1, sizeof(gnss_simple_data_t));

    const size_t gnssMessageBufferSizeBytes = 100;
    gnssMessageBuffer = xMessageBufferCreate(gnssMessageBufferSizeBytes);

    gnss_run_mode_e mode = GNSS_RUN_HOT;
    uint16_t fixFreq = 100;
    uint32_t timeoutS = 1;

    gnss_setup();
    gnss_ret_e res = gnss_start(mode, fixFreq, timeoutS);

    while (true)
    {
        gnss_process_loop(true);
        gnss_getData(&gnssData);
        xQueueSend(gnssDataQueue, &gnssData, 0);
        gnss_printState();
        osDelay(1000);
    }
}

/**
 * Queue a GNSS message so that it can be printed later
 */
void osQueueGNSSprocessMessage(const char* gnssmess)
{
    GNSSMessage_t pcGNSSMessage = (GNSSMessage_t)pvPortMalloc(strlen(gnssmess));

    if (pcGNSSMessage == NULL)
    {
        osQueueUSARTMessage("ERROR! Not enough memory to store GNSS string\r\n");
    }
    else
    {
        strcpy(pcGNSSMessage, buffer);

        // TODO: Show a warning if the queue is full (e.g. replace the last
        // message in the queue)
        if (xQueueSend(xGNSSQueue, (void *)(&pcGNSSMessage), (TickType_t)0) == pdFAIL)
        {
            // Make sure to deallocate the failed message
            vPortFree(pcGNSSMessage);
        }
    }
}