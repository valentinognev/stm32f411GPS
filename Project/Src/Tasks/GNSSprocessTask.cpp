#include "GNSSprocessTask.h"
#include "SERIALcommTXTask.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "GNSScommRXTask.h"


#include <string.h>

#define GNSS_PROCESS_QUEUE_SIZE 100
static QueueHandle_t xGNSSprocessQueue;
TaskHandle_t xGNSSprocessTaskHandle;
SemaphoreHandle_t xGNSSprocessSemaphore;

gnss_simple_data_t xGNSSData;

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

    gnss_setup();
    gnss_ret_e res = gnss_start(mode, fixFreq, timeoutS);
    setupGNSScommRX();

    GNSSprocessMessage_t message;
    while (1)
    {
        if (xQueueReceive(xGNSSprocessQueue, &message, portMAX_DELAY))
        {
            gnss_processString(message);
            vPortFree(message);

            gnss_getData(&gnssData);

            gnss_printState();
        }
    }
}

/**
 * Queue a GNSS message so that it can be printed later
 */
void osQueueGNSSprocessMessage(const char* gnssmess)
{
    GNSSprocessMessage_t pcGNSSprocessMessage = (GNSSprocessMessage_t)pvPortMalloc(strlen(gnssmess));

    if (pcGNSSprocessMessage == NULL)
    {
        osQueueSERIALMessage("ERROR! Not enough memory to store GNSS string\r\n");
        return;
    }
    
    strcpy(pcGNSSprocessMessage, gnssmess);

    // TODO: Show a warning if the queue is full (e.g. replace the last
    // message in the queue)
    if (xQueueSend(xGNSSprocessQueue, (void *)(&pcGNSSprocessMessage), (TickType_t)0) == pdFAIL)
    {
        // Make sure to deallocate the failed message
        vPortFree(pcGNSSprocessMessage);
    }
    
}


void setupGNSSprocess()
{
    xGNSSprocessSemaphore = xSemaphoreCreateBinary();
    xGNSSprocessQueue = xQueueCreate(GNSS_PROCESS_QUEUE_SIZE, sizeof(GNSSprocessMessage_t));
    xTaskCreate(vGNSSprocessTask, "GNSSprocess", STACK_SIZE_WORDS, NULL, tskIDLE_PRIORITY + 2, NULL);
}
