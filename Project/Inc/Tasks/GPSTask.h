#ifndef GPSTASK_H_
#define GPSTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// TODO: Use dynamic allocation for message strings
typedef uint8_t *GPSMessage_t;
extern TaskHandle_t xGPSMessageRXTaskHandle;
extern QueueHandle_t xGPSQueue;

void setupGPS();
void vGPSMessageRXTask(void *pvParameters);
void vGNSSTask(void *pvParameters);

#ifdef __cplusplus
extern "C"
{
#endif

    void DMA_GPS_TX_ISR(void);
    void DMA_GPS_RX_ISR(void);

#ifdef __cplusplus
}
#endif

#endif /* GPSTASK_H_ */