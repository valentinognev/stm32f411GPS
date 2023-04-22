#ifndef GPSTASK_H_
#define GPSTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

// TODO: Use dynamic allocation for message strings
typedef uint8_t *GPSMessage_t;
extern TaskHandle_t xGPSMessageRXTaskHandle;
extern QueueHandle_t xGPSQueue;

void setupGNSScommRX();
void vGNSScommRXTask(void *pvParameters);


EXTERNC void DMA_GNSS_RX_ISR(void);


#endif /* GPSTASK_H_ */