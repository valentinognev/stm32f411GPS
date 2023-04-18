#ifndef LCDTASK_H_
#define LCDTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// TODO: Use dynamic allocation for message strings
typedef uint8_t *LCDMessage_t;
extern QueueHandle_t xLCDQueue;

void vLCDTransmitTask(void *pvParameters);

#endif /* LCDTASK_H_ */