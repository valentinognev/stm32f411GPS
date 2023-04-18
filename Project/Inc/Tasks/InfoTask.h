#ifndef INFOTASK_H_
#define INFOTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// TODO: Use dynamic allocation for message strings
typedef uint8_t *INFOMessage_t;
extern QueueHandle_t xINFOQueue;

void vTaskInfoTransmitTask(void *pvParameters);

#endif /* INFOTASK_H_ */