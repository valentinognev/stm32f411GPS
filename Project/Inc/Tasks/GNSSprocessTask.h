#ifndef GNSS_TASK_H_
#define GNSS_TASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

typedef char *GNSSMessage_t;

void setupGNSSprocess();
void vGNSSprocessTask(void *pvParameters);
void osQueueGNSSprocessMessage(const char *gnssmess);




#endif /* GNSS_TASK_H_ */