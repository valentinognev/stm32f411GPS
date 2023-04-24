#ifndef GNSSPROCESSTASK_H_
#define GNSSPROCESSTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC 
{
    #include "gnss.h"
}

typedef char *GNSSprocessMessage_t;

void setupGNSSprocess();
void vGNSSprocessTask(void *pvParameters);
void osQueueGNSSprocessMessage(const char *gnssmess);
void osQueueGNSSprocessMessageFromISR(const char *gnssmess);

#undef EXTERNC
#endif /* GNSS_TASK_H_ */