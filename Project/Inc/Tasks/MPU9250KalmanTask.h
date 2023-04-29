#ifndef IMUPROCESSTASK_H_
#define IMUPROCESSTASK_H_

#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

// EXTERNC 
// {
//     #include "IMU.h"
// }

typedef char *IMUprocessMessage_t;

void setupIMUprocess();
void vIMUprocessTask(void *pvParameters);
void osQueueIMUprocessMessageFromISR(const char *IMUmess);

#undef EXTERNC
#endif /* IMU_TASK_H_ */