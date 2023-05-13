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

void setupMPU9250Kalman();
void MPU9250KalmanTask(void *pvParameters);

#undef EXTERNC
#endif /* IMU_TASK_H_ */