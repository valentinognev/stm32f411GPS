#ifndef VLC53L5CX_TASK_H_
#define VLC53L5CX_TASK_H_

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

void setupVL53L5CX();
void VL53L5CXTask(void *pvParameters);

#undef EXTERNC
#endif /* VLC53L5CX_TASK_H_ */