#ifndef LCDTASK_H_
#define LCDTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

// TODO: Use dynamic allocation for message strings
typedef uint8_t *LCDMessage_t;
extern QueueHandle_t xLCDQueue;

void vTFTcommTXTask(void *pvParameters);
void setupTFTcommTX();
EXTERNC void TFT_DMA_TransmitComplete_Callback();
EXTERNC void TFT_DMA_TransferError_Callback();

#endif /* LCDTASK_H_ */