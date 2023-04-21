#include "GPSTask.h"
#include "main.h"
#include "SWO.h"
#include <string.h>
#include <stdio.h>

#include "USARTTask.h"
extern "C"
{
#include "gnss.h"
}

#define DMA_RX_BUFFER_SIZE          550
#define COMMAND_REQUEST_SIZE        12

typedef char *GNSSMessage_t;

// Private function prototypes
void prvGPSDMAMessageTX(char *pcTxMessage);
void prvGPSDMAMessageRX(void);

// Private variables
char cDMA_RX_Buffer[DMA_RX_BUFFER_SIZE] = {"\0"};
char cDMA_TX_Buffer[COMMAND_REQUEST_SIZE + 1];

// Task handle
TaskHandle_t xGPSMessageRXTaskHandle;
TaskHandle_t xGNSSTaskHandle;

gnss_simple_data_t xGPSData;

LL_DMA_InitTypeDef dma_usart_rx;  // Define the DMA structure for RX
LL_DMA_InitTypeDef dma_usart_tx;  // Define the DMA structure for TX

/************************ Sample NMEA cSentences *************************
 * $GPGLL,4916.45,N,12311.12,W,225444,A,*1D                             *
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A *
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47    *
 ************************************************************************/

/*********************************************
 * USART1 DMA TX Channel --> DMA1, Channel 4 *
 * USART1 DMA RX Channel --> DMA1, Channel 5 *
 * *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* *
 * USART3 DMA TX Channel --> DMA1, Channel 2 *
 * USART3 DMA RX Channel --> DMA1, Channel 3 *
 *********************************************/

// void vGPSUSARTTask(void *pvParameters) 
// {
//     char cRF24Msg[32] = {"\0"};

// #if GPS_DEBUGGING_MSGS
//     osQueueUSARTMessage("GPS task started!.....\r\n");
// #endif

//     while(1) 
//     {
//         if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) 
//         {    
//             sprintf(cRF24Msg, "Time: %d:%d:%d", xGPSData.hour,
//                     xGPSData.minute, xGPSData.second);
//             USART_PrintBuffer(cRF24Msg, 32);

//             sprintf(cRF24Msg, "Date: %d/%d/%d", xGPSData.day,
//                     xGPSData.month, xGPSData.year);
//             USART_PrintBuffer(cRF24Msg, 32);

//             sprintf(cRF24Msg, "Lat: %.4f", xGPSData.latitude);
//             USART_PrintBuffer(cRF24Msg, 32);

//             sprintf(cRF24Msg, "Lon: %.4f", xGPSData.longitude);
//             USART_PrintBuffer(cRF24Msg, 32);

//             sprintf(cRF24Msg, "Speed: %.2f km/h", xGPSData.speed);
//             USART_PrintBuffer(cRF24Msg, 32);            
//         }
//     }
// }

void vGPSMessageRXTask(void *pvParameters) 
{
	while(1) 
    {
        if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) 
        {
            prvGPSDMAMessageRX();
        }
    }
}

void prvGPSDMAMessageTX(char *pcTxMessage) 
{
    LL_DMA_SetDataLength(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX), strlen(pcTxMessage));
    LL_DMA_ConfigAddresses(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX),
            (uint32_t)pcTxMessage, LL_USART_DMA_GetRegAddr(GPS_USART),
            LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
    LL_DMA_EnableStream(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX));
}

void prvGPSDMAMessageRX(void) 
{
    size_t xLen = 0;
    size_t xBufDataLen = LL_DMA_GetDataLength(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX));
    char *pcTokSstr = NULL;

    if(xBufDataLen == DMA_RX_BUFFER_SIZE)
        xLen = DMA_RX_BUFFER_SIZE;
    else
        xLen = DMA_RX_BUFFER_SIZE - xBufDataLen;
    cDMA_RX_Buffer[xLen] = '\0';  // Append a null terminator

    pcTokSstr = strtok(cDMA_RX_Buffer, "\r\n");
    while(pcTokSstr) 
    {
        gnss_processString(pcTokSstr);
        pcTokSstr = strtok(NULL, "\r\n");  // Get the other strings, if any
    }

    // Reset the flags in the DMA to prepare for the next transaction
    LL_DMA_ClearFlag_HT2(GPS_DMA);
    LL_DMA_ClearFlag_TC2(GPS_DMA);
    LL_DMA_ClearFlag_TE2(GPS_DMA);

    LL_DMA_DisableStream(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX));
    LL_DMA_SetDataLength(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX),
            (uint32_t)DMA_RX_BUFFER_SIZE);
    LL_DMA_EnableStream(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX));
}

void DMA_GPS_TX_ISR(void) 
{
    if(LL_DMA_IsActiveFlag_TC7(GPS_DMA)) 
    {
        LL_DMA_ClearFlag_TC7(GPS_DMA);
        LL_DMA_DisableStream(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX));
    }
}

void DMA_GPS_RX_ISR(void) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send a notification to FREERTOS for the task to take priority
    vTaskNotifyGiveFromISR(xGPSMessageRXTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setupGPS()
{
    // RX initialization
	LL_DMA_SetDataLength(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX),(uint32_t)DMA_RX_BUFFER_SIZE);
	LL_DMA_ConfigAddresses(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX),
			LL_USART_DMA_GetRegAddr(GPS_USART), (uint32_t)cDMA_RX_Buffer,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_USART_EnableDMAReq_RX(GPS_USART);
    LL_DMA_EnableStream(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_RX));

    // TX initialization
	LL_DMA_SetDataLength(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX), (uint32_t)COMMAND_REQUEST_SIZE);
	LL_DMA_ConfigAddresses(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX),
						   (uint32_t)cDMA_TX_Buffer, LL_USART_DMA_GetRegAddr(GPS_USART),
						   LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_USART_EnableDMAReq_TX(GPS_USART);

    LL_DMA_EnableIT_TC(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX));
    LL_DMA_EnableIT_TE(GPS_DMA, __LL_DMA_GET_STREAM(GPS_DMA_STREAM_TX));

    LL_USART_EnableIT_IDLE(GPS_USART);
    //xUSARTQueue = xQueueCreate(USART_QUEUE_SIZE, sizeof(UARTMessage_t *));
}

void vGNSSTask(void *pvParameters)
{
    gnss_simple_data_t gnssData = {
        .hour = 0,
        .minute = 0,
        .second = 0,
        .year = 0,
        .month = 0,
        .day = 0,
        .fix = 0,
        .satellites = 0,
        .latitude = 0,
        .longitude = 0,
        .altitude = 0,
        .speed = 0,
        .course = 0,
    };
    gnssDataQueue = xQueueCreate(1, sizeof(gnss_simple_data_t));

    const size_t gnssMessageBufferSizeBytes = 100;
    gnssMessageBuffer = xMessageBufferCreate(gnssMessageBufferSizeBytes);

	LL_TIM_EnableCounter(TIM5);
    // gps setup
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_ERROR(USART1);

    // USART_PrintString("A simple demonstration of TinyGPSPlus with an attached GPS module\n");
    // USART_PrintString("Testing TinyGPSPlus library v. \n");

    gnss_run_mode_e mode = GNSS_RUN_HOT;
    uint16_t fixFreq = 100;
    uint32_t timeoutS = 1;

    gnss_setup();
    gnss_ret_e res = gnss_start(mode, fixFreq, timeoutS);

    while (true)
    {
        gnss_process_loop(true);
        gnss_getData(&gnssData);
        xQueueSend(gnssDataQueue, &gnssData, 0);
        gnss_printState();
        osDelay(1000);
    }
}

/**
 * Queue a UART message so that it can be printed later
 */
void osQueueUSARTMessage(const char *format, ...)
{
	// TODO: Less copying around bits

	va_list arg;
	char buffer[128];

	va_start(arg, format);
	vsnprintf(buffer, 128, format, arg);
	va_end(arg);

	// configASSERT(strlen(message) < 127);
	GNSSMessage_t pcGNSSMessage = (GNSSMessage_t)pvPortMalloc(strlen(buffer) + 1);

	if (pcGNSSMessage == NULL)
	{
		USART_PrintString("ERROR! Not enough memory to store UART string\r\n");
	}
	else
	{
		strcpy(pcGNSSMessage, buffer);

		// TODO: Show a warning if the queue is full (e.g. replace the last
		// message in the queue)
		if (xQueueSend(xGNSSQueue, (void *)(&pcGNSSMessage), (TickType_t)0) == pdFAIL)
		{
			// Make sure to deallocate the failed message
			vPortFree(pcGNSSMessage);
		}
	}
}