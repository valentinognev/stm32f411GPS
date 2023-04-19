#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "math.h"
#include "gnss.h"

#include "usb_device.h"

#include "FreeRTOS.h"
// #include "message_buffer.h"
// #include "queue.h"
// #include "cmsis_os.h"

#include "task.h"

#include "GPSTask.h"
#include "USARTTask.h"
#include "LCDTask.h"
#include "InfoTask.h"

extern "C"
{ // another way
#include "Lcd_Driver.h"
#include "GUI.h"
#include "TFT_demo.h"
#include "gnss.h"
};

extern "C"
{ // another way
	gnss_ret_e gnss_setup();
	void gnss_process_loop(bool force);
	gnss_ret_e gnss_start(gnss_run_mode_e mode, uint16_t fixFreq, uint32_t timeoutS);
    void gnss_printState(void);
    void gnss_getData(gnss_simple_data_t *gnssData);
};

// static void gpsCommand(char *msg);
// void displayInfo();
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GPSBaud = 9600;


TaskHandle_t xGPSMessageRXTaskHandle;
TaskHandle_t xUSARTTaskHandle;

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);
void ProjectMain()
{
    // SWO_PrintString("Hello World!\r
    configSTACK_DEPTH_TYPE xStackSize = 512;

    xTaskCreate(vGPSMessageRXTask,        /* The function that implements the task. */
                "GPS_Msg_RX",             /* Human readable name for the task. */
                200,                      /* Stack size (in words!). */
                NULL,                     /* Task parameter is not used. */
                tskIDLE_PRIORITY + 3,     /* The priority at which the task is created. */
                &xGPSMessageRXTaskHandle);/* No use for the task handle. */
    xTaskCreate(vUSARTTask, "USART", 300, NULL, tskIDLE_PRIORITY + 3, &xUSARTTaskHandle);
    xTaskCreate(vLCDTransmitTask, "LCD_TX", 250, NULL, tskIDLE_PRIORITY + 1, NULL);
    // xTaskCreate(vTaskInfoTransmitTask, "NRF_TX_TaskInfo", 300, NULL, tskIDLE_PRIORITY + 2, NULL);
}

// void gnssMain()
// {
//     gnss_simple_data_t gnssData = {
//         .hour = 0,
//         .minute = 0,
//         .second = 0,
//         .year = 0,
//         .month = 0,
//         .day = 0,
//         .fix = 0,
//         .satellites = 0,
//         .latitude = 0,
//         .longitude = 0,
//         .altitude = 0,
//         .speed = 0,
//         .course = 0,
//     };
//     gnssDataQueue = xQueueCreate(1, sizeof(gnss_simple_data_t));

//     const size_t gnssMessageBufferSizeBytes = 100;
//     gnssMessageBuffer = xMessageBufferCreate(gnssMessageBufferSizeBytes);

// 	LL_TIM_EnableCounter(TIM5);
//     // gps setup
//     LL_USART_EnableIT_RXNE(USART1);
//     LL_USART_EnableIT_ERROR(USART1);

//     // USART_PrintString("A simple demonstration of TinyGPSPlus with an attached GPS module\n");
//     // USART_PrintString("Testing TinyGPSPlus library v. \n");

//     gnss_run_mode_e mode = GNSS_RUN_HOT;
//     uint16_t fixFreq = 100;
//     uint32_t timeoutS = 1;

//     gnss_setup();
//     gnss_ret_e res = gnss_start(mode, fixFreq, timeoutS);

//     // gpsCommand("$PGCMD,33,0*6D\r\n");   // turn off antenna data nuesance
//     // gpsCommand("$PMTK220,1000*1F\r\n"); // set update frequency to 1Hz
//     // gpsCommand("$PMTK010,001*2E\r\n");  // This message is used to automatically output system messages by GPS module
//     // gpsCommand("$PMTK011,MTKGPS*08\r\n");  // This message is used to automatically output system messages by GPS module
//     // gpsCommand("$PMTK101*32\r\n");         // This message is used to hot start the GPS module
//     // gpsCommand("$PMTK102*31\r\n");         // This message is used to warm start the GPS module
//     // gpsCommand("$PMTK103*30\r\n");         // This message is used to cold start the GPS module
//     // gpsCommand("$PMTK104*37\r\n");         // This message is used to factory reset the GPS module, no information about last gps location
//     // gpsCommand("$PMTK313,1*2E\r\n");       // This message is used to enable the NMEA message output of the GPS module
 
//     while (true)
//     {
//         gnss_process_loop(true);
//         gnss_getData(&gnssData);
//         xQueueSend(gnssDataQueue, &gnssData, 0);
//         gnss_printState();
//         osDelay(1000);
//     }
// }


// void lcdMain()
// {
//     gnss_simple_data_t gnssData;
//     char buf[100];
//     Lcd_Init();	 //1.8 Inch LCD screen -- Initialization configuration 
//     Lcd_Clear(GRAY0);// Clear the screen 									  // initialization LCD  
    
//     while (true)
//     {
//         xQueueReceive(gnssDataQueue, &gnssData, portMAX_DELAY);
//         if (gnssData.satellites == 0)
//         {
//             TFT_PrintString(0, "No GPS signal");
//             continue;
//         }
//         else
//         {
//             sprintf(buf, "Sat: %d", gnssData.satellites);
//             TFT_PrintString(1, buf);
//             sprintf(buf, " %02d/%02d/%02d", gnssData.day, gnssData.month, gnssData.year);
//             TFT_PrintString(2, buf);
//             sprintf(buf, "%02d:%02d:%02d", gnssData.hour, gnssData.minute, gnssData.second);
//             TFT_PrintString(3, buf);
//             sprintf(buf, "Lat: %f", gnssData.latitude);
//             TFT_PrintString(4, buf);
//             sprintf(buf, "Lon: %f", gnssData.longitude);
//             TFT_PrintString(5, buf);
//             sprintf(buf, "Alt: %f", gnssData.altitude);
//             TFT_PrintString(6, buf);
//             sprintf(buf, "Spd: %f", gnssData.speed);
//             TFT_PrintString(7, buf);
//         }
//     }
// }


