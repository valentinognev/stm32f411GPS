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
	void gnss_process_loop(bool force);
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

extern TaskHandle_t xGPSMessageRXTaskHandle;
extern TaskHandle_t xUSARTTaskHandle;
extern TaskHandle_t xGNSSTaskHandle;

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);
void ProjectMain()
{
    setupUSART();
    setupLCD();
    setupGPS();
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
    xTaskCreate(vGNSSTask, "GNSS", 300, NULL, tskIDLE_PRIORITY + 3, &xGNSSTaskHandle);
    // xTaskCreate(vTaskInfoTransmitTask, "NRF_TX_TaskInfo", 300, NULL, tskIDLE_PRIORITY + 2, NULL);
    osQueueUSARTMessage("Hello world %d from FreeRTOS\r\n", xTaskGetTickCount());
    osQueueUSARTMessage("Compiled at " __DATE__ " " __TIME__ "\r\n");
    vTaskStartScheduler();
}



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


