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

#include "GNSScommTXTask.h"
#include "GNSScommRXTask.h"
#include "GNSSprocessTask.h"
#include "SERIALcommTXTask.h"
#include "LCDTask.h"
#include "InfoTask.h"

extern "C"
{ // another way
#include "Lcd_Driver.h"
#include "GUI.h"
#include "TFT_demo.h"
};


// static void GNSSCommand(char *msg);
// void displayInfo();
/*
   This sample sketch demonstrates the normal use of a TinyGNSSPlus (TinyGNSSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GNSS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GNSSBaud = 9600;

extern TaskHandle_t xGNSScommRXTaskHandle;
extern TaskHandle_t xGNSScommTXTaskHandle;
extern TaskHandle_t xGNSSprocessTaskHandle;
extern TaskHandle_t xSERIALcommTXTaskHandle;

// The serial connection to the GNSS device
// SoftwareSerial ss(RXPin, TXPin);
void ProjectMain()
{
    setupLCD();    
    setupSERIALcommTX();
    setupGNSScommTX();
    setupGNSScommRX();
    setupGNSSprocess();

    // SWO_PrintString("Hello World!\r
    configSTACK_DEPTH_TYPE xStackSize = 512;

    const int stackSizeWords = 300;
    xTaskCreate(vGNSScommRXTask, "GNSScommRX", stackSizeWords, NULL, tskIDLE_PRIORITY + 3, &xGNSScommRXTaskHandle);
    xTaskCreate(vGNSScommTXTask, "GNSScommTX", stackSizeWords, NULL, tskIDLE_PRIORITY + 3, &xGNSScommTXTaskHandle);
    xTaskCreate(vGNSSprocessTask, "GNSSprocess", stackSizeWords, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vSERIALcommTXTask, "SERIALcommTX", stackSizeWords, NULL, tskIDLE_PRIORITY + 3, &xSERIALcommTXTaskHandle);
    xTaskCreate(vLCDTransmitTask, "LCD_TX", stackSizeWords, NULL, tskIDLE_PRIORITY + 1, NULL);

    // xTaskCreate(vTaskInfoTransmitTask, "NRF_TX_TaskInfo", 300, NULL, tskIDLE_PRIORITY + 2, NULL);
    osQueueSERIALMessage("Hello world %d from FreeRTOS\r\n", xTaskGetTickCount());
    osQueueSERIALMessage("Compiled at " __DATE__ " " __TIME__ "\r\n");

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
//             TFT_PrintString(0, "No GNSS signal");
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


