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
#include "GNSSprocessTask.h"
#include "SERIALcommRXTask.h"
#include "SERIALcommTXTask.h"
#include "TFTcommTXTask.h"
#include "MPU9250KalmanTask.h"
#include "VL53L5CXTask.h"
#include "InfoTask.h"

void ProjectMain()
{
//     setupTFTcommTX();    
//  //   setupSERIALcommRX();
      setupSERIALcommTX();
//    setupGNSSprocess();
//    setupGNSScommTX();
//    setupMPU9250Kalman();
    setupVL53L5CX();


    // SWO_PrintString("Hello World!\r
    configSTACK_DEPTH_TYPE xStackSize = 512;
    //osQueueGNSStransmitMessage("const char *gnssmess");        
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


