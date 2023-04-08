#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "math.h"
#include "gnss.h"

#include "usb_device.h"

extern "C"
{ // another way
#include "Lcd_Driver.h"
#include "GUI.h"
#include "TFT_demo.h"
};

extern "C"
{ // another way
	gnss_ret_e gnss_setup();
	void gnss_process_loop(bool force);
	gnss_ret_e gnss_start(gnss_run_mode_e mode, uint16_t fixFreq, uint32_t timeoutS);
    void gnss_printState(void);
};

// static void gpsCommand(char *msg);
// void displayInfo();
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GPSBaud = 9600;

char NMEA_Sent[NMEA_GPRMC_SENTENCE_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);

void projectMain()
{
	LL_TIM_EnableCounter(TIM5);

    Lcd_Init();	 //1.8 Inch LCD screen -- Initialization configuration 
	Lcd_Clear(GRAY0);// Clear the screen 									  // initialization LCD  
	
	// USART_PrintString("A simple demonstration of TinyGPSPlus with an attached GPS module\n");
    // USART_PrintString("Testing TinyGPSPlus library v. \n");

    gnss_run_mode_e mode = GNSS_RUN_HOT;
    uint16_t fixFreq = 100;
    uint32_t timeoutS = 1;

    gnss_setup();
    gnss_ret_e res = gnss_start(mode, fixFreq, timeoutS);

    // gpsCommand("$PGCMD,33,0*6D\r\n");   // turn off antenna data nuesance
    // gpsCommand("$PMTK220,1000*1F\r\n"); // set update frequency to 1Hz
    // gpsCommand("$PMTK010,001*2E\r\n");  // This message is used to automatically output system messages by GPS module
    // gpsCommand("$PMTK011,MTKGPS*08\r\n");  // This message is used to automatically output system messages by GPS module
    // gpsCommand("$PMTK101*32\r\n");         // This message is used to hot start the GPS module
    // gpsCommand("$PMTK102*31\r\n");         // This message is used to warm start the GPS module
    // gpsCommand("$PMTK103*30\r\n");         // This message is used to cold start the GPS module
    // gpsCommand("$PMTK104*37\r\n");         // This message is used to factory reset the GPS module, no information about last gps location
    // gpsCommand("$PMTK313,1*2E\r\n");       // This message is used to enable the NMEA message output of the GPS module
    Lcd_Clear(GRAY0);

    while (true)
    {
        //Test_Demo();	// LCD screen test DEMO	   
 
        gnss_process_loop(true);
        gnss_printState();
        LL_mDelay(1000);

        Gui_DrawFont_GBK16_l(8, 0, BLUE, GRAY0, (uint8_t*)NMEA_Sent,10);

        // gpsCommand("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); // RMC NMEA Sentence
        // USART_PrintString("-----------\n");
        // USART_PrintString(NMEA_Sent);
        // USART_PrintString("\n-----------");
    }
}



