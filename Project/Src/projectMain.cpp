#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "math.h"
#include "gnss.h"

#include "usb_device.h"
#include "TinyGPS++.h"

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

// The TinyGPSPlus object
TinyGPSPlus gps;
char NMEA_Sent[NMEA_GPRMC_SENTENCE_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);

void projectMain()
{
	LL_TIM_EnableCounter(TIM5);

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

    while (true)
    {
        gnss_process_loop(true);
        gnss_printState();
        LL_mDelay(1000);
        // gpsCommand("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); // RMC NMEA Sentence
        // USART_PrintString("-----------\n");
        // USART_PrintString(NMEA_Sent);
        // USART_PrintString("\n-----------");

        // This sketch displays information every time a new sentence is correctly encoded.
        // for (int i = 0; i < strlen(NMEA_Sent); i++)
        //     if (gps.encode(NMEA_Sent[i]))
        //         displayInfo();
        // LL_mDelay(1000);
        // if (millis() > 5000 && gps.charsProcessed() < 10)
        // {
        //     USART_PrintString("No GPS detected: check wiring.");
        //     while (true)
        //         ;
        // }
    }
}

static void gpsCommand(char *msg)
{
    for (int i = 0; i < strlen(msg); i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(USART1))
            ;
        LL_USART_TransmitData8(USART1, msg[i]);
    }
    LL_mDelay(100);
}

void displayInfo()
{
    char buf[100];
    USART_PrintString("Location: ");
    if (gps.location.isValid())
    {
        sprintf(buf, "%f,%f\n", gps.location.lat(), gps.location.lng());
        USART_PrintString(buf);
    }
    else
    {
        USART_PrintString("INVALID");
    }

    USART_PrintString("  Date/Time: ");
    if (gps.date.isValid())
    {
        sprintf(buf, "%02d/%02d/%04d ", gps.date.day(), gps.date.month(), gps.date.year());
        USART_PrintString(buf);
    }
    else
    {
        USART_PrintString("INVALID");
    }

    USART_PrintString(" ");
    if (gps.time.isValid())
    {
        sprintf(buf, "%02d:%02d:%02d.%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
        USART_PrintString(buf);
    }
    else
    {
        USART_PrintString("INVALID");
    }

}
