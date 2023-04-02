#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "math.h"

#include "usb_device.h"
#include "TinyGPS++.h"

/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);

void projectMain()
{
    USART_PrintString("A simple demonstration of TinyGPSPlus with an attached GPS module\n");
    USART_PrintString("Testing TinyGPSPlus library v. \n");

    while (true)
    {
        // This sketch displays information every time a new sentence is correctly encoded.
        // while (ss.available() > 0)
        //     if (gps.encode(ss.read()))
        //         displayInfo();

        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
            USART_PrintString("No GPS detected: check wiring.");
            while (true)
                ;
        }
    }
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
