/* ==========================================================
 * configDrivers.h - 
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 7 nov. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU LESSER General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------
 * 
 *
 * ==========================================================
 */

#ifndef INC_IT_SDK_CONFIGDRIVERS_H_
#define INC_IT_SDK_CONFIGDRIVERS_H_

#include "main.h"
/**
 * List of user expected option to filter the unneeded NMEA messages
 */
#define __GNSS_WITH_2DPOS 0x0001
#define __GNSS_WITH_3DPOS 0x0002
#define __GNSS_WITH_TIME 0x0004
#define __GNSS_WITH_DATE 0x0008
#define __GNSS_WITH_HDOP 0x0010
#define __GNSS_WITH_PDOP_VDOP 0x0020
#define __GNSS_WITH_SAT_DETAILS 0x0040
#define __GNSS_WITH_SPEED 0x0080
#define __GNSS_WITH_COG 0x0100 // Direction

// ******************************************* GNSS *********************************************************************

// -------------------------------------------------------------------------
// Gnss : COMMON

#define ITSDK_DRIVERS_WITH_GNSS_DRIVER                __ENABLE
#if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE

#define ITSDK_DRIVERS_GNSS_SERIAL                     __UART_USART1                // Select the Serial port used for the communication
#define ITSDK_DRIVERS_GNSS_LINEBUFFER                256                            // Buffer to store the serial chars until we get a full line                                                                                                                            //  select __UART_NONE if none

#define ITSDK_DRIVERS_GNSS_WITHGPSSAT                __ENABLE                    // Store details of the GPS Sat in memory
#define ITSDK_DRIVERS_GNSS_WITHGLOSAT                __ENABLE                    // Store details of the GLONASS Sat in memory
#define ITSDK_DRIVERS_GNSS_WITHGALSAT                __DISABLE                    // Store details of the GALILEO Sat in memory

#define ITSDK_DRIVERS_GNSS_WITH_UTCDATE_FULL        __DISABLE                    // Convert the Date+Time into UTC timestamp
                                                                                //  this feature coset up to 6KB of flash footprint.
                                                                                //  when _DISABLE, only HH:MM:SS is taken into consideration

                                                                                // What GPS information is needed, this helps to filter the
                                                                                // unused GPS NEMA messages and saves processing / energy
#define ITSDK_DRIVERS_GNSS_POSINFO        (   __GNSS_WITH_2DPOS                     /* Lat / Lng */ \
                                          | __GNSS_WITH_3DPOS                    /* Altitude */ \
                                          | __GNSS_WITH_TIME                    /* UTC time of the day */\
                                          | __GNSS_WITH_DATE                    /* UTC Date */\
                                          | __GNSS_WITH_HDOP                    /* Hdop */\
                                          | __GNSS_WITH_PDOP_VDOP                /* VDOP + PDOP */\
                                          | __GNSS_WITH_SAT_DETAILS                /* Sat in view and signal level*/\
                                          | __GNSS_WITH_SPEED                    /* Speed */\
                                          | __GNSS_WITH_COG                        /* Course over ground - direction*/\
                                        )
#include <gnss.h>


// -------------------------------------------------------------------------
// GNSS : L80 & L86

#define ITSDK_DRIVERS_GNSS_QUECTEL            __ENABLE
#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
    #include <quectel.h>
    #define ITSDK_DRIVERS_GNSS_QUECTEL_MODEL    DRIVER_GNSS_QUECTEL_MODEL_L80
    #if ITSDK_WITH_UART_RXIRQ == __UART_NONE ||  ITSDK_WITH_UART_RXIRQ_BUFSZ < 64
      #warning "For GNSS, UART under interrupt is recommended and buffer size higher than 64 is also recommended"
    #endif
#endif
#define __LP_GPIO_NONE (0)
#define ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK        GNSS_RST_GPIO_Port                    // Pin to control the quectel GNSS Reset signal
#define ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN         GNSS_RST_Pin                    //    __LP_GPIO_NONE if not used
// #define ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK    __BANK_B                    // Pin to control the L86 Force On signal. When __LP_GPIO_NONE the backup mode is disabled
// #define ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN    __LP_GPIO_15
// #define ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK    __BANK_B                    // Pin to control the L80/L86 VCC_ENABLE on
// #define ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN    __LP_GPIO_NONE                //  an external power switch. When __LP_GPIO_NONE the backup mode is disabled
                                                                                                    // L86 backup mode not working correctly, you need to use it also for this module.
// #define ITSDK_DRIVERS_GNSS_QUECTEL_L8X_SERIAL_DISC    __DISABLE                    // The way serial line works on quectel is source of trouble, you could have to disable it when the
                                                                                //  GNSS is on hold. Putting the Serial Line with a pull-up sound a good choice and could avoid the use of this.
#endif // ITSDK_DRIVERS_WITH_GNSS_DRIVER


#endif /* INC_IT_SDK_CONFIGDRIVERS_H_ */