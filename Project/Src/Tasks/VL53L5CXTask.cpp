// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <cmath>
#include <stdio.h>

#include "VL53L5CXTask.h"
#include "SERIALcommTXTask.h"
#include "FreeRTOS.h"
#include "SWO.h"

#include <string.h>

#include "I2Cdev.h"
#include "vl53l5cx_class.h"

TaskHandle_t xVL53L5CXTaskHandle;
I2Cdev i2cdev;
VL53L5CX sensor_vl53l5cx_sat(&i2cdev);

void VL53L5CXTask(void *pvParameters)
{
    char report[64];
    uint8_t status;

    // Configure VL53L5CX satellite component.
    sensor_vl53l5cx_sat.begin();

    sensor_vl53l5cx_sat.init_sensor();

    /*********************************/
    /*  Set ranging mode autonomous  */
    /*********************************/

    status = sensor_vl53l5cx_sat.vl53l5cx_set_ranging_mode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
    if (status)
    {
        osQueueSERIALMessage("vl53l5cx_set_ranging_mode failed, status %u\r\n", status);
        //blink_led_loop();
    }

    /* Using autonomous mode, the integration time can be updated (not possible
     * using continuous) */
    status = sensor_vl53l5cx_sat.vl53l5cx_set_integration_time_ms(20);

    if (status)
    {
        osQueueSERIALMessage("vl53l5cx_set_integration_time_ms failed, status %u\r\n", status);
        //link_led_loop();
    }

    // Start Measurements
    sensor_vl53l5cx_sat.vl53l5cx_start_ranging();

    while (1)
    {
        static uint8_t loop_count = 0;
        VL53L5CX_ResultsData Results;
        uint8_t NewDataReady = 0;
        char report[64];
        uint8_t status;

        if (loop_count < 10)
        {

            do
            {
                status = sensor_vl53l5cx_sat.vl53l5cx_check_data_ready(&NewDataReady);
            } while (!NewDataReady);

            // Led on
            //digitalWrite(LedPin, HIGH);

            if ((!status) && (NewDataReady != 0))
            {
                status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);

                /* As the sensor is set in 4x4 mode by default, we have a total
                 * of 16 zones to print.
                 */

                osQueueSERIALMessage("Print data no : %3u\r\n", sensor_vl53l5cx_sat.get_stream_count());
                // SerialPort.print(report);
                for (int i = 0; i < 16; i++)
                {
                    osQueueSERIALMessage("Zone : %3d, Status : %3u, Distance : %4d mm\r\n",
                                         i,
                                         Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                                         Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
                }
                osQueueSERIALMessage("\n");
                loop_count++;
            }

            //digitalWrite(LedPin, LOW);
        }
        else if (loop_count == 10)
        {
            /* Stop measurements */
            status = sensor_vl53l5cx_sat.vl53l5cx_stop_ranging();
            if (status)
            {
                osQueueSERIALMessage("vl53l5cx_stop_ranging failed, status %u\r\n", status);
                // blink_led_loop();
            }

            osQueueSERIALMessage("\r\n\r\nStop ranging mode autonomous and Start ranging mode continuous\r\n\r\n");

            /*********************************/
            /* Set ranging mode continuous   */
            /*********************************/

            /* In continuous mode, the integration time cannot be programmed
             * (automatically set to maximum value) */
            status = sensor_vl53l5cx_sat.vl53l5cx_set_ranging_mode(VL53L5CX_RANGING_MODE_CONTINUOUS);
            if (status)
            {
                osQueueSERIALMessage("vl53l5cx_set_ranging_mode failed, status %u\r\n", status);
                //blink_led_loop();
            }

            // Restart Measurements
            sensor_vl53l5cx_sat.vl53l5cx_start_ranging();

            loop_count++;
        }
        else if (loop_count < 21)
        {
            do
            {
                status = sensor_vl53l5cx_sat.vl53l5cx_check_data_ready(&NewDataReady);
            } while (!NewDataReady);

            // Led on
            //digitalWrite(LedPin, HIGH);

            if ((!status) && (NewDataReady != 0))
            {
                status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);

                /* As the sensor is set in 4x4 mode by default, we have a total
                 * of 16 zones to print.
                 */

                osQueueSERIALMessage("Print data no : %3u\r\n", sensor_vl53l5cx_sat.get_stream_count());
                for (int i = 0; i < 16; i++)
                {
                    osQueueSERIALMessage("Zone : %3d, Status : %3u, Distance : %4d mm\r\n",
                                         i,
                                         Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                                         Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
                }
                osQueueSERIALMessage("\n");
                loop_count++;
            }

            //digitalWrite(LedPin, LOW);
        }
        else if (loop_count == 21)
        {
            /* Stop measurements */
            status = sensor_vl53l5cx_sat.vl53l5cx_stop_ranging();
            if (status)
            {
                osQueueSERIALMessage("vl53l5cx_stop_ranging failed, status %u\r\n", status);
                //blink_led_loop();
            }

            loop_count++;
            /* End of the demo */
            osQueueSERIALMessage("End of ULD demo\n");
        }
        else
        {
            delay(1000);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    // Never reach here
    vTaskDelete(NULL);
}

void setupVL53L5CX()
{
    xTaskCreate(VL53L5CXTask, "VL53L5CX", STACK_SIZE_WORDS, NULL, MPU9250KalmanTaskPriority, &xVL53L5CXTaskHandle);
}
