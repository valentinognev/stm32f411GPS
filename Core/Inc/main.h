/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
 * Basic enable / disable
 */
#define __DISABLE					0x00
#define __ENABLE					0x01
#define itsdk_time_get_ms HAL_GetTick
#define NMEA_GPRMC_SENTENCE_SIZE 80

  void printmsg(char *msg);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SERIAL_TX_Pin LL_GPIO_PIN_2
#define SERIAL_TX_GPIO_Port GPIOA
#define SERIAL_RX_Pin LL_GPIO_PIN_3
#define SERIAL_RX_GPIO_Port GPIOA
#define FLASH_CS_Pin LL_GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_SCK_Pin LL_GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MOSI_Pin LL_GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define TFT_BLK_Pin LL_GPIO_PIN_0
#define TFT_BLK_GPIO_Port GPIOB
#define TFT_RS_Pin LL_GPIO_PIN_1
#define TFT_RS_GPIO_Port GPIOB
#define TFT_CS_Pin LL_GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define TFT_SCK_Pin LL_GPIO_PIN_13
#define TFT_SCK_GPIO_Port GPIOB
#define TFT_RST_Pin LL_GPIO_PIN_14
#define TFT_RST_GPIO_Port GPIOB
#define TFT_MOSI_Pin LL_GPIO_PIN_15
#define TFT_MOSI_GPIO_Port GPIOB
#define GNSS_RST_Pin LL_GPIO_PIN_9
#define GNSS_RST_GPIO_Port GPIOA
#define GNSS_RX_Pin LL_GPIO_PIN_10
#define GNSS_RX_GPIO_Port GPIOA
#define GNSS_TX_Pin LL_GPIO_PIN_15
#define GNSS_TX_GPIO_Port GPIOA
#define FLASH_MISO_Pin LL_GPIO_PIN_4
#define FLASH_MISO_GPIO_Port GPIOB
#define SENSOR_SCL_Pin LL_GPIO_PIN_6
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin LL_GPIO_PIN_7
#define SENSOR_SDA_GPIO_Port GPIOB
#define VLINT_Pin LL_GPIO_PIN_8
#define VLINT_GPIO_Port GPIOB
#define VLLPN_Pin LL_GPIO_PIN_9
#define VLLPN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define GNSScommTxTaskPriority 3
#define GNSSprocessTaskPriority 2
#define InfoTaskPriority 3
#define MPU9250KalmanTaskPriority 3
#define SERIALcommRXTaskPriority 3
#define SERIALcommTXTaskPriority 4
#define TFTcommTXTaskPriority 4
#define VL53L5CXTaskPriority 3

#define GNSS_USART USART1
#define SERIAL_USART USART2
#define SENSOR_I2C I2C1
#define TFT_SPI SPI2
#define FLASH_SPI SPI1

#define TFT_DMA                DMA1
#define TFT_DMA_STREAM_TX      LL_DMA_STREAM_4
#define SENSOR_DMA             DMA1
#define SENSOR_DMA_STREAM_TX   LL_DMA_STREAM_1
#define SENSOR_DMA_STREAM_RX   LL_DMA_STREAM_0
#define SERIAL_DMA             DMA1
#define SERIAL_DMA_STREAM_TX   LL_DMA_STREAM_6
#define SERIAL_DMA_STREAM_RX   LL_DMA_STREAM_5
#define GNSS_DMA               DMA2
#define GNSS_DMA_STREAM_TX     LL_DMA_STREAM_7
#define GNSS_DMA_STREAM_RX     LL_DMA_STREAM_2
#define FLASH_DMA              DMA2
#define FLASH_DMA_STREAM_TX    LL_DMA_STREAM_3
#define FLASH_DMA_STREAM_RX    LL_DMA_STREAM_0

#define SENSOR_EVIRQn I2C1_EV_IRQn
#define SENSOR_ERIRQn I2C1_ER_IRQn
#define SENSOR_I2C_PERIPH LL_APB1_GRP1_PERIPH_I2C1
#define SENSOR_SCL_PERIPH LL_AHB1_GRP1_PERIPH_GPIOB
#define SENSOR_SDA_PERIPH LL_AHB1_GRP1_PERIPH_GPIOB
#define SENSOR_DMA_PERIPH LL_AHB1_GRP1_PERIPH_DMA1
#define SENSOR_GPIO_AF LL_GPIO_AF_4
#define SENSOR_DMA_CHANNEL_RX LL_DMA_CHANNEL_1
#define SENSOR_DMA_CHANNEL_TX LL_DMA_CHANNEL_0
#define SENSOR_DMA_IRQ_RX DMA1_Stream0_IRQn
#define SENSOR_DMA_IRQ_TX DMA1_Stream1_IRQn
#define SENSOR_DMA_RX_ClearFlag_TC LL_DMA_ClearFlag_TC0
#define SENSOR_DMA_RX_ClearFlag_TE LL_DMA_ClearFlag_TE0
#define SENSOR_DMA_RX_IsActiveFlag_TC LL_DMA_IsActiveFlag_TC0
#define SENSOR_DMA_RX_IsActiveFlag_TE LL_DMA_IsActiveFlag_TE0
#define SENSOR_DMA_TX_ClearFlag_TC LL_DMA_ClearFlag_TC1
#define SENSOR_DMA_TX_ClearFlag_TE LL_DMA_ClearFlag_TE1
#define SENSOR_DMA_TX_IsActiveFlag_TC LL_DMA_IsActiveFlag_TC1
#define SENSOR_DMA_TX_IsActiveFlag_TE LL_DMA_IsActiveFlag_TE1

#define STACK_SIZE_WORDS 256
#define map(x, in_min, in_max, out_min, out_max) (long)((x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min)
#define micros xTaskGetTickCount
#define I2C_BUFFER_LENGTH 128
#define SENSOR_BUFFER_LENGTH I2C_BUFFER_LENGTH


//////////////////////////////////////////////////////////////////////
/////// VL52L5CX data
//////////////////////////////////////////////////////////////////////
/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#ifndef VL53L5CX_NB_TARGET_PER_ZONE
#define VL53L5CX_NB_TARGET_PER_ZONE 1U
#endif

/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this macro
 * allows to use the firmware format instead of user format. The firmware format allows
 * an increased precision.
 */

// #define  VL53L5CX_USE_RAW_FORMAT

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

// #define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
// #define VL53L5CX_DISABLE_NB_SPADS_ENABLED
// #define VL53L5CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
// #define VL53L5CX_DISABLE_RANGE_SIGMA_MM
// #define VL53L5CX_DISABLE_DISTANCE_MM
// #define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
// #define VL53L5CX_DISABLE_TARGET_STATUS
// #define VL53L5CX_DISABLE_MOTION_INDICATOR
//////////////////////////////////////////////////////////////////////
/////// End of VL52L5CX data
//////////////////////////////////////////////////////////////////////


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
