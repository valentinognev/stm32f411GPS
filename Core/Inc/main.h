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
#define TFT_SCK_Pin LL_GPIO_PIN_5
#define TFT_SCK_GPIO_Port GPIOA
#define TFT_MOSI_Pin LL_GPIO_PIN_7
#define TFT_MOSI_GPIO_Port GPIOA
#define LCD_BLK_Pin LL_GPIO_PIN_0
#define LCD_BLK_GPIO_Port GPIOB
#define TFT_RS_Pin LL_GPIO_PIN_1
#define TFT_RS_GPIO_Port GPIOB
#define TFT_CS_Pin LL_GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define LCD_SCL_Pin LL_GPIO_PIN_13
#define LCD_SCL_GPIO_Port GPIOB
#define TFT_RST_Pin LL_GPIO_PIN_14
#define TFT_RST_GPIO_Port GPIOB
#define LCD_SDI_Pin LL_GPIO_PIN_15
#define LCD_SDI_GPIO_Port GPIOB
#define GNSS_RST_Pin LL_GPIO_PIN_9
#define GNSS_RST_GPIO_Port GPIOA
#define GNSS_RX_Pin LL_GPIO_PIN_10
#define GNSS_RX_GPIO_Port GPIOA
#define GNSS_TX_Pin LL_GPIO_PIN_15
#define GNSS_TX_GPIO_Port GPIOA
#define SENSOR_SCL_Pin LL_GPIO_PIN_6
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin LL_GPIO_PIN_7
#define SENSOR_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define GNSS_USART USART1
#define SERIAL_USART USART2
#define SENSOR_I2C I2C1
#define TFT_SPI SPI1

#define TFT_DMA                DMA2
#define TFT_DMA_STREAM_TX      LL_DMA_STREAM_3
#define SENSOR_DMA             DMA1
#define SENSOR_DMA_STREAM_TX   LL_DMA_Stream_1
#define SENSOR_DMA_STREAM_RX   LL_DMA_Stream_0
#define SERIAL_DMA             DMA1
#define SERIAL_DMA_STREAM_TX   LL_DMA_STREAM_6
#define SERIAL_DMA_STREAM_RX   LL_DMA_STREAM_5
#define GNSS_DMA               DMA2
#define GNSS_DMA_STREAM_TX     LL_DMA_STREAM_7
#define GNSS_DMA_STREAM_RX     LL_DMA_STREAM_2

#define STACK_SIZE_WORDS 256
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
