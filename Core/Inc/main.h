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
#include "stm32f4xx_ll_dma.h"

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
#define GPS_RST_Pin LL_GPIO_PIN_4
#define GPS_RST_GPIO_Port GPIOA
#define LCD_BLK_Pin LL_GPIO_PIN_0
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RS_Pin LL_GPIO_PIN_1
#define LCD_RS_GPIO_Port GPIOB
#define LCD_CS_Pin LL_GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCL_Pin LL_GPIO_PIN_13
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_RST_Pin LL_GPIO_PIN_14
#define LCD_RST_GPIO_Port GPIOB
#define LCD_SDI_Pin LL_GPIO_PIN_15
#define LCD_SDI_GPIO_Port GPIOB
#define GPS_RX_Pin LL_GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define GPS_TX_Pin LL_GPIO_PIN_15
#define GPS_TX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define UART_PORT_GPS USART1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
