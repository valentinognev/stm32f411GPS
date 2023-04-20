/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "GPSTask.h"
#include "stdbool.h"
#include <string.h>
#include "NMEAQueue.h"
#include "USARTTask.h"

/* USER CODE END Includes */
#include "projectMain.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern NMEAQueue nmea_buff;
char buff[NMEA_GPRMC_SENTENCE_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */
extern uint8_t deviceAddress;
extern uint8_t ubMasterNbDataToTransmit;
extern uint8_t ubMasterNbDataToReceive;
extern uint8_t ubMasterXferDirection;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
    // I2C RX
    /*Check for transfer complete flag*/
    if (LL_DMA_IsActiveFlag_TC0(DMA1))
    {
        i2c_receive_complete_callback();
        LL_DMA_ClearFlag_TC0(DMA1);
    }

    /*Check for transfer error flag*/
    if (LL_DMA_IsActiveFlag_TE0(DMA1))
    {
        i2c_receive_error_callback();
    }

  /* USER CODE END DMA1_Stream0_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
    // I2C TX
    /*Check for transfer complete flag*/
    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        i2c_transfer_complete_callback();
        LL_DMA_ClearFlag_TC1(DMA1);
    }

    /*Check for transfer error flag*/
    if (LL_DMA_IsActiveFlag_TE1(DMA1))
    {
        i2c_transfer_error_callback();
    }

  /* USER CODE END DMA1_Stream1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
// RX USART DEBUG
  /* USER CODE END DMA1_Stream5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
  // TX USART DEBUG
    if (LL_DMA_IsActiveFlag_TC6(DMA1))
    {
        LL_DMA_ClearFlag_TC6(DMA1);
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);

        DMA_USART_TX_ISR();
    }
  /* USER CODE END DMA1_Stream6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
   /* Check SB flag value in ISR register */
    if (LL_I2C_IsActiveFlag_SB(I2C1))
    {
        /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a ubMasterRequestDirection request */
        LL_I2C_TransmitData8(I2C1, deviceAddress);
    }
    /* Check ADDR flag value in ISR register */
    else if (LL_I2C_IsActiveFlag_ADDR(I2C1))
    {
        /* Verify the transfer direction */
        if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ)
        {
            ubMasterXferDirection = LL_I2C_DIRECTION_READ;

            if (ubMasterNbDataToReceive == 1)
            {
                /* Prepare the generation of a Non ACKnowledge condition after next received byte */
                LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

                /* Enable DMA transmission requests */
                LL_I2C_EnableDMAReq_RX(I2C1);
            }
            else if (ubMasterNbDataToReceive == 2)
            {
                /* Prepare the generation of a Non ACKnowledge condition after next received byte */
                LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

                /* Enable Pos */
                LL_I2C_EnableBitPOS(I2C1);
            }
            else
            {
                /* Enable Last DMA bit */
                LL_I2C_EnableLastDMA(I2C1);

                /* Enable DMA transmission requests */
                LL_I2C_EnableDMAReq_RX(I2C1);
            }
        }
        else
        {
            ubMasterXferDirection = LL_I2C_DIRECTION_WRITE;

            /* Enable DMA transmission requests */
            LL_I2C_EnableDMAReq_TX(I2C1);
        }

        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C1);
    }

  /* USER CODE END I2C1_EV_IRQn 0 */

  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */
  Error_Callback();
  /* USER CODE END I2C1_ER_IRQn 0 */

  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
 	if (LL_USART_IsActiveFlag_IDLE(GPS_USART)) 
  {
		LL_USART_ClearFlag_IDLE(GPS_USART);

		DMA_GPS_RX_ISR();
  }

  // static int flag = 0;
  // static int i = 0;
  // uint16_t data_byte;

  // if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  // {
  //   // a data byte is received from the user
  //   data_byte = LL_USART_ReceiveData8(USART1);
  //   if (data_byte == '$')
  //   {
  //     flag = 1;
  //   }
  //   else if (data_byte == '\r')
  //   {
  //     flag = 0;
  //     buff[i]='\0';
  //     if (strlen(buff) > 3)
  //       NMEAQueue_push(&nmea_buff, buff);
  // //    USART_PrintString("\n-----------\n");
  // //    USART_PrintString(NMEA_Sent);
  // //    USART_PrintString("\n-----------\n");
  //   }
  //   // extract the NMEA sentence
  //   if ((i < NMEA_GPRMC_SENTENCE_SIZE - 1) & (flag == 1))
  //   {
  //     buff[i] = (char)data_byte;
  //     // __gnss_processChar((char)data_byte);
  //     i++;
  //   }
  //   else
  //   {
  //     i = 0;
  //     flag = 0;
  // //    USART_PrintString("\n-----------\n");
  // //    USART_PrintString(NMEA_Sent);
  // //    USART_PrintString("\n-----------\n");
  //   }
  // }
  // else
  // {
  //   if (LL_USART_IsActiveFlag_ORE(USART1))
  //   {
  //     (void)USART1->DR;
  //   }
  //   else if (LL_USART_IsActiveFlag_FE(USART1))
  //   {
  //     (void)USART1->DR;
  //   }
  //   else if (LL_USART_IsActiveFlag_NE(USART1))
  //   {
  //     (void)USART1->DR;
  //   }
  // }

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
  //RX GPS
  DMA_GPS_TX_ISR();
  /* USER CODE END DMA2_Stream2_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */
  // SPI TX
  /* USER CODE END DMA2_Stream3_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */
//TX GPS
  /* USER CODE END DMA2_Stream7_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
