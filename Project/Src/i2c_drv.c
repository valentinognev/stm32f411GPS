/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 *
 * @note
 * For some reason setting CR1 reg in sequence with
 * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
 * instant start->stop condition (3.9us long) which I found out with an I2C
 * analyzer. This fast start->stop is only possible to generate if both
 * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
 * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
 * problem is gone. Go figure...
 */

// Standard includes.
#include <string.h>
// Scheduler include files.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"
// Application includes.
#include "i2c_drv.h"

#define I2C_MASTER_WRITE (0)
#define I2C_MASTER_READ (1)
// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED 400000

// Definition of eeprom and deck I2C buss
#define I2C_DEFAULT_DECK_CLOCK_SPEED 400000

// Misc constants.
#define I2C_NO_BLOCK 0
#define I2C_SLAVE_ADDRESS7 0x30
#define I2C_MAX_RETRIES 2
#define I2C_MESSAGE_TIMEOUT (1000)

// Delay is approx 0.06us per loop @168Mhz
#define I2CDEV_LOOPS_PER_US 17
#define I2CDEV_LOOPS_PER_MS (16789) // measured

// Defines to unlock bus
#define I2CDEV_CLK_TS (10 * I2CDEV_LOOPS_PER_US)
#define GPIO_WAIT_FOR_HIGH(gpio, pin, timeoutcycles)         \
  {                                                          \
    int i = timeoutcycles;                                   \
    while (LL_GPIO_IsInputPinSet(gpio, pin) == false && i--) \
      ;                                                      \
  }

#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles)         \
  {                                                         \
    int i = timeoutcycles;                                  \
    while (LL_GPIO_IsInputPinSet(gpio, pin) == true && i--) \
      ;                                                     \
  }

#ifdef I2CDRV_DEBUG_LOG_EVENTS
// Debug variables
uint32_t eventDebug[1024][2];
uint32_t eventPos = 0;
#endif

/* Private functions */
/**
 * Low level i2c init funciton
 */
static void i2cdrvInitBus(I2cDrv *i2c);
/**
 * Low level dma init funciton
 */
static void i2cdrvDmaSetupBus(I2cDrv *i2c);
/**
 * Start the i2c transfer
 */
static void i2cdrvStartTransfer(I2cDrv *i2c);
/**
 * Try to restart a hanged buss
 */
static void i2cdrvTryToRestartBus(I2cDrv *i2c);
/**
 * Unlocks the i2c bus if needed.
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef *portSCL, GPIO_TypeDef *portSDA, uint16_t pinSCL, uint16_t pinSDA);
/**
 * Clear DMA stream
 */
static void i2cdrvClearDMA(I2cDrv *i2c);
/**
 * Event interrupt service routine
 */
// static void i2cdrvEventIsrHandler(I2cDrv *i2c);
/**
 * Error interrupt service routine
 */
// static void i2cdrvErrorIsrHandler(I2cDrv *i2c);
/**
 * DMA interrupt service routine
 */
// static void i2cdrvDmaIsrHandler(I2cDrv *i2c);

// // Cost definitions of busses
static const I2cDef sensorBusDef =
    {
        .i2cPort = SENSOR_I2C,
        .i2cPerif = SENSOR_I2C_PERIPH,
        .i2cEVIRQn = SENSOR_EVIRQn,
        .i2cERIRQn = SENSOR_ERIRQn,
        .i2cClockSpeed = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
        .gpioSCLPerif = SENSOR_SCL_PERIPH,
        .gpioSCLPort = SENSOR_SCL_GPIO_Port,
        .gpioSCLPin = SENSOR_SCL_Pin,
        .gpioSDAPerif = SENSOR_SDA_PERIPH,
        .gpioSDAPort = SENSOR_SDA_GPIO_Port,
        .gpioSDAPin = SENSOR_SDA_Pin,
        .gpioAF = SENSOR_GPIO_AF,
        .dmaPerif = SENSOR_DMA_PERIPH,
        .dmaRxChannel = SENSOR_DMA_CHANNEL_RX,
        .dmaRxStream = SENSOR_DMA_STREAM_RX,
        .dmaRxIRQ = SENSOR_DMA_IRQ_RX,
        .dma = SENSOR_DMA,
        .DMA_ClearFlag_TC = SENSOR_DMA_RX_ClearFlag_TC,
        .DMA_ClearFlag_TE = SENSOR_DMA_RX_ClearFlag_TE,
        .DMA_IsActiveFlag_TC = SENSOR_DMA_RX_IsActiveFlag_TC,
        .DMA_IsActiveFlag_TE = SENSOR_DMA_RX_IsActiveFlag_TE,
};

I2cDrv sensorsBus =
    {
        .def = &sensorBusDef,
};

static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
  volatile uint32_t delay = 0;
  for (delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay)
  {
  };
}

static void i2cdrvStartTransfer(I2cDrv *i2c)
{
  //ASSERT_DMA_SAFE(i2c->txMessage.buffer);

  if (i2c->txMessage.direction == i2cRead)
  {
    // i2c->DMAStruct.DMA_BufferSize = i2c->txMessage.messageLength;
    // i2c->DMAStruct.DMA_Memory0BaseAddr = (uint32_t)i2c->txMessage.buffer;
    LL_DMA_ConfigAddresses(i2c->def->dma, i2c->def->dmaRxStream, (uint32_t)i2c->txMessage.buffer,
                           (uint32_t)LL_I2C_DMA_GetRegAddr(i2c->def->i2cPort), LL_DMA_GetDataTransferDirection(i2c->def->dma, i2c->def->dmaRxStream));

    LL_DMA_SetDataLength(i2c->def->dma, i2c->def->dmaRxStream, i2c->txMessage.messageLength);
    LL_DMA_Init(i2c->def->dma, i2c->def->dmaRxStream, &i2c->DMAStruct);
    LL_DMA_EnableStream(i2c->def->dma, i2c->def->dmaRxStream);
  }

  LL_I2C_DisableIT_BUF(SENSOR_I2C);
  LL_I2C_EnableIT_EVT(SENSOR_I2C);

  LL_I2C_ClearFlag_STOP(SENSOR_I2C); //   i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
  LL_I2C_GenerateStartCondition(SENSOR_I2C);
}

static void i2cTryNextMessage(I2cDrv *i2c)
{
  LL_I2C_ClearFlag_STOP(SENSOR_I2C); //   i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
  LL_I2C_GenerateStopCondition(SENSOR_I2C);

  LL_I2C_DisableIT_BUF(SENSOR_I2C);
  LL_I2C_DisableIT_EVT(SENSOR_I2C);
}

static void i2cNotifyClient(I2cDrv *i2c)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(i2c->isBusFreeSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void i2cdrvTryToRestartBus(I2cDrv *i2c)
{
  LL_I2C_InitTypeDef I2C_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  // Enable GPIOA clock
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  // Enable I2C_SENSORS clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  // Configure I2C_SENSORS pins to unlock bus.
  GPIO_InitStruct.Pin = SENSOR_SCL_Pin | SENSOR_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);

  // I2C_SENSORS configuration
  LL_I2C_DisableOwnAddress2(SENSOR_I2C);
  LL_I2C_DisableGeneralCall(SENSOR_I2C);
  LL_I2C_EnableClockStretching(SENSOR_I2C);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(SENSOR_I2C, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(SENSOR_I2C, 0);

  // Enable I2C_SENSORS error interrupts
  LL_I2C_EnableIT_ERR(SENSOR_I2C); // I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);

  NVIC_SetPriority(SENSOR_EVIRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(SENSOR_EVIRQn);
  NVIC_SetPriority(SENSOR_ERIRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(SENSOR_ERIRQn);

  i2cdrvDmaSetupBus(i2c);
}

static void i2cdrvDmaSetupBus(I2cDrv *i2c)
{
  // Enable DMA1 clock
  LL_AHB1_GRP1_EnableClock(i2c->def->dmaPerif);

  // RX DMA Channel Config
  i2c->DMAStruct.Channel = i2c->def->dmaRxChannel;
  // i2c->DMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
  // i2c->DMAStruct.DMA_Memory0BaseAddr = 0;
  i2c->DMAStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  // i2c->DMAStruct.DMA_BufferSize = 0;
  i2c->DMAStruct.PeriphInc = LL_DMA_PERIPH_NOINCREMENT;
  i2c->DMAStruct.MemInc = LL_DMA_MEMORY_NOINCREMENT;
  i2c->DMAStruct.PeriphDataAlignment = LL_DMA_PDATAALIGN_BYTE;
  i2c->DMAStruct.MemDataAlignment = LL_DMA_MDATAALIGN_BYTE;
  i2c->DMAStruct.Mode = LL_DMA_MODE_NORMAL;
  i2c->DMAStruct.Priority = LL_DMA_PRIORITY_HIGH;
  i2c->DMAStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  i2c->DMAStruct.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
  i2c->DMAStruct.MemBurst = LL_DMA_MBURST_SINGLE;
  i2c->DMAStruct.PeriphBurst = LL_DMA_PBURST_SINGLE;

  NVIC_SetPriority(SENSOR_IRQRX, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 12, 0));
  NVIC_EnableIRQ(SENSOR_IRQRX);
}

static void i2cdrvInitBus(I2cDrv *i2c)
{
  i2cdrvTryToRestartBus(i2c);

  i2c->isBusFreeSemaphore = xSemaphoreCreateBinaryStatic(&i2c->isBusFreeSemaphoreBuffer);
  i2c->isBusFreeMutex = xSemaphoreCreateMutexStatic(&i2c->isBusFreeMutexBuffer);
}

static void i2cdrvdevUnlockBus(GPIO_TypeDef *portSCL, GPIO_TypeDef *portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
  LL_GPIO_SetOutputPin(portSDA, pinSDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while (LL_GPIO_IsInputPinSet(portSDA, pinSDA) == false)
  {
    /* Set clock high */
    LL_GPIO_SetOutputPin(portSCL, pinSCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    LL_GPIO_ResetOutputPin(portSCL, pinSCL);
    i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
    LL_GPIO_SetOutputPin(portSCL, pinSCL);
    i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  LL_GPIO_SetOutputPin(portSCL, pinSCL);
  i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
  LL_GPIO_ResetOutputPin(portSDA, pinSDA);
  i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
  LL_GPIO_ResetOutputPin(portSCL, pinSCL);
  i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  LL_GPIO_SetOutputPin(portSDA, pinSDA);
  LL_GPIO_SetOutputPin(portSCL, pinSCL);
  GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, 10 * I2CDEV_LOOPS_PER_MS);
}

//-----------------------------------------------------------

void i2cdrvInit(I2cDrv *i2c)
{
  i2cdrvInitBus(i2c);
}

void i2cdrvCreateMessage(I2cMessage *message,
                         uint8_t slaveAddress,
                         I2cDirection direction,
                         uint32_t length,
                         const uint8_t *buffer)
{
  ASSERT_DMA_SAFE(buffer);

  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = false;
  message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = (uint8_t *)buffer;
  message->nbrOfRetries = I2C_MAX_RETRIES;
}

void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                                uint8_t slaveAddress,
                                bool IsInternal16,
                                uint16_t intAddress,
                                I2cDirection direction,
                                uint32_t length,
                                const uint8_t *buffer)
{
  //ASSERT_DMA_SAFE(buffer);

  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = IsInternal16;
  message->internalAddress = intAddress;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = (uint8_t *)buffer;
  message->nbrOfRetries = I2C_MAX_RETRIES;
}

bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message)
{
  bool status = false;

  xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); // Protect message data
  // Copy message
  memcpy((char *)&i2c->txMessage, (char *)message, sizeof(I2cMessage));
  // We can now start the ISR sending this message.
  i2cdrvStartTransfer(i2c);
  // Wait for transaction to be done
  if (xSemaphoreTake(i2c->isBusFreeSemaphore, I2C_MESSAGE_TIMEOUT) == pdTRUE)
  {
    if (i2c->txMessage.status == i2cAck)
    {
      status = true;
    }
  }
  else
  {
    i2cdrvClearDMA(i2c);
    i2cdrvTryToRestartBus(i2c);
    // TODO: If bus is really hanged... fail safe
  }
  xSemaphoreGive(i2c->isBusFreeMutex);

  return status;
}

void i2cdrvEventIsrHandler(I2cDrv *i2c)
{
  // uint16_t SR1;
  // uint16_t SR2;

  // read the status register first
  // SR1 = i2c->def->i2cPort->SR1;

  // Start bit event
  if (LL_I2C_IsActiveFlag_SB(i2c->def->i2cPort))
  {
    i2c->messageIndex = 0;

    if (i2c->txMessage.direction == i2cWrite ||
        i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      LL_I2C_TransmitData8(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1 & I2C_MASTER_WRITE);
    }
    else
    {
      LL_I2C_AcknowledgeNextData(i2c->def->i2cPort, LL_I2C_ACK); // I2C_AcknowledgeConfig(i2c->def->i2cPort, ENABLE);
      LL_I2C_TransmitData8(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1 | I2C_MASTER_READ);
    }
  }
  // Address event
  else if (LL_I2C_IsActiveFlag_ADDR(i2c->def->i2cPort))
  {
    if (i2c->txMessage.direction == i2cWrite ||
        i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      // SR2 = i2c->def->i2cPort->SR2;                               // clear ADDR
      // In write mode transmit is always empty so can send up to two bytes
      if (i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
      {
        if (i2c->txMessage.isInternal16bit)
        {
          LL_I2C_TransmitData8(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0xFF00) >> 8);
          LL_I2C_TransmitData8(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
        }
        else
        {
          LL_I2C_TransmitData8(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
        }
        i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
      }
      LL_I2C_EnableIT_BUF(i2c->def->i2cPort); // allow us to have an EV7
    }
    else // Reading, start DMA transfer
    {
      if (i2c->txMessage.messageLength == 1)
      {
        LL_I2C_AcknowledgeNextData(i2c->def->i2cPort, LL_I2C_NACK); // I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
      }
      else
      {
        LL_I2C_EnableLastDMA(i2c->def->i2cPort); // I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); // No repetitive start
      }
      // Disable buffer I2C interrupts
      LL_I2C_DisableIT_BUF(i2c->def->i2cPort);
      LL_I2C_DisableIT_EVT(i2c->def->i2cPort);
      // Enable the Transfer Complete interrupt
      LL_DMA_IsEnabledIT_TC(i2c->def->dma, i2c->def->dmaRxStream);
      LL_DMA_IsEnabledIT_TE(i2c->def->dma, i2c->def->dmaRxStream);
      LL_I2C_EnableDMAReq_RX(i2c->def->i2cPort); // Enable before ADDR clear

      // Workaround to enable DMA for Renode simulation.
      // The I2C uses the DMA for reading, but the Renode implementation lacks some functionality
      // and for a message to be read the DMA needs to be enabled manually.
      // Without setting this bit the I2C reading fails.
      // With added functionality it should be possible to remove.
      LL_DMA_EnableStream(i2c->def->dma, i2c->def->dmaRxStream); // Workaround

      __DMB();                                  // Make sure instructions (clear address) are in correct order
      LL_I2C_ClearFlag_ADDR(i2c->def->i2cPort); // Clear ADDR flag
    }
  }
  // Byte transfer finished
  else if (LL_I2C_IsActiveFlag_BTF(i2c->def->i2cPort))
  {
    if (LL_I2C_GetTransferDirection(i2c->def->i2cPort) == LL_I2C_DIRECTION_WRITE) // (SR2 & I2C_SR2_TRA) // In write mode?
    {
      if (i2c->txMessage.direction == i2cRead) // internal address read
      {
        /* Internal address written, switch to read */
        LL_I2C_Enable(i2c->def->i2cPort); // i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
        LL_I2C_GenerateStartCondition(i2c->def->i2cPort);
      }
      else
      {
        i2cNotifyClient(i2c);
        // Are there any other messages to transact? If so stop else repeated start.
        i2cTryNextMessage(i2c);
      }
    }
    else // Reading. Shouldn't happen since we use DMA for reading.
    {
      //ASSERT(1);
      i2c->txMessage.buffer[i2c->messageIndex++] = LL_I2C_ReceiveData8(i2c->def->i2cPort);
      if (i2c->messageIndex == i2c->txMessage.messageLength)
      {
        i2cNotifyClient(i2c);
        // Are there any other messages to transact?
        i2cTryNextMessage(i2c);
      }
    }
    // A second BTF interrupt might occur if we don't wait for it to clear.
    // TODO Implement better method.
    while (READ_BIT(i2c->def->i2cPort->CR1, I2C_CR1_START))
    {
      ;
    } // 0x0100
  }
  // Byte received
  else if (LL_I2C_IsActiveFlag_RXNE((i2c->def->i2cPort))) // Should not happen when we use DMA for reception.
  {
    i2c->txMessage.buffer[i2c->messageIndex++] = LL_I2C_ReceiveData8(i2c->def->i2cPort);
    if (i2c->messageIndex == i2c->txMessage.messageLength)
    {
      LL_I2C_DisableIT_BUF(i2c->def->i2cPort);
    }
  }
  // Byte ready to be transmitted
  else if (LL_I2C_IsActiveFlag_TXE(i2c->def->i2cPort))
  {
    if (i2c->txMessage.direction == i2cRead)
    {
      // Disable TXE to flush and get BTF to switch to read.
      // Switch must be done in BTF or strange things happen.
      LL_I2C_DisableIT_BUF(i2c->def->i2cPort);
    }
    else
    {
      LL_I2C_TransmitData8(i2c->def->i2cPort, i2c->txMessage.buffer[i2c->messageIndex++]);
      if (i2c->messageIndex == i2c->txMessage.messageLength)
      {
        // Disable TXE to allow the buffer to flush and get BTF
        LL_I2C_DisableIT_BUF(i2c->def->i2cPort);
        // If an instruction is not here an extra byte gets sent, don't know why...
        // Is is most likely timing issue but STM32F405 I2C peripheral is bugged so
        // this is the best solution so far.
        __DMB();
      }
    }
  }
}

void i2cdrvErrorIsrHandler(I2cDrv *i2c)
{
  if (LL_I2C_IsActiveFlag_AF(i2c->def->i2cPort))
  {
    if (i2c->txMessage.nbrOfRetries-- > 0)
    {
      // Retry by generating start
      LL_I2C_Enable(i2c->def->i2cPort); // i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
      LL_I2C_GenerateStartCondition(i2c->def->i2cPort);
    }
    else
    {
      // Failed so notify client and try next message if any.
      i2c->txMessage.status = i2cNack;
      i2cNotifyClient(i2c);
      i2cTryNextMessage(i2c);
    }
    LL_I2C_ClearFlag_AF(i2c->def->i2cPort);
  }
  if (LL_I2C_IsActiveFlag_BERR(i2c->def->i2cPort))
  {
    LL_I2C_ClearFlag_BERR(i2c->def->i2cPort);
  }
  if (LL_I2C_IsActiveFlag_OVR(i2c->def->i2cPort))
  {
    LL_I2C_ClearFlag_OVR(i2c->def->i2cPort);
  }
  if (LL_I2C_IsActiveFlag_ARLO(i2c->def->i2cPort))
  {
    LL_I2C_ClearFlag_ARLO(i2c->def->i2cPort);
  }
}

static void i2cdrvClearDMA(I2cDrv *i2c)
{
  LL_DMA_DisableStream(i2c->def->dma, i2c->def->dmaRxStream);
  i2c->def->DMA_ClearFlag_TC(i2c->def->dmaRxStream);
  LL_I2C_EnableDMAReq_RX(i2c->def->i2cPort);
  LL_I2C_DisableLastDMA(i2c->def->i2cPort);
  LL_DMA_DisableIT_TC(i2c->def->dma, i2c->def->dmaRxStream);
  LL_DMA_DisableIT_TE(i2c->def->dma, i2c->def->dmaRxStream);
}

void i2cdrvDmaIsrHandler(I2cDrv *i2c)
{
  if (i2c->def->DMA_IsActiveFlag_TC(i2c->def->dma)) // Transfer complete
  {
    i2cdrvClearDMA(i2c);
    i2cNotifyClient(i2c);
    // Are there any other messages to transact?
    i2cTryNextMessage(i2c);
  }
  if (i2c->def->DMA_IsActiveFlag_TE(i2c->def->dma)) // Transfer error
  {
    i2c->def->DMA_ClearFlag_TE(i2c->def->dma);
    // TODO: Best thing we could do?
    i2c->txMessage.status = i2cNack;
    i2cNotifyClient(i2c);
    i2cTryNextMessage(i2c);
  }
}

// void __attribute__((used)) I2C1_ER_IRQHandler(void)
// {
//   i2cdrvErrorIsrHandler(&deckBus);
// }

// void __attribute__((used)) I2C1_EV_IRQHandler(void)
// {
//   i2cdrvEventIsrHandler(&deckBus);
// }

// #ifdef CONFIG_DECK_USD_USE_ALT_PINS_AND_SPI
// void __attribute__((used)) DMA1_Stream5_IRQHandler(void)
// #else
// void __attribute__((used)) DMA1_Stream0_IRQHandler(void)
// #endif
// {
//   i2cdrvDmaIsrHandler(&deckBus);
// }

// void __attribute__((used)) I2C3_ER_IRQHandler(void)
// {
//   i2cdrvErrorIsrHandler(&sensorsBus);
// }

// void __attribute__((used)) I2C3_EV_IRQHandler(void)
// {
//   i2cdrvEventIsrHandler(&sensorsBus);
// }

// void __attribute__((used)) DMA1_Stream2_IRQHandler(void)
// {
//   i2cdrvDmaIsrHandler(&sensorsBus);
// }
