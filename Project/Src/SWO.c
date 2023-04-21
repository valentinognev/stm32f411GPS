#include "SWO.h"
#include "main.h"
#include "GUI.h"
#include "Lcd_Driver.h"
#include <stdbool.h>
//extern UART_HandleTypeDef huart2;

bool transmitFinished = false;

void SWO_PrintChar(char const c, uint8_t const portNumber)
{
    volatile int timeout;

    /* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
    /* check Trace Control Register if ITM trace is enabled*/
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0)
    {
        return; /* not enabled? */
    }
    /* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
    /* check Trace Enable Register if requested port is enabled */
    if ((ITM->TER & (1ul << portNumber)) == 0)
    {
        return; /* requested port not enabled? */
    }
    timeout = 5000; /* arbitrary timeout value */
    while (ITM->PORT[0].u32 == 0)
    {
        /* Wait until STIMx is ready, then send data */
        if (--timeout == 0)
        {
            return; /* not able to send */
        }
    }
    ITM->PORT[0].u8 = (uint8_t)c;
}

void SWO_PrintString(char const* s, uint8_t const portNumber)
{
    while (*s != '\0')
    {
        SWO_PrintChar(*s++, portNumber);
    }
}

void SWO_PrintDefault(char const* str)
{
    SWO_PrintString(str, 0);
}

void SWO_PrintDefaultN(char const* str, size_t const len)
{
    for (size_t i = 0; i < len; ++i)
    {
        SWO_PrintChar(str[i], 0);
    }
}


void USART_PrintChar(char const c)
{
    while (LL_USART_IsActiveFlag_TXE(USART2))
        ;
    LL_USART_TransmitData8(USART2, c);
}
void USART_PrintString(char const* s)
{
    for (int i = 0; i < strlen(s); i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(USART2))
            ;
        LL_USART_TransmitData8(USART2, s[i]);
    }
}
void USART_PrintDefaultN(char const* str, size_t const len)
{
    for (int i = 0; i < len; i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(USART2))
            ;
        LL_USART_TransmitData8(USART2, str[i]);
    }
}


void USART_PrintStringDMA(char const* s)
{

    for (int i = 0; i < strlen(s); i++)
    {
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, strlen(s)); // Set amount of copied bits for DMA
        LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)s,
                (uint32_t)&(USART1->DR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
        LL_USART_EnableDMAReq_TX(USART1); // Enable DMA in the USART registers
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7); // Enable the DMA transaction

        // wait untill DR empty
        while (transmitFinished)
            ;
    }
}



void TFT_PrintString(int16_t lineNum, char const *s)
{
    Gui_DrawFont_GBK16(8, (lineNum-1)*16 , BLUE, GRAY0, s);
}