#include "SWO.h"
#include "main.h"
#include "GUI.h"
#include "Lcd_Driver.h"
//extern UART_HandleTypeDef huart2;

#define HEX_CHARS      "0123456789ABCDEF"

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

/// @brief Prints a character to the serial port in blocking mode
/// @param c - character to print
void USART_PrintChar(char const c)
{
    while (LL_USART_IsActiveFlag_TXE(SERIAL_USART))
        ;
    LL_USART_TransmitData8(SERIAL_USART, c);
	/* **************************************************** *
	 * The second while loops serves as guarantee that		*
	 * the data is transmitted through the channel.			*
	 * This is to prevent any data corruption.				*
	 * **************************************************** */
	while (!LL_USART_IsActiveFlag_TC(SERIAL_USART));
}

void USART_PrintString(char const* s)
{
	uint16_t len = strlen(s); 
	if (len == 0)
		return;
    for (int i = 0; i < len; i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(SERIAL_USART))
            ;
        LL_USART_TransmitData8(SERIAL_USART, s[i]);
    }
	if (s[len-1] != '\n')
    {
		while (!LL_USART_IsActiveFlag_TXE(SERIAL_USART))
            ;
		LL_USART_TransmitData8(SERIAL_USART, '\n');
	}
}
void USART_PrintBuffer(char const* str, size_t const len)
{
    for (int i = 0; i < len; i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(SERIAL_USART))
            ;
        LL_USART_TransmitData8(SERIAL_USART, str[i]);
    }
}

void USART_PrintInt(int32_t num) 
{
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		USART_PrintChar('-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) USART_PrintChar(str[i]);
}

void USART_PrintInt0(int32_t num) 
{
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		USART_PrintChar('-');
		num *= -1;
	}
	if ((num < 10) && (num >= 0)) USART_PrintChar('0');
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) USART_PrintChar(str[i]);
}

void USART_PrintHex8(uint16_t num) 
{
	USART_PrintChar(HEX_CHARS[(num >> 4)   % 0x10]);
	USART_PrintChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void USART_PrintHex16(uint16_t num) 
{
	uint8_t i;
	for (i = 12; i > 0; i -= 4) USART_PrintChar(HEX_CHARS[(num >> i) % 0x10]);
	USART_PrintChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void USART_PrintHex32(uint32_t num) 
{
	uint8_t i;
	for (i = 28; i > 0; i -= 4)	USART_PrintChar(HEX_CHARS[(num >> i) % 0x10]);
	USART_PrintChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void USART_PrintBufPrintable(char *buf, uint16_t bufsize, char subst) 
{
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		USART_PrintChar(ch > 32 ? ch : subst);
	}
}

void USART_PrintBufHex(char *buf, uint16_t bufsize) 
{
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		USART_PrintChar(HEX_CHARS[(ch >> 4)   % 0x10]);
		USART_PrintChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}

void USART_PrintBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width, char subst) 
{
	uint16_t i = 0,len,pos;
	char buffer[column_width];

	while (i < bufsize) {
		// Line number
		USART_PrintHex16(i);
		USART_PrintChar(':'); USART_PrintChar(' '); // Faster and less code than USART_PrintStr(": ");

		// Copy one line
		if (i+column_width >= bufsize) len = bufsize - i; else len = column_width;
		memcpy(buffer,&buf[i],len);

		// Hex data
		pos = 0;
		while (pos < len) USART_PrintHex8(buffer[pos++]);
		USART_PrintChar(' ');

		// Raw data
		pos = 0;
		do USART_PrintChar(buffer[pos] > 32 ? buffer[pos] : subst); while (++pos < len);
		USART_PrintChar('\n');

		i += len;
	}
}

/// @brief The function prints a string on the TFT screen, starting at line lineNum.
/// @param lineNum - the line number to start printing the string on.
/// @param s - the string to print.
void TFT_PrintString(int16_t lineNum, char const *s)
{
    // This function prints a string on the screen, starting at line lineNum.
    Gui_DrawFont_GBK16(8, (lineNum-1)*16 , BLUE, GRAY0, s);
}
