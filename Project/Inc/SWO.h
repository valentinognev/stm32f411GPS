#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void SWO_PrintChar(char const c, uint8_t const portNumber);
    void SWO_PrintString(char const* s, uint8_t const portNumber);
    void SWO_PrintDefault(char const* str);
    void SWO_PrintDefaultN(char const* str, size_t const len);

    void USART_PrintChar(char const c);
    void USART_PrintString(char const* s);
    void USART_PrintBuffer(char const* str, size_t const len);
    void USART_PrintInt(int32_t num);
    void USART_PrintInt0(int32_t num);
    void USART_PrintHex8(uint16_t num);
    void USART_PrintHex16(uint16_t num);
    void USART_PrintHex32(uint32_t num);
    void USART_PrintBufPrintable(char *buf, uint16_t bufsize, char subst); 
    void USART_PrintBufHex(char *buf, uint16_t bufsize);
    void USART_PrintBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width, char subst);

    void TFT_PrintString(int16_t lineNum, char const *s);

#ifdef __cplusplus
}
#endif
