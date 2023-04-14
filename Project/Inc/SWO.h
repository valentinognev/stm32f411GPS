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
    void USART_PrintDefaultN(char const* str, size_t const len);
    void TFT_PrintString(int16_t lineNum, char const *s);

#ifdef __cplusplus
}
#endif
