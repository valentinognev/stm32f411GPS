#ifndef __GUI_H
#define __GUI_H

#include "stdint.h"

//********************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************


//STM32 Core board routine 

// Library function version routine 
//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	



uint16_t LCD_BGR2RGB(uint16_t c);
void Gui_Circle(uint16_t X,uint16_t Y,uint16_t R,uint16_t fc); 
void Gui_DrawLine(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint16_t Color);  
void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t bc);
void Gui_box2(uint16_t x,uint16_t y,uint16_t w,uint16_t h, uint8_t mode);
void DisplayButtonDown(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void Gui_DrawFont_GBK16_l(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s, uint8_t len);
void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s);
void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s);
void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num) ;





#endif

//********************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com	

//********************************************************************************















