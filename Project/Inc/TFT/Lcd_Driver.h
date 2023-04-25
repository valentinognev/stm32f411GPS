#ifndef __Lcd_Driver_H
#define __Lcd_Driver_H

#include "stdint.h"
#include "main.h"

#define X_MAX_PIXEL            64
#define Y_MAX_PIXEL            128


//********************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

//********************************************************************************


//STM32 Core board routine 

// Library function version routine 



#define RED        0xf800
#define GREEN      0x07e0
#define BLUE       0x001f
#define WHITE      0xffff
#define BLACK      0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D       // grey 0 
#define GRAY1   0x8410    // grey 1 
#define GRAY2   0x4208    // grey 2 




// LCD control mouth 1 Definition of operating statement macro 
#define    LCD_SDA_SET      LL_GPIO_SetOutputPin(LCD_SDI_GPIO_Port,LCD_SDI_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)        //PB15 Set 1 
#define    LCD_SCL_SET      LL_GPIO_SetOutputPin(LCD_SCL_GPIO_Port,LCD_SCL_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET)         //PB13 Set 1 
#define    TFT_CS_SET       LL_GPIO_SetOutputPin(TFT_CS_GPIO_Port,TFT_CS_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)         //PB12 Set 1  
#define    TFT_RST_SET      LL_GPIO_SetOutputPin(TFT_RST_GPIO_Port,TFT_RST_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)       //PB14 Set 1 
#define    TFT_RS_SET       LL_GPIO_SetOutputPin(TFT_RS_GPIO_Port,TFT_RS_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET)           //PB1 Set 1  
#define    LCD_BLK_SET      LL_GPIO_SetOutputPin(LCD_BLK_GPIO_Port,LCD_BLK_Pin)   //AL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)          //PB0 Set 1 

// LCD control mouth 0 Definition of operating statement macro 
// LCD control mouth 1 Definition of operating statement macro 
#define    LCD_SDA_CLR      LL_GPIO_ResetOutputPin(LCD_SDI_GPIO_Port,LCD_SDI_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)        //PB15 Set 1 
#define    LCD_SCL_CLR      LL_GPIO_ResetOutputPin(LCD_SCL_GPIO_Port,LCD_SCL_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET)         //PB13 Set 1 
#define    TFT_CS_CLR       LL_GPIO_ResetOutputPin(TFT_CS_GPIO_Port,TFT_CS_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)         //PB12 Set 1  
#define    TFT_RST_CLR      LL_GPIO_ResetOutputPin(TFT_RST_GPIO_Port,TFT_RST_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)       //PB14 Set 1 
#define    TFT_RS_CLR       LL_GPIO_ResetOutputPin(TFT_RS_GPIO_Port,TFT_RS_Pin)   //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)           //PB1 Set 1  
#define    LCD_BLK_CLR      LL_GPIO_ResetOutputPin(LCD_BLK_GPIO_Port,LCD_BLK_Pin)   //AL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)          //PB0 Set 1 


void LCD_GPIO_Init(void);
void Lcd_WriteIndex(uint8_t Index);
void Lcd_WriteData(uint8_t Data);
void Lcd_WriteReg(uint8_t Index,uint8_t Data);
uint16_t Lcd_ReadReg(uint8_t LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(uint16_t Color);
void Lcd_SetXY(uint16_t x,uint16_t y);
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data);
void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end);
void LCD_WriteData_16Bit(uint16_t Data);

#endif



//********************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com    

//********************************************************************************













