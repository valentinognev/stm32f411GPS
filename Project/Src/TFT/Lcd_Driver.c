
#include "Lcd_Driver.h"






//********************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************


//STM32 Core board routine 
// Library function version routine 

// This test program uses analog SPI Interface driver 
// Can be free to change the interface IO Configuration ， Use any at least 4 IO You can complete this LCD display display 

/******************************************************************************
 Interface definition Lcd_Driver.h Internal definition ， Please modify and modify the corresponding according to the wiring IO initialization LCD_GPIO_Init()

//  ----------------------------------------------------------------
// VCC   catch 5V or 3.3v power supply 
// GND   Power supply 
// DI    catch PB15（SDI）
// SC    catch PB13（SCL）
// CS    catch PB12  Chip Select     
// RST   catch PB14  System reset 
// RS    catch PB1   Order / data   
// BLK   catch PB0  Backlight control     
// ----------------------------------------------------------------

//	#define LCD_SDI        	//PB15
//	#define LCD_SCL        	//PB13
//	#define LCD_CS        	//PB12
//	#define LCD_RST     	//PB14 
//	#define LCD_RS         	//PB1
//	#define LCD_BLK         //PB0    	
*******************************************************************************/

/**************************************************************************************

 Function name : void LCD_GPIO_Init(void)

 Function description :  liquid crystal IO Initialization configuration 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void LCD_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();					          // Open PB clock 
	

    GPIO_Initure.Pin=GPIO_PIN_0 |GPIO_PIN_1 | GPIO_PIN_12| GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15;	//PB
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		    // Push output 
    GPIO_Initure.Pull=GPIO_PULLUP;         			    // pull up 
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	// high speed 
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     	     	// initialization 
	  
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	  //PB0 Set 1 	
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	  //PB1 Set 1 
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);	//PB12 Set 1 
 	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);	//PB13 Set 1 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	//PB14 Set 1 
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);	//PB15 Set 1 
			
		LCD_BLK_SET;// Backlight 
		LCD_BLK_CLR;// Turn off the backlight 
		LCD_BLK_SET;// Backlight 
      
}

/**************************************************************************************

 Function name : void  SPI_WriteData(uint8_t Data)

 Function description :  Towards SPI A bus transmission one 8 Position data 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void  SPI_WriteData(uint8_t Data)
{
  	unsigned char i=0;
		for(i=8;i>0;i--)
		{
			if(Data&0x80)	
			LCD_SDA_SET; // Output Data 
				else LCD_SDA_CLR;
			 
				LCD_SCL_CLR;       
				LCD_SCL_SET;
				Data<<=1; 
		}
}





/**************************************************************************************

 Function name : void  SPI_WriteData(uint8_t Data)

 Function description :  Write a LCD screen 8 Bit instruction 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void Lcd_WriteIndex(uint8_t Index)
{
   //SPI  Write the order of the command to start 
   LCD_CS_CLR;
   LCD_RS_CLR;
	 SPI_WriteData(Index);
   LCD_CS_SET;
}


/**************************************************************************************

 Function name : void Lcd_WriteData(uint8_t Data)

 Function description :  Write a LCD screen 8 Position data 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void Lcd_WriteData(uint8_t Data)
{
   LCD_CS_CLR;
   LCD_RS_SET;
   SPI_WriteData(Data);
   LCD_CS_SET; 
}



/**************************************************************************************

 Function name : void LCD_WriteData_16Bit(uint16_t Data)

 Function description :  Write a LCD screen 16 Position data 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void LCD_WriteData_16Bit(uint16_t Data)
{
   LCD_CS_CLR;
   LCD_RS_SET;
	 SPI_WriteData(Data>>8); 	// Write high 8 Position data 
	 SPI_WriteData(Data); 			// Write down 8 Position data 
   LCD_CS_SET; 
}


/**************************************************************************************

 Function name : void Lcd_WriteReg(uint8_t Index,uint8_t Data)

 Function description :  Write LCD screen register 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/


void Lcd_WriteReg(uint8_t Index,uint8_t Data)
{
	Lcd_WriteIndex(Index);
  Lcd_WriteData(Data);
}


/**************************************************************************************

 Function name : void Lcd_Reset(void)

 Function description :  hardware IO Control the LCD screen ， If you use the screen of the font version version ， This function is invalid 
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void Lcd_Reset(void)
{
  LCD_RST_CLR;//RST High pins output 
	LL_mDelay(300);
  LCD_RST_SET;//RST Point output is low 
	LL_mDelay(100);
}



/**************************************************************************************

 Function name : void Lcd_Init(void)

 Function description : 1.44 inch   LCD screen initialization function ， LCD screen controller is ：ST7735R.
 lose      enter : 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void Lcd_Init(void)
{	
	LCD_GPIO_Init();
	Lcd_Reset(); //Reset before LCD Init.

	//LCD Init For 1.44Inch LCD Panel with ST7735R.
	Lcd_WriteIndex(0x11);//Sleep exit 
	LL_mDelay (120);
		
	//ST7735R Frame Rate
	Lcd_WriteIndex(0xB1); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB2); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB3); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	
	Lcd_WriteIndex(0xB4); //Column inversion 
	Lcd_WriteData(0x07); 
	
	//ST7735R Power Sequence
	Lcd_WriteIndex(0xC0); 
	Lcd_WriteData(0xA2); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x84); 
	Lcd_WriteIndex(0xC1); 
	Lcd_WriteData(0xC5); 

	Lcd_WriteIndex(0xC2); 
	Lcd_WriteData(0x0A); 
	Lcd_WriteData(0x00); 

	Lcd_WriteIndex(0xC3); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0x2A); 
	Lcd_WriteIndex(0xC4); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0xEE); 
	
	Lcd_WriteIndex(0xC5); //VCOM 
	Lcd_WriteData(0x0E); 
	
	Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
	Lcd_WriteData(0xC8); 
	
	//ST7735R Gamma Sequence
	Lcd_WriteIndex(0xe0); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1a); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x18); 
	Lcd_WriteData(0x2f); 
	Lcd_WriteData(0x28); 
	Lcd_WriteData(0x20); 
	Lcd_WriteData(0x22); 
	Lcd_WriteData(0x1f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x23); 
	Lcd_WriteData(0x37); 
	Lcd_WriteData(0x00); 	
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x10); 

	Lcd_WriteIndex(0xe1); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x17); 
	Lcd_WriteData(0x33); 
	Lcd_WriteData(0x2c); 
	Lcd_WriteData(0x29); 
	Lcd_WriteData(0x2e); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x39); 
	Lcd_WriteData(0x3f); 
	Lcd_WriteData(0x00); 
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x03); 
	Lcd_WriteData(0x10);  
	
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x7f);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x9f);
	
	Lcd_WriteIndex(0xF0); //Enable test command  
	Lcd_WriteData(0x01); 
	Lcd_WriteIndex(0xF6); //Disable ram power save mode 
	Lcd_WriteData(0x00); 
	
	Lcd_WriteIndex(0x3A); //65k mode 
	Lcd_WriteData(0x05); 
	
	
	Lcd_WriteIndex(0x29);//Display on	 
}


/***************************************************************************************

 Function name ：LCD_Set_Region
 Function ： set up lcd Display area ， Write some data in this area to automatically change the line 
 Entry parameter ：xy Starting point and end point 
 return value ： none 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{		
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_start+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_end+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_start+3);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_end+3);
	
	Lcd_WriteIndex(0x2c);

}

/***************************************************************************************

 Function name ：LCD_Set_XY
 Function ： set up lcd Show the starting point 
 Entry parameter ：xy coordinate 
 return value ： none 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/
void Lcd_SetXY(uint16_t x,uint16_t y)
{
  	Lcd_SetRegion(x,y,x,y);
}

	
/***************************************************************************************

 Function name ：LCD_DrawPoint
 Function ： Draw a point 
 Entry parameter ： none 
 return value ： none 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Data);

}    


/***************************************************************************************

 Function name ：Lcd_Clear
 Function ： Full screen clear screen function 
 Entry parameter ： Fill color COLOR
 return value ： none 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/
void Lcd_Clear(uint16_t Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	  	LCD_WriteData_16Bit(Color);
    }   
}





































/**************************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com	

**************************************************************************************/

