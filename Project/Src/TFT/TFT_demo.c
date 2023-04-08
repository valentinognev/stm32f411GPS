

/**************************************************************************************/

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/


/* Includes ------------------------------------------------------------------*/


#include "Lcd_Driver.h"
#include "GUI.h"
#include "Picture.h"
#include "TFT_demo.h"

unsigned char Num[10]={0,1,2,3,4,5,6,7,8,9};



/**************************************************************************************/

// Demonstration routine menu 


//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/


void Redraw_Mainmenu(void)
{

    Lcd_Clear(GRAY0);
    
    Gui_DrawFont_GBK16(8,0,BLUE,GRAY0,"STM32 Electronic technology ");
    // Gui_DrawFont_GBK16(16,20,RED,GRAY0," LCD test program ");

    // DisplayButtonUp(15,38,113,58); //x1,y1,x2,y2
    // Gui_DrawFont_GBK16(16,40,GREEN,GRAY0," Color filling test ");

    // DisplayButtonUp(15,68,113,88); //x1,y1,x2,y2
    // Gui_DrawFont_GBK16(16,70,BLUE,GRAY0," Text display test ");

    // DisplayButtonUp(15,98,113,118); //x1,y1,x2,y2
    // Gui_DrawFont_GBK16(16,100,RED,GRAY0," Picture display test ");
    // LL_mDelay(1500);
}


/**************************************************************************************/

// Digital test display 


//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/

void Num_Test(void)
{
    uint8_t i=0;
    Lcd_Clear(GRAY0);
    Gui_DrawFont_GBK16(16,20,RED,GRAY0,"Num Test");
    LL_mDelay(1000);
    Lcd_Clear(GRAY0);

    for(i=0;i<10;i++)
    {
    Gui_DrawFont_Num32((i%3)*40,32*(i/3)+5,RED,GRAY0,Num[i+1]);
    LL_mDelay(100);
    }
    
}


/**************************************************************************************/

// Chinese character test show 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/

void Font_Test(void)
{
    Lcd_Clear(GRAY0);
    Gui_DrawFont_GBK16(16,10,BLUE,GRAY0," Text display test ");

    LL_mDelay(1000);
    Lcd_Clear(GRAY0);
    Gui_DrawFont_GBK16(8,8,BLACK,GRAY0,"STM32 Electronic technology ");
    Gui_DrawFont_GBK16(16,28,GREEN,GRAY0," Focus on LCD wholesale ");
    Gui_DrawFont_GBK16(16,48,RED,GRAY0, " Full technical support ");
    Gui_DrawFont_GBK16(0,68,BLUE,GRAY0," Tel:1234567890");
    Gui_DrawFont_GBK16(0,88,RED,GRAY0, " mcudev.taobao");    
    LL_mDelay(1800);    
}


/**************************************************************************************/

// Color filling test 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/

void Color_Test(void)
{
    uint8_t i=1;
    Lcd_Clear(GRAY0);
    
    Gui_DrawFont_GBK16(20,10,BLUE,GRAY0,"Color Test");
    LL_mDelay(500);

    while(i--)
    {
        Lcd_Clear(WHITE);
        LL_mDelay(500);
        Lcd_Clear(BLACK);
        LL_mDelay(500);
        Lcd_Clear(RED);
        LL_mDelay(500);
      Lcd_Clear(GREEN);
        LL_mDelay(500);
      Lcd_Clear(BLUE);
        LL_mDelay(500);
    }        
}


/**************************************************************************************

// Picture display test 

// Mold method   Horizontal scanning   From left to right   Low the front 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

**************************************************************************************/


void showimage(const unsigned char *p) // show 40*40 QQ picture 
{
      int i,j,k; 
    unsigned char picH,picL;
    Lcd_Clear(WHITE); // Clear the screen   
    
    for(k=0;k<3;k++)
    {
           for(j=0;j<3;j++)
        {    
            Lcd_SetRegion(40*j,40*k,40*j+39,40*k+39);        // Coordinate settings 
            for(i=0;i<40*40;i++)
             {    
                 picL=*(p+i*2);    // Low data 
                picH=*(p+i*2+1);                
                LCD_WriteData_16Bit(picH<<8|picL);                          
             }    
         }
    }        
}


/**************************************************************************************/
// Full screen display picture 

// Mold method   Horizontal scanning   From left to right   Low the front 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/



void Fullscreen_showimage(const unsigned char *p) // show 128*128  picture 
{
  int i; 
    unsigned char picH,picL;
    
    Lcd_Clear(WHITE); // Clear the screen   
    
    
            Lcd_SetRegion(0,0,127,127);        // Coordinate settings : Scan the starting point to the end 0 arrive 127， just 128 One point 
            for(i=0;i<128*128;i++)
                 {    
                    picL=*(p+i*2);    // Low data 
                    picH=*(p+i*2+1);                
                    LCD_WriteData_16Bit(picH<<8|picL);                          
                 }    
        
}

/**************************************************************************************/

// LCD screen display routine 


//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com    

/**************************************************************************************/

void Test_Demo(void)
{

    Redraw_Mainmenu();// Draw the main menu ( Some content may not be displayed because the resolution exceeds the physical value )
    
    // Color_Test();// Simple pure color fill test 
    
    // Num_Test();// Digital tube font test 
    
    // Font_Test();// Chinese and English display test         
    
    // showimage(gImage_qq);// Image display example : The larger the picture of the example of the example ， Will occupy more FLASH space ， You can appropriately reduce the number of pictures display according to the situation 
    
    // LL_mDelay(1500);
    
    // Fullscreen_showimage(gImage_XHR128);// Image display example : The larger the picture of the example of the example ， Will occupy more FLASH space ， You can appropriately reduce the number of pictures display according to the situation 
    // LL_mDelay(1500);
    // Fullscreen_showimage(gImage_XNH128);// Image display example 
    // LL_mDelay(1500);
    // Fullscreen_showimage(gImage_ATM128);// Image display example 
    // LL_mDelay(1500);
    
    
}




/**************************************************************************************/

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com    

/**************************************************************************************/
