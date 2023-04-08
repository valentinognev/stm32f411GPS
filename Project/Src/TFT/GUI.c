

#include "Lcd_Driver.h"

#include "GUI.h"
#include "Font.h"


//********************************************************************************

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************

// When we write RGB Format 。
// Converted through this function 
//c:GBR Format color value 
// return value ：RGB Format color value 
uint16_t LCD_BGR2RGB(uint16_t c)
{
  uint16_t  r,g,b,rgb;   
  b=(c>>0)&0x1f;
  g=(c>>5)&0x3f;
  r=(c>>11)&0x1f;	 
  rgb=(b<<11)+(g<<5)+(r<<0);		 
  return(rgb);

}


//********************************************************************************

// Function name : void Gui_Circle(uint16_t X,uint16_t Y,uint16_t R,uint16_t fc) 

// Function function : Draw a circular function 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************

void Gui_Circle(uint16_t X,uint16_t Y,uint16_t R,uint16_t fc) 
{//Bresenham algorithm  
    unsigned short  a,b; 
    int c; 
    a=0; 
    b=R; 
    c=3-2*R; 
    while (a<b) 
    { 
        Gui_DrawPoint(X+a,Y+b,fc);     //        7 
        Gui_DrawPoint(X-a,Y+b,fc);     //        6 
        Gui_DrawPoint(X+a,Y-b,fc);     //        2 
        Gui_DrawPoint(X-a,Y-b,fc);     //        3 
        Gui_DrawPoint(X+b,Y+a,fc);     //        8 
        Gui_DrawPoint(X-b,Y+a,fc);     //        5 
        Gui_DrawPoint(X+b,Y-a,fc);     //        1 
        Gui_DrawPoint(X-b,Y-a,fc);     //        4 

        if(c<0) c=c+4*a+6; 
        else 
        { 
            c=c+4*(a-b)+10; 
            b-=1; 
        } 
       a+=1; 
    } 
    if (a==b) 
    { 
        Gui_DrawPoint(X+a,Y+b,fc); 
        Gui_DrawPoint(X+a,Y+b,fc); 
        Gui_DrawPoint(X+a,Y-b,fc); 
        Gui_DrawPoint(X-a,Y-b,fc); 
        Gui_DrawPoint(X+b,Y+a,fc); 
        Gui_DrawPoint(X-b,Y+a,fc); 
        Gui_DrawPoint(X+b,Y-a,fc); 
        Gui_DrawPoint(X-b,Y-a,fc); 
    } 
	
} 

//********************************************************************************

// Function name : void Gui_DrawLine(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint16_t Color) 

// Function function : Draw line 

// Drawing function ， use Bresenham  Drawing algorithm 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************

void Gui_DrawLine(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint16_t Color)   
{
int dx,             // difference in x's
    dy,             // difference in y's
    dx2,            // dx,dy * 2
    dy2, 
    x_inc,          // amount in pixel space to move during drawing
    y_inc,          // amount in pixel space to move during drawing
    error,          // the discriminant i.e. error i.e. decision variable
    index;          // used for looping	


	Lcd_SetXY(x0,y0);
	dx = x1-x0;// calculate x distance 
	dy = y1-y0;// calculate y distance 

	if (dx>=0)
	{
		x_inc = 1;
	}
	else
	{
		x_inc = -1;
		dx    = -dx;  
	} 
	
	if (dy>=0)
	{
		y_inc = 1;
	} 
	else
	{
		y_inc = -1;
		dy    = -dy; 
	} 

	dx2 = dx << 1;
	dy2 = dy << 1;

	if (dx > dy)//x More than y distance ， Then x There is only one point on the shaft ， Each y There are several points on the axis 
	{// The points of the line are equal to x distance ， by x Increased drawing point 
		// initialize error term
		error = dy2 - dx; 

		// draw the line
		for (index=0; index <= dx; index++)// The points you want to draw will not exceed more x distance 
		{
			// Draw point 
			Gui_DrawPoint(x0,y0,Color);
			
			// test if error has overflowed
			if (error >= 0) // Do you need to increase y Coordinate 
			{
				error-=dx2;

				// move to next line
				y0+=y_inc;// Increase y Coordinate 
			} // end if error overflowed

			// adjust the error term
			error+=dy2;

			// move to the next pixel
			x0+=x_inc;//x The coordinate value increases after each drawing point 1
		} // end for
	} // end if |slope| <= 1
	else//y Axis is greater than x axis ， Each y There is only one point on the shaft ，x Several points axis 
	{// by y The axis is an increasing drawing point 
		// initialize error term
		error = dx2 - dy; 

		// draw the line
		for (index=0; index <= dy; index++)
		{
			// set the pixel
			Gui_DrawPoint(x0,y0,Color);

			// test if error overflowed
			if (error >= 0)
			{
				error-=dy2;

				// move to next line
				x0+=x_inc;
			} // end if error overflowed

			// adjust the error term
			error+=dx2;

			// move to the next pixel
			y0+=y_inc;
		} // end for
	} // end else |slope| > 1
}


//********************************************************************************

// Function name : void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t bc)

// Function function : Draw the box button 

// Drawing function ， use Bresenham  Drawing algorithm 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************

void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t bc)
{
	Gui_DrawLine(x,y,x+w,y,0xEF7D);
	Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
	Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
	Gui_DrawLine(x,y,x,y+h,0xEF7D);
    Gui_DrawLine(x+1,y+1,x+1+w-2,y+1+h-2,bc);
}

//********************************************************************************

// Function name : void Gui_box2(uint16_t x,uint16_t y,uint16_t w,uint16_t h, uint8_t mode)

// Function function : Draw the box button  2

// Drawing function ， use Bresenham  Drawing algorithm 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

//********************************************************************************

void Gui_box2(uint16_t x,uint16_t y,uint16_t w,uint16_t h, uint8_t mode)
{
	if (mode==0)	{
		Gui_DrawLine(x,y,x+w,y,0xEF7D);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
		Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
		Gui_DrawLine(x,y,x,y+h,0xEF7D);
		}
	if (mode==1)	{
		Gui_DrawLine(x,y,x+w,y,0x2965);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xEF7D);
		Gui_DrawLine(x,y+h,x+w,y+h,0xEF7D);
		Gui_DrawLine(x,y,x,y+h,0x2965);
	}
	if (mode==2)	{
		Gui_DrawLine(x,y,x+w,y,0xffff);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xffff);
		Gui_DrawLine(x,y+h,x+w,y+h,0xffff);
		Gui_DrawLine(x,y,x,y+h,0xffff);
	}
}


/**************************************************************************************

 Function name : void DisplayButtonDown(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)

 Function description :  Display a raised button box on the screen 
 lose      enter : uint16_t x1,y1,x2,y2  In the upper left and lower right corner coordinates of the button frame 
 lose      out :  none 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/
void DisplayButtonDown(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	Gui_DrawLine(x1,  y1,  x2,y1, GRAY2);  //H
	Gui_DrawLine(x1+1,y1+1,x2,y1+1, GRAY1);  //H
	Gui_DrawLine(x1,  y1,  x1,y2, GRAY2);  //V
	Gui_DrawLine(x1+1,y1+1,x1+1,y2, GRAY1);  //V
	Gui_DrawLine(x1,  y2,  x2,y2, WHITE);  //H
	Gui_DrawLine(x2,  y1,  x2,y2, WHITE);  //V
}

/**************************************************************************************

 Function name : void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)

 Function description :  Display the button box under the screen 
 lose      enter : uint16_t x1,y1,x2,y2  In the upper left and lower right corner coordinates of the button frame 
 lose      out :  none 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/
void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	Gui_DrawLine(x1,  y1,  x2,y1, WHITE); //H
	Gui_DrawLine(x1,  y1,  x1,y2, WHITE); //V
	
	Gui_DrawLine(x1+1,y2-1,x2,y2-1, GRAY1);  //H
	Gui_DrawLine(x1,  y2,  x2,y2, GRAY2);  //H
	Gui_DrawLine(x2-1,y1+1,x2-1,y2, GRAY1);  //V
    Gui_DrawLine(x2  ,y1  ,x2,y2, GRAY2); //V
}

/**************************************************************************************

 Function name : void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)

 Function description :  Display the point line is 16x16  String 
 lose      enter : x,y, Show the starting point position ，  fc： font color   bc： background color    *s： String to be displayed 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/
void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
	Gui_DrawFont_GBK16_l(x, y, fc, bc, s, strlen(s));
}

void Gui_DrawFont_GBK16_l(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s, uint8_t len)
{
	unsigned char i,j;
	unsigned short k,x0;
	x0=x;
    int count=0;

	while(*s) 
	{	
		count ++;
		if (count > len)
			break;
		if((*s) < 128) 
		{
			k=*s;
			if (k==13) 
			{
				x=x0;
				y+=16;
			}
			else 
			{
				if (k>32) k-=32; else k=0;
	
			    for(i=0;i<16;i++)
				for(j=0;j<8;j++) 
					{
				    	if(asc16[k*16+i]&(0x80>>j))	
						    Gui_DrawPoint(x+j,y+i,fc);
						else 
						{
							if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
						}
					}
				x+=8;
			}
			s++;
		}
			
		else 
		{
		

			for (k=0;k<hz16_num;k++) 
			{
			  if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<16;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
								else {
									if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
								}
							}
				    }
				}
			  }
			s+=2;x+=16;
		} 
		
	}
}


/**************************************************************************************

 Function name : void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)

 Function description :  Display the point line is 24x24  String 
 lose      enter : x,y, Show the starting point position ，  fc： font color   bc： background color    *s： String to be displayed 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/

void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
	unsigned char i,j;
	unsigned short k;

	while(*s) 
	{
		if( *s < 0x80 ) 
		{
			k=*s;
			if (k>32) k-=32; else k=0;

		    for(i=0;i<16;i++)
			for(j=0;j<8;j++) 
				{
			    	if(asc16[k*16+i]&(0x80>>j))	
					Gui_DrawPoint(x+j,y+i,fc);
					else 
					{
						if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
					}
				}
			s++;x+=8;
		}
		else 
		{

			for (k=0;k<hz24_num;k++) 
			{
			  if ((hz24[k].Index[0]==*(s))&&(hz24[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<24;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3]&(0x80>>j))
								Gui_DrawPoint(x+j,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
								else {
									if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+2]&(0x80>>j))	
								Gui_DrawPoint(x+j+16,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j+16,y+i,bc);
								}
							}
				    }
			  }
			}
			s+=2;x+=24;
		}
	}
}



/**************************************************************************************

 Function name : void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num)

 Function description :  Display the point line is 32x32  Number 
 lose      enter : x,y, Show the starting point position ，  fc： font color   bc： background color    num： Numbers to be displayed 
 lose      out : 

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
// Taobao shop ：shop389957290.taobao.com	

**************************************************************************************/


void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num)
{
	unsigned char i,j,k,c;
	//lcd_text_any(x+94+i*42,y+34,32,32,0x7E8,0x0,sz32,knum[i]);
//	w=w/8;

    for(i=0;i<32;i++)
	{
		for(j=0;j<4;j++) 
		{
			c=*(sz32+num*32*4+i*4+j);
			for (k=0;k<8;k++)	
			{
	
		    	if(c&(0x80>>k))	Gui_DrawPoint(x+j*8+k,y+i,fc);
				else {
					if (fc!=bc) Gui_DrawPoint(x+j*8+k,y+i,bc);
				}
			}
		}
	}
}




//**************************************************************************************/

//DevEBox   Dayue Electronics （ Embedded Development Network ）
// Taobao shop ：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com	

//**************************************************************************************/









