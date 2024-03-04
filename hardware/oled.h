#ifndef __OLED_H
#define __OLED_H			  	 
#include "gpio.h"

//Oled port macro definition
//OLED端口宏定义
#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET)   //RST
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET)     //RST

#define OLED_RS_Clr()  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET)   //DC
#define OLED_RS_Set()  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET)   //DC

#define OLED_SCLK_Clr()  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET)   //SCL
#define OLED_SCLK_Set()  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET)     //SCL

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET)   //SDA
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET)     //SDA
#define OLED_CMD  0	//Command //写命令
#define OLED_DATA 1	//Data //写数据

//Oled control function
//OLED控制用函数
void OLED_WR_Byte(unsigned char dat,unsigned char cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(unsigned char x,unsigned char y,unsigned char t);
void OLED_ShowChar(unsigned char x,unsigned char y,unsigned char chr,unsigned char size,unsigned char mode);
void OLED_ShowNumber(unsigned char x,unsigned char y,unsigned int num,unsigned char len,unsigned char size);
void OLED_ShowString(unsigned char x,unsigned char y,const unsigned char *p);

#define CNSizeWidth  16
#define CNSizeHeight 16
//extern char Hzk16[][16];
void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char no,unsigned char font_width,unsigned char font_height);	
void OLED_Set_Pos(unsigned char x, unsigned char y);
#endif  
	 
