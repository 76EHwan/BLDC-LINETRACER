#ifndef BSP_ST7735_ST7735_LCD_H
#define BSP_ST7735_ST7735_LCD_H

#include "main.h"
#include "st7735.h"
#include <stdio.h>
#include "SDcard.h"

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07
#define GRAY  			 0X8430
#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C
#define GRAYBLUE       	 0X5458

extern ST7735_Object_t st7735_pObj;
extern uint32_t st7735_id;

extern uint16_t LCD7735_POINT_COLOR;
extern uint16_t LCD7735_BACK_COLOR;
extern uint16_t LCD7735_BACK_BRIGHT;

void LCD7735_Test(void);
void LCD7735_SetBrightness(uint32_t Brightness);
uint32_t LCD7735_GetBrightness(void);
void LCD7735_SoftPWMEnable(uint8_t enable);
uint8_t LCD7735_SoftPWMIsEnable(void);
void LCD7735_SoftPWMCtrlInit(void);
void LCD7735_SoftPWMCtrlDeInit(void);
void LCD7735_SoftPWMCtrlRun(void);
void LCD7735_Light(uint32_t Brightness_Dis, uint32_t time);
void LCD7735_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode);
void LCD7735_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p);
void LCD7735_Clear(void);
void LCD7735_Printf(uint8_t x, uint8_t y, const char *text, ...);
void LCD7735_Display_Random_BMP_From_SD(const TCHAR *address);

#endif /* __ST7735_LCD_H */
