#ifndef BSP_ST7789_ST7789_LCD_H
#define BSP_ST7789_ST7789_LCD_H

#include "main.h"
#include "st7789.h"
#include <stdio.h>
#include "ff.h"

#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40
#define BRRED            0XFC07
#define GRAY             0X8430
#define DARKBLUE         0X01CF
#define LIGHTBLUE        0X7D7C
#define GRAYBLUE         0X5458

extern ST7789_Object_t st7789_pObj;

extern uint16_t LCD7789_POINT_COLOR;
extern uint16_t LCD7789_BACK_COLOR;
extern uint16_t LCD7789_BACK_BRIGHT;

void LCD7789_Test(void);
void LCD7789_SetBrightness(uint32_t Brightness);
uint32_t LCD7789_GetBrightness(void);
void LCD7789_SoftPWMEnable(uint8_t enable);
uint8_t LCD7789_SoftPWMIsEnable(void);
void LCD7789_SoftPWMCtrlInit(void);
void LCD7789_SoftPWMCtrlDeInit(void);
void LCD7789_SoftPWMCtrlRun(void);
void LCD7789_Light(uint32_t Brightness_Dis, uint32_t time);
void LCD7789_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode);
void LCD7789_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p);
void LCD7789_Clear(void);
void LCD7789_Printf(uint16_t x, uint16_t y, const char *text, ...);
void LCD7789_Display_Random_BMP_From_SD(const TCHAR *address);
void LCD7789_Set_Color(uint16_t point, uint16_t back);
#endif /* BSP_ST7789_ST7789_LCD_H */
