/*
 * user_init.h
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */

#ifndef USER_INIT_H_
#define USER_INIT_H

#include "st7789_lcd.h"

void User_Init(void);

#define TEST_DISPLAY "/display"
#define LCD_Test		LCD7789_Test
#define LCD_Printf		LCD7789_Printf
#define LCD_Clear		LCD7789_Clear
#define LCD_Set_Color 	LCD7789_Set_Color
#define LCD_Sleep_Mode	LCD7789_Sleep

#define FOC_CONTROL
//#define SENSOR_TRAP_CONTROL

#if defined(FOC_CONTROL) & defined(SENSOR_TRAP_CONTROL)
error "FOC & SENSOT TRAP Both Crashed."
#endif

#if !defined(FOC_CONTROL) & !defined(SENSOR_TRAP_CONTROL)
error "Undefined FOC & SENSOT TRAP."

#endif

#endif /* USER_INIT_H_ */
