/*
 * user_init.h
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */

#ifndef USER_INIT_H_
#define USER_INIT_H

#include "st7735_lcd.h"


void User_Init(void);


#define TEST_DISPLAY "/display"
#define LCD_Printf LCD7735_Printf
#define LCD_Clear LCD7735_Clear
#define LCD_Set_Color LCD7735_Set_Color

#define FOC_CONTROL
//#define SENSOR_TRAP_CONTROL

#if defined(FOC_CONTROL) & defined(SENSOR_TRAP_CONTROL)
error "FOC & SENSOT TRAP Both Crashed."
#endif

#if !defined(FOC_CONTROL) & !defined(SENSOR_TRAP_CONTROL)
error "Undefined FOC & SENSOT TRAP."

#endif

#endif /* USER_INIT_H_ */
