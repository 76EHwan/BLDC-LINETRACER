/*
 * init.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_INIT_H_
#define MAIN_INC_INIT_H_

#include "lcd.h"
#include "tim.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "lptim.h"
#include "switch.h"
#include "mcf8316c.h"

typedef struct {
	char name[10];
	void (*func) (void);
} menu_t;

void Init();
void Custom_LCD_Clear();
void Custom_LCD_Printf(int x, int y, const char *text, ...);



#endif /* MAIN_INC_INIT_H_ */
