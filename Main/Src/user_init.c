/*
 * user_init.c
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */
#include "main.h"
#include "rng.h"

#include "user_init.h"
#include "st7789_lcd.h"
#include "w25qxx.h"
#include "SDcard.h"
#include "button.h"
#include "buzzer.h"

void User_Init() {
	Button_init();
	Buzzer_Init();

}
