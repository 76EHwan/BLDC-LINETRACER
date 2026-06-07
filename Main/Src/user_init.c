/*
 * user_init.c
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */
#include "main.h"
#include "rng.h"
#include "button.h"
#include "buzzer.h"

#include "user_init.h"

void User_Init() {
	Button_init();
	Buzzer_Init();
	Buzzer_Start();
	LCD7789_Test();
	Buzzer_Stop();
}
