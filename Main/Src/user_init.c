/*
 * user_init.c
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */
#include "main.h"
#include "rng.h"

#include "drv8316crq1.h"
#include "button.h"
#include "buzzer.h"
#include "mct8316z.h"
#include "lsm6ds3tr-c.h"
#include "SDcard.h"

#include "user_init.h"
#include "menu.h"
#include "foc.h"

void User_Init() {
	Button_init();
	Buzzer_Init();
	Buzzer_Start();
	LCD7789_Test();
	LSM6DS3_Init();
	Buzzer_Stop();
	MX_DRV8316C_Init();

	FOC_Init_Motor(&foc_L, &htim3, &hadc2, &hlptim2);
	FOC_Init_Motor(&foc_R, &htim4, &hadc1, &hlptim1);

	FOC_Calibrate_Offset(&foc_L);
	FOC_Calibrate_Offset(&foc_R);

//	SDCard_DebugTest();

	FRESULT res;
	if ((res = SDCard_Test()) != FR_OK) {
		LCD_Printf(0, 6, "SDcard Fail");
		while (1) {
			LED_Blink(250);
		}
	}

	if ((res = FOC_Parameters_InitOrLoad()) != FR_OK) {
		LCD_Printf(0, 7, "FOC param save Fail");
	}

	HAL_Delay(2500);

}
