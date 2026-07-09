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
#include "drv8316crq1.h"
#include "mct8316z.h"
#include "lsm6ds3tr-c.h"

#include "user_init.h"
#include "foc.h"

void User_Init() {
	Button_init();
	LSM6DS3_Init();
#ifdef FOC_CONTROL
	MX_DRV8316C_Init();
	FOC_Init_Motor(&foc_L, &htim3, &hadc2, &hlptim2);
	FOC_Init_Motor(&foc_R, &htim4, &hadc1, &hlptim1);

#endif

#ifdef SENSOR_TRAP_CONTROL
	MX_MCT8316Z_Init();
#endif
	Buzzer_Init();
	Buzzer_Start();
	LCD7789_Test();
	Buzzer_Stop();
}
