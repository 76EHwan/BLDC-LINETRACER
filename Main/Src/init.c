/*
 * init.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "init.h"

#define CELL 		8				// Li-po 배터리 셀 개수
#define CELL_MIN	(3.f * CELL)	// 셀당 최소 3V
#define CELL_DIF	(1.2 * CELL)	// 셀당 최대 4.2V로 차이는 셀당 1.2V
#define CELL_REF	(3.7 * CELL)	// 셀당 기준 전압 3.7V

#define MOTOR_VOLTAGE	36

void Show_Remain_Battery() {
	uint8_t percent = (uint8_t) (fmin(
			fmaxf((sensor.voltage - CELL_MIN), 0) * 100 / CELL_DIF, 100));
	Custom_LCD_Printf(0, 9, "%3d%%", percent);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 40, 146, 2, 12, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 40, 144, 37, 2, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 40, 158, 37, 2, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 75, 146, 2, 12, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 77, 149, 3, 6, 0xFFFF);

	uint32_t percentColor;
	if (sensor.voltage < CELL_REF)
		percentColor = RED;
	else if (sensor.voltage < MOTOR_VOLTAGE)
		percentColor = YELLOW;
	else
		percentColor = GREEN;

	ST7735_LCD_Driver.FillRect(&st7735_pObj, 43, 147,
			(uint32_t) (31 * percent / 100), 10, percentColor);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 43 + (31 * percent / 100), 147,
			31 - (31 * percent / 100), 10, BLACK);
}

void Setting() {

}

menu_t menu[] = {
		{ "1.Cali    ", Sensor_Calibration}, { "2.D 1st   ",
		Drive_First }, { "3.D 2nd   ", Drive_Second }, { "4.D 3rd   ",
		Drive_Third }, { "5.D 4th   ", Drive_Forth }, { "6.Setting ", Setting },
		{ "7.S Test  ", Sensor_Test_Menu }, { "8.M Test  ", Motor_Test_Menu } };

void Init() {
	//	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	//	__HAL_TIM_SET_AUTORELOAD(&htim16, 500);
	//	__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1, 250);
	Encoder_Start();
	static uint8_t maxMenu = sizeof(menu) / sizeof(menu_t);
	static uint8_t beforeMenu = 0;
	static uint8_t cnt_l = 0;
//	static uint8_t cnt_r = 0;
	while(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET);
	while (1) {
		cnt_l = ((hlptim1.Instance->CNT + 128) / 256 + beforeMenu) % maxMenu;
//		cnt_r = ((hlptim2.Instance->CNT + 1024) / 2048) % 2;
		Custom_LCD_Printf(0, 0, "Main Menu", hlptim1.Instance->CNT);
		for (uint8_t i = 0; i < maxMenu; i++) {
			Set_Color(cnt_l, i);
			Custom_LCD_Printf(0, i + 1, "%s", (menu + i)->name);
		}

		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
//		Show_Remain_Battery();

		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET);
			Custom_LCD_Clear();
			Encoder_Stop();

			MCF8316C_Set_EEPROM();
			MCF8316C_MPET();
//			(menu + cnt_l)->func();

			Encoder_Start();
			Custom_LCD_Clear();
			beforeMenu = cnt_l;
		}
	}
}
