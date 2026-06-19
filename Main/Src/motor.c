/*
 * motor.c
 *
 *  Created on: 2026. 6. 9.
 *      Author: kth59
 */

#include "user_init.h"
#include "button.h"
#include "motor.h"

#ifdef FOC_CONTROL
#include "drv8316crq1.h"

#define MTR_L DRV8316C_L
#define MTR_R DRV8316C_R

#define MTR_ReadRegister DRV8316C_ReadRegister
#define MTR_REG_IC_STATUS DRV_REG_IC_STATUS
#define MTR_REG_STATUS_1 DRV_REG_STATUS_1
#define MTR_REG_STATUS_2 DRV_REG_STATUS_2
#define MTR_REG_CTRL_2 DRV_REG_CTRL_2
#define MTR_REG_CTRL_2 DRV_REG_CTRL_2
#define MTR_REG_CTRL_3 DRV_REG_CTRL_3
#define MTR_REG_CTRL_4 DRV_REG_CTRL_4
#define MTR_REG_CTRL_5 DRV_REG_CTRL_5
#define MTR_REG_CTRL_6 DRV_REG_CTRL_6
#define MTR_REG_CTRL_7 DRV_REG_CTRL_7
#define MTR_REG_CTRL_8 DRV_REG_CTRL_8
#define MTR_REG_CTRL_9 DRV_REG_CTRL_9
#endif

#ifdef SENSOR_TRAP_CONTROL
#include "mct8316z.h"

#define MTR_L MCT8316Z_L
#define MTR_R MCT8316Z_R

#define MTR_ReadRegister MCT8316Z_ReadRegister
#define MTR_REG_IC_STATUS MCT_REG_IC_STATUS
#define MTR_REG_STATUS_1 MCT_REG_STATUS_1
#define MTR_REG_STATUS_2 MCT_REG_STATUS_2
#define MTR_REG_CTRL_2 MCT_REG_CTRL_2
#define MTR_REG_CTRL_3 MCT_REG_CTRL_3
#define MTR_REG_CTRL_4 MCT_REG_CTRL_4
#define MTR_REG_CTRL_5 MCT_REG_CTRL_5
#define MTR_REG_CTRL_6 MCT_REG_CTRL_6
#define MTR_REG_CTRL_7 MCT_REG_CTRL_7
#define MTR_REG_CTRL_8 MCT_REG_CTRL_8
#define MTR_REG_CTRL_9 MCT_REG_CTRL_9

#endif

void MTR_Start() {
#ifdef FOC_CONTROL
	DRV8316C_FOC_PWM_EN();

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

#endif
#ifdef SENSOR_TRAP_CONTROL
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
#endif
}

void MTR_Stop() {
#ifdef FOC_CONTROL
	DRV8316C_FOC_PWM_EN();

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

#endif
#ifdef SENSOR_TRAP_CONTROL
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
	MCT8316Z_SLEEP(&MTR_L);
	MCT8316Z_SLEEP(&MTR_R);
#endif
}

void MTR_Read_Register() {
	uint8_t lcd_left_x_bias = 5;
	uint8_t lcd_right_x_bias = 8;
	LCD_Printf(lcd_left_x_bias, 0, "Left");
	LCD_Printf(lcd_right_x_bias, 0, "Right");

	LCD_Printf(0, 1, "IC:");
	LCD_Printf(0, 2, "ST1");
	LCD_Printf(0, 3, "ST2");
	LCD_Printf(0, 4, "CTR2");
	LCD_Printf(0, 5, "CTR3");
	LCD_Printf(0, 6, "CTR4");
	LCD_Printf(0, 7, "CTR5");
	LCD_Printf(0, 8, "CTR6");
	LCD_Printf(0, 9, "CTR7");
	LCD_Printf(0, 10, "CTR8");
	LCD_Printf(0, 11, "CTR9");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		uint8_t mtr_L_pData;
		uint8_t mtr_R_pData;

		MTR_ReadRegister(&MTR_L, MTR_REG_IC_STATUS, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 2, "%02x", mtr_L_pData);
		MTR_ReadRegister(&MTR_L, MTR_REG_STATUS_1, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 3, "%02x", mtr_L_pData);
		MTR_ReadRegister(&MTR_L, MTR_REG_STATUS_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 4, "%02x", mtr_L_pData);
		MTR_ReadRegister(&MTR_L, MTR_REG_CTRL_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 5, "%02x", mtr_L_pData);

		MTR_ReadRegister(&MTR_R, MTR_REG_IC_STATUS, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 2, "%02x", mtr_R_pData);
		MTR_ReadRegister(&MTR_R, MTR_REG_STATUS_1, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 3, "%02x", mtr_R_pData);
		MTR_ReadRegister(&MTR_R, MTR_REG_STATUS_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 4, "%02x", mtr_R_pData);
		MTR_ReadRegister(&MTR_R, MTR_REG_CTRL_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 5, "%02x", mtr_R_pData);
	}
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}

void MTR_Simple_Control() {
	UserInput_t bt;
#ifdef FOC_CONTROL
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		if (bt == INPUT_CMD_L_HOLD) {

		}
		if (bt == INPUT_CMD_R_HOLD) {

		}
		if (bt == INPUT_CMD_U_HOLD) {

		}
	}
#endif
#ifdef SENSOR_TRAP_CONTROL
	uint16_t duty = 2000;
	MTR_Start();
	for (uint16_t i = 0; i < 1000; i = i + 5) {
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2 * i);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 2 * i);
		HAL_Delay(1);
	}
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 0, "%5d", duty);
		if (bt == INPUT_CMD_L_HOLD) {
			duty = duty - ((duty < 1000) ? 0 : 20);
		}
		if (bt == INPUT_CMD_R_HOLD) {
			duty = duty + ((duty > 8000) ? 0 : 20);
		}
		if (bt == INPUT_CMD_U_HOLD) {
			HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin, GPIO_PIN_SET);
		}
		if (bt == INPUT_CMD_D_HOLD) {

			HAL_GPIO_WritePin(MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin, GPIO_PIN_RESET);

			MCT8316Z_ClearFaults(&MTR_L);
			HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin,
					GPIO_PIN_RESET);
			MCT8316Z_ClearFaults(&MTR_R);
			HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin,
					GPIO_PIN_RESET);
		}
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, duty);
	}
#endif
	Button_Wait_Release(&btn_k);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);

}
