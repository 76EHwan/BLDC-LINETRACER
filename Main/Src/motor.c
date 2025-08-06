/*
 * motor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */


#include "motor.h"


void Motor_Init() {
	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL, 0);

	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin, GPIO_PIN_SET);
}

menu_t motorMenu[] = {
		{ "1.M I2CSET", MCF8316C_Set_EEPROM},
		{ "2.M FAULT ", MCF8316C_Get_Fault},
		{ "3.M VOLT  ", MCF8316C_Get_Voltage},
		{ "4.M ENC   ", Motor_Test_Encoder},
		{ "5.M MPET ",  MCF8316C_MPET },
		{ "6.M PI CTL", },
		{ "7.M SPEED ", },
		{ "8.OUT     ", }
};

void Motor_Test_Menu() {
	Encoder_Start();
	static uint8_t maxMenu = sizeof(motorMenu) / sizeof(menu_t);
	static uint8_t beforeMenu = 0;
	while (1) {
		uint32_t cnt = ((hlptim1.Instance->CNT + 128) / 256 + beforeMenu)
				% maxMenu;
		Custom_LCD_Printf(0, 0, "Main Menu", hlptim1.Instance->CNT);
		for (uint8_t i = 0; i < maxMenu; i++) {
			Set_Color(cnt, i);
			Custom_LCD_Printf(0, i + 1, "%s", (motorMenu + i)->name);
		}
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
		//		Show_Remain_Battery();
		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
			while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
				;
			Custom_LCD_Clear();
			Encoder_Stop();
			if (cnt == maxMenu - 1)
				return;
			(motorMenu + cnt)->func();
			Encoder_Start();
			Custom_LCD_Clear();
			beforeMenu = cnt;
		}
	}
}
typedef struct {
	float_t id_ref, iq_ref;
	float_t id, iq;
	float_t vd, vq;
	float_t theta;
	int32_t encD, cmdD;
	float_t curV;
	uint16_t encPst;
} motor_t;

motor_t motor[2];

void Motor_Start() {
	HAL_TIM_PWM_Start(MOTOR_L_TIM, MOTOR_L_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_R_TIM, MOTOR_R_CHANNEL);

	HAL_LPTIM_Counter_Start_IT(MOTOR_PID_TIM, MOTOR_PID_PERIOD);

	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
			GPIO_PIN_RESET);
}

void Encoder_Start() {
	HAL_LPTIM_Encoder_Start(ENCODER_L_TIM, ENCODER_PERIOD);
	HAL_LPTIM_Encoder_Start(ENCODER_R_TIM, ENCODER_PERIOD);
}

void Motor_Stop() {
	HAL_TIM_PWM_Stop(MOTOR_L_TIM, MOTOR_L_CHANNEL);
	HAL_TIM_PWM_Stop(MOTOR_R_TIM, MOTOR_R_CHANNEL);

	HAL_LPTIM_Counter_Stop_IT(MOTOR_PID_TIM);

	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin, GPIO_PIN_SET);
}

void Encoder_Stop() {
	HAL_LPTIM_Encoder_Stop(ENCODER_L_TIM);
	HAL_LPTIM_Encoder_Stop(ENCODER_R_TIM);
}

void Motor_LPTIM4_IRQ() {

}

void Motor_Test_Encoder() {
	Encoder_Start();
	Custom_LCD_Printf(0, 0, "EncoderL");
	Custom_LCD_Printf(0, 2, "EncoderR");
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Custom_LCD_Printf(0, 1, "%5d", *ENCODER_L_TIM.Instance->CNT);
		Custom_LCD_Printf(0, 3, "%5d", *ENCODER_R_TIM.Instance->CNT);
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
	Encoder_Stop();
}

