/*
 * motor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "motor.h"

motor_t motor[2];

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

menu_t motorMenu[] = { { "1.M I2CSET", MCF8316C_Set_EEPROM }, { "2.M FAULT ",
		MCF8316C_Get_Fault }, { "3.M VOLT  ", MCF8316C_Get_Voltage }, {
		"4.M ENC   ", Motor_Test_Encoder }, { "5.M MPET ", MCF8316C_MPET }, {
		"6.M PI CTL", MCF8316C_PID_CONTROL }, { "7.M SPEED ", MCF8316C_SPEED },
		{ "8.OUT     ", } };

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

void Motor_Start() {
	HAL_GPIO_WritePin(Motor_L_Dir_GPIO_Port, Motor_L_Dir_Pin, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(MOTOR_L_TIM, MOTOR_L_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_R_TIM, MOTOR_R_CHANNEL);

	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL, 0);

	HAL_LPTIM_Counter_Start_IT(MOTOR_PID_TIM, MOTOR_PID_PERIOD);

	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
			GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin,
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

	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL, 0);

	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin, GPIO_PIN_SET);
}

void Encoder_Stop() {
	HAL_LPTIM_Encoder_Stop(ENCODER_L_TIM);
	HAL_LPTIM_Encoder_Stop(ENCODER_R_TIM);
}

uint16_t encoder_tick_past_l = 0;
uint16_t encoder_tick_past_r = 0;
int32_t encoder_tick_err_l = 0;
int32_t encoder_tick_err_r = 0;
float_t encoder_rotor_per_sec_l = 0;
float_t encoder_rotor_per_sec_r = 0;
float_t err_p_l = 0;
float_t err_p_r = 0;
float_t err_i_l = 0;
float_t err_i_r = 0;
float_t gain_p = 0.01f;
float_t gain_i = 0;
float_t iq_ref_l = 0;
float_t iq_ref_r = 0;
uint32_t duty_l = 0;
uint32_t duty_r = 0;

void Motor_LPTIM4_IRQ() {
	motor[ML].rps = motor[ML].mps / METER_PER_ROTOR;
	motor[MR].rps = motor[MR].mps / METER_PER_ROTOR;

	uint32_t dutyL = fabsf(motor[ML].rps)
			* *(MOTOR_L_TIM.Instance->ARR)/ MAX_SPEED_VALUE;

	uint32_t dutyR = (fabsf)(motor[MR].rps)
			* *(MOTOR_R_TIM.Instance->ARR)/ MAX_SPEED_VALUE;

	HAL_GPIO_WritePin(Motor_L_Dir_GPIO_Port, Motor_L_Dir_Pin,
			(motor[ML].mps > 0) ? 1 : 0);
	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, dutyL);

	HAL_GPIO_WritePin(Motor_R_Dir_GPIO_Port, Motor_R_Dir_Pin,
			(motor[MR].mps > 0) ? 0 : 1);
	__HAL_TIM_SET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL, dutyR);


//	uint16_t encoder_tick_present_l = __HAL_TIM_GET_COUNTER(ENCODER_L_TIM);
//	uint16_t encoder_tick_present_r = __HAL_TIM_GET_COUNTER(ENCODER_R_TIM);
//
//	encoder_tick_err_l = (int32_t) (encoder_tick_present_l) - (int32_t) (encoder_tick_past_l);
//	encoder_tick_err_r = (int32_t) (encoder_tick_present_r) - (int32_t) (encoder_tick_past_r);
//
//	encoder_tick_past_l = encoder_tick_present_l;
//	encoder_tick_past_r = encoder_tick_present_r;
//
//	encoder_rotor_per_sec_l = encoder_tick_err_l;
//	encoder_rotor_per_sec_r = encoder_tick_err_r;
//
//	err_p_l = encoder_rotor_per_sec_l - motor[ML].rps;
//	err_p_r = encoder_rotor_per_sec_r - motor[MR].rps;
//
//	err_i_l += err_p_l * dt;
//	err_i_r += err_p_r * dt;
//
//	iq_ref_l = -(err_p_l * gain_p + err_i_l * gain_i);
//	iq_ref_r = -(err_p_r * gain_p + err_i_r * gain_i);
//
//	HAL_GPIO_WritePin(Motor_L_Dir_GPIO_Port, Motor_L_Dir_Pin, (iq_ref_l > 0.f));
//	HAL_GPIO_WritePin(Motor_L_Dir_GPIO_Port, Motor_L_Dir_Pin, (iq_ref_r < 0.f));
//
//	duty_l = (uint32_t) (fabsf(iq_ref_l) / 3.f * __HAL_TIM_GET_AUTORELOAD(MOTOR_L_TIM));
//	duty_r = (uint32_t) (fabsf(iq_ref_r) / 3.f * __HAL_TIM_GET_AUTORELOAD(MOTOR_R_TIM));
//
//	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, duty_l);
//	__HAL_TIM_SET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL, duty_r);
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

