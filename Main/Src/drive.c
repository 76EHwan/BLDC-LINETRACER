/*
 * drive.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "drive.h"

#define MARK_NONE				0x0
#define MARK_LEFT				0x1
#define MARK_RIGHT				0x2
#define MARK_END				0x3
#define MARK_CROSS				0x7

#define MARK_STATE_IDLE			0x0
#define MARK_STATE_READ			0x1
#define MARK_STATE_CROSS		0x2
#define MARK_STATE_DECISION		0x3

#define WINDOW_MARK				0xC000
#define WINDOW_POSITION			0x3FFF
#define WINDOW_ALL				0xFFFF

float target_mps = 0.f;	// target velocity
float curve_decel = 1.f;
float kp = 0.6f;

uint16_t sensor_accum = 0;

void Drive_Start() {
	HAL_LPTIM_Counter_Start_IT(DRIVE_TIM, DRIVE_PSC);
}

void Drive_Stop() {
	HAL_LPTIM_Counter_Stop_IT(DRIVE_TIM);
}

void Drive_LPTIM5_IRQ() {
	float center_mps = target_mps * curve_decel
			/ (curve_decel + fabsf(sensor.position));
	motor[ML].mps = center_mps * (1 + kp * sensor.position);
	motor[MR].mps = center_mps * (1 - kp * sensor.position);
}

__STATIC_INLINE uint8_t Mark_State_Machine(uint8_t *mark_state) {
	sensor_accum |= sensor.state;
	switch (*mark_state) {
	case MARK_STATE_IDLE:
		if (sensor_accum & WINDOW_MARK)
			*mark_state = MARK_STATE_READ;
		if ((sensor_accum & WINDOW_POSITION) == WINDOW_POSITION)
			*mark_state = MARK_STATE_CROSS;
		break;

	case MARK_STATE_READ:
		if (!(sensor.state & WINDOW_MARK))
			*mark_state = MARK_STATE_DECISION;
		break;

	case MARK_STATE_CROSS:
		if (!(sensor.state & WINDOW_MARK))
			*mark_state = MARK_STATE_DECISION;
		break;

	case MARK_STATE_DECISION:
		uint16_t decision_accum = sensor_accum;
		sensor_accum = 0;
		*mark_state = MARK_STATE_IDLE;
		if (decision_accum == WINDOW_ALL)
			return MARK_CROSS;
		else if (decision_accum & WINDOW_MARK)
			return MARK_END;
		else if (decision_accum >> 15)
			return MARK_RIGHT;
		else
			return MARK_LEFT;
	}
	return MARK_NONE;
}

void Drive_First() {
	motor[ML].mps = 0;
	motor[MR].mps = 0;

	Sensor_Start();
	Motor_Start();
	HAL_Delay(10);
	Drive_Start();
	uint8_t mark_state = MARK_STATE_IDLE;
	target_mps = 2.0f;
	while (sensor.state) {
		Custom_LCD_Printf(0, 0, "%f", motor[ML].mps);
		Custom_LCD_Printf(0, 1, "%f", motor[MR].mps);
		Custom_LCD_Printf(0, 2, "%f  ", sensor.position);

		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin,
				(sensor.state >> 14) & 0x1);
		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin,
				(sensor.state >> 15) & 0x1);
		Custom_LCD_Printf(0, 3, "%d", mark_state);

		if((sensor.state >> 14) == 0x3) break;
		uint8_t mark = Mark_State_Machine(&mark_state);
		if (!mark) {
			Custom_LCD_Printf(0, 4, "%d", mark);
		}
	}
	target_mps = 0.f;
	HAL_Delay(1000);
	Drive_Stop();
	Motor_Stop();
	Sensor_Stop();
}

void Drive_Second() {

}

void Drive_Third() {

}

void Drive_Forth() {

}
