/*
 * motor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_MOTOR_H_
#define MAIN_INC_MOTOR_H_

#define ENCODER_L_TIM			&hlptim2
#define ENCODER_R_TIM			&hlptim1
#define MOTOR_PID_TIM			&hlptim4
#define MOTOR_L_TIM				&htim8
#define MOTOR_R_TIM				&htim8

#define MOTOR_L_CHANNEL			TIM_CHANNEL_1
#define MOTOR_R_CHANNEL			TIM_CHANNEL_2

#define ML						0
#define MR						1

#define MOTOR_PID_PERIOD		0
#define ENCODER_PERIOD			65535

#define SPUR_GEAR				39.f
#define PINION_GEAR				11.f

#define WHEEL_DIAMETER			0.0215f

#define METER_PER_WHEEL			(WHEEL_DIAMETER * M_PI)

#define WHEEL_PER_ROTOR			(PINION_GEAR / SPUR_GEAR)

#define METER_PER_ROTOR			(METER_PER_WHEEL * WHEEL_PER_ROTOR)

#define ROTOR_PER_TICK			(1.f/2048.f)

#define METER_PER_TICK			(METER_PER_WHEEL * WHEEL_PER_ROTOR * ROTOR_PER_TICK)



#define dt	0.0005f




#include <stdbool.h>
#include "init.h"
#include "sensor.h"
#include "tim.h"
#include "lptim.h"
#include "math.h"
#include "lcd.h"
#include "mcf8316c.h"

typedef struct {
	float_t mps;
	float_t rps;
} motor_t;

extern uint16_t encoder_tick_past_l;
extern uint16_t encoder_tick_past_r;
extern int32_t encoder_tick_err_l;
extern int32_t encoder_tick_err_r;
extern float_t encoder_rotor_per_sec_l;
extern float_t encoder_rotor_per_sec_r;
extern float_t err_p_l;
extern float_t err_p_r;
extern float_t err_i_l;
extern float_t err_i_r;
extern float_t gain_p;
extern float_t gain_i;
extern float_t iq_ref_l;
extern float_t iq_ref_r;
extern uint32_t duty_l;
extern uint32_t duty_r;

extern motor_t motor[2];

void Motor_Init();


void Motor_Test_Menu(void);

void Motor_Start();
void Encoder_Start();

void Motor_Stop();
void Encoder_Stop();

void Motor_LPTIM4_IRQ();

void Motor_Test_Encoder();

void Motor_Test_Kp();

void Motor_Test_Speed();



#endif /* MAIN_INC_MOTOR_H_ */
