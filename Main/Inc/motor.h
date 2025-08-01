/*
 * motor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_MOTOR_H_
#define MAIN_INC_MOTOR_H_

#define ENCODER_L_TIM			&hlptim1
#define ENCODER_R_TIM			&hlptim2
#define MOTOR_PID_TIM			&hlptim4
#define MOTOR_L_TIM				&htim8
#define MOTOR_R_TIM				&htim8

#define MOTOR_L_CHANNEL			TIM_CHANNEL_1
#define MOTOR_R_CHANNEL			TIM_CHANNEL_2

#define MOTOR_PID_PERIOD		0
#define ENCODER_PERIOD			65535

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
