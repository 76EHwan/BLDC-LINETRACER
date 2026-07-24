/*
 * motor.h
 *
 *  Created on: 2026. 6. 9.
 *      Author: kth59
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "arm_math.h"

#define TIRE_DIAMETER	0.023f
#define INV_TIRE_RADIUS	(2.f / TIRE_DIAMETER)

#define GEAR_RATIO (39.f/11.f)

#define THREAD		0.225f	// 바퀴 끝에서 끝 거리는 22.5cm, 바퀴 중심 거리는 18.6cm
#define THREAD_DIV2	(THREAD / 2.f)

#define RAMP_DT	0.0005f
#define Ramp_TIM_IRQ_Handler TIM14_IRQ_Handler

typedef enum {
    FOC_MODE_NO_SVPWM_SPIN  = 0, // 모터 구동 O, SVPWM 연산 X (단순 PWM/수동 제어)
    FOC_MODE_SVPWM_NO_SPIN  = 1, // 모터 구동 X, SVPWM 연산 O (전류/각도 연산 디버깅용)
    FOC_MODE_SVPWM_SPIN     = 2, // 모터 구동 O, SVPWM 연산 O (정상 FOC 제어)
	FOC_MODE_SPEED_LOOP		= 3, // 모터 구동 O, SVPWM 연산 O (속도 Closed Loop 실행/주행용)
} FOC_DriveMode_t;

extern float_t accel;
extern float_t decel;
extern volatile uint8_t g_is_braking;
extern volatile float g_target_base_mps;
extern volatile float g_current_base_mps;

extern volatile uint16_t buzzer_timer_count;
extern float_t g_buzzer_duration;

void MTR_Setup_And_Start(FOC_DriveMode_t mode);
void MTR_Safe_Stop(void);


void MTR_Read_Register(void);
void MTR_Simple_Control(void);
void MTR_Update_Setup(void);
void MTR_Simple_FOC(void);
void MTR_Encoder_Test(void);
void MTR_Speed_FOC(void);
void MTR_Current_Tune_Loop(void);

void Fan_Mtr_Start(void);
void Fan_Mtr_Set_Duty(uint8_t duty);
void Fan_Mtr_Stop(void);
void Fan_Test(void);
void Magnet_Encoder_Test(void);

void Ramp_Start(void);
void Ramp_Stop(void);

#endif /* INC_MOTOR_H_ */
