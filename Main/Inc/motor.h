/*
 * motor.h
 *
 *  Created on: 2026. 6. 9.
 *      Author: kth59
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define TIRE_DIAMETER	0.023f
#define INV_TIRE_RADIUS	(2.f / TIRE_DIAMETER)

#define GEAR_RATIO (39.f/11.f)

typedef enum {
    FOC_MODE_NO_SVPWM_SPIN  = 0, // 모터 구동 O, SVPWM 연산 X (단순 PWM/수동 제어)
    FOC_MODE_SVPWM_NO_SPIN  = 1, // 모터 구동 X, SVPWM 연산 O (전류/각도 연산 디버깅용)
    FOC_MODE_SVPWM_SPIN     = 2, // 모터 구동 O, SVPWM 연산 O (정상 FOC 제어)
	FOC_MODE_SPEED_LOOP		= 3, // 모터 구동 O, SVPWM 연산 O (속도 Closed Loop 실행/주행용)
} FOC_DriveMode_t;


void MTR_Setup_And_Start(FOC_DriveMode_t mode);
void MTR_Safe_Stop(void);


void MTR_Read_Register(void);
void MTR_Simple_Control(void);
void MTR_Update_Setup(void);
void MTR_Simple_FOC(void);
void MTR_Encoder_Test(void);
void MTR_Speed_FOC(void);
void MTR_Current_Tune_Loop();
void FAN_Test(void);

#endif /* INC_MOTOR_H_ */
