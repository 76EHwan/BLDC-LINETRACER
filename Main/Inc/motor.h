/*
 * motor.h
 *
 *  Created on: 2026. 6. 9.
 *      Author: kth59
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

typedef enum {
    FOC_MODE_NO_SVPWM_SPIN  = 0, // 모터 구동 O, SVPWM 연산 X (단순 PWM/수동 제어)
    FOC_MODE_SVPWM_NO_SPIN  = 1, // 모터 구동 X, SVPWM 연산 O (전류/각도 연산 디버깅용)
    FOC_MODE_SVPWM_SPIN     = 2  // 모터 구동 O, SVPWM 연산 O (정상 FOC 제어)
} FOC_DriveMode_t;

void MTR_Read_Register(void);
void MTR_Simple_Control(void);
void MTR_Update_Setup(void);
void MTR_Simple_FOC(void);
void MTR_Encoder_Test(void);
void MTR_Speed_FOC(void);
void MTR_Current_Tune_Loop();

#endif /* INC_MOTOR_H_ */
