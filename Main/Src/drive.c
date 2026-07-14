/*
 * drive.c
 *
 *  Created on: 2026. 7. 2.
 *      Author: kth59
 */

#include "main.h"

#include "button.h"
#include "foc.h"

#include "user_init.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"

DriveParam_t driveDataL;
DriveParam_t driveDataR;

static float g_base_mps = 1.f;   // 기준 직진 속도 (m/s)
static float g_steer_gain = 0.75f;   // 라인 위치 -> 좌우 속도차 게인
static float g_max_mps = 3.f;   // 좌/우 바퀴 속도 상한

__STATIC_INLINE void Ramp_Omega(FOC_Handle_t *hfoc) {
	float d = hfoc->omega_setpoint - hfoc->target_omega;
	float step = hfoc->omega_ramp_rate * SPD_DT;
	if (d > step)
		hfoc->target_omega += step;
	else if (d < -step)
		hfoc->target_omega -= step;
	else
		hfoc->target_omega = hfoc->omega_setpoint;
}

__STATIC_INLINE void MTR_Set_Speed(float mps_L, float mps_R) {
	foc_L.omega_setpoint = mps_L * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;
	foc_R.omega_setpoint = -mps_R * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;
}

void Ramp_TIM_IRQ_Handler() {
	Ramp_Omega(&foc_L);
	Ramp_Omega(&foc_R);
}

void Ramp_Start() {
	foc_L.omega_setpoint = 0.f;
	foc_R.omega_setpoint = 0.f;
	driveDataL.mps = 0;
	driveDataR.mps = 0;
	HAL_TIM_Base_Start_IT(RAMP_TIM);
}

void Ramp_Stop() {
	driveDataL.mps = 0;
	driveDataR.mps = 0;
	foc_L.omega_setpoint = 0.f;
	foc_R.omega_setpoint = 0.f;
	HAL_TIM_Base_Stop_IT(RAMP_TIM);
}

void Line_Follow_Drive(void) {
	UserInput_t bt = INPUT_CMD_NONE;
	if (!IR_Sensor.is_calibration) {
		FRESULT res = Sensor_Load_Calibration();
		if (res == FR_OK) {
			IR_Sensor.is_calibration = 1;
		} else {
			LCD_Printf(0, 0, "Fail: %d", res);
			HAL_Delay(1000);
			return;

		}
	}

	Sensor_Start();
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();

	HAL_Delay(10);

	uint8_t sel = 0;
	const float step_base = 0.05f;
	const float step_steer = 0.05f;
	MTR_Set_Speed(g_base_mps, g_base_mps);

	while (IR_Sensor.data.state & 0xFFFF) {
		bt = Button_Get_Input();

		switch (bt) {
		case INPUT_CMD_K_SINGLE:
			sel = (sel + 1) % 2;
			break;

		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			if (sel == 0) {
				g_base_mps += step_base;
				if (g_base_mps > g_max_mps)
					g_base_mps = g_max_mps;
			} else {
				g_steer_gain += step_steer;
			}
			break;

		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			if (sel == 0) {
				g_base_mps -= step_base;
				if (g_base_mps < 0.0f)
					g_base_mps = 0.0f;
			} else {
				g_steer_gain -= step_steer;
				if (g_steer_gain < 0.0f)
					g_steer_gain = 0.0f;
			}
			break;

		default:
			break;
		}

		float line_pos = IR_Sensor.data.line_position;
		float steer = g_steer_gain * line_pos;

		float mps_L = g_base_mps * (1 + steer);
		float mps_R = g_base_mps * (1 - steer);

		if (mps_L > g_max_mps)
			mps_L = g_max_mps;
		if (mps_L < -g_max_mps)
			mps_L = -g_max_mps;
		if (mps_R > g_max_mps)
			mps_R = g_max_mps;
		if (mps_R < -g_max_mps)
			mps_R = -g_max_mps;

		// 4. Ramp 목표 속도 갱신 (실제 반영은 Ramp_TIM_IRQ_Handler에서 처리)
		MTR_Set_Speed(mps_L, mps_R);

		LCD_Printf(0, 0, "%cBase:%5.2f", sel == 0 ? '>' : ' ', g_base_mps);
		LCD_Printf(0, 1, "%cSteer:%5.2f", sel == 1 ? '>' : ' ', g_steer_gain);
		LCD_Printf(0, 2, "line:%6.3f", line_pos);
		LCD_Printf(0, 3, "mL:%5.2f", mps_L);
		LCD_Printf(0, 4, "mL:%5.2f", mps_R);
		LCD_Printf(0, 5, "wL:%6.1f", foc_L.omega_e_meas);
		LCD_Printf(0, 6, "wL:%6.1f", foc_L.omega_e_meas);
	}

	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	LCD_Clear();
	return;
}
