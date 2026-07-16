/*
 * drive.c
 *
 *  Created on: 2026. 7. 2.
 *      Author: kth59
 */

#include "main.h"
#include "math.h"

#include "button.h"
#include "foc.h"

#include "user_init.h"
#include "motor.h"
#include "drive.h"

// @formatter:off
DriveParam_t driveData = {
		.base_mps = 1.5f,
		.max_mps = 6.f,
		.steer_gain = 0.75,
		.pos_atten_gain = 0.25f,
};
// @formatter:on

__STATIC_INLINE void Ramp_Omega(FOC_Handle_t *hfoc) {
	float d = hfoc->omega_setpoint - hfoc->target_omega;
	const float_t accel_step = driveData.accel * SPD_DT;
	const float_t decel_step = driveData.decel * SPD_DT;

	if (d > accel_step)
		hfoc->target_omega += accel_step;
	else if (d < -decel_step)
		hfoc->target_omega -= decel_step;
	else
		hfoc->target_omega = hfoc->omega_setpoint;
}

__STATIC_INLINE void MTR_Set_Speed(float mps_L, float mps_R) {
	foc_L.omega_setpoint = mps_L * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;
	foc_R.omega_setpoint = -mps_R * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;
}

// ============================================================================
// 주행거리 적산 (Odometry) - 마커 간 거리 저장용
//   FOC 측정 전기각속도(omega_e_meas) -> 바퀴 선속도(mps) 역변환 후,
//   메인 루프 dt(ms)만큼 적분하여 누적 거리를 구한다.
//   부호는 방향 구분 없이 주행 "거리"만 필요하므로 절대값을 사용한다.
// ============================================================================
static float g_odom_distance_m = 0.f;

CrossMarkerLog_t g_cross_log[CROSS_LOG_MAX];
uint8_t g_cross_log_count = 0;

__STATIC_INLINE float FOC_Meas_Mps(FOC_Handle_t *hfoc) {
	return fabsf(hfoc->omega_e_meas)
			/ (INV_TIRE_RADIUS * MOTOR_POLE_PAIRS * GEAR_RATIO);
}

__STATIC_INLINE void Odom_Reset(void) {
	g_odom_distance_m = 0.f;
}

__STATIC_INLINE void Odom_Accumulate(uint32_t dt_ms) {
	float mps = 0.5f * (FOC_Meas_Mps(&foc_L) + FOC_Meas_Mps(&foc_R));
	g_odom_distance_m += mps * ((float) dt_ms * 0.001f);
}

__STATIC_INLINE void Cross_Log_Push(CrossEvent_t type) {
	CrossMarkerLog_t *log = &g_cross_log[g_cross_log_count % CROSS_LOG_MAX];
	log->type = type;
	log->dist_from_prev_m = g_odom_distance_m;
	g_cross_log_count++;
	Odom_Reset();   // 다음 마커까지 새로 누적 시작
}

// 위치 창(POS_WINDOW) 바깥쪽 센서에서 검출되는 cross_left/cross_right 후보 플래그를 사용한다.
// (idx16/17 전용 마커 포토인터럽터는 예비용이라 사용하지 않는다.)
// 좌/우가 동시에 켜지면 STOP(정지 지점)으로 본다.
// 에지(rising) 검출로 한 마커당 1회만 이벤트가 발생하도록 한다.
static CrossEvent_t Cross_Detect_Update(void) {
	static uint8_t prev_left = 0, prev_right = 0;

	uint8_t left = IR_Sensor.data->cross_left;
	uint8_t right = IR_Sensor.data->cross_right;

	CrossEvent_t event = CROSS_NONE;

	if (left && right) {
		if (!(prev_left && prev_right)) {
			event = CROSS_STOP;
		}
	} else if (left && !prev_left) {
		event = CROSS_LEFT;
	} else if (right && !prev_right) {
		event = CROSS_RIGHT;
	}

	prev_left = left;
	prev_right = right;

	if (event != CROSS_NONE) {
		Cross_Log_Push(event);
	}

	return event;
}

void Ramp_TIM_IRQ_Handler() {
	Ramp_Omega(&foc_L);
	Ramp_Omega(&foc_R);
}

void Ramp_Start() {
	foc_L.omega_setpoint = 0.f;
	foc_R.omega_setpoint = 0.f;
	driveData.mpsL = 0;
	driveData.mpsR = 0;
	HAL_TIM_Base_Start_IT(RAMP_TIM);
}

void Ramp_Stop() {
	driveData.mpsL = 0;
	driveData.mpsR = 0;
	foc_L.omega_setpoint = 0.f;
	foc_R.omega_setpoint = 0.f;
	HAL_TIM_Base_Stop_IT(RAMP_TIM);
}

void Line_Follow_Drive(void) {

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

	float_t g_base_mps = driveData.base_mps;
	float_t g_steer_gain = driveData.steer_gain;
//	float_t g_max_mps = driveData.max_mps;
	float_t g_pos_atten_gain = driveData.pos_atten_gain;

	Sensor_Start();
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();

	HAL_Delay(10);

	MTR_Set_Speed(g_base_mps, g_base_mps);

	Odom_Reset();
	g_cross_log_count = 0;

	uint32_t last_tick = HAL_GetTick();

	while (!IR_Sensor.is_lost_position) {
		uint32_t now_tick = HAL_GetTick();
		Odom_Accumulate(now_tick - last_tick);
		last_tick = now_tick;

		CrossEvent_t cross = Cross_Detect_Update();
		if (cross == CROSS_STOP) {
			// 좌/우 마커 동시 검출 -> 교차로 정지 지점. 주행 종료.
			static uint8_t count = 0;
			if (count)
				break;
			count++;
		}

		float line_pos = IR_Sensor.data->line_position;
		float line_pos_abs = fabsf(line_pos);
		float steer = g_steer_gain * line_pos;

		float mps_L = g_base_mps * (1 + steer)
				* (1 - line_pos_abs * g_pos_atten_gain);
		float mps_R = g_base_mps * (1 - steer)
				* (1 - line_pos_abs * g_pos_atten_gain);

		MTR_Set_Speed(mps_L, mps_R);

		if (g_cross_log_count > 0) {
			CrossMarkerLog_t *last = &g_cross_log[(g_cross_log_count - 1)
					% CROSS_LOG_MAX];
			const char *tag = (last->type == CROSS_LEFT) ? "L" :
								(last->type == CROSS_RIGHT) ? "R" : "?";
			LCD_Printf(0, 7, "cross:%s d:%5.2f", tag, last->dist_from_prev_m);
		}
	}
	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	LCD_Clear();
	return;
}
