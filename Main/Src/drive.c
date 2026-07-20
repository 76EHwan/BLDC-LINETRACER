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

#define RAMP_DT	0.0005f

uint32_t count_irq = 0;
uint32_t count_polling = 0;

// @formatter:off
DriveParam_t driveData = {
		.base_mps = 1.5f,
		.max_mps = 6.f,
		.accel = 6.f,
		.decel = 6.f,
		.steer_gain_p = 0.9f,
		.steer_gain_d = 0.15f,
		.pos_atten_gain = 0.5f,
		.fan_en = 0,
};

typedef enum {
	MARKER_STATE_IDLE = 0,
	MARKER_STATE_READING,
} MarkerState_t;

// @formatter:on

// ============================================================================
// 중심 속도 제어용 전역 변수
// ============================================================================
static volatile float g_target_base_mps = 0.0f;
static volatile float g_current_base_mps = 0.0f;
static volatile float g_current_steer = 0.0f;

// ============================================================================
// 주행거리 적산 (Odometry) - 마커 간 거리 저장용
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
	Odom_Reset();
}

// ============================================================================
// 마커(십자/좌/우) 감지
// ============================================================================
static MarkerState_t g_marker_state = MARKER_STATE_IDLE;
static uint8_t g_accum_left = 0;
static uint8_t g_accum_right = 0;
static uint16_t g_accum_center_state = 0;

__STATIC_INLINE void Cross_Detect_Reset(void) {
	g_marker_state = MARKER_STATE_IDLE;
	g_accum_left = 0;
	g_accum_right = 0;
	g_accum_center_state = 0;
}

static CrossEvent_t Cross_Detect_Update(void) {
	uint8_t left_marker = IR_Sensor.data->mark_left;
	uint8_t right_marker = IR_Sensor.data->mark_right;
	uint16_t current_center_state = (uint16_t) (IR_Sensor.data->state & 0xFFFF);

	CrossEvent_t event = CROSS_NONE;

	switch (g_marker_state) {
	case MARKER_STATE_IDLE:
		if (left_marker || right_marker) {
			g_marker_state = MARKER_STATE_READING;
			g_accum_left = left_marker;
			g_accum_right = right_marker;
			g_accum_center_state = current_center_state;
		}
		break;

	case MARKER_STATE_READING:
		if (left_marker) g_accum_left = 1;
		if (right_marker) g_accum_right = 1;
		g_accum_center_state |= current_center_state;

		if (!left_marker && !right_marker) {
			if (g_accum_left && g_accum_right) {
				uint8_t center_on_count = 0;
				for (int i = 0; i < 16; i++) {
					if (g_accum_center_state & (1 << i)) {
						center_on_count++;
					}
				}
				if (center_on_count >= 12) {
					event = CROSS_CROSS;
				} else {
					event = CROSS_STOP;
				}
			} else if (g_accum_left) {
				event = CROSS_LEFT;
			} else if (g_accum_right) {
				event = CROSS_RIGHT;
			}

			if (event != CROSS_NONE && g_cross_log_count > 0) {
				CrossMarkerLog_t *prev = &g_cross_log[(g_cross_log_count - 1) % CROSS_LOG_MAX];
				if (prev->type == event) {
					event = CROSS_NONE;
				}
			}

			Cross_Detect_Reset();
		}
		break;
	}

	if (event != CROSS_NONE) {
		Cross_Log_Push(event);
	}

	return event;
}

// ============================================================================
// 모터 제어 및 Ramp 로직
// ============================================================================
void Ramp_TIM_IRQ_Handler() {
	// 1. 중심 속도(Center Speed) 가감속(Ramp) 계산
	float d_mps = g_target_base_mps - g_current_base_mps;
	float accel_step = driveData.accel * RAMP_DT;
	float decel_step = driveData.decel * RAMP_DT;

	if (d_mps > accel_step) {
		g_current_base_mps += accel_step;
	} else if (d_mps < -decel_step) {
		g_current_base_mps -= decel_step;
	} else {
		g_current_base_mps = g_target_base_mps;
	}

	// 2. 부드럽게 변하는 중심 속도에 조향(Steer)을 즉시 적용
	float mps_L = g_current_base_mps * (1.f + g_current_steer);
	float mps_R = g_current_base_mps * (1.f - g_current_steer);

	// 3. FOC 목표 속도로 변환하여 즉시 갱신 (내부 Ramp 무시)
	foc_L.target_omega = mps_L * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS * GEAR_RATIO;
	foc_R.target_omega = -mps_R * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS * GEAR_RATIO;

	foc_L.omega_setpoint = foc_L.target_omega;
	foc_R.omega_setpoint = foc_R.target_omega;

	count_irq++;
}

void Ramp_Start() {
	g_target_base_mps = 0.f;
	g_current_base_mps = 0.f;
	g_current_steer = 0.f;

	foc_L.omega_setpoint = 0.f;
	foc_L.target_omega = 0.f;
	foc_R.omega_setpoint = 0.f;
	foc_R.target_omega = 0.f;

	driveData.mpsL = 0;
	driveData.mpsR = 0;

	HAL_TIM_Base_Start_IT(RAMP_TIM);
}

void Ramp_Stop() {
	g_target_base_mps = 0.f;
	g_current_base_mps = 0.f;
	g_current_steer = 0.f;

	driveData.mpsL = 0;
	driveData.mpsR = 0;

	foc_L.omega_setpoint = 0.f;
	foc_L.target_omega = 0.f;
	foc_R.omega_setpoint = 0.f;
	foc_R.target_omega = 0.f;

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
	count_sensor_irq = 0;
	count_irq = 0;
	count_polling = 0;

	float_t g_base_mps = driveData.base_mps;
	float_t g_steer_gain_p = driveData.steer_gain_p;
	float_t g_steer_gain_d = driveData.steer_gain_d;
	float_t g_pos_atten_gain = driveData.pos_atten_gain;

	float_t prev_line_pos = 0.0f;
	uint8_t end_count = 0;
	float_t filtered_atten = 1.0f;

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		HAL_Delay(2000);
	}
	Odom_Reset();
	g_cross_log_count = 0;

	Cross_Detect_Reset();

	Sensor_Start();
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);

	Ramp_Start();

	HAL_Delay(100);
	Sensor_Get_Position();

	// 초기 타겟 설정
	g_target_base_mps = g_base_mps;
	g_current_steer = 0.0f;

	while (!IR_Sensor.is_lost_position) {
		float_t line_pos = Sensor_Get_Position();
		float_t line_pos_abs = fabsf(line_pos);

		CrossEvent_t cross = Cross_Detect_Update();

		if (cross == CROSS_STOP) {
			if (end_count != 0)
				break;
			else
				end_count++;
		} else if (cross != CROSS_NONE) {
			CrossMarkerLog_t *last = &g_cross_log[(g_cross_log_count - 1)
					% CROSS_LOG_MAX];
			const char *tag = (last->type == CROSS_LEFT) ? "LEFT " :
								(last->type == CROSS_RIGHT) ? "RIGHT" :
								(last->type == CROSS_CROSS) ? "CROSS" : "UNK";

			LCD_Printf(0, 7, "M:%s #%d d:%4.2f", tag, g_cross_log_count,
					last->dist_from_prev_m);
		}

		float_t d_line_pos = line_pos - prev_line_pos;
		prev_line_pos = line_pos;

		float_t steer = (g_steer_gain_p * line_pos)
				+ (g_steer_gain_d * d_line_pos);

		// 1. 현재 센서 위치 기반의 목표 감속 비율
		float_t target_atten = 1.f - (line_pos_abs * g_pos_atten_gain);

		// 2. 비대칭 Low Pass Filter 적용
		if (target_atten < filtered_atten) {
			filtered_atten = (0.5f * target_atten) + (0.5f * filtered_atten);
		} else {
			filtered_atten = (0.02f * target_atten) + (0.98f * filtered_atten);
		}

		// 3. 필터링된 목표 중심 속도 및 조향값 업데이트
		// Ramp_TIM_IRQ_Handler 에서 실시간으로 가져가서 계산 및 반영합니다.
		g_target_base_mps = g_base_mps * filtered_atten;
		g_current_steer = steer;
	}

	uint16_t last_normalized[NUM_SENSORS];
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		last_normalized[i] = IR_Sensor.data->normalized[i];
	}
	Fan_Mtr_Stop();
	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	LCD_Clear();

	// 종료 사유 확인 및 출력
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
		for (uint8_t i = 0; i < NUM_SENSORS; i++) {
			Sensor_Printf(i, last_normalized);
		}
	} else {
		LCD_Printf(0, 0, "Cross End");
	}
	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();

}
