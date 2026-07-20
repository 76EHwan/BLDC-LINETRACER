/*
 * drive.c
 *
 *  Created on: 2026. 7. 2.
 *      Author: kth59
 *
 *  [수정 이력]
 *  - 구버전 마커 감지 알고리즘 상태머신 재구성
 *  - 좌우 모터 개별 Ramp 방식에서 중심 속도(Base Speed) Ramp 방식으로 변경
 *  - 목표 거리 제동 함수(Drive_Stop_At_Distance) 추가 및 관성 제동 적용
 *  - 조향 제어에 CMSIS-DSP 하드웨어 가속 PID (arm_pid_f32) 적용
 */

#include "main.h"
#include "math.h"
#include "arm_math.h" // CMSIS-DSP 라이브러리

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
		.base_mps = 2.2f,
		.max_mps = 6.f,
		.accel = 6.f,
		.decel = 6.f,
		.steer_gain_p = 0.65f,
		.steer_gain_d = 0.03f,
		.pos_atten_gain = 0.5f,
		.pit_in_distance_m = 0.2f,
		.fan_en = 0,
};

typedef enum {
	MARKER_STATE_IDLE = 0,
	MARKER_STATE_READING,
} MarkerState_t;

// @formatter:on

// ============================================================================
// 조향 제어 및 중심 속도 제어용 전역 변수
// ============================================================================
static arm_pid_instance_f32 steer_pid;

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

float_t accel;
float_t decel;

void Ramp_TIM_IRQ_Handler() {
	// 1. 중심 속도(Center Speed) 가감속(Ramp) 계산
	float d_mps = g_target_base_mps - g_current_base_mps;
	float accel_step = accel * RAMP_DT;
	float decel_step = decel * RAMP_DT;

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

// ============================================================================
// 지정 거리 제동 함수 (Active Braking with Line Tracking & arm_pid)
// ============================================================================
void Drive_Stop_At_Distance(float target_distance_m) {
	float v1 = g_current_base_mps;

	if (v1 <= 0.0f || target_distance_m <= 0.0f) {
		g_target_base_mps = 0.0f;
		g_current_steer = 0.0f;
		HAL_Delay(500);
		return;
	}

	float required_decel = (v1 * v1) / (2.0f * target_distance_m);

	decel = required_decel;
	g_target_base_mps = 0.0f;

	// 제동 중 조향 제어 유지
	while (g_current_base_mps > 0.001f) {
		float_t line_pos = Sensor_Get_Position();

		// CMSIS-DSP PID 제어기 적용
		g_current_steer = arm_pid_f32(&steer_pid, line_pos);

		if (IR_Sensor.is_lost_position) {
			break;
		}
	}

	// 완전히 멈춘 후 관성 정지 대기 (Active Braking)
	g_current_base_mps = 0.0f;
	g_target_base_mps = 0.0f;
	g_current_steer = 0.0f;

	foc_L.omega_setpoint = 0.0f;
	foc_R.omega_setpoint = 0.0f;

	uint32_t start_tick = HAL_GetTick();
	while ((HAL_GetTick() - start_tick) < 500) {
		// 속도 0 상태를 강제로 유지하며 대기
	}
}

// ============================================================================
// 메인 라인 트레이싱 주행
// ============================================================================
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

	accel = driveData.accel;
	decel = driveData.decel;

	float_t g_base_mps = driveData.base_mps;
	float_t g_pos_atten_gain = driveData.pos_atten_gain;

	uint8_t end_count = 0;
	float_t filtered_atten = 1.0f;

	// CMSIS-DSP PID 제어기 세팅 및 초기화 (1 = 상태 변수 리셋)
	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f; // 필요 시 적분 제어 추가
	steer_pid.Kd = driveData.steer_gain_d;
	arm_pid_init_f32(&steer_pid, 1);

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
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

		// CMSIS-DSP PID 연산 (이전의 수동 PD 연산 대체)
		float_t steer = arm_pid_f32(&steer_pid, line_pos);

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

	// ★ 루프 탈출 후 목표 거리에서 안전하게 정지 (예: 0.15m)
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

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
