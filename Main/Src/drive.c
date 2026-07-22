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
 *  - 센서 읽기, 조향 PID, LPF 감속 제어를 Ramp 인터럽트로 이관하여 제어 대역폭 2kHz 고정
 */

#include "main.h"
#include "math.h"
#include "arm_math.h" // CMSIS-DSP 라이브러리

#include "button.h"
#include "foc.h"
#include "buzzer.h"

#include "user_init.h"
#include "motor.h"
#include "drive.h"

#define RAMP_DT	0.0005f

uint32_t count_irq = 0;
uint32_t count_polling = 0;

// 인터럽트 기반 부저 제어 타이머
volatile uint16_t buzzer_timer_count = 0;

// 시간 기반 마커 필터링 쿨다운 타이머
static uint32_t last_marker_tick = 0;

// 제어 루프용 전역 상태 변수
static volatile float filtered_atten = 1.0f;
static volatile uint8_t g_is_braking = 0; // 정지 함수 진입 여부 플래그

// @formatter:off
DriveParam_t driveData = {
		.base_mps = 2.7f,
		.max_mps = 6.f,
		.accel = 4.f,
		.decel = 8.f,
		.steer_gain_p = 0.6f,
		.steer_gain_d = 0.0f,
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

__STATIC_INLINE void Odom_Accumulate(float dt_sec) {
	float mps = 0.5f * (FOC_Meas_Mps(&foc_L) + FOC_Meas_Mps(&foc_R));
	g_odom_distance_m += mps * dt_sec;
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
		if (left_marker)
			g_accum_left = 1;
		if (right_marker)
			g_accum_right = 1;
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

			// 시간 기반(200ms) 마커 노이즈 필터링
			if (event != CROSS_NONE) {
				uint32_t current_tick = HAL_GetTick();

				if ((current_tick - last_marker_tick) < 200) {
					event = CROSS_NONE;
				} else {
					last_marker_tick = current_tick;
				}
			}

			Cross_Detect_Reset();
		}
		break;
	}

	if (event != CROSS_NONE) {
		Cross_Log_Push(event);
		Buzzer_Start();
		buzzer_timer_count = 1000;
	}

	return event;
}

// ============================================================================
// 모터 제어 및 Ramp 로직
// ============================================================================

float_t accel;
float_t decel;

void Ramp_TIM_IRQ_Handler() {
	// 0.5ms(RAMP_DT) 주기로 현재 속도를 바탕으로 이동 거리 적산
	Odom_Accumulate(RAMP_DT);

	// --- [추가됨] 고정 대역폭 보장을 위한 제어 로직 ---
	float_t line_pos = Sensor_Get_Position();
	float_t line_pos_abs = fabsf(line_pos);

	// 제동 중이든 주행 중이든 조향 제어는 인터럽트에서 실시간 유지 (D항 미분 오차 방지)
	g_current_steer = arm_pid_f32(&steer_pid, line_pos);

	if (!g_is_braking) {
		// 정상 주행 중에만 감쇠 비율 목표치 생성 및 LPF 연산 수행
		float_t target_atten = 1.f - (line_pos_abs * driveData.pos_atten_gain);

		const float_t pos_atten_alpha_low = 0.002f;
		const float_t pos_atten_alpha_high = 1.f - pos_atten_alpha_low;

		if (target_atten < filtered_atten) {
			filtered_atten = (pos_atten_alpha_high * target_atten) + (pos_atten_alpha_low * filtered_atten);
		} else {
			filtered_atten = (pos_atten_alpha_low * target_atten) + (pos_atten_alpha_high * filtered_atten);
		}

		g_target_base_mps = driveData.base_mps * filtered_atten;
	}
	// --------------------------------------------------

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
	foc_L.target_omega =
			mps_L * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS * GEAR_RATIO;
	foc_R.target_omega = -mps_R * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;

	foc_L.omega_setpoint = foc_L.target_omega;
	foc_R.omega_setpoint = foc_R.target_omega;

	count_irq++;

	// 부저 비동기 타이머 처리
	if (buzzer_timer_count > 0) {
		buzzer_timer_count--;
		if (buzzer_timer_count == 0) {
			Buzzer_Stop(); // 0.5초 경과 시 부저 OFF
		}
	}
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

	// [추가됨] 인터럽트 쪽에 브레이킹 모드 진입을 알려 목표 속도 덮어쓰기 방지
	g_is_braking = 1;

	// 제동 중 조향 제어는 인터럽트가 알아서 수행하므로 여기선 이탈만 체크
	while (g_current_base_mps > 0.001f) {
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

	g_is_braking = 0; // 정지 완료 후 브레이킹 모드 해제
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

	uint8_t end_count = 0;

	// --- [추가됨] 마커 종류별 개수 카운트를 위한 변수 선언 ---
	uint8_t total_left_marker = 0;
	uint8_t total_right_marker = 0;
	uint8_t total_cross_marker = 0;
	// ----------------------------------------------------

	// 루프 진입 시 제어 상태 초기화
	filtered_atten = 1.0f;
	g_is_braking = 0;

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

	// 초기 타겟 설정
	g_target_base_mps = driveData.base_mps;
	g_current_steer = 0.0f;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();

		if (cross == CROSS_STOP) {
			if (end_count != 0)
				break;
			else
				end_count++;
		} else if (cross != CROSS_NONE) {

			// --- [추가됨] 발견된 마커 종류에 따라 카운트 증가 ---
			if (cross == CROSS_LEFT) {
				total_left_marker++;
			} else if (cross == CROSS_RIGHT) {
				total_right_marker++;
			} else if (cross == CROSS_CROSS) {
				total_cross_marker++;
			}
			// ------------------------------------------------

			CrossMarkerLog_t *last = &g_cross_log[(g_cross_log_count - 1)
					% CROSS_LOG_MAX];
			const char *tag = (last->type == CROSS_LEFT) ? "LEFT " :
								(last->type == CROSS_RIGHT) ? "RIGHT" :
								(last->type == CROSS_CROSS) ? "CROSS" : "UNK";

			LCD_Printf(0, 7, "M:%s #%d d:%4.2f", tag, g_cross_log_count,
					last->dist_from_prev_m);
		}
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
		// --- [수정됨] 정상 종료 시 마커별 누적 개수 출력 ---
		LCD_Printf(0, 0, "Cross End");
		// 2번째 줄에 L(좌), R(우), C(십자) 개수 간략 출력
		LCD_Printf(0, 2, "L:%-3d", total_left_marker);
		LCD_Printf(0, 3, "R:%-3d", total_right_marker);
		LCD_Printf(0, 4, "C:%-3d", total_cross_marker);
		// 3번째 줄에 전체 로깅된 개수 출력
		LCD_Printf(0, 5, "Total: %d", g_cross_log_count);
		// ------------------------------------------------
	}
	while (Button_Get_Input() != INPUT_CMD_K_HOLD);
	LCD_Clear();
}
