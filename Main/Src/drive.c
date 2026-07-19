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
		.accel = 30.f,
		.decel = 30.f,
		.steer_gain = 1.f,
		.pos_atten_gain = 0.5f,
		.fan_en = 0,
};

typedef enum {
	MARKER_STATE_IDLE = 0, // 평상시 라인 주행 중 (마커 없음)
	MARKER_STATE_IN_ZONE   // 마커 구간 진입함 (센서 감지 중)
} MarkerState_t;

// @formatter:on

__STATIC_INLINE void Ramp_Omega(FOC_Handle_t *hfoc) {
	float d = hfoc->omega_setpoint - hfoc->target_omega;
	const float_t accel_step = driveData.accel * RAMP_DT * INV_TIRE_RADIUS;
	const float_t decel_step = driveData.decel * RAMP_DT * INV_TIRE_RADIUS;

	if (d > accel_step)
		hfoc->target_omega += accel_step;
	else if (d < -decel_step)
		hfoc->target_omega -= decel_step;
	else
		hfoc->target_omega = hfoc->omega_setpoint;
	count_irq++;
}

__STATIC_INLINE void MTR_Set_Speed(float mps_L, float mps_R) {
	foc_L.omega_setpoint = mps_L * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;
	foc_R.omega_setpoint = -mps_R * INV_TIRE_RADIUS * MOTOR_POLE_PAIRS
			* GEAR_RATIO;
}

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
	Odom_Reset();   // 다음 마커까지 새로 누적 시작
}

// === 마커 판독용 상태 변수를 전역(파일 스코프)으로 분리 ===
static MarkerState_t g_marker_state = MARKER_STATE_IDLE;
static uint8_t g_accum_left = 0;
static uint8_t g_accum_right = 0;
static uint16_t g_accum_center_state = 0;

// 주행 시작 전 마커 상태를 초기화하는 함수
__STATIC_INLINE void Cross_Detect_Reset(void) {
	g_marker_state = MARKER_STATE_IDLE;
	g_accum_left = 0;
	g_accum_right = 0;
	g_accum_center_state = 0;
}

// 위치 창(POS_WINDOW) 바깥쪽 센서에서 검출되는 cross_left/cross_right 후보 플래그를 사용한다.
static CrossEvent_t Cross_Detect_Update(void) {
	uint8_t left = 0;
	uint8_t right = 0;

	// 현재 중앙 16개 센서의 상태만 추출 (하위 16비트)
	uint16_t current_center_state = (uint16_t) (IR_Sensor.data->state & 0xFFFF);

	CrossEvent_t event = CROSS_NONE;

	switch (g_marker_state) {
	case MARKER_STATE_IDLE:
		// 1. 좌/우 마커 센서가 하나라도 켜지면 마커 구간 진입
		if (left || right) {
			g_marker_state = MARKER_STATE_IN_ZONE;
			g_accum_left = left;
			g_accum_right = right;
			g_accum_center_state = current_center_state; // 진입 순간의 상태 기록
		}
		break;

	case MARKER_STATE_IN_ZONE:
		// 2. 마커 구간을 통과하는 동안 켜지는 모든 상태를 bitwise OR로 누적
		if (left)
			g_accum_left = 1;
		if (right)
			g_accum_right = 1;
		g_accum_center_state |= current_center_state;

		// 3. 양쪽 마커 센서가 모두 꺼지면 구간 탈출 (Falling Edge) -> 최종 판정
		if (!left && !right) {
			if (g_accum_left && g_accum_right) {
				// 십자 코스 vs 도착 지점 구분 로직
				// 노이즈를 고려해 여유를 두고 싶다면 (g_accum_center_state == 0xFFFF) 대신 비트 수를 세는 방식을 적용할 수 있습니다.
				if (g_accum_center_state == 0xFFFF) {
					event = CROSS_CROSS; // 16개 모두 1이 됨 -> 십자 교차로
				} else {
					event = CROSS_STOP;  // 전부 1이 되지는 않음 -> 도착(End) 지점
				}
			} else if (g_accum_left) {
				event = CROSS_LEFT;
			} else if (g_accum_right) {
				event = CROSS_RIGHT;
			}

			// 상태 및 누적 변수 초기화하여 다음 마커 대기
			g_marker_state = MARKER_STATE_IDLE;
			g_accum_left = 0;
			g_accum_right = 0;
			g_accum_center_state = 0;
		}
		break;
	}

	// 4. 유효한 이벤트가 발생했을 때만 로그 저장
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
	count_sensor_irq = 0;
	count_irq = 0;
	count_polling = 0;

	float_t g_base_mps = driveData.base_mps;
	float_t g_steer_gain = driveData.steer_gain;
//	float_t g_max_mps = driveData.max_mps;
	float_t g_pos_atten_gain = driveData.pos_atten_gain;

	// PD 제어를 위한 변수 추가 (kd 값은 로봇의 반응성에 맞게 튜닝하세요)
	float_t prev_line_pos = 0.0f;
	float_t kd = 0.0f;
	uint8_t end_count = 0;

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		HAL_Delay(2000);
	}
	Odom_Reset();
	g_cross_log_count = 0;

	// 주행 시작 전 마커 판독기 강제 초기화 (오작동 방지)
	Cross_Detect_Reset();

	Sensor_Start();
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);

	Ramp_Start();

	HAL_Delay(10);

	MTR_Set_Speed(g_base_mps, g_base_mps);

//	uint32_t last_tick = HAL_GetTick();
//	uint32_t lcd_update_tick = last_tick; // LCD 갱신용 타이머 추가

	while (!IR_Sensor.is_lost_position) {
//		count_polling++;
//		uint32_t now_tick = HAL_GetTick();
//		Odom_Accumulate(now_tick - last_tick);
//		last_tick = now_tick;

		CrossEvent_t cross = Cross_Detect_Update();

		if (cross == CROSS_STOP) {
			if (end_count != 0)
				break;
			else
				end_count++;
		}
		// ★ 추가된 마커 출력 로직: 마커가 감지된 순간(이벤트 발생)에만 딱 1번 출력하여 루프 지연 방지
		else if (cross != CROSS_NONE) {
			CrossMarkerLog_t *last = &g_cross_log[(g_cross_log_count - 1)
					% CROSS_LOG_MAX];
			const char *tag = (last->type == CROSS_LEFT) ? "LEFT " :
								(last->type == CROSS_RIGHT) ? "RIGHT" :
								(last->type == CROSS_CROSS) ? "CROSS" : "UNK";

			// 화면 7번째 줄에 마커 종류, 번호, 이전 마커로부터의 거리를 출력
			LCD_Printf(0, 7, "M:%s #%d d:%4.2f", tag, g_cross_log_count,
					last->dist_from_prev_m);
		}

		// ★ 최적화: 센서 위치가 갱신되었을 때만 PD 연산 수행 (이전 대화 내용 반영)

		if (IR_Sensor.is_position) {
			IR_Sensor.is_position = 0;
			float_t line_pos = 0;
			float_t line_pos_abs = fabsf(line_pos);

			float_t d_line_pos = line_pos - prev_line_pos;
			prev_line_pos = line_pos;

			float_t steer = (g_steer_gain * line_pos) + (kd * d_line_pos);

			float_t mps_L = g_base_mps * (1.f + steer)
					* (1.f - line_pos_abs * g_pos_atten_gain);
			float_t mps_R = g_base_mps * (1.f - steer)
					* (1.f - line_pos_abs * g_pos_atten_gain);

			MTR_Set_Speed(mps_L, mps_R);
		}
		// 100ms (0.1초) 주기로 기본 정보 갱신
//		if (now_tick - lcd_update_tick >= 100) {
//			LCD_Printf(0, 0, "%d", count_irq);
//			LCD_Printf(0, 1, "%d", count_polling);
//			LCD_Printf(0, 2, "%d", count_sensor_irq);
//			lcd_update_tick = now_tick;
//		}
	}
	Fan_Mtr_Stop();
	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	LCD_Clear();

	// 종료 사유 확인 및 출력
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Stop: Line Lost");
	} else {
		LCD_Printf(0, 0, "Stop: Cross End");
	}
	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();

}
