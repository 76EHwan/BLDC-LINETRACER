#include "main.h"
#include "drive.h"
#include "sensor.h"
#include "foc.h"
#include "motor.h"
#include "user_init.h"
#include "button.h"
#include "sd_ui.h"
#include "buzzer.h"
#include "lptim.h"

#define RAMP_TIM		(&htim14)
#define Buzzer_LPTIM	(&hlptim3)
#define Buzzer_LPTIM_IRQ_Handler	LPTIM3_IRQ_Handler

// @formatter:off
DriveParam_t driveData = {
		.base_mps = 2.3f,
		.max_mps = 10.f,
		.accel = 5.f,
		.decel = 8.f,
		.steer_gain_p = 12.f,
		.steer_gain_d = 0.0f,
		.pos_atten_gain = 0.f,
		.pit_in_distance_m = 0.15f,
		.fan_en = 0,
};
// @formatter:on

uint8_t g_total_L = 0;
uint8_t g_total_R = 0;
uint8_t g_total_C = 0;
uint8_t g_total_STOP = 0;

void Buzzer_LPTIM_IRQ_Handler() {
	if (buzzer_timer_count > 0) {
		buzzer_timer_count--;
		if (buzzer_timer_count == 0) {
			Buzzer_Stop();
		}
	}
}
// ============================================================================
// 타이머 및 가감속(Ramp) 변수
// ============================================================================

float_t accel;
float_t decel;
volatile float_t g_target_base_mps = 0.0f;
volatile float_t g_current_base_mps = 0.0f;
volatile uint8_t g_is_braking = 0;
uint32_t count_irq = 0;

void Ramp_TIM_IRQ_Handler() {
	Odom_Accumulate(RAMP_DT);

	float d_mps = g_target_base_mps - g_current_base_mps;
	if (d_mps > accel * RAMP_DT)
		g_current_base_mps += accel * RAMP_DT;
	else if (d_mps < -decel * RAMP_DT)
		g_current_base_mps -= decel * RAMP_DT;
	else
		g_current_base_mps = g_target_base_mps;
	Steer_Motor();

}

void Ramp_Start() {
	g_target_base_mps = 0.f;
	g_current_base_mps = 0.f;
	HAL_TIM_Base_Start_IT(RAMP_TIM); // RAMP_TIM
}

void Ramp_Stop() {
	HAL_TIM_Base_Stop_IT(RAMP_TIM);
}

void Buzzer_Discount_Start() {
	HAL_LPTIM_Counter_Start_IT(Buzzer_LPTIM, 0);
}

void Buzzer_Discount_Stop() {
	HAL_LPTIM_Counter_Stop_IT(Buzzer_LPTIM);
}

// ============================================================================
// 주행 제어 공통 함수
// ============================================================================
void Drive_Stop_At_Distance(float target_distance_m) {
	if (g_current_base_mps <= 0.0f || target_distance_m <= 0.0f) {
		g_target_base_mps = 0.0f;
		HAL_Delay(500);
		return;
	}

	decel = (g_current_base_mps * g_current_base_mps)
			/ (2.0f * target_distance_m);
	g_target_base_mps = 0.0f;
	g_is_braking = 1;

	while (g_current_base_mps > 0.001f) {

	}
	g_is_braking = 0;
}

// ============================================================================
// 1회차 주행 함수 모음
// ============================================================================
__STATIC_INLINE uint8_t Drive_Init_Sequence(void) {
	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0;
		}
	}

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();
	g_cross_log_count = 0;
	Cross_Detect_Reset();

	IR_Sensor.is_lost_position = 0;
	IR_Sensor.data->mark_left = 0;
	IR_Sensor.data->mark_right = 0;

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;
	arm_pid_init_f32(&steer_pid, 1);

	Buzzer_Discount_Start();

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();

	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event(CrossEvent_t cross) {
	if (cross == CROSS_STOP) {
		g_total_STOP++;
		if (g_total_STOP >= 2)
			return 1;
		return 0;
	}

	if (cross == CROSS_LEFT)
		g_total_L++;
	else if (cross == CROSS_RIGHT)
		g_total_R++;
	else if (cross == CROSS_CROSS)
		g_total_C++;
	return 0;
}

void Drive_First() {
	if (!Drive_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event(cross))
				break;
		}
	}

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Buzzer_Discount_Stop();
	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;

	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time); // ★ 랩타임 출력 추가
		uint8_t slot = Select_Save_Slot();
		Save_MarkerLog_To_SD(slot);
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}

// ============================================================================
// 2회차 주행 함수 모음
// ============================================================================
typedef struct {
	uint8_t accel_ok;
} SegmentPlan_t;

static CrossMarkerLog_t ref_log[CROSS_LOG_MAX];
static uint16_t ref_log_count = 0;
static SegmentPlan_t seg_plan[CROSS_LOG_MAX];

#define BRAKE_MARGIN_M 0.03f
// ★ 본체가 마커를 통과하기 위한 여유 거리 (약 8cm로 설정, 필요시 조절)
#define ACCEL_START_MARGIN_M 0.08f

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
} DriveSecondState_t;

__STATIC_INLINE void Build_Segment_Plan(void) {
	uint8_t curve_state = 0; // 0: 직선, 1: 좌(L) 곡선 중, 2: 우(R) 곡선 중

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t is_straight = 0;

		// ★ 규칙 반영: 진입/탈출 상태 토글 로직
		if (cur == CROSS_CROSS) {
			curve_state = 0; // 십자 마커는 무조건 직선으로 초기화
			is_straight = 1;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				// 이미 L 곡선 중이었는데 L이 또 나옴 -> 곡선 닫힘(탈출), 이후 구간 직선
				curve_state = 0;
				is_straight = 1;
			} else {
				// 직선 또는 R 곡선 중 L 발견 -> 새로운 L 곡선 진입, 이후 구간 곡선
				curve_state = 1;
				is_straight = 0;
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				// 이미 R 곡선 중이었는데 R이 또 나옴 -> 곡선 닫힘(탈출), 이후 구간 직선
				curve_state = 0;
				is_straight = 1;
			} else {
				// 직선 또는 L 곡선 중 R 발견 -> 새로운 R 곡선 진입, 이후 구간 곡선
				curve_state = 2;
				is_straight = 0;
			}
		}

		// 안전을 위해 정지(STOP) 마커가 포함된 구간은 무조건 가속 금지
		if (cur == CROSS_STOP
				|| (i + 1 < ref_log_count && ref_log[i + 1].type == CROSS_STOP)) {
			is_straight = 0;
		}

		seg_plan[i].accel_ok = is_straight;
	}
}

__STATIC_INLINE uint8_t Drive_Second_Init_Sequence(void) {
	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0;
		}
	}

	ref_log_count = g_cross_log_count;

	if (ref_log_count == 0) {
		LCD_Printf(0, 0, "No Log Data");
		HAL_Delay(1000);
		return 0;
	}

	for (uint16_t i = 0; i < ref_log_count; i++) {
		ref_log[i] = g_cross_log[i];
	}

	Build_Segment_Plan();

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();

	g_cross_log_count = 0;
	Cross_Detect_Reset();

	IR_Sensor.is_lost_position = 0;
	IR_Sensor.data->mark_left = 0;
	IR_Sensor.data->mark_right = 0;

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;
	arm_pid_init_f32(&steer_pid, 1);

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();

	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event_Second(CrossEvent_t cross,
		DriveSecondState_t *state) {
	if (cross == CROSS_STOP) {
		g_total_STOP++;
		if (g_total_STOP >= 2)
			return 1;
	} else {
		if (cross == CROSS_LEFT)
			g_total_L++;
		else if (cross == CROSS_RIGHT)
			g_total_R++;
		else if (cross == CROSS_CROSS)
			g_total_C++;
	}

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	if (!state->mismatch && state->idx < ref_log_count
			&& seg_plan[state->idx].accel_ok
			&& state->idx + 1 < ref_log_count) {
		state->accel_active = 1;

		// 1. 다음 곡선/정지 마커가 나올 때까지의 연속된 직선 구간 거리를 모두 합산
		float total_straight_dist = 0.0f;
		for (uint16_t i = state->idx + 1; i < ref_log_count; i++) {
			total_straight_dist += ref_log[i].dist_from_prev_m;
			if (!seg_plan[i].accel_ok) {
				break;
			}
		}
		state->seg_len_predicted = total_straight_dist;

		// 2. 이미 직전 구간부터 직선이었다면 즉시 강제 가속
		if (state->idx > 0 && seg_plan[state->idx - 1].accel_ok) {
			// 이미 가속 중인 상태이므로 8cm 대기를 무시하고 목표 속도를 최고 속도로 고정
			g_target_base_mps = driveData.max_mps;
		} else {
			// 곡선에서 갓 빠져나왔을 때는 안정성을 위해 기본 속도로 잠시 대기
			g_target_base_mps = driveData.base_mps;
		}

	} else {
		state->accel_active = 0;
		g_target_base_mps = driveData.base_mps;
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;
	state->idx++;

	return 0;
}

__STATIC_INLINE void Check_Distance_And_Brake(DriveSecondState_t *state) {
	if (state->accel_active && !state->braking_started) {
		float_t traveled = g_odom_distance_m - state->marker_start_dist;
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = driveData.base_mps;
		float_t brake_dist = 0.0f;

		// 최고 속도가 아닌 현재 속도(v1)를 기준으로 제동 거리를 동적으로 계산
		if (v1 > v2) {
			brake_dist = (v1 * v1 - v2 * v2)
					/ (2.0f * driveData.decel)+ BRAKE_MARGIN_M;
		}

		if (remaining <= brake_dist) {
			// 감속 구간 돌입
			g_target_base_mps = driveData.base_mps;
			state->braking_started = 1;
		} else if (traveled >= ACCEL_START_MARGIN_M) {
			// 누적 거리가 여유 거리(마커를 완전히 빠져나온 시점)를 초과하면 본격적으로 가속
			g_target_base_mps = driveData.max_mps;
		}
	}
}

void Drive_Second() {
	if (!Drive_Second_Init_Sequence())
		return;
	uint32_t start_tick = HAL_GetTick();
	DriveSecondState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Second(cross, &state))
				break;
		}
		Check_Distance_And_Brake(&state);
	}
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);
	uint32_t end_tick = HAL_GetTick();

	Buzzer_Discount_Stop();
	HAL_Delay(500);
	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(2nd)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time); // ★ 랩타임 출력 추가
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}
