#include "main.h"
#include "dac.h"
#include "drive.h"
#include "sensor.h"
#include "foc.h"
#include "motor.h"
#include "user_init.h"
#include "button.h"
#include "sd_ui.h"
#include "buzzer.h"
#include "lptim.h"
#include "lsm6ds3tr-c.h"

#define RAMP_TIM        (&htim14)
#define VIB_SAMPLE_MAX  10000

// @formatter:off
DriveParam_t driveData = {
    .base_mps = 1.8f,
    .max_mps = 8.f,
    .accel = 5.f,
    .decel = 5.f,
    .steer_gain_p = 22.4f,
    .steer_gain_d = 600.f,
    .pos_atten_gain = 0.0f,
    .pit_in_distance_m = 0.15f,
    .fan_en = 0,
    .target_shift_val = 5, // 초기값: 5 (5/15 비율)
};
// @formatter:on

uint8_t g_total_L = 0;
uint8_t g_total_R = 0;
uint8_t g_total_C = 0;
uint8_t g_total_STOP = 0;

// ============================================================================
// 타이머 및 가감속(Ramp) 변수
// ============================================================================

float_t accel;
float_t decel;
volatile float_t g_target_base_mps = 0.0f;
volatile float_t g_current_base_mps = 0.0f;
volatile uint8_t g_is_braking = 0;
uint32_t count_irq = 0;

void Ramp_TIM_IRQ_Handler(void) {
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

void Ramp_Start(void) {
	g_target_base_mps = 0.f;
	g_current_base_mps = 0.f;
	HAL_TIM_Base_Start_IT(RAMP_TIM); // RAMP_TIM
}

void Ramp_Stop(void) {
	HAL_TIM_Base_Stop_IT(RAMP_TIM);
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
static uint8_t is_ref_log_copied = 0;

void Drive_Reset_Ref_Log_Flag(void) {
	is_ref_log_copied = 0;
}

__STATIC_INLINE uint8_t Drive_Init_Sequence(void) {
	LCD7789_Invert(0);

	is_ref_log_copied = 0;

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
	IR_Sensor.data->target_pos = 0.0f;

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;

	arm_pid_init_f32(&steer_pid, 1);
	arm_pid_init_f32(&steer_pid, 0);

	Buzzer_Discount_Start();

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();
	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event(CrossEvent_t cross) {
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
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

extern uint16_t g_last_stop_state;
extern uint8_t g_last_stop_count;

void Drive_First(void) {
	if (!Drive_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();

	uint8_t exit_reason_lost = 0;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event(cross))
				break;
		}
	}

	if (IR_Sensor.is_lost_position) {
		exit_reason_lost = 1;
	}
	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;

	if (exit_reason_lost) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
		LCD_Printf(0, 5, "U_Bit:0x%04X", g_last_stop_state);
		LCD_Printf(0, 6, "U_Cnt:%d (<12)", g_last_stop_count);
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
#define ACCEL_START_MARGIN_M 0.08f

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
} DriveSecondState_t;

SecondDriveLog_t g_second_log[CROSS_LOG_MAX];
uint16_t g_second_log_count = 0;

__STATIC_INLINE void Build_Segment_Plan(void) {
	uint8_t curve_state = 0;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t is_straight = 0;

		if (cur == CROSS_CROSS) {
			curve_state = 0;
			is_straight = 1;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 1;
				is_straight = 0;
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 2;
				is_straight = 0;
			}
		}

		if (cur == CROSS_STOP) {
			is_straight = 0;
		}

		seg_plan[i].accel_ok = is_straight;
	}
}

__STATIC_INLINE uint8_t Drive_Second_Init_Sequence(void) {
	LCD7789_Invert(0);

	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0;
		}
	}

	if (!is_ref_log_copied) {
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
		is_ref_log_copied = 1;
	} else {
		if (ref_log_count == 0) {
			LCD_Printf(0, 0, "No Log Data");
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
	IR_Sensor.data->target_pos = 0.0f;

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;

	arm_pid_init_f32(&steer_pid, 1);
	arm_pid_init_f32(&steer_pid, 0);

	Buzzer_Discount_Start();

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();
	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event_Second(CrossEvent_t cross,
		DriveSecondState_t *state) {
	uint8_t is_stop = 0;

	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2)
			is_stop = 1;
	} else {
		if (cross == CROSS_LEFT)
			g_total_L++;
		else if (cross == CROSS_RIGHT)
			g_total_R++;
		else if (cross == CROSS_CROSS)
			g_total_C++;
	}

	if (state->mismatch) {
		// [수정] 복구 로직: 놓친 마커를 현재 마커(cross)와 일치하는 가장 가까운 다음 마커로 찾음
		for (uint16_t i = state->idx; i < ref_log_count; i++) {
			// ★ 수정점: 마커 종류(C, L, R)에 상관없이, 곡선 구간에서는 절대로 인덱스 점프(복구)를 하지 않음.
			if (!seg_plan[i].accel_ok)
				continue;

			if (ref_log[i].type == cross) {
				state->idx = i;
				state->mismatch = 0;
				break; // ★ 찾았으면 루프 종료
			}
		}
	}

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	if (!state->mismatch) {
		if (state->idx < ref_log_count && seg_plan[state->idx].accel_ok
				&& state->idx + 1 < ref_log_count) {
			state->accel_active = 1;

			float total_straight_dist = 0.0f;
			for (uint16_t i = state->idx + 1; i < ref_log_count; i++) {
				total_straight_dist += ref_log[i].dist_from_prev_m;
				if (!seg_plan[i].accel_ok)
					break;
			}
			state->seg_len_predicted = total_straight_dist;

			if (state->idx > 0 && seg_plan[state->idx - 1].accel_ok) {
				g_target_base_mps = driveData.max_mps;
			} else {
				g_target_base_mps = driveData.base_mps;
			}
		} else {
			state->accel_active = 0;
			g_target_base_mps = driveData.base_mps;
		}

		state->marker_start_dist = g_odom_distance_m;
		state->braking_started = 0;
		state->idx++;
	} else {
		// ★수정점: Mismatch 발생 시 무조건 base_mps로 감속하는 문제 수정
		if (!state->accel_active) {
			g_target_base_mps = driveData.base_mps;
		}
	}

	// ★ 2차 주행 로그 기록
	if (g_second_log_count < CROSS_LOG_MAX) {
		g_second_log[g_second_log_count].ref_idx =
				state->idx > 0 ? state->idx - 1 : 0;
		g_second_log[g_second_log_count].type = cross;
		g_second_log[g_second_log_count].accel_active = state->accel_active;
		g_second_log[g_second_log_count].mismatch = state->mismatch;
		g_second_log[g_second_log_count].dist = g_odom_distance_m;
		g_second_log_count++;
	}

	return is_stop;
}

__STATIC_INLINE void Check_Distance_And_Brake(DriveSecondState_t *state) {
	if (state->accel_active) {
		float_t traveled = g_odom_distance_m - state->marker_start_dist;
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = driveData.base_mps;

		float_t brake_dist = BRAKE_MARGIN_M;
		if (v1 > v2) {
			brake_dist += (v1 * v1 - v2 * v2) / (2.0f * driveData.decel);
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = driveData.base_mps;
			state->braking_started = 1;
		} else {
			float_t hysteresis = state->braking_started ? 0.05f : 0.0f;
			if (remaining > (brake_dist + hysteresis)) {
				if (traveled >= ACCEL_START_MARGIN_M) {
					g_target_base_mps = driveData.max_mps;
					state->braking_started = 0;
				}
			}
		}
	}
}

void Drive_Second(void) {
	if (!Drive_Second_Init_Sequence())
		return;

	g_second_log_count = 0; // 로그 버퍼 초기화

	uint32_t start_tick = HAL_GetTick();
	DriveSecondState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;

	if (ref_log_count > 0) {
		state.accel_active = 1;
		state.seg_len_predicted = ref_log[0].dist_from_prev_m;

		if (state.seg_len_predicted > 0.3f) {
			g_target_base_mps = driveData.max_mps;
		} else {
			g_target_base_mps = driveData.base_mps;
		}
	}

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Second(cross, &state))
				break;
		}
		Check_Distance_And_Brake(&state);
	}

	LCD7789_Invert(0);

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
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
		LCD_Printf(0, 4, "T:%.2fs", lap_time);

		// 주행 완료 후 SD카드에 2차 주행 상세 로그 저장
		uint8_t slot = Select_Save_Slot();
		Save_SecondDriveLog_To_SD(slot);
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}

// ============================================================================
// 3회차 주행 함수 모음 (가속 부활 + 상태 기반 Target Pos 예지 이동)
// ============================================================================

typedef struct {
	uint8_t accel_ok;
	uint8_t curve_dir; // 0: 직선, 1: 좌(L) 곡선, 2: 우(R) 곡선
} SegmentPlanThird_t;

static SegmentPlanThird_t seg_plan_3[CROSS_LOG_MAX];

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
} DriveThirdState_t;

// 3차 주행용 구간 계획 빌드 (곡선 방향 정보 포함)
__STATIC_INLINE void Build_Segment_Plan_Third(void) {
	uint8_t curve_state = 0; // 0: 직선, 1: L 곡선, 2: R 곡선

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t is_straight = 0;

		if (cur == CROSS_CROSS) {
			curve_state = 0;
			is_straight = 1;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 1;
				is_straight = 0;
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 2;
				is_straight = 0;
			}
		}

		if (cur == CROSS_STOP) {
			is_straight = 0;
			curve_state = 0;
		}

		seg_plan_3[i].accel_ok = is_straight;
		seg_plan_3[i].curve_dir = curve_state;
	}
}

// 3차 주행 초기화
__STATIC_INLINE uint8_t Drive_Third_Init_Sequence(void) {
	LCD7789_Invert(0);

	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0;
		}
	}

	if (!is_ref_log_copied) {
		ref_log_count = g_cross_log_count;
		if (ref_log_count == 0) {
			LCD_Printf(0, 0, "No Log Data");
			HAL_Delay(1000);
			return 0;
		}
		for (uint16_t i = 0; i < ref_log_count; i++) {
			ref_log[i] = g_cross_log[i];
		}
		Build_Segment_Plan_Third();
		is_ref_log_copied = 1;
	} else {
		if (ref_log_count == 0) {
			LCD_Printf(0, 0, "No Log Data");
			HAL_Delay(1000);
			return 0;
		}
		Build_Segment_Plan_Third();
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
	IR_Sensor.data->target_pos = 0.0f; // 초기 시작은 센서 중앙

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;

	arm_pid_init_f32(&steer_pid, 1);
	arm_pid_init_f32(&steer_pid, 0);

	Buzzer_Discount_Start();
	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();
	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event_Third(CrossEvent_t cross,
		DriveThirdState_t *state) {
	uint8_t is_stop = 0;

	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2)
			is_stop = 1;
	} else {
		if (cross == CROSS_LEFT)
			g_total_L++;
		else if (cross == CROSS_RIGHT)
			g_total_R++;
		else if (cross == CROSS_CROSS)
			g_total_C++;
	}

	if (state->mismatch) {
		// [수정] 복구 로직: 놓친 마커를 현재 마커(cross)와 일치하는 가장 가까운 다음 마커로 찾음
		for (uint16_t i = state->idx; i < ref_log_count; i++) {
			// ★ 수정점: 마커 종류(C, L, R)에 상관없이, 곡선 구간에서는 절대로 인덱스 점프(복구)를 하지 않음.
			if (!seg_plan_3[i].accel_ok)
				continue;

			if (ref_log[i].type == cross) {
				state->idx = i;
				state->mismatch = 0;
				break; // ★ 찾았으면 루프 종료
			}
		}
	}

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	if (!state->mismatch) {
		if (state->idx < ref_log_count && seg_plan_3[state->idx].accel_ok
				&& state->idx + 1 < ref_log_count) {
			state->accel_active = 1;

			float total_straight_dist = 0.0f;
			for (uint16_t i = state->idx + 1; i < ref_log_count; i++) {
				total_straight_dist += ref_log[i].dist_from_prev_m;
				if (!seg_plan_3[i].accel_ok)
					break;
			}
			state->seg_len_predicted = total_straight_dist;

			if (state->idx > 0 && seg_plan_3[state->idx - 1].accel_ok) {
				g_target_base_mps = driveData.max_mps;
			} else {
				g_target_base_mps = driveData.base_mps;
			}
		} else {
			state->accel_active = 0;
			g_target_base_mps = driveData.base_mps;
		}

		state->marker_start_dist = g_odom_distance_m;
		state->braking_started = 0;
		state->idx++;
	} else {
		// ★수정점: Mismatch 발생 시 무조건 base_mps로 감속 방지
		if (!state->accel_active) {
			g_target_base_mps = driveData.base_mps;
		}
	}
	return is_stop;
}

// ★ 가속 부활: 감속/가속 판정 로직 복구
__STATIC_INLINE void Check_Distance_And_Brake_Third(DriveThirdState_t *state) {
	if (state->accel_active) {
		float_t traveled = g_odom_distance_m - state->marker_start_dist;
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = driveData.base_mps;

		float_t brake_dist = BRAKE_MARGIN_M;
		if (v1 > v2) {
			brake_dist += (v1 * v1 - v2 * v2) / (2.0f * driveData.decel);
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = driveData.base_mps;
			state->braking_started = 1;
		} else {
			float_t hysteresis = state->braking_started ? 0.05f : 0.0f;
			if (remaining > (brake_dist + hysteresis)) {
				if (traveled >= ACCEL_START_MARGIN_M) {
					g_target_base_mps = driveData.max_mps;
					state->braking_started = 0;
				}
			}
		}
	}
}

// ★ 3차 주행 전용 Target Position 예지 이동 로직 (상태 기반 Out-In-Out)
__STATIC_INLINE void Update_Target_Pos_Third(DriveThirdState_t *state) {
	// 에러 났거나 이미 끝났다면 정중앙 유지
	if (state->mismatch || state->idx >= ref_log_count) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	// 현재 속해있는 구간의 곡선 상태 (마커 idx 도착 전이므로 idx-1의 결과 상태 참조)
	uint8_t current_curve =
			(state->idx > 0) ? seg_plan_3[state->idx - 1].curve_dir : 0;
	// 다가올 다음 마커를 지나친 후의 곡선 상태
	uint8_t next_curve = seg_plan_3[state->idx].curve_dir;

	// 정수형 타겟 쉬프트 값(-15 ~ 15)을 float 비율(-1.0f ~ 1.0f)로 변환
	float shift_val = (float) driveData.target_shift_val / 15.0f;
	float ideal_target = 0.0f;

	// 상태 기반 이상적인 타겟 산출 (마커 단위로 즉시 변경)
	if (current_curve == 1) {
		// [Curve L 진행 중]
		ideal_target = shift_val;       // 인코스 유지 (+Shift로 좌측 파고들기)
	} else if (current_curve == 2) {
		// [Curve R 진행 중]
		ideal_target = -shift_val;      // 인코스 유지 (-Shift로 우측 파고들기)
	} else {
		// [직선(Straight) 진행 중]
		if (next_curve == 1) {
			ideal_target = -shift_val;  // 다가올 L 커브를 위해 아웃코스(우측)로 미리 진입
		} else if (next_curve == 2) {
			ideal_target = shift_val;   // 다가올 R 커브를 위해 아웃코스(좌측)로 미리 진입
		} else {
			ideal_target = 0.0f;        // 순수 직선
		}
	}

	// 부드러운 전환을 위한 로우패스 필터 (LPF)
	// 마커 통과 직후 ideal_target이 바뀌면, 목표점을 향해 부드럽고 빠르게 미끄러져 들어감
	IR_Sensor.data->target_pos += (ideal_target - IR_Sensor.data->target_pos)
			* 0.05f;
}

void Drive_Third(void) {
	if (!Drive_Third_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveThirdState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;

	// ★ 첫 번째 직선 구간 출발 가속 처리
	if (ref_log_count > 0) {
		state.accel_active = 1;
		state.seg_len_predicted = ref_log[0].dist_from_prev_m;
		if (state.seg_len_predicted > 0.3f) {
			g_target_base_mps = driveData.max_mps;
		} else {
			g_target_base_mps = driveData.base_mps;
		}
	}

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Third(cross, &state))
				break;
		}

		Check_Distance_And_Brake_Third(&state); // ★ 가속 로직 복구
		Update_Target_Pos_Third(&state);        // ★ 상태 기반 타겟 동적 이동
	}

	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	IR_Sensor.data->target_pos = 0.0f; // ★ 주행 종료 후 다른 모드 작동을 위해 반드시 0으로 원상복구

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(3rd)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}

// ============================================================================
// 진동 분석을 위한 테스트 주행 모드
// ============================================================================

static float vib_data_buf[VIB_SAMPLE_MAX];
static uint32_t vib_sample_count = 0;

void Drive_Vibration_Test(void) {
	if (!Drive_Init_Sequence())
		return;
	vib_sample_count = 0;
	uint32_t start_tick = HAL_GetTick();
	uint32_t last_sample_tick = start_tick;
	g_target_base_mps = driveData.base_mps;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross == CROSS_STOP) {
			break;
		}

		uint32_t current_tick = HAL_GetTick();
		if ((current_tick - last_sample_tick) >= 1) {
			last_sample_tick = current_tick;

			if (vib_sample_count < VIB_SAMPLE_MAX) {
				vib_data_buf[vib_sample_count++] = imu_data.Gyro_Z;
			} else {
				break;
			}
		}
	}

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	LCD_Clear();
	LCD_Printf(0, 0, "Saving Vib Data...");

	if (SDCard_Mount() == FR_OK) {
		FIL file;
		UINT bw;
		char filepath[64];

		sprintf(filepath, "/Drive_Data/vib_log_%lu.csv", HAL_GetTick() % 1000);

		if (f_open(&file, filepath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
			char line_buf[64];

			f_write(&file, "Index,Value", 12, &bw);

			for (uint32_t i = 0; i < vib_sample_count; i++) {
				int len = snprintf(line_buf, sizeof(line_buf), "%lu,%.4f", i,
						vib_data_buf[i]);
				f_write(&file, line_buf, len, &bw);
			}
			f_close(&file);
			LCD_Printf(0, 1, "Saved!");
		} else {
			LCD_Printf(0, 1, "SD Open Fail");
		}

		SDCard_Unmount();
	} else {
		LCD_Printf(0, 1, "SD Mount Fail");
	}

	LCD_Printf(0, 3, "Samples: %lu", vib_sample_count);
	LCD_Printf(0, 4, "Press Hold");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}
