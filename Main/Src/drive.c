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

#define RAMP_TIM		(&htim14)
#define Buzzer_LPTIM	(&hlptim3)
#define Buzzer_LPTIM_IRQ_Handler	LPTIM3_IRQ_Handler

// @formatter:off
DriveParam_t driveData = {
		.base_mps = 1.8f,
		.max_mps = 10.f,
		.accel = 4.f,
		.decel = 4.f,
		.steer_gain_p = 2.f,
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
// 사용자 로봇의 실제 하드웨어 제원에 맞게 반드시 실측하여 입력해야 하는 값 (단위: 미터)
// ============================================================================
// 조향 마진 (이상적인 물리적 회전값의 몇 배까지 PID를 허용할 것인가)
// 1.0에 가까울수록 차가 둔해지고(언더스티어), 너무 크면 다시 오버스티어가 납니다.
#define STEER_SAFETY_MARGIN 1.5f

void Steer_Motor_With_Anti_Oversteer(void) {
	// 1. 센서 에러값을 가져오고 기존처럼 PID 연산 수행 (-1.0 ~ 1.0 범위)
	float_t sensor_pos = Sensor_Get_Position();
	float_t pid_steer = arm_pid_f32(&steer_pid, 0.0f - sensor_pos);

	float_t final_steer = pid_steer; // 최종 적용될 조향값

	// 2. 에러가 있을 때만 오버스티어 방지 로직 개입
	if (sensor_pos != 0.0f) {
		// 비율값(-1.0 ~ 1.0)을 실제 물리적 측면 오차 거리 x(미터)로 변환
		float_t x = sensor_pos * SENSOR_HALF_WIDTH;

		// 기하학적 곡률 반경 R 계산 (퓨어 퍼슈트 원리)
		float_t R = (SENSOR_DIST_L * SENSOR_DIST_L + (x * x)) / (2.0f * x);
		if (R < 0)
			R = -R; // 반경은 절대값 처리

		// 현재 직진 속도(V)에서 반경 R을 돌기 위해 필요한 이상적인 조향 속도차
		// g_current_base_mps는 drive.c에서 관리되는 현재 베이스 속도
		float_t ideal_steer = g_current_base_mps * (WHEEL_TRACK_W / R);

		// 3. 허용 가능한 최대 조향 한계치(Limit) 설정
		float_t max_steer_limit = ideal_steer * STEER_SAFETY_MARGIN;

		// 4. PID 제어값이 물리적 한계를 넘어가려 하면 강제로 잘라버림 (Clamp)
		if (final_steer > max_steer_limit) {
			final_steer = max_steer_limit;
		} else if (final_steer < -max_steer_limit) {
			final_steer = -max_steer_limit;
		}
	}

	// 5. 최종 안전하게 Clamp된 조향값을 양쪽 모터에 인가
	float_t mps_L = g_current_base_mps - final_steer;
	float_t mps_R = g_current_base_mps + final_steer;

	foc_L.target_omega = mps_L * MPS_TO_OMEGA;
	foc_R.target_omega = -mps_R * MPS_TO_OMEGA;
	foc_L.omega_setpoint = foc_L.target_omega;
	foc_R.omega_setpoint = foc_R.target_omega;
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
//	Steer_Motor();
	Steer_Motor_With_Anti_Oversteer();
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
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
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

void Drive_First() {
	if (!Drive_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();

	// ★ 추가: 라인 이탈 상태를 영구히 기억할 변수 선언
	uint8_t exit_reason_lost = 0;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event(cross))
				break;
		}
	}

	// ★ 추가: 루프를 빠져나온 즉시, 당시의 이탈 상태를 캡처하여 저장
	if (IR_Sensor.is_lost_position) {
		exit_reason_lost = 1;
	}

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;

	// ★ 수정: 변질될 위험이 있는 IR_Sensor 변수 대신, 캡처해둔 exit_reason_lost 사용
	if (exit_reason_lost) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
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
	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event_Second(CrossEvent_t cross,
		DriveSecondState_t *state) {
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
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
		LCD_Printf(0, 4, "T:%.2fs", lap_time); // ★ 랩타임 출력 추가
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}
