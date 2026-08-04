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
    .base_mps = 1.2f,
    .max_mps = 10.f,
    .accel = 6.f,
    .decel = 6.f,
    .steer_gain_p = 16.4f,
    .steer_gain_d = 0.1f,
    .pos_atten_gain = 0.0f,
    .pit_in_distance_m = 0.15f,
    .fan_en = 0,
	.turn45_len_s_m = 0.19f,
	.turn45_len_c_m = 0.30f,
	.turn90_len_s_m = 0.35f,
	.turn90_len_c_m = 0.46f,
	.add45_mps = 0.8f,
	.add90_mps = 0.4f,
	.zero_offset = 0.33f,
	.zero_shift_rate = 2.0f,
	.zero_out_turn_m = 0.10f,
	.zero_out_straight_m = 0.15f,
};
// @formatter:on

uint8_t g_total_L = 0;
uint8_t g_total_R = 0;
uint8_t g_total_C = 0;
uint8_t g_total_STOP = 0;

// ===== 영점이동 상태 (4회차 주행에서만 사용) =====
// g_line_offset_rate 가 0 이면 슬루가 멈추므로 1~3회차 주행은 영향을 받지 않는다.
volatile float_t g_line_offset = 0.0f;
volatile float_t g_line_offset_target = 0.0f;
volatile float_t g_line_offset_rate = 0.0f;

// 오프셋을 목표값 쪽으로 조금씩 옮긴다.
// ★ 증분이 시간이 아니라 "전진 거리"에 비례하므로, 주행 속도와 무관하게
//   지면에 그려지는 이동 궤적이 항상 같다.
__STATIC_INLINE void Line_Offset_Update(void) {
	if (g_line_offset_rate <= 0.0f) {
		return;
	}

	// 이번 주기에 옮길 수 있는 최대량 = (1m당 이동량) x (이번 주기 전진 거리)
	float_t step = g_line_offset_rate * g_current_base_mps * RAMP_DT;
	float_t diff = g_line_offset_target - g_line_offset;

	if (diff > step)
		g_line_offset += step;
	else if (diff < -step)
		g_line_offset -= step;
	else
		g_line_offset = g_line_offset_target;
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
	Line_Offset_Update();
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
__STATIC_INLINE uint8_t Drive_Init_Sequence(void) {
	LCD7789_Invert(0);

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

	// ★ 수정: 변질될 위험이 있는 IR_Sensor 변수 대신, 캡처해둔 exit_reason_lost 사용
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
	LCD7789_Invert(0);

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

void Drive_Second(void) {
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
		LCD_Printf(0, 4, "T:%.2fs", lap_time); // ★ 랩타임 출력 추가
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

	// 3. 정지 및 모터 릴리즈
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

			// CSV 헤더 작성
			f_write(&file, "Index,Value\n", 12, &bw);

			// 한 줄씩 변환하여 기록 (대용량 RAM 오버플로우 방지)
			for (uint32_t i = 0; i < vib_sample_count; i++) {
				int len = snprintf(line_buf, sizeof(line_buf), "%lu,%.4f\n", i,
						vib_data_buf[i]);
				f_write(&file, line_buf, len, &bw);
			}
			f_close(&file);
			LCD_Printf(0, 1, "Saved!");
		} else {
			LCD_Printf(0, 1, "SD Open Fail");
		}

		// ★ 수정: 사용 완료 후 반드시 언마운트
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

// ============================================================================
// 3 / 4회차 주행 공용 : 구간 계획
// ============================================================================
// 2회차는 "직선 구간만 가속"이었다.
// 3회차는 곡선을 길이로 45도 / 90도 / 긴 곡선으로 나누고, 짧은 곡선은
// 감속하지 않고 오히려 가산 속도를 얹어 통과한다.
// 4회차는 여기에 곡선 안쪽으로 붙는 영점이동을 더한다.

typedef enum {
	SEG_STRAIGHT = 0, SEG_TURN_45, SEG_TURN_90, SEG_TURN_LONG,
} SegmentKind_t;

typedef enum {
	SEG_DIR_STRAIGHT = 0, SEG_DIR_LEFT, SEG_DIR_RIGHT,
} SegmentDir_t;

typedef struct {
	uint8_t kind;			// SegmentKind_t
	uint8_t dir;			// SegmentDir_t
	float_t len_m;			// 이 구간의 길이 (마커 i -> i+1)
	float_t cruise_mps;		// 이 구간을 통과할 목표 속도
	float_t zero_mid;		// 구간 주행 중 유지할 영점 오프셋
	float_t zero_end;		// 구간 탈출 시 취할 영점 오프셋
	float_t zero_out_m;		// 탈출 전환에 쓸 추가 여유 거리
} SegmentPlan3_t;

static SegmentPlan3_t seg3[CROSS_LOG_MAX];

// 마커 종류로부터 각 구간이 직선인지 곡선인지, 곡선이면 어느 방향인지 판정한다.
// ★ 2회차 Build_Segment_Plan() 과 동일한 상태 천이 규칙을 사용한다.
//   (L 마커가 좌곡선을 열고, 다음 L 마커가 그 곡선을 닫는다)
__STATIC_INLINE void Build_Segment_Direction(uint16_t count) {
	uint8_t curve_state = 0; // 0: 직선, 1: 좌곡선 중, 2: 우곡선 중

	for (uint16_t i = 0; i < count; i++) {
		CrossEvent_t cur = ref_log[i].type;

		if (cur == CROSS_CROSS) {
			curve_state = 0;
		} else if (cur == CROSS_LEFT) {
			curve_state = (curve_state == 1) ? 0 : 1;
		} else if (cur == CROSS_RIGHT) {
			curve_state = (curve_state == 2) ? 0 : 2;
		}

		if (curve_state == 1)
			seg3[i].dir = SEG_DIR_LEFT;
		else if (curve_state == 2)
			seg3[i].dir = SEG_DIR_RIGHT;
		else
			seg3[i].dir = SEG_DIR_STRAIGHT;

		// 구간 길이는 "다음 마커까지의 거리"이다. 마지막 구간은 알 수 없다.
		seg3[i].len_m =
				(i + 1 < count) ? ref_log[i + 1].dist_from_prev_m : 0.0f;
	}
}

// 곡선 구간의 길이로 45도 / 90도 / 긴 곡선을 판별한다.
// 다음 구간이 곡선이면 마커 간 거리가 더 길게 잡히므로 임계값을 따로 쓴다.
__STATIC_INLINE void Build_Segment_Kind(uint16_t count) {
	for (uint16_t i = 0; i < count; i++) {
		if (seg3[i].dir == SEG_DIR_STRAIGHT) {
			seg3[i].kind = SEG_STRAIGHT;
			continue;
		}

		uint8_t next_is_curve = (i + 1 < count)
				&& (seg3[i + 1].dir != SEG_DIR_STRAIGHT);
		float_t len = seg3[i].len_m;
		float_t th45 =
				next_is_curve ?
						driveData.turn45_len_c_m : driveData.turn45_len_s_m;
		float_t th90 =
				next_is_curve ?
						driveData.turn90_len_c_m : driveData.turn90_len_s_m;

		// 길이를 모르는 마지막 구간은 안전하게 긴 곡선으로 본다.
		if (len <= 0.0f)
			seg3[i].kind = SEG_TURN_LONG;
		else if (len < th45)
			seg3[i].kind = SEG_TURN_45;
		else if (len < th90)
			seg3[i].kind = SEG_TURN_90;
		else
			seg3[i].kind = SEG_TURN_LONG;
	}
}

// 구간 종류별 통과 속도를 배정한다.
__STATIC_INLINE void Build_Segment_Speed(uint16_t count) {
	for (uint16_t i = 0; i < count; i++) {
		float_t v;

		switch (seg3[i].kind) {
		case SEG_STRAIGHT:
			v = driveData.max_mps;
			break;
		case SEG_TURN_45:
			v = driveData.base_mps + driveData.add45_mps;
			break;
		case SEG_TURN_90:
			v = driveData.base_mps + driveData.add90_mps;
			break;
		default:
			v = driveData.base_mps;
			break;
		}

		// ★ 정지 마커가 걸린 구간은 가산 속도 없이 기본 속도로만 통과한다.
		if (ref_log[i].type == CROSS_STOP
				|| (i + 1 < count && ref_log[i + 1].type == CROSS_STOP)) {
			v = driveData.base_mps;
		}

		if (v > driveData.max_mps)
			v = driveData.max_mps;
		if (v < driveData.base_mps)
			v = driveData.base_mps;

		seg3[i].cruise_mps = v;
	}
}

// 구간별 영점 오프셋을 미리 계획한다. (4회차 전용)
// 규칙은 "항상 다음에 올 곡선의 안쪽으로 미리 붙는다".
// ★ 부호: 음수 = 로봇이 왼쪽으로, 양수 = 로봇이 오른쪽으로.
__STATIC_INLINE void Build_Segment_ZeroShift(uint16_t count, uint8_t enable) {
	const float_t z = driveData.zero_offset;

	for (uint16_t i = 0; i < count; i++) {
		if (!enable) {
			seg3[i].zero_mid = 0.0f;
			seg3[i].zero_end = 0.0f;
			seg3[i].zero_out_m = 0.0f;
			continue;
		}

		uint8_t next_dir = (i + 1 < count) ? seg3[i + 1].dir : SEG_DIR_STRAIGHT;

		if (seg3[i].dir == SEG_DIR_LEFT) {
			seg3[i].zero_mid = -z;	// 좌곡선 안쪽 = 왼쪽
			// 다음이 우회전이면 중앙으로 빠져나가고, 아니면 그대로 유지
			seg3[i].zero_end = (next_dir == SEG_DIR_RIGHT) ? 0.0f : -z;
			seg3[i].zero_out_m = driveData.zero_out_turn_m;
		} else if (seg3[i].dir == SEG_DIR_RIGHT) {
			seg3[i].zero_mid = z;	// 우곡선 안쪽 = 오른쪽
			seg3[i].zero_end = (next_dir == SEG_DIR_LEFT) ? 0.0f : z;
			seg3[i].zero_out_m = driveData.zero_out_turn_m;
		} else {
			seg3[i].zero_mid = 0.0f;	// 직선에서는 가운데
			// 다음 곡선 방향으로 미리 붙어서 빠져나간다
			if (next_dir == SEG_DIR_LEFT)
				seg3[i].zero_end = -z;
			else if (next_dir == SEG_DIR_RIGHT)
				seg3[i].zero_end = z;
			else
				seg3[i].zero_end = 0.0f;
			seg3[i].zero_out_m = driveData.zero_out_straight_m;
		}

		// ★ 정지 마커 주변에서는 영점이동을 하지 않는다.
		if (ref_log[i].type == CROSS_STOP
				|| (i + 1 < count && ref_log[i + 1].type == CROSS_STOP)) {
			seg3[i].zero_mid = 0.0f;
			seg3[i].zero_end = 0.0f;
		}
	}
}

// ============================================================================
// 3 / 4회차 주행 : 초기화 및 주행 루프
// ============================================================================
typedef struct {
	uint16_t idx;			// 다음에 볼 마커 번호
	uint8_t mismatch;		// 맵과 어긋났는가
	uint8_t braking_started;
	uint8_t zero_exit_started;
	uint8_t use_zero_shift;	// 4회차이면 1
	float_t marker_start_dist;
	float_t seg_len;
	float_t cruise_mps;
	float_t exit_mps;
} DriveThirdState_t;

__STATIC_INLINE uint8_t Drive_Third_Init_Sequence(uint8_t use_zero_shift) {
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

	// ★ 로그가 버퍼보다 많이 쌓였으면 잘라낸다. 넘치면 배열 밖을 건드린다.
	if (ref_log_count > CROSS_LOG_MAX)
		ref_log_count = CROSS_LOG_MAX;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		ref_log[i] = g_cross_log[i];
	}

	Build_Segment_Direction(ref_log_count);
	Build_Segment_Kind(ref_log_count);
	Build_Segment_Speed(ref_log_count);
	Build_Segment_ZeroShift(ref_log_count, use_zero_shift);

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

	// ★ 마커 부저를 자동으로 끄는 LPTIM. 이걸 켜지 않으면 마커에서 울린 부저가
	//   계속 울린다. (1회차 Drive_Init_Sequence 와 동일)
	Buzzer_Discount_Start();

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();	// 여기서 g_line_offset 계열이 모두 0으로 초기화된다

	// 4회차이면 영점이동 슬루를 켠다.
	if (use_zero_shift)
		g_line_offset_rate = driveData.zero_shift_rate;

	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

// 맵과 어긋났을 때의 안전 복귀. 가속과 영점이동을 모두 끈다.
__STATIC_INLINE void Drive_Third_Fallback(DriveThirdState_t *state) {
	state->mismatch = 1;
	state->cruise_mps = driveData.base_mps;
	state->exit_mps = driveData.base_mps;
	state->seg_len = 0.0f;
	g_target_base_mps = driveData.base_mps;
	g_line_offset_target = 0.0f;
}

__STATIC_INLINE uint8_t Process_Marker_Event_Third(CrossEvent_t cross,
		DriveThirdState_t *state) {
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
			// ★ 맵과 어긋났다. 이후로는 기본 속도로만 달린다.
			Drive_Third_Fallback(state);
		}
	}

	if (!state->mismatch && (state->idx + 1 < ref_log_count)) {
		const SegmentPlan3_t *seg = &seg3[state->idx];

		state->seg_len = seg->len_m;
		state->cruise_mps = seg->cruise_mps;
		state->exit_mps = seg3[state->idx + 1].cruise_mps;

		// 구간 진입 시에는 일단 구간 중 자세를 목표로 잡는다.
		if (state->use_zero_shift)
			g_line_offset_target = seg->zero_mid;
	} else {
		state->cruise_mps = driveData.base_mps;
		state->exit_mps = driveData.base_mps;
		state->seg_len = 0.0f;
		g_target_base_mps = driveData.base_mps;
		if (state->use_zero_shift)
			g_line_offset_target = 0.0f;
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;
	state->zero_exit_started = 0;
	state->idx++;

	return 0;
}

// 구간 안에서의 가감속 판단. 2회차 Check_Distance_And_Brake 를 임의 속도로 확장한 것.
__STATIC_INLINE void Check_Distance_And_Brake_Third(DriveThirdState_t *state) {
	if (state->mismatch || state->seg_len <= 0.0f)
		return;

	float_t traveled = g_odom_distance_m - state->marker_start_dist;
	float_t remaining = state->seg_len - traveled;

	if (!state->braking_started) {
		float_t v1 = g_current_base_mps;
		float_t v2 = state->exit_mps;
		float_t brake_dist = 0.0f;

		// 지금 속도에서 다음 구간 진입 속도까지 줄이는 데 필요한 거리
		// ★ decel 이 0 이면 0으로 나누게 되므로 반드시 막는다.
		if (v1 > v2 && driveData.decel > 0.0f) {
			brake_dist = (v1 * v1 - v2 * v2)
					/ (2.0f * driveData.decel)+ BRAKE_MARGIN_M;
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = v2;
			state->braking_started = 1;
		} else if (traveled >= ACCEL_START_MARGIN_M) {
			// 마커를 완전히 빠져나온 뒤부터 이 구간의 통과 속도로 올린다
			g_target_base_mps = state->cruise_mps;
		}
	}
}

// 구간 안에서의 영점이동 전환 판단. (4회차 전용)
__STATIC_INLINE void Check_Distance_And_Zero_Shift(DriveThirdState_t *state) {
	if (!state->use_zero_shift || state->mismatch || state->seg_len <= 0.0f)
		return;
	if (state->zero_exit_started || state->idx == 0)
		return;

	const SegmentPlan3_t *seg = &seg3[state->idx - 1];

	float_t traveled = g_odom_distance_m - state->marker_start_dist;
	float_t remaining = state->seg_len - traveled;

	// ★ 탈출 오프셋까지 옮기는 데 필요한 전진 거리.
	//   이 항을 빼먹으면 이동이 끝나기 전에 구간이 끝나버린다.
	float_t shift_need_m = 0.0f;
	if (driveData.zero_shift_rate > 0.0f) {
		float_t gap = seg->zero_end - g_line_offset;
		if (gap < 0.0f)
			gap = -gap;
		shift_need_m = gap / driveData.zero_shift_rate;
	}

	if (remaining <= (shift_need_m + seg->zero_out_m)) {
		g_line_offset_target = seg->zero_end;
		state->zero_exit_started = 1;
	}
}

__STATIC_INLINE void Drive_Third_Common(uint8_t use_zero_shift,
		const char *title) {
	if (!Drive_Third_Init_Sequence(use_zero_shift))
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveThirdState_t state = { 0 };
	state.use_zero_shift = use_zero_shift;
	state.marker_start_dist = g_odom_distance_m;
	state.cruise_mps = driveData.base_mps;
	state.exit_mps = driveData.base_mps;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Third(cross, &state))
				break;
		}
		Check_Distance_And_Brake_Third(&state);
		Check_Distance_And_Zero_Shift(&state);
	}

	// ★ 정지 구간에서는 영점을 중앙으로 되돌린다.
	//   여기서 g_line_offset 을 곧바로 0 으로 써버리면 아직 달리는 중에
	//   조향이 크게 튀므로, 슬루는 살려둔 채 목표만 중앙으로 준다.
	g_line_offset_target = 0.0f;

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	// 완전히 멈춘 뒤 정리한다. rate 를 먼저 꺼야 ISR 이 다시 건드리지 않는다.
	g_line_offset_rate = 0.0f;
	g_line_offset = 0.0f;
	g_line_offset_target = 0.0f;

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
		LCD_Printf(0, 0, "%s", title);
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
		LCD_Printf(0, 5, "Miss:%d", state.mismatch);
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}

void Drive_Third() {
	Drive_Third_Common(0, "End(3rd)");
}

void Drive_Fourth() {
	Drive_Third_Common(1, "End(4th)");
}
