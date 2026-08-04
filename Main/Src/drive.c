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

    // 3, 4회차용 추가 파라미터 초기화
    .turn45_len_s_m = 0.2f,
    .turn45_len_c_m = 0.3f,
    .turn90_len_s_m = 0.4f,
    .turn90_len_c_m = 0.5f,
    .add45_mps = 0.8f,
    .add90_mps = 0.4f,

    .zero_offset = 0.15f,
    .zero_out_turn_m = 0.1f,
    .zero_out_straight_m = 0.15f
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

		float total_straight_dist = 0.0f;
		for (uint16_t i = state->idx + 1; i < ref_log_count; i++) {
			total_straight_dist += ref_log[i].dist_from_prev_m;
			if (!seg_plan[i].accel_ok) {
				break;
			}
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

	return 0;
}

__STATIC_INLINE void Check_Distance_And_Brake(DriveSecondState_t *state) {
	if (state->accel_active && !state->braking_started) {
		float_t traveled = g_odom_distance_m - state->marker_start_dist;
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = driveData.base_mps;
		float_t brake_dist = 0.0f;

		if (v1 > v2) {
			brake_dist = (v1 * v1 - v2 * v2)
					/ (2.0f * driveData.decel)+ BRAKE_MARGIN_M;
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = driveData.base_mps;
			state->braking_started = 1;
		} else if (traveled >= ACCEL_START_MARGIN_M) {
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

			f_write(&file, "Index,Value\n", 12, &bw);

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

typedef enum {
	SEG_STRAIGHT = 0, SEG_TURN_45, SEG_TURN_90, SEG_TURN_LONG,
} SegmentKind_t;

typedef enum {
	SEG_DIR_STRAIGHT = 0, SEG_DIR_LEFT, SEG_DIR_RIGHT,
} SegmentDir_t;

typedef struct {
	uint8_t kind;
	uint8_t dir;
	float_t len_m;
	float_t cruise_mps;
	float_t zero_mid;
	float_t zero_end;
	float_t zero_out_m;
} SegmentPlan3_t;

static SegmentPlan3_t seg3[CROSS_LOG_MAX];

__STATIC_INLINE void Build_Segment_Direction(uint16_t count) {
	uint8_t curve_state = 0;

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

		seg3[i].len_m =
				(i + 1 < count) ? ref_log[i + 1].dist_from_prev_m : 0.0f;
	}
}

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
			seg3[i].zero_mid = -z;
			seg3[i].zero_end = (next_dir == SEG_DIR_RIGHT) ? 0.0f : -z;
			seg3[i].zero_out_m = driveData.zero_out_turn_m;
		} else if (seg3[i].dir == SEG_DIR_RIGHT) {
			seg3[i].zero_mid = z;
			seg3[i].zero_end = (next_dir == SEG_DIR_LEFT) ? 0.0f : z;
			seg3[i].zero_out_m = driveData.zero_out_turn_m;
		} else {
			seg3[i].zero_mid = 0.0f;
			if (next_dir == SEG_DIR_LEFT)
				seg3[i].zero_end = -z;
			else if (next_dir == SEG_DIR_RIGHT)
				seg3[i].zero_end = z;
			else
				seg3[i].zero_end = 0.0f;
			seg3[i].zero_out_m = driveData.zero_out_straight_m;
		}

		if (ref_log[i].type == CROSS_STOP
				|| (i + 1 < count && ref_log[i + 1].type == CROSS_STOP)) {
			seg3[i].zero_mid = 0.0f;
			seg3[i].zero_end = 0.0f;
		}
	}
}

// 3회차 상태 추적용 구조체
typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t braking_started;
	float marker_start_dist;
	float current_shifted_pos;
	float target_shifted_pos;
} DriveThirdState_t;

// 3회차 초기화 시퀀스
__STATIC_INLINE uint8_t Drive_Third_Init_Sequence(void) {
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

	Build_Segment_Direction(ref_log_count);
	Build_Segment_Kind(ref_log_count);
	Build_Segment_Speed(ref_log_count);
	Build_Segment_ZeroShift(ref_log_count, 1);

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

// 3회차 마커 이벤트 처리기
__STATIC_INLINE uint8_t Process_Marker_Event_Third(CrossEvent_t cross, DriveThirdState_t *state) {
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2) return 1;
	} else {
		if (cross == CROSS_LEFT) g_total_L++;
		else if (cross == CROSS_RIGHT) g_total_R++;
		else if (cross == CROSS_CROSS) g_total_C++;
	}

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;
	state->idx++;

	if (!state->mismatch && state->idx < ref_log_count) {
		g_target_base_mps = seg3[state->idx].cruise_mps;
	} else {
		g_target_base_mps = driveData.base_mps;
	}
	return 0;
}

// 영점 이동 및 거리 제어 로직
__STATIC_INLINE void Check_Distance_And_Control_Third(DriveThirdState_t *state) {
	if (state->mismatch || state->idx >= ref_log_count) {
		g_target_base_mps = driveData.base_mps;
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	float traveled = g_odom_distance_m - state->marker_start_dist;
	float remaining = seg3[state->idx].len_m - traveled;

	float v1 = g_current_base_mps;
	float v2 = (state->idx + 1 < ref_log_count) ? seg3[state->idx + 1].cruise_mps : driveData.base_mps;
	float brake_dist = 0.0f;

	if (v1 > v2) {
		brake_dist = (v1 * v1 - v2 * v2) / (2.0f * driveData.decel) + BRAKE_MARGIN_M;
	}

	if (remaining <= brake_dist && state->idx + 1 < ref_log_count) {
		g_target_base_mps = v2;
	} else {
		g_target_base_mps = seg3[state->idx].cruise_mps;
	}

	float z_mid = seg3[state->idx].zero_mid;
	float z_end = seg3[state->idx].zero_end;
	float z_out_m = seg3[state->idx].zero_out_m;

	float zero_shift_rate_m_per_unit = 0.25f;
	float zero_end_safe_dist = fabsf(z_end - state->current_shifted_pos) * zero_shift_rate_m_per_unit;

	if (remaining <= (zero_end_safe_dist + z_out_m) && state->idx + 1 < ref_log_count) {
		state->target_shifted_pos = z_end;
	} else {
		state->target_shifted_pos = z_mid;
	}

	float shift_step = 0.01f * (g_current_base_mps > 0.1f ? g_current_base_mps : 0.1f);

	if (state->target_shifted_pos >= state->current_shifted_pos) {
		if (shift_step >= state->target_shifted_pos - state->current_shifted_pos) {
			state->current_shifted_pos = state->target_shifted_pos;
		} else {
			state->current_shifted_pos += shift_step;
		}
	} else {
		if (shift_step >= state->current_shifted_pos - state->target_shifted_pos) {
			state->current_shifted_pos = state->target_shifted_pos;
		} else {
			state->current_shifted_pos -= shift_step;
		}
	}

	IR_Sensor.data->target_pos = state->current_shifted_pos;
}

// 실제 Drive_Third 메인 루프
void Drive_Third(void) {
	if (!Drive_Third_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveThirdState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;
	state.current_shifted_pos = 0.0f;
	state.target_shifted_pos = 0.0f;

	if (ref_log_count > 0) {
		g_target_base_mps = seg3[0].cruise_mps;
	}

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Third(cross, &state))
				break;
		}

		Check_Distance_And_Control_Third(&state);

		HAL_Delay(1);
	}

	LCD7789_Invert(0);
	IR_Sensor.data->target_pos = 0.0f;

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

void Drive_Fourth() {
}
