#include "main.h"
#include "cmsis_gcc.h"

#include "button.h"
#include "buzzer.h"
#include "lsm6ds.h"

#include "foc.h"
#include "motor.h"
#include "sensor.h"
#include "sd_ui.h"
#include "user_init.h"
#include "drive.h"

#define SENSOR_ADC_HANDLE		&hadc3
#define SENSOR_TIM_IR_HANDLE	&htim7
#define SENSOR_TIM_TRIG_HANDLE	&htim2

#define SENSOR_IR_HIGH()  (SENSOR_IR_EN_GPIO_Port->BSRR = SENSOR_IR_EN_Pin)
#define SENSOR_IR_LOW()   (SENSOR_IR_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_IR_EN_Pin << 16U))
#define SENSOR_PT_HIGH()  (SENSOR_PT_EN_GPIO_Port->BSRR = SENSOR_PT_EN_Pin)
#define SENSOR_PT_LOW()   (SENSOR_PT_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_PT_EN_Pin << 16U))

#define LINE_WEIGHT(n)      (uint8_t)(n)

#define LPTIM_TICK_DT 0.001f

// 1D convolution 기반 라인 state (0.0f = 기존 단순 threshold 방식)
#ifndef SENSOR_CONV_GAIN_DEFAULT
#define SENSOR_CONV_GAIN_DEFAULT 0.05f
#endif

#define CONV_KERNEL_LEN	3
#define CONV_IN_LEN		(LINE_N_SENSORS + 2)                    // M_L, IR_0..15, M_R
#define CONV_OUT_LEN	(CONV_IN_LEN + CONV_KERNEL_LEN - 1)     // full convolution length

// @formatter:off
static const uint8_t scan_group1[SCAN_GROUP_LEN] = { 7, 8, 10, 5, 3, 12, 14, 1, 16, 17 };
static const uint8_t scan_group2[SCAN_GROUP_LEN] = { 9, 6, 4, 11, 13, 2, 0, 15, 16, 17 };

const float32_t line_sensor_pos[LINE_N_SENSORS] = {
		-1.5f,
		-1.3f,
		-1.1f,
		-0.9f,
		-0.7f,
		-0.5f,
		-0.3f,
		-0.1f,
		 0.1f,
		 0.3f,
		 0.5f,
		 0.7f,
		 0.9f,
		 1.1f,
		 1.3f,
		 1.5f
};

volatile SensorData_TypeDef sensorData = {
        .idx = 0,
		.raw = { 0 },
		.blackmax = { 0 },
		.whitemax = { 0 },
        .normalized = { 0 },
		.target_pos = 0.0f,
		.state = 0,
		.threshold = 95,
        .line_lost_sum_min = 20,
		.mark_left = 0,
		.mark_right = 0,
		.conv = { 0 },
		.conv_gain = SENSOR_CONV_GAIN_DEFAULT,
};

volatile Sensor_TypeDef IR_Sensor = {
		.scan_group = 0,
		.is_calibration = 0,
		.is_lost_position = 0,
		.is_position = 0,
		.is_cross = 0,
		.data = &sensorData,
};
// @formatter:on

__attribute__((section(".ram_d3"), aligned(32)))         uint16_t adc3_buffer[1];

volatile uint32_t count_sensor_irq = 0;

volatile uint16_t buzzer_timer_count;
float32_t g_buzzer_duration = 0.05f;

__STATIC_INLINE uint8_t Scan_Slot_To_Phys(uint8_t slot) {
	if (slot < SCAN_CYCLE_LEN_HALF)
		return scan_group1[slot];
	return scan_group2[slot - SCAN_CYCLE_LEN_HALF];
}

void Sensor_Printf(uint8_t idx, volatile uint16_t *sensor_data) {
	LCD_Printf(8 * (idx & 0x1), idx / 2 + 1, "0x%03X", *(sensor_data + idx));
}

void Sensor_Start() {
	IR_Sensor.data->idx = 0;
	IR_Sensor.scan_group = 0;
	HAL_ADCEx_Calibration_Start(SENSOR_ADC_HANDLE, ADC_CALIB_OFFSET,
	ADC_SINGLE_ENDED);
	HAL_StatusTypeDef ret = HAL_ADC_Start_DMA(SENSOR_ADC_HANDLE,
			(uint32_t*) adc3_buffer, 1);
	if (ret != HAL_OK) {
		LCD_Printf(0, 15, "ADC ERR:%d", ret);
		return;
	}
	HAL_TIM_Base_Start_IT(SENSOR_TIM_IR_HANDLE);

}

void Sensor_Stop() {
	HAL_TIM_Base_Stop(SENSOR_TIM_TRIG_HANDLE);
	HAL_TIM_Base_Stop_IT(SENSOR_TIM_IR_HANDLE);
	HAL_ADC_Stop_DMA(SENSOR_ADC_HANDLE);
}

__STATIC_INLINE void Set_Mux_Channel_Fast(uint8_t index) {
	if (index & 0x01)
		SENSOR_MUX0_GPIO_Port->BSRR = SENSOR_MUX0_Pin;
	else
		SENSOR_MUX0_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX0_Pin << 16U;

	if (index & 0x02)
		SENSOR_MUX1_GPIO_Port->BSRR = SENSOR_MUX1_Pin;
	else
		SENSOR_MUX1_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX1_Pin << 16U;

	if (index & 0x04)
		SENSOR_MUX2_GPIO_Port->BSRR = SENSOR_MUX2_Pin;
	else
		SENSOR_MUX2_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX2_Pin << 16U;

	if (index & 0x08)
		SENSOR_MUX3_GPIO_Port->BSRR = SENSOR_MUX3_Pin;
	else
		SENSOR_MUX3_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX3_Pin << 16U;
}

__STATIC_INLINE uint16_t Sensor_Normalize(uint8_t idx) {
	const uint16_t raw = IR_Sensor.data->raw[idx];
	const uint16_t bmax = IR_Sensor.data->blackmax[idx];
	const uint16_t bias = IR_Sensor.data->normalized_coef_bias[idx];
	uint16_t diff = (raw > bmax) ? (raw - bmax) : 0;
	uint32_t result = ((uint32_t) diff * bias) >> 8;
	return (result > 255U) ? 255U : (uint8_t) result;
}

// ==============================================
// 1D Convolution (CMSIS-DSP arm_conv_f32)
// ==============================================
// 입력: [M_L(16), IR_0 .. IR_15, M_R(17)] (LineSim 뷰어 및 실제 센서 배치 일치)
// 커널: [-1, 2, -1] (1D 라플라시안: 라인 피크는 +, 완만한 어깨/배경 오프셋은 <= 0)
// arm_conv_f32를 통해 16개 메인 라인 센서의 공간 2차 미분(샤프닝) 신호를 고속 연산
static const float32_t conv_kernel[CONV_KERNEL_LEN] = { -1.0f, 2.0f, -1.0f };

static void Sensor_Conv_Update(void) {
	float32_t conv_in[CONV_IN_LEN];
	float32_t conv_out[CONV_OUT_LEN];
	volatile SensorData_TypeDef *d = IR_Sensor.data;

	conv_in[0] = (float32_t) d->normalized[LEFT_MARK_SENSOR_INDEX];
	for (uint8_t i = 0; i < LINE_N_SENSORS; i++) {
		conv_in[i + 1] = (float32_t) d->normalized[i];
	}
	conv_in[CONV_IN_LEN - 1] = (float32_t) d->normalized[RIGHT_MARK_SENSOR_INDEX];

	// CMSIS-DSP full convolution: out[n] = sum_k in[n-k] * h[k]
	// IR_i (= conv_in[i+1])에 대응하는 출력은 out[i+2]
	arm_conv_f32(conv_in, CONV_IN_LEN, conv_kernel, CONV_KERNEL_LEN, conv_out);

	for (uint8_t i = 0; i < LINE_N_SENSORS; i++) {
		d->conv[i] = (int16_t) conv_out[i + 2];
	}
}

void TIM7_IRQ_Handler() {
	uint8_t slot = IR_Sensor.data->idx % SCAN_CYCLE_LEN_HALF;
	if (slot < SCAN_GROUP_LEN) {
		uint8_t phys = Scan_Slot_To_Phys(IR_Sensor.data->idx);
		Set_Mux_Channel_Fast(phys);
		SENSOR_IR_LOW();
		SENSOR_PT_LOW();
	} else {
		SENSOR_IR_HIGH();
		SENSOR_PT_HIGH();
	}

	*SENSOR_TIM_TRIG_HANDLE.Instance->CR1 &= ~TIM_CR1_CEN;
	*SENSOR_TIM_TRIG_HANDLE.Instance->CNT = 0;
	*SENSOR_TIM_TRIG_HANDLE.Instance->SR = ~TIM_SR_UIF;
	*SENSOR_TIM_TRIG_HANDLE.Instance->CR1 |= TIM_CR1_CEN;
}

void ADC3_IRQ_Cplt_Handler() {
	uint8_t idx = IR_Sensor.data->idx;
	uint32_t state = IR_Sensor.data->state;
	uint8_t phys = Scan_Slot_To_Phys(idx);
	uint16_t raw = *adc3_buffer;
	static uint8_t normalized;
	if (IR_Sensor.is_calibration) {
		normalized = Sensor_Normalize(phys);
		state &= ~(0x01 << phys);
		state |= ((normalized > IR_Sensor.data->threshold) << phys);
		IR_Sensor.data->normalized[phys] = normalized;
		IR_Sensor.data->state = state;
	}
	IR_Sensor.data->raw[phys] = raw;
	idx = (idx + 1) % SCAN_CYCLE_LEN;
	IR_Sensor.data->idx = idx;

	// 스캔 그룹(반주기, 라인 센서 8개 + 마크 센서 2개) 완료 시점마다 CMSIS-DSP convolution 갱신
	if (IR_Sensor.is_calibration && (idx % SCAN_CYCLE_LEN_HALF) == 0) {
		Sensor_Conv_Update();
	}
}

typedef enum {
	MARKER_STATE_IDLE = 0,
	MARKER_STATE_READING,
	MARKER_STATE_WAIT_CLEAR,
} MarkerState_t;

static MarkerState_t g_marker_state = MARKER_STATE_IDLE;
static uint16_t g_left_on_count = 0;
static uint16_t g_right_on_count = 0;
static uint8_t g_left_src_mask = 0;
static uint8_t g_right_src_mask = 0;
static uint16_t g_accum_center_state = 0;
static uint8_t g_both_marker_coincident = 0;
static uint8_t g_is_curve_state = 0;
static uint16_t g_marker_on_count = 0;
static uint16_t g_marker_off_count = 0;
static int8_t g_marker_min_peak = 15;
static int8_t g_marker_max_peak = 0;

static int8_t prev_peak_idx = LINE_N_SENSORS / 2;
static float32_t prev_position = 0.0f; // 직전 절대 물리 위치 기억
static uint16_t lost_counter = 0;
static uint8_t force_full_scan = 0;
static uint8_t cross_state = 0;
static uint16_t g_cross_accum_mask = 0;
static uint8_t g_cross_event_pending = 0;

void Sensor_Reset_Position(void) {
	prev_peak_idx = LINE_N_SENSORS / 2;
	prev_position = 0.0f;
	lost_counter = 0;
	force_full_scan = 1;
	cross_state = 0;
	g_cross_accum_mask = 0;
	g_cross_event_pending = 0;
	IR_Sensor.is_cross = 0;
}

float32_t Sensor_Get_Last_Position(void) {
	return prev_position;
}

float32_t Sensor_Get_Position(void) {

	int8_t current_peak_idx = LINE_N_SENSORS / 2;
	uint8_t is_cross_line = 0;

	uint16_t state16 = (uint16_t) (IR_Sensor.data->state & 0xFFFF);

	float32_t best_metric = -1e9f;
	const float32_t cgain = IR_Sensor.data->conv_gain;

	if (force_full_scan || IR_Sensor.is_lost_position) {
		for (int8_t i = 0; i < LINE_N_SENSORS; i++) {
			float32_t metric = (float32_t) IR_Sensor.data->normalized[i];
			if (cgain > 0.0f) {
				metric += cgain * (float32_t) IR_Sensor.data->conv[i];
			}
			if (metric > best_metric) {
				best_metric = metric;
				current_peak_idx = i;
			}
		}
		force_full_scan = 0;
		cross_state = 0;
	} else {
		int8_t search_start = prev_peak_idx - POS_WINDOW_HALF;
		int8_t search_end = prev_peak_idx + POS_WINDOW_HALF;

		if (search_start < 0)
			search_start = 0;
		if (search_end >= LINE_N_SENSORS)
			search_end = LINE_N_SENSORS - 1;

		uint8_t window_len = search_end - search_start + 1;
		uint32_t window_mask = ((1U << window_len) - 1) << search_start;

		// 메인 라인과 갭(검은색 공간)으로 독립된 외곽 마커 비트 분리
		uint16_t left_marker_bits = 0;
		if (search_start >= 2) {
			uint8_t has_gap = 0;
			for (int8_t i = search_start - 1; i >= 0; i--) {
				if ((state16 & (1U << i)) == 0) {
					has_gap = 1;
				} else if (has_gap) {
					left_marker_bits |= (1U << i);
				}
			}
		}
		uint16_t right_marker_bits = 0;
		if (search_end <= 13) {
			uint8_t has_gap = 0;
			for (int8_t i = search_end + 1; i < LINE_N_SENSORS; i++) {
				if ((state16 & (1U << i)) == 0) {
					has_gap = 1;
				} else if (has_gap) {
					right_marker_bits |= (1U << i);
				}
			}
		}

		// 교차로 판정 시에는 갭으로 분리된 독립 사이드 마커 비트를 제외한 순수 라인 영역만 검사
		uint16_t line_check_state = state16 & ~(left_marker_bits | right_marker_bits);
		uint8_t active_in_window = __builtin_popcount(line_check_state & window_mask);
		uint8_t active_out_window = __builtin_popcount(line_check_state & ~window_mask);
		uint8_t total_active = active_in_window + active_out_window;

		uint8_t left_adjacent = 0, right_adjacent = 0;
		if (search_start >= 1 && (state16 & (1U << (search_start - 1)))) left_adjacent = 1;
		if (search_start >= 2 && (state16 & (1U << (search_start - 2)))) left_adjacent = 1;
		if (search_end + 1 < LINE_N_SENSORS && (state16 & (1U << (search_end + 1)))) right_adjacent = 1;
		if (search_end + 2 < LINE_N_SENSORS && (state16 & (1U << (search_end + 2)))) right_adjacent = 1;

		uint8_t has_continuous_cross = (left_adjacent || right_adjacent);

		// 크로스(교차로) 감지 조건:
		// 1) total_active >= 5: 단일 20mm 라인은 센서 간격(10.9mm)상 최대 3~4개만 켤 수 있으므로 5개 이상은 100% 교차선
		// 2) active_out_window >= 3: 윈도우 외곽 센서 3개 이상 활성은 교차선 통과
		// 3) has_continuous_cross와 결합된 윈도우 확장 조건
		uint8_t cross_condition = (total_active >= 5) || (active_out_window >= 3) ||
		                          (has_continuous_cross && (
		                              (total_active >= 4) ||
		                              (active_in_window >= 4)));

		if (cross_state == 0) {
			if (cross_condition) {
				// 크로스 진입: 진입 즉시 크로스 플래그를 세워 진입 순간 왜곡 위치가 prev_position에 오염되는 것 차단
				cross_state = 1;
				is_cross_line = 1;
				current_peak_idx = prev_peak_idx; // 메인 라인 피크 위치 보존 (zero position shift 시에도 피크 불변)
				g_cross_accum_mask = state16;
			} else {
				// 정상 단일 라인: 윈도우 내 피크 탐색
				for (int8_t i = search_start; i <= search_end; i++) {
					float32_t metric = (float32_t) IR_Sensor.data->normalized[i];
					if (cgain > 0.0f) {
						metric += cgain * (float32_t) IR_Sensor.data->conv[i];
					}
					if (metric > best_metric) {
						best_metric = metric;
						current_peak_idx = i;
					}
				}
				is_cross_line = 0;
			}
		} else {
			// 크로스 상태 유지 중: 교차선 통과 중 센서 누적
			g_cross_accum_mask |= state16;

			uint8_t markers_active = (IR_Sensor.data->state & ((1U << LEFT_MARK_SENSOR_INDEX) | (1U << RIGHT_MARK_SENSOR_INDEX))) != 0;
			uint8_t raw_out_window = __builtin_popcount(state16 & ~window_mask);

			// 탈출 조건: 교차선이 완전히 통과하여 단일 라인으로 복귀하고 외곽 마커 센서 및 메인바 외곽 센서(IR_0..15)가 완전히 꺼졌는지 확인
			if (!markers_active && raw_out_window == 0 && active_in_window <= 3) {
				cross_state = 0;
				is_cross_line = 0;

				// 교차로 통과 완료 판정: 교차선은 센서바 전역(0~15)을 가로지르므로 누적 센서 수 >= 12
				uint8_t cross_pop = __builtin_popcount(g_cross_accum_mask);
				if (cross_pop >= 12) {
					g_cross_event_pending = 1;
				}
				g_cross_accum_mask = 0;

				best_metric = -1e9f;
				for (int8_t i = search_start; i <= search_end; i++) {
					float32_t metric = (float32_t) IR_Sensor.data->normalized[i];
					if (cgain > 0.0f) {
						metric += cgain * (float32_t) IR_Sensor.data->conv[i];
					}
					if (metric > best_metric) {
						best_metric = metric;
						current_peak_idx = i;
					}
				}
			} else {
				is_cross_line = 1;
				// 크로스 통과 중: target_idx 등으로 튀지 않고 직전의 메인 라인 피크 위치를 그대로 고정
				current_peak_idx = prev_peak_idx;
			}
		}
	}

	prev_peak_idx = current_peak_idx;

	int8_t calc_start = current_peak_idx - POS_WINDOW_HALF;
	int8_t calc_end = current_peak_idx + POS_WINDOW_HALF;

	if (calc_start < 0)
		calc_start = 0;
	if (calc_end >= LINE_N_SENSORS)
		calc_end = LINE_N_SENSORS - 1;

	uint8_t mark_left = 0, mark_right = 0;

	// 1. 전용 외곽 마커 센서 (16: M_L, 17: M_R) - Bit 0
	if (IR_Sensor.data->state & (1U << LEFT_MARK_SENSOR_INDEX)) {
		mark_left |= 0x01;
	}
	if (IR_Sensor.data->state & (1U << RIGHT_MARK_SENSOR_INDEX)) {
		mark_right |= 0x01;
	}

	// 2. 센서바 최외곽 끝단(IR_0..2, IR_13..15) 보조 - Bit 1
	if (!is_cross_line && cross_state == 0) {
		if (current_peak_idx >= 3 && (state16 & 0x0007)) {
			mark_left |= 0x02;
		}
		if (current_peak_idx <= 12 && (state16 & 0xE000)) {
			mark_right |= 0x02;
		}
	}

	IR_Sensor.data->mark_left = mark_left;
	IR_Sensor.data->mark_right = mark_right;

	// 바닥 잔류 노이즈 플로어: 윈도우 내 최소 센서 값 탐색
	uint16_t min_noise = IR_Sensor.data->normalized[calc_start];
	uint32_t raw_sum = IR_Sensor.data->normalized[calc_start];
	for (int8_t i = calc_start + 1; i <= calc_end; i++) {
		uint16_t v = IR_Sensor.data->normalized[i];
		raw_sum += v;
		if (v < min_noise) {
			min_noise = v;
		}
	}

	float32_t weighted_sum = 0.0f;
	float32_t total_weight = 0.0f;

	for (int8_t i = calc_start; i <= calc_end; i++) {
		int32_t net_val = (int32_t) IR_Sensor.data->normalized[i] - (int32_t) min_noise;
		if (net_val > 0) {
			float32_t net_f = (float32_t) net_val;
			if (cgain > 0.0f && IR_Sensor.data->conv[i] > 0) {
				net_f += cgain * (float32_t) IR_Sensor.data->conv[i];
			}
			// 노이즈 플로어 차감 및 제곱 가중으로 계단형 점프(윈도우 경계 불연속) 제거
			float32_t w = net_f * net_f;
			weighted_sum += w * line_sensor_pos[i];
			total_weight += w;
		}
	}

	if (raw_sum > IR_Sensor.data->line_lost_sum_min) {
		IR_Sensor.is_lost_position = 0;
		lost_counter = 0;

		if (is_cross_line) {
			IR_Sensor.is_cross = 1;
			return prev_position;
		}
		IR_Sensor.is_cross = 0;

		float32_t current_position = (total_weight > 0.0f) ? (weighted_sum / total_weight) : line_sensor_pos[current_peak_idx];

		// 교차로 통과 직후 잔류 노이즈나 고주파 센서 스파이크에 대한 변화율 제한(Slew Rate Limiter)
		// 500us(2kHz) 제어 주기 기준 최대 0.015f (초당 약 30 units/s, 물리적 한계 약 1.6 m/s)
		float32_t pos_diff = current_position - prev_position;
		const float32_t MAX_POS_STEP = 0.015f;
		if (pos_diff > MAX_POS_STEP) {
			current_position = prev_position + MAX_POS_STEP;
		} else if (pos_diff < -MAX_POS_STEP) {
			current_position = prev_position - MAX_POS_STEP;
		}

		prev_position = current_position; // 정상 라인일 때 절대 위치 갱신

		return current_position;
	} else {
		force_full_scan = 1;
		lost_counter++;

		if (lost_counter > 50) {
			IR_Sensor.is_lost_position = 1;
			lost_counter = 0;
		}

		IR_Sensor.is_cross = 0;
		// 라인을 잃었을 때도 급격히 튀지 않도록 직전 위치 반환
		return prev_position;
	}
}

// ==============================================
// 마커 인식 알고리즘 (동적 오프셋 & 래핑 보정 적용)
// ==============================================

CrossMarkerLog_t g_cross_log[CROSS_LOG_MAX];
uint16_t g_cross_log_count = 0;

uint16_t g_last_stop_state = 0;
uint8_t g_last_stop_count = 0;

static float32_t g_prev_imu_yaw = 0.0f;
static float32_t g_continuous_yaw = 0.0f;
static float32_t g_segment_start_yaw = 0.0f;
static float32_t g_max_yaw_change = 0.0f;
static int16_t g_pending_log_idx = -1;
static float32_t g_pending_target_dist = 0.10f;
static float32_t g_post_cross_lockout_dist = 0.0f;
static void Marker_FSM_Reset(void) {
	g_marker_state = MARKER_STATE_IDLE;
	g_left_on_count = 0;
	g_right_on_count = 0;
	g_left_src_mask = 0;
	g_right_src_mask = 0;
	g_accum_center_state = 0;
	g_marker_on_count = 0;
	g_marker_off_count = 0;
	g_marker_min_peak = 15;
	g_marker_max_peak = 0;
	g_both_marker_coincident = 0;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

void Cross_Detect_Reset(void) {
	Marker_FSM_Reset();

	g_prev_imu_yaw = imu_data.Yaw_Angle;
	g_continuous_yaw = 0.0f;
	g_segment_start_yaw = 0.0f;
	g_max_yaw_change = 0.0f;
	g_pending_log_idx = -1;
	g_pending_target_dist = 0.10f;
	g_post_cross_lockout_dist = 0.0f;
}

CrossEvent_t Cross_Detect_Update(void) {
	float32_t dyaw = imu_data.Yaw_Angle - g_prev_imu_yaw;
	if (dyaw > 180.0f) dyaw -= 360.0f;
	if (dyaw < -180.0f) dyaw += 360.0f;
	g_continuous_yaw += dyaw;
	g_prev_imu_yaw = imu_data.Yaw_Angle;

	if (g_pending_log_idx >= 0) {
		float32_t current_yaw_change = g_continuous_yaw - g_segment_start_yaw;

		if (fabsf(current_yaw_change) > fabsf(g_max_yaw_change)) {
			g_max_yaw_change = current_yaw_change;
			g_cross_log[g_pending_log_idx].yaw_angle = g_max_yaw_change;
		}

		if (g_odom_distance_m >= g_pending_target_dist) {
			g_pending_log_idx = -1;
			g_segment_start_yaw = g_continuous_yaw;
			g_max_yaw_change = 0.0f;
		}
	} else {
		float32_t current_yaw_change = g_continuous_yaw - g_segment_start_yaw;
		if (fabsf(current_yaw_change) > fabsf(g_max_yaw_change)) {
			g_max_yaw_change = current_yaw_change;
		}
	}

	uint8_t left_marker = IR_Sensor.data->mark_left;
	uint8_t right_marker = IR_Sensor.data->mark_right;
	uint16_t current_center_state = (uint16_t) (IR_Sensor.data->state & 0xFFFF);
	CrossEvent_t event = CROSS_NONE;

	// 1. 교차로 통과 완료 이벤트 최우선 처리
	if (g_cross_event_pending) {
		g_cross_event_pending = 0;
		Marker_FSM_Reset();
		g_post_cross_lockout_dist = 0.15f; // 크로스 통과 후 최소 15cm 동안 교차선 꼬리/대각선 잔여물로 인한 사이드 마커 오인식 차단
		event = CROSS_CROSS;
	} else if (cross_state) {
		Marker_FSM_Reset();
	} else if (g_post_cross_lockout_dist > 0.0f) {
		Marker_FSM_Reset();
		if (g_odom_distance_m >= g_post_cross_lockout_dist) {
			g_post_cross_lockout_dist = 0.0f; // 15cm 경과 완료 -> 락아웃 해제
		}
	} else {
		// 2. 일반 사이드 마커 FSM (출발선/정지선 STOP 포함)
		switch (g_marker_state) {
		case MARKER_STATE_IDLE:
			if (left_marker || right_marker) {
				g_marker_state = MARKER_STATE_READING;
				g_left_on_count = left_marker ? 1 : 0;
				g_right_on_count = right_marker ? 1 : 0;
				g_left_src_mask = left_marker;
				g_right_src_mask = right_marker;
				g_marker_on_count = 1;
				g_marker_off_count = 0;
				g_marker_min_peak = prev_peak_idx;
				g_marker_max_peak = prev_peak_idx;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
			}
			break;
		case MARKER_STATE_READING:
			if (left_marker) {
				g_left_on_count++;
				g_left_src_mask |= left_marker;
			}
			if (right_marker) {
				g_right_on_count++;
				g_right_src_mask |= right_marker;
			}

			if (prev_peak_idx < g_marker_min_peak) g_marker_min_peak = prev_peak_idx;
			if (prev_peak_idx > g_marker_max_peak) g_marker_max_peak = prev_peak_idx;

			if (left_marker || right_marker) {
				g_marker_on_count++;
				g_marker_off_count = 0;
			} else {
				g_marker_off_count++;
				// 마커 통과 완료 디바운스: 센서간 핸드오프 및 대각선 진입 깜빡임 방지 (60사이클 = 3ms 연속 오프 시 확정)
				if (g_marker_off_count >= 60) {
					const uint16_t MIN_VALID_ON = 80; // 최소 4ms 지속 감지 시에만 정규 마커로 인정 (잔류 스파이크 차단)
					// 인접 트랙 스침(Ghost) 방어:
					// 차체가 중앙에 있을 때(peak <= 8) 발생한 M_R 단독 감지는 85mm 밖 인접 트랙을 스친 것이므로 기각.
					// 차체가 우측으로 쏠려(max_peak >= 9) 메인바 끝단을 벗어났을 때의 M_R 감지이거나,
					// 메인바(IR_13..15)가 함께 감지된 경우(src_mask & 0x02)는 정규 마커로 인정.
					uint8_t left_ok  = ((g_left_src_mask & 0x02) != 0) || (g_marker_min_peak <= 6);
					uint8_t right_ok = ((g_right_src_mask & 0x02) != 0) || (g_marker_max_peak >= 9);

					uint8_t valid_left = (g_left_on_count >= MIN_VALID_ON) && left_ok;
					uint8_t valid_right = (g_right_on_count >= MIN_VALID_ON) && right_ok;

					if (valid_left && valid_right) {
						event = CROSS_STOP;
						g_last_stop_state = current_center_state;
					} else if (valid_left) {
						event = CROSS_LEFT;
					} else if (valid_right) {
						event = CROSS_RIGHT;
					}

					// 마커 통과 후 센서가 테이프를 완전히 벗어날 때까지 대기 (동일 마커 중복 트리거 원천 차단)
					g_marker_state = MARKER_STATE_WAIT_CLEAR;
					g_marker_off_count = 0;
				}
			}
			break;
		case MARKER_STATE_WAIT_CLEAR:
			if (left_marker || right_marker) {
				g_marker_off_count = 0;
			} else {
				g_marker_off_count++;
				if (g_marker_off_count >= 20) { // 1ms 연속 클린 플로어 확인 후 IDLE 복귀
					Marker_FSM_Reset();
				}
			}
			break;
		}
	}

	if (event != CROSS_NONE) {
		Cross_Log_Push(event);

		if (g_cross_log_count > 0) {
			g_pending_log_idx = (g_cross_log_count - 1) % CROSS_LOG_MAX;
		}

		if (event == CROSS_CROSS || event == CROSS_STOP) {
			g_pending_target_dist = 0.10f;
		} else {
			if (g_is_curve_state == 0) {
				g_pending_target_dist = 0.05f;
			} else if ((g_is_curve_state == 1 && event == CROSS_LEFT) ||
					   (g_is_curve_state == 2 && event == CROSS_RIGHT)) {
				g_pending_target_dist = 0.12f;
			} else {
				g_pending_target_dist = 0.08f;
			}
		}

		Buzzer_Start();
		buzzer_timer_count = (uint16_t) (g_buzzer_duration / LPTIM_TICK_DT);

		if (event == CROSS_CROSS || event == CROSS_STOP) {
			g_is_curve_state = 0;
			LCD7789_Invert(0);
		} else if (event == CROSS_LEFT) {
			if (g_is_curve_state == 1) {
				g_is_curve_state = 0;
				LCD7789_Invert(0);
			} else {
				g_is_curve_state = 1;
				LCD7789_Invert(1);
			}
		} else if (event == CROSS_RIGHT) {
			if (g_is_curve_state == 2) {
				g_is_curve_state = 0;
				LCD7789_Invert(0);
			} else {
				g_is_curve_state = 2;
				LCD7789_Invert(1);
			}
		}
	}
	return event;
}

void Cross_Log_Push(CrossEvent_t type) {
	CrossMarkerLog_t *log = &g_cross_log[g_cross_log_count % CROSS_LOG_MAX];
	log->type = type;
	log->dist_from_prev_m = g_odom_distance_m;

	log->yaw_angle = g_max_yaw_change;

	g_cross_log_count++;
	Odom_Reset();
}

void Sensor_Calibration() {
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		IR_Sensor.data->whitemax[i] = 0;
		IR_Sensor.data->blackmax[i] = 0;
	}

	UserInput_t btn;
	FRESULT load_res = Sensor_Load_Calibration();
	LCD_Printf(0, 0, "Load:%d", load_res);
	HAL_Delay(500);
	LCD_Clear();

	IR_Sensor.is_calibration = 0;

	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "White Max");
	LCD_Printf(0, 10, "Clear: K Double");
	while ((btn = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		if (btn == INPUT_CMD_K_DOUBLE) {
			for (uint8_t j = 0; j < NUM_SENSORS; j++) {
				IR_Sensor.data->whitemax[j] = 0;
			}
		}
		if (IR_Sensor.data->whitemax[i] < IR_Sensor.data->raw[i]) {
			IR_Sensor.data->whitemax[i] = IR_Sensor.data->raw[i];
		}
		Sensor_Printf(i, IR_Sensor.data->whitemax);
		i = (i + 1) % NUM_SENSORS;
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);

	Sensor_Start();
	LCD_Printf(0, 0, "Black Max");
	LCD_Printf(0, 10, "Clear: K Double");

	while ((btn = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		if (btn == INPUT_CMD_K_DOUBLE) {
			for (uint8_t j = 0; j < NUM_SENSORS; j++) {
				IR_Sensor.data->blackmax[j] = 0;
			}
		}
		if (IR_Sensor.data->blackmax[i] < IR_Sensor.data->raw[i]) {
			IR_Sensor.data->blackmax[i] = IR_Sensor.data->raw[i];
		}
		Sensor_Printf(i, IR_Sensor.data->blackmax);
		i = (i + 1) % NUM_SENSORS;
	}

	for (i = 0; i < 18; i++) {
		const uint16_t wmax = IR_Sensor.data->whitemax[i];
		const uint16_t bmax = IR_Sensor.data->blackmax[i];
		uint16_t range = (wmax > bmax) ? (wmax - bmax) : 0;
		IR_Sensor.data->normalized_coef_bias[i] =
				(range > 0) ? (uint16_t) ((255U << 8) / range) : 0;
	}

	IR_Sensor.is_calibration = 1;

	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);

	FRESULT save_res = Sensor_Save_Calibration();
	LCD_Printf(0, 0, "Save:%d", save_res);
	HAL_Delay(500);
	LCD_Clear();

	Sensor_State_Printf();
}

void Sensor_Raw_Printf() {
	uint8_t old_calib = IR_Sensor.is_calibration;
	IR_Sensor.is_calibration = 0;

	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "Sensor Raw");
	UserInput_t bt;
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		Sensor_Printf(i, IR_Sensor.data->raw);
		LCD_Printf(0, 10, "%2d", IR_Sensor.data->idx);
		i = (i + 1) % 18;
	}

	LCD_Clear();
	Sensor_Stop();

	IR_Sensor.is_calibration = old_calib;
	Button_Wait_Release(&btn_k);
}

void Sensor_Normalize_Printf() {
	Sensor_Start();
	uint8_t i = 0;
	uint32_t sum = 0;
	LCD_Printf(0, 0, "Sensor Normal");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		if (i == 0)
			sum = 0;
		sum += IR_Sensor.data->normalized[i];
		Sensor_Printf(i, IR_Sensor.data->normalized);
		if (i == 15)
			LCD_Printf(0, 13, "%-5d", sum);
		i = (i + 1) % 18;
	}

	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void Sensor_State_Printf() {
	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "Sensor State");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		char state = (IR_Sensor.data->state & 0x01 << i) ? '1' : '0';
		if (i < LINE_N_SENSORS)
			LCD_Printf(i, 2, "%c", state);
		if (i == 16)
			LCD_Printf(0, 3, "%c", state);
		if (i == 17)
			LCD_Printf(15, 3, "%c", state);
		i = (i + 1) % 18;
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void Sensor_Conv_Printf() {
	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "Sensor Conv");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(8 * (i & 0x1), i / 2 + 1, "%+04d", (int) IR_Sensor.data->conv[i]);
		i = (i + 1) % LINE_N_SENSORS;
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void Sensor_Position_Printf() {
	Sensor_Start();
	LCD_Printf(0, 0, "Position");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		float32_t weighted_sum = 0.0f;
		uint32_t total_weight = 0;

		for (uint8_t i = 0; i < LINE_N_SENSORS; i++) {
			uint16_t weight = IR_Sensor.data->normalized[i];
			weighted_sum += (float32_t) weight * line_sensor_pos[i];
			total_weight += weight;
		}

		if (total_weight > IR_Sensor.data->line_lost_sum_min) {
			float32_t position = weighted_sum / (float32_t) total_weight;
			LCD_Printf(0, 2, "Pos: %+6.3f", position);
		} else {
			LCD_Printf(0, 2, "Pos: LOST   ");
		}

		LCD_Printf(0, 3, "Sum: %-5lu", total_weight);
		HAL_Delay(10);
	}

	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void IMU_Test(void) {
	LCD_Clear();
	LCD_Printf(0, 0, "IMU Test");
	LCD_Printf(0, 5, "[U] Cali [D] 0Yaw");
	LCD_Printf(0, 6, "[K Hold] Exit");

	LSM6DS3_Reset_Yaw();

	while (1) {
		uint8_t cmd = Button_Get_Input();
		if (cmd == INPUT_CMD_K_HOLD) {
			break;
		} else if (cmd == INPUT_CMD_U_SINGLE || cmd == INPUT_CMD_U_HOLD) {
			LCD_Printf(0, 4, "Calibrating...  ");
			LSM6DS3_Gyro_Calibrate_Z_Only();
			LCD_Printf(0, 4, "Cali OK!        ");
		} else if (cmd == INPUT_CMD_D_SINGLE || cmd == INPUT_CMD_D_HOLD) {
			LSM6DS3_Reset_Yaw();
			LCD_Printf(0, 4, "Yaw Reset!      ");
		}

		LCD_Printf(0, 1, "Yaw : %+7.2f deg", (double)imu_data.Yaw_Angle);
		LCD_Printf(0, 2, "Gz  : %+7.2f dps", (double)(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset));
		LCD_Printf(0, 3, "Ofst: %+7.2f dps", (double)imu_data.Gyro_Z_Offset);

		HAL_Delay(50);
	}

	LCD_Clear();
	Button_Wait_Release(&btn_k);
}

void IMU_Calibrate_Menu(void) {
	LCD_Clear();
	LCD_Printf(0, 0, "IMU Calibration");
	LCD_Printf(0, 2, "Keep Still 1s...");
	HAL_Delay(200);

	LSM6DS3_Gyro_Calibrate_Z_Only();

	LCD_Printf(0, 2, "Calibrated OK!  ");
	LCD_Printf(0, 4, "Offset: %+6.2fdps", (double)imu_data.Gyro_Z_Offset);
	LCD_Printf(0, 6, "[K] Exit");

	uint8_t btn;
	while ((btn = Button_Get_Input()) != INPUT_CMD_K_HOLD && btn != INPUT_CMD_K_SINGLE) {
		HAL_Delay(20);
	}
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}


void Sensor_SysID_Log(void) {
	LCD_Clear();
	LCD_Printf(0, 0, "Sensor SysID");
	LCD_Printf(0, 1, "Keep Still 1.0s");

	if (!IR_Sensor.is_calibration) {
		Sensor_Load_Calibration();
	}

	LSM6DS3_Gyro_Calibrate_Z_Only();

	Sensor_Start();
	HAL_Delay(20);

	FRESULT res = SD_SysID_Open(SYSID_SENSOR_PATH);
	if (res != FR_OK) {
		Sensor_Stop();
		LCD_Printf(0, 3, "SD Fail: %d", res);
		while (Button_Get_Input() != INPUT_CMD_K_HOLD)
			;
		LCD_Clear();
		return;
	}

	char line[320];
	int len = snprintf(line, sizeof(line), "# whitemax=");
	for (int ch = 0; ch < NUM_SENSORS; ch++) {
		len += snprintf(line + len, sizeof(line) - len, "%u%s",
				(unsigned) IR_Sensor.data->whitemax[ch],
				(ch + 1 < NUM_SENSORS) ? "," : "\n");
	}
	SD_SysID_WriteChunk(line, (UINT) len);

	len = snprintf(line, sizeof(line), "# blackmax=");
	for (int ch = 0; ch < NUM_SENSORS; ch++) {
		len += snprintf(line + len, sizeof(line) - len, "%u%s",
				(unsigned) IR_Sensor.data->blackmax[ch],
				(ch + 1 < NUM_SENSORS) ? "," : "\n");
	}
	SD_SysID_WriteChunk(line, (UINT) len);

	len = snprintf(line, sizeof(line),
			"# gyro_z_offset=%.4f\n"
			"sample,t_ms,raw_00,raw_01,raw_02,raw_03,raw_04,raw_05,raw_06,raw_07,"
			"raw_08,raw_09,raw_10,raw_11,raw_12,raw_13,raw_14,raw_15,raw_16,raw_17,"
			"gyro_z_dps,gyro_z_offset\n",
			imu_data.Gyro_Z_Offset);
	SD_SysID_WriteChunk(line, (UINT) len);

	float ir_mean[LINE_N_SENSORS] = { 0 };
	float ir_m2[LINE_N_SENSORS] = { 0 };
	float gz_mean = 0.0f;
	float gz_m2 = 0.0f;

	const int N_SAMPLES = 200;
	uint32_t t0 = HAL_GetTick();
	for (int k = 0; k < N_SAMPLES; k++) {
		uint32_t dt_ms = HAL_GetTick() - t0;
		float n_f = (float) (k + 1);

		len = snprintf(line, sizeof(line), "%d,%lu", k, (unsigned long) dt_ms);
		for (int ch = 0; ch < NUM_SENSORS; ch++) {
			uint16_t r = IR_Sensor.data->raw[ch];
			len += snprintf(line + len, sizeof(line) - len, ",%u", (unsigned) r);
			if (ch < LINE_N_SENSORS) {
				float d = (float) r - ir_mean[ch];
				ir_mean[ch] += d / n_f;
				ir_m2[ch] += d * ((float) r - ir_mean[ch]);
			}
		}
		float gz = imu_data.Gyro_Z;
		float d_gz = gz - gz_mean;
		gz_mean += d_gz / n_f;
		gz_m2 += d_gz * (gz - gz_mean);

		len += snprintf(line + len, sizeof(line) - len, ",%.4f,%.4f\n",
				gz, imu_data.Gyro_Z_Offset);
		SD_SysID_WriteChunk(line, (UINT) len);
		HAL_Delay(5);
	}

	FRESULT close_res = SD_SysID_Close();
	Sensor_Stop();

	float ir_std_sum = 0.0f;
	for (int ch = 0; ch < LINE_N_SENSORS; ch++) {
		ir_std_sum += sqrtf(ir_m2[ch] / (float) (N_SAMPLES - 1));
	}
	float ir_noise_lsb = ir_std_sum / (float) LINE_N_SENSORS;
	float gz_noise_dps = sqrtf(gz_m2 / (float) (N_SAMPLES - 1));

	LCD_Clear();
	LCD_Printf(0, 0, "Sensor SysID");
	LCD_Printf(0, 1, (close_res == FR_OK) ? "Saved SD OK!" : "Save Err:%d", close_res);
	LCD_Printf(0, 2, "/SysID_Data/");
	LCD_Printf(0, 3, "sensor_sysid.csv");
	LCD_Printf(0, 5, "IR Noise:%.1fLSB", ir_noise_lsb);
	LCD_Printf(0, 6, "Gz Noise:%.3fdps", gz_noise_dps);
	LCD_Printf(0, 7, "Gz Bias :%.3fdps", fabsf(gz_mean));
	LCD_Printf(0, 9, "[K Hold] Exit");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}
