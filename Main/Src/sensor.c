#include "main.h"
#include "cmsis_gcc.h"

#include "button.h"
#include "buzzer.h"
#include "lsm6ds3tr-c.h"

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

// @formatter:off
static const uint8_t scan_group1[SCAN_GROUP_LEN] = { 7, 8, 10, 5, 3, 12, 14, 1, 16, 17 };
static const uint8_t scan_group2[SCAN_GROUP_LEN] = { 9, 6, 4, 11, 13, 2, 0, 15, 16, 17 };

// ★ float -> float32_t 로 변경
const float32_t line_sensor_pos[LINE_N_SENSORS] = {
		 -1.0f,
		 -13.0f / 15.0f,
		 -11.0f / 15.0f,
		 -9.0f / 15.0f,
		 -7.0f / 15.0f,
		 -5.0f / 15.0f,
		 -3.0f / 15.0f,
		 -1.0f / 15.0f,
		 1.0f / 15.0f,
		 3.0f / 15.0f,
		 5.0f / 15.0f,
		 7.0f / 15.0f,
		 9.0f / 15.0f,
		 11.0f / 15.0f,
		 13.0f / 15.0f,
		 1.0f
};

__STATIC_INLINE uint8_t Scan_Slot_To_Phys(uint8_t slot) {
	if (slot < SCAN_CYCLE_LEN_HALF) return scan_group1[slot];
	return scan_group2[slot - SCAN_CYCLE_LEN_HALF];
}

__attribute__((section(".ram_d3"), aligned(32))) uint16_t adc3_buffer[1];

volatile SensorData_TypeDef sensorData = {
        .idx = 0,
		.raw = { 0 },
		.blackmax = { 0 },
		.whitemax = { 0 },
        .normalized = { 0 },
		.target_pos = 0.0f,
		.state = 0,
		.threshold = 100,
        .line_lost_sum_min = 20,
		.mark_left = 0,
		.mark_right = 0,
};

volatile Sensor_TypeDef IR_Sensor = {
		.scan_group = 0,
		.is_calibration = 0,
		.is_lost_position = 0,
		.is_position = 0,
		.data = &sensorData,
};
// @formatter:on

volatile uint32_t count_sensor_irq = 0;

volatile uint16_t buzzer_timer_count;
float32_t g_buzzer_duration = 0.05f;

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
}

typedef enum {
	MARKER_STATE_IDLE = 0, MARKER_STATE_READING,
} MarkerState_t;

static MarkerState_t g_marker_state = MARKER_STATE_IDLE;
static uint8_t g_accum_left = 0;
static uint8_t g_accum_right = 0;
static uint16_t g_accum_center_state = 0;
static uint8_t g_is_curve_state = 0;

// ★ 반환형 및 내부 실수형 변수를 모두 float32_t로 통일
float32_t Sensor_Get_Position(void) {
	static int8_t prev_peak_idx = LINE_N_SENSORS / 2;
	static float32_t prev_position = 0.0f;

	static uint16_t lost_counter = 0;
	static uint8_t force_full_scan = 0;

	uint16_t max_val = 0;
	int8_t current_peak_idx = LINE_N_SENSORS / 2;
	uint8_t is_cross_line = 0;

	uint16_t state16 = (uint16_t) (IR_Sensor.data->state & 0xFFFF);

	if (force_full_scan || IR_Sensor.is_lost_position) {
		for (int8_t i = 0; i < LINE_N_SENSORS; i++) {
			if (IR_Sensor.data->normalized[i] > max_val) {
				max_val = IR_Sensor.data->normalized[i];
				current_peak_idx = i;
			}
		}
		force_full_scan = 0;
	} else {
		int8_t search_start = prev_peak_idx - POS_WINDOW_HALF;
		int8_t search_end = prev_peak_idx + POS_WINDOW_HALF;

		if (search_start < 0)
			search_start = 0;
		if (search_end >= LINE_N_SENSORS)
			search_end = LINE_N_SENSORS - 1;

		uint8_t window_len = search_end - search_start + 1;
		uint32_t window_mask = ((1U << window_len) - 1) << search_start;
		uint8_t active_in_window = __builtin_popcount(state16 & window_mask);

		for (int8_t i = search_start; i <= search_end; i++) {
			if (IR_Sensor.data->normalized[i] > max_val) {
				max_val = IR_Sensor.data->normalized[i];
				current_peak_idx = i;
			}
		}

		// ★ 판단: 십자 마커 발견 시
		if (active_in_window > 6) {
			current_peak_idx = prev_peak_idx;
			max_val = IR_Sensor.data->normalized[current_peak_idx];
			is_cross_line = 1;
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
	for (int8_t i = 0; i < calc_start; i++) {
		if (state16 & (1U << i)) {
			mark_left = 1;
			break;
		}
	}
	for (int8_t i = calc_end + 1; i < LINE_N_SENSORS; i++) {
		if (state16 & (1U << i)) {
			mark_right = 1;
			break;
		}
	}
	IR_Sensor.data->mark_left = mark_left;
	IR_Sensor.data->mark_right = mark_right;

	uint32_t scale_factor = 256;
	if (max_val > 20 && max_val < 255) {
		scale_factor = (255U << 8) / max_val;
	}

	float32_t weighted_sum = 0.0f;
	uint32_t total_weight = 0;

	for (int8_t i = calc_start; i <= calc_end; i++) {
		uint32_t amplified_val = (IR_Sensor.data->normalized[i] * scale_factor)
				>> 8;
		if (amplified_val > 255)
			amplified_val = 255;
		// ★ 형변환을 float32_t로 명시
		weighted_sum += (float32_t) amplified_val * line_sensor_pos[i];
		total_weight += amplified_val;
	}

	// ★ 핵심: max_val > 40 조건을 추가하여 미세 노이즈가 라인으로 둔갑하는 현상 차단
	if (total_weight > IR_Sensor.data->line_lost_sum_min && max_val > 40) {
		IR_Sensor.is_lost_position = 0;
		lost_counter = 0;

		// ★ 핵심: 십자 마커에서는 이전 위치를 유지하여 흔들림 최소화 (앞선 수정안 반영 시)
		if (is_cross_line) {
			return prev_position;
		}

		// ★ 나눗셈 시 분모도 float32_t로 정확히 캐스팅
		float32_t current_position = weighted_sum / (float32_t) total_weight;
		prev_position = current_position;
		return current_position;
	} else {
		force_full_scan = 1;
		lost_counter++;

		if (lost_counter > 50) {
			IR_Sensor.is_lost_position = 1;
			lost_counter = 0;
		}

		return prev_position;
	}
}

// ==============================================
// 마커 인식 알고리즘
// ==============================================

CrossMarkerLog_t g_cross_log[CROSS_LOG_MAX];
uint16_t g_cross_log_count = 0;

uint16_t g_last_stop_state = 0;
uint8_t g_last_stop_count = 0;

void Cross_Detect_Reset(void) {
	g_marker_state = MARKER_STATE_IDLE;
	g_accum_left = 0;
	g_accum_right = 0;
	g_accum_center_state = 0;
}

CrossEvent_t Cross_Detect_Update(void) {
	uint8_t left_marker = IR_Sensor.data->mark_left;
	uint8_t right_marker = IR_Sensor.data->mark_right;
	uint16_t current_center_state = (uint16_t) (IR_Sensor.data->state & 0xFFFF);
	CrossEvent_t event = CROSS_NONE;

	switch (g_marker_state) {
	// ... (IDLE 상태 처리 생략) ...

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
					if (g_accum_center_state & (1 << i))
						center_on_count++;
				}

				if (center_on_count >= 12) {
					event = CROSS_CROSS;
				} else {
					event = CROSS_STOP;

					// ★ 추가: STOP(U) 마커로 판정되는 순간의 센서 누적 비트맵과 개수를 저장
					g_last_stop_state = g_accum_center_state;
					g_last_stop_count = center_on_count;
				}
			} else if (g_accum_left) {
				event = CROSS_LEFT;
			} else if (g_accum_right) {
				event = CROSS_RIGHT;
			}
			Cross_Detect_Reset();

			// 마커 판독 종료: E3 LED OFF
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		break;
	}

	if (event != CROSS_NONE) {
		Cross_Log_Push(event);
		Buzzer_Start();
		buzzer_timer_count = (uint16_t) (g_buzzer_duration / LPTIM_TICK_DT);

		// ★ 수정된 화면 반전 로직 (drive.c의 Build_Segment_Plan 곡선 논리와 동일)
		if (event == CROSS_CROSS || event == CROSS_STOP) {
			// 십자, 정지 마커는 무조건 직선으로 초기화
			g_is_curve_state = 0;
			LCD7789_Invert(0); // 검은색 (직선)
		} else if (event == CROSS_LEFT) {
			if (g_is_curve_state == 1) {
				// 이미 L 곡선 중이었는데 L이 또 나옴 -> 곡선 닫힘(탈출), 직선 전환
				g_is_curve_state = 0;
				LCD7789_Invert(0); // 검은색
			} else {
				// 직선 또는 R 곡선 중 L 발견 -> 새로운 L 곡선 진입/유지
				g_is_curve_state = 1;
				LCD7789_Invert(1); // 흰색
			}
		} else if (event == CROSS_RIGHT) {
			if (g_is_curve_state == 2) {
				// 이미 R 곡선 중이었는데 R이 또 나옴 -> 곡선 닫힘(탈출), 직선 전환
				g_is_curve_state = 0;
				LCD7789_Invert(0); // 검은색
			} else {
				// 직선 또는 L 곡선 중 R 발견 -> 새로운 R 곡선 진입/유지
				g_is_curve_state = 2;
				LCD7789_Invert(1); // 흰색
			}
		}
	}
	return event;
}

void Cross_Log_Push(CrossEvent_t type) {
	CrossMarkerLog_t *log = &g_cross_log[g_cross_log_count % CROSS_LOG_MAX];
	log->type = type;
	log->dist_from_prev_m = g_odom_distance_m;
	log->yaw_angle = imu_data.Yaw_Angle; // ★ 현재 IMU Yaw 각도 기록

	g_cross_log_count++;
	Odom_Reset();
}

// ==============================================
// 센서 값 디버깅 함수
// ==============================================

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

void Sensor_Position_Printf() {
	Sensor_Start();
	LCD_Printf(0, 0, "Position");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		// ★ float -> float32_t 로 통일
		float32_t weighted_sum = 0.0f;
		uint32_t total_weight = 0;

		for (uint8_t i = 0; i < LINE_N_SENSORS; i++) {
			uint16_t weight = IR_Sensor.data->normalized[i];
			// ★ 캐스팅도 정확하게 지정
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
	LCD_Printf(0, 0, "IMU Test");

	// 테스트 시작 전 초기 각도를 0으로 리셋 (새로 만드신 함수 사용)
	LSM6DS3_Reset_Yaw();

	// K 버튼을 길게 누를 때까지 반복
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		// 백그라운드 인터럽트에서 알아서 계산 중인 각도를 그대로 가져와 출력
		LCD_Printf(0, 1, "Yaw: %+6.3f", imu_data.Yaw_Angle);

		// LCD 화면이 너무 빠르게 갱신되어 깜빡거리는 것을 방지
		HAL_Delay(50);
	}

	LCD_Clear();
	Button_Wait_Release(&btn_k);
}
