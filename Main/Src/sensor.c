/*
 * sensor.c
 *
 *  Created on: 2026. 5. 2.
 *      Author: kth59
 *
 *  [수정 이력]
 *  - idx16/17 전용 마커 포토인터럽터를 더 이상 쓰지 않게 되어,
 *    Sensor_Get_Position()의 위치추정 윈도우(POS_WINDOW_HALF) 바깥쪽
 *    idx 0~15 센서 중 활성(threshold 초과) 상태가 있으면 좌/우 마커
 *    후보(mark_left/mark_right)로 판정하도록 변경.
 *    drive.c의 Cross_Detect_Update()는 이제 IR_Sensor.data->mark_left,
 *    mark_right를 사용한다.
 *  - 동적 스케일링(Auto Gain) 추가: 센서 들림 현상 발생 시 현재 프레임의
 *    최댓값을 기준으로 전체 센서 값을 255로 증폭시켜 라인 이탈 오판(is_lost_position) 방지.
 */
#include "sensor.h"
#include "main.h"
#include "user_init.h"
#include "button.h"
#include "lsm6ds3tr-c.h"
#include "cmsis_gcc.h"
#include "SDcard.h"

#define SENSOR_CALIB_PATH "/Sensor_Data/calibration_result.txt"
#define SENSOR_CALIB_COUNT (sizeof(sensor_calib_table) / sizeof(sensor_calib_table[0]))

#define SENSOR_ADC_HANDLE		&hadc3
#define SENSOR_TIM_IR_HANDLE	&htim7
#define SENSOR_TIM_TRIG_HANDLE	&htim2

/*
 * IR_EN 극성 (schematic 기준):
 *   IR_EN = HIGH -> 좌우 mark 2N7002 ON, CD4067 INH=HIGH -> 중앙 mux 차단
 *   IR_EN = LOW  -> 좌우 mark OFF,        CD4067 INH=LOW  -> 중앙 mux 통과
 */
#define SENSOR_IR_HIGH()  (SENSOR_IR_EN_GPIO_Port->BSRR = SENSOR_IR_EN_Pin)
#define SENSOR_IR_LOW()   (SENSOR_IR_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_IR_EN_Pin << 16U))
#define SENSOR_PT_HIGH()  (SENSOR_PT_EN_GPIO_Port->BSRR = SENSOR_PT_EN_Pin)
#define SENSOR_PT_LOW()   (SENSOR_PT_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_PT_EN_Pin << 16U))

// === 라인 위치 추정 (weighted centroid, 2단계 interleaved 스캔) ==========
#define LINE_WEIGHT(n)      (uint8_t)(n)

static const uint8_t scan_group1[SCAN_GROUP_LEN] = { 7, 8, 10, 5, 3, 12, 14, 1,
		16, 17 };

static const uint8_t scan_group2[SCAN_GROUP_LEN] = { 9, 6, 4, 11, 13, 2, 0, 15,
		16, 17 };

const float line_sensor_pos[LINE_N_SENSORS] = { -1.0f, -13.0f / 15.0f, -11.0f
		/ 15.0f, -9.0f / 15.0f, -7.0f / 15.0f, -5.0f / 15.0f, -3.0f / 15.0f,
		-1.0f / 15.0f, 1.0f / 15.0f, 3.0f / 15.0f, 5.0f / 15.0f, 7.0f / 15.0f,
		9.0f / 15.0f, 11.0f / 15.0f, 13.0f / 15.0f, 1.0f };

__STATIC_INLINE uint8_t Scan_Slot_To_Phys(uint8_t slot) {
	if (slot < SCAN_CYCLE_LEN_HALF) {
		return scan_group1[slot];
	}
	return scan_group2[slot - SCAN_CYCLE_LEN_HALF];
}

// @formatter:off

// DMA Circular 모드 16-rank 결과 버퍼
__attribute__((section(".ram_d3"), aligned(32))) uint16_t adc3_buffer[1];

volatile SensorDataTypeDef sensorData = {
        .idx = 0,
        .raw = { 0 },
        .blackmax = { 0 },
        .whitemax = { 0 },
        .normalized = { 0 },
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

float Sensor_Get_Position(void) {
	// 이전 피크(중심) 인덱스를 기억하기 위한 정적 변수
	static int8_t prev_peak_idx = LINE_N_SENSORS / 2;

	uint16_t max_val = 0;
	int8_t current_peak_idx = LINE_N_SENSORS / 2;

	// 1. 피크(가장 강한 신호) 찾기
	if (IR_Sensor.is_lost_position) {
		// 선을 놓친 상태 (Global Search: 전체 센서 범위 탐색)
		for (int8_t i = 0; i < LINE_N_SENSORS; i++) {
			if (IR_Sensor.data->normalized[i] > max_val) {
				max_val = IR_Sensor.data->normalized[i];
				current_peak_idx = i;
			}
		}
	} else {
		// 선을 추종 중인 상태 (Local Search: 이전 피크 주변 윈도우 내에서만 탐색)
		int8_t search_start = prev_peak_idx - POS_WINDOW_HALF;
		int8_t search_end = prev_peak_idx + POS_WINDOW_HALF;

		// 인덱스 범위 제한
		if (search_start < 0) search_start = 0;
		if (search_end >= LINE_N_SENSORS) search_end = LINE_N_SENSORS - 1;

		for (int8_t i = search_start; i <= search_end; i++) {
			if (IR_Sensor.data->normalized[i] > max_val) {
				max_val = IR_Sensor.data->normalized[i];
				current_peak_idx = i;
			}
		}
	}

	// 다음 주기를 위해 피크 위치 갱신
	prev_peak_idx = current_peak_idx;

	// 2. 찾아낸 피크를 중심으로 가중 산술 평균을 구할 계산 윈도우 설정
	int8_t calc_start = current_peak_idx - POS_WINDOW_HALF;
	int8_t calc_end = current_peak_idx + POS_WINDOW_HALF;

	if (calc_start < 0) calc_start = 0;
	if (calc_end >= LINE_N_SENSORS) calc_end = LINE_N_SENSORS - 1;

	// ★ idx16/17 전용 마커 센서 미사용 대체: 위치추정 윈도우 [calc_start, calc_end]
	//   바깥쪽에 있는 idx 0~15 센서 중 활성(threshold 초과) 상태가 하나라도 있으면
	//   그쪽을 좌/우 마커 후보로 판정한다. drive.c의 Cross_Detect_Update()가 이 값을 읽는다.
	{
		uint16_t state16 = (uint16_t) (IR_Sensor.data->state & 0xFFFF);
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
	}

	// ★ 동적 스케일링 (Auto Gain) 적용 ★
	// max_val이 너무 낮을 때(예: 완전히 벗어남, 20 이하)는 노이즈 증폭 방지를 위해 기본 배율 사용
	uint32_t scale_factor = 256; // 기본 배율 (1.0배 = 256)
	if (max_val > 20 && max_val < 255) {
		scale_factor = (255U << 8) / max_val; // 고정 소수점 연산을 위한 8비트 시프트
	}

	float weighted_sum = 0.0f;
	uint32_t total_weight = 0;

	// 3. 계산 윈도우 내부에 있는 센서 값만 합산 (스케일링 적용)
	for (int8_t i = calc_start; i <= calc_end; i++) {
		// 기존 normalized 값에 scale_factor를 곱해 증폭시킴
		uint32_t amplified_val = (IR_Sensor.data->normalized[i] * scale_factor) >> 8;

		// 255를 넘지 않도록 클램핑
		if (amplified_val > 255) amplified_val = 255;

		weighted_sum += (float) amplified_val * line_sensor_pos[i];
		total_weight += amplified_val;
	}

	// 4. 작성해주신 로직 적용: 총합을 기준으로 이탈 여부 갱신 및 위치 반환
	if (total_weight > IR_Sensor.data->line_lost_sum_min) {
		IR_Sensor.is_lost_position = 0;
		return weighted_sum / (float) total_weight;
	} else {
		IR_Sensor.is_lost_position = 1;
		return 0.0f;
	}
}

// =========================================================
// [SD카드 저장 및 불러오기]
// =========================================================
// @formatter:off
static const SDCard_ConfigEntry sensor_calib_table[] = {
        { "whitemax_00",    (void*)&sensorData.whitemax[0],                 SDCFG_UINT16 },
        { "whitemax_01",    (void*)&sensorData.whitemax[1],                 SDCFG_UINT16 },
        { "whitemax_02",    (void*)&sensorData.whitemax[2],                 SDCFG_UINT16 },
        { "whitemax_03",    (void*)&sensorData.whitemax[3],                 SDCFG_UINT16 },
        { "whitemax_04",    (void*)&sensorData.whitemax[4],                 SDCFG_UINT16 },
        { "whitemax_05",    (void*)&sensorData.whitemax[5],                 SDCFG_UINT16 },
        { "whitemax_06",    (void*)&sensorData.whitemax[6],                 SDCFG_UINT16 },
        { "whitemax_07",    (void*)&sensorData.whitemax[7],                 SDCFG_UINT16 },
        { "whitemax_08",    (void*)&sensorData.whitemax[8],                 SDCFG_UINT16 },
        { "whitemax_09",    (void*)&sensorData.whitemax[9],                 SDCFG_UINT16 },
        { "whitemax_10",    (void*)&sensorData.whitemax[10],                SDCFG_UINT16 },
        { "whitemax_11",    (void*)&sensorData.whitemax[11],                SDCFG_UINT16 },
        { "whitemax_12",    (void*)&sensorData.whitemax[12],                SDCFG_UINT16 },
        { "whitemax_13",    (void*)&sensorData.whitemax[13],                SDCFG_UINT16 },
        { "whitemax_14",    (void*)&sensorData.whitemax[14],                SDCFG_UINT16 },
        { "whitemax_15",    (void*)&sensorData.whitemax[15],                SDCFG_UINT16 },
        { "whitemax_16",    (void*)&sensorData.whitemax[16],                SDCFG_UINT16 },
        { "whitemax_17",    (void*)&sensorData.whitemax[17],                SDCFG_UINT16 },
        { "blackmax_00",    (void*)&sensorData.blackmax[0],                 SDCFG_UINT16 },
        { "blackmax_01",    (void*)&sensorData.blackmax[1],                 SDCFG_UINT16 },
        { "blackmax_02",    (void*)&sensorData.blackmax[2],                 SDCFG_UINT16 },
        { "blackmax_03",    (void*)&sensorData.blackmax[3],                 SDCFG_UINT16 },
        { "blackmax_04",    (void*)&sensorData.blackmax[4],                 SDCFG_UINT16 },
        { "blackmax_05",    (void*)&sensorData.blackmax[5],                 SDCFG_UINT16 },
        { "blackmax_06",    (void*)&sensorData.blackmax[6],                 SDCFG_UINT16 },
        { "blackmax_07",    (void*)&sensorData.blackmax[7],                 SDCFG_UINT16 },
        { "blackmax_08",    (void*)&sensorData.blackmax[8],                 SDCFG_UINT16 },
        { "blackmax_09",    (void*)&sensorData.blackmax[9],                 SDCFG_UINT16 },
        { "blackmax_10",    (void*)&sensorData.blackmax[10],                SDCFG_UINT16 },
        { "blackmax_11",    (void*)&sensorData.blackmax[11],                SDCFG_UINT16 },
        { "blackmax_12",    (void*)&sensorData.blackmax[12],                SDCFG_UINT16 },
        { "blackmax_13",    (void*)&sensorData.blackmax[13],                SDCFG_UINT16 },
        { "blackmax_14",    (void*)&sensorData.blackmax[14],                SDCFG_UINT16 },
        { "blackmax_15",    (void*)&sensorData.blackmax[15],                SDCFG_UINT16 },
        { "blackmax_16",    (void*)&sensorData.blackmax[16],                SDCFG_UINT16 },
        { "blackmax_17",    (void*)&sensorData.blackmax[17],                SDCFG_UINT16 },
        { "coef_bias_00",   (void*)&sensorData.normalized_coef_bias[0],     SDCFG_UINT16 },
        { "coef_bias_01",   (void*)&sensorData.normalized_coef_bias[1],     SDCFG_UINT16 },
        { "coef_bias_02",   (void*)&sensorData.normalized_coef_bias[2],     SDCFG_UINT16 },
        { "coef_bias_03",   (void*)&sensorData.normalized_coef_bias[3],     SDCFG_UINT16 },
        { "coef_bias_04",   (void*)&sensorData.normalized_coef_bias[4],     SDCFG_UINT16 },
        { "coef_bias_05",   (void*)&sensorData.normalized_coef_bias[5],     SDCFG_UINT16 },
        { "coef_bias_06",   (void*)&sensorData.normalized_coef_bias[6],     SDCFG_UINT16 },
        { "coef_bias_07",   (void*)&sensorData.normalized_coef_bias[7],     SDCFG_UINT16 },
        { "coef_bias_08",   (void*)&sensorData.normalized_coef_bias[8],     SDCFG_UINT16 },
        { "coef_bias_09",   (void*)&sensorData.normalized_coef_bias[9],     SDCFG_UINT16 },
        { "coef_bias_10",   (void*)&sensorData.normalized_coef_bias[10],    SDCFG_UINT16 },
        { "coef_bias_11",   (void*)&sensorData.normalized_coef_bias[11],    SDCFG_UINT16 },
        { "coef_bias_12",   (void*)&sensorData.normalized_coef_bias[12],    SDCFG_UINT16 },
        { "coef_bias_13",   (void*)&sensorData.normalized_coef_bias[13],    SDCFG_UINT16 },
        { "coef_bias_14",   (void*)&sensorData.normalized_coef_bias[14],    SDCFG_UINT16 },
        { "coef_bias_15",   (void*)&sensorData.normalized_coef_bias[15],    SDCFG_UINT16 },
        { "coef_bias_16",   (void*)&sensorData.normalized_coef_bias[16],    SDCFG_UINT16 },
        { "coef_bias_17",   (void*)&sensorData.normalized_coef_bias[17],    SDCFG_UINT16 },
};
// @formatter:on

FRESULT Sensor_Save_Calibration(void) {
	return SDCard_SaveConfig(SENSOR_CALIB_PATH, sensor_calib_table,
	SENSOR_CALIB_COUNT);
}

FRESULT Sensor_Load_Calibration(void) {
	FRESULT res = SDCard_LoadConfig(SENSOR_CALIB_PATH, sensor_calib_table,
	SENSOR_CALIB_COUNT);
	if (res != FR_OK)
		return res;

	IR_Sensor.is_calibration = 1;
	return FR_OK;
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

	// ★ 전체 센서 측정을 위해 로테이션 스캔 모드로 강제 전환
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

	// ★ 캘리브레이션 완료 후 라인 위치 추정 모드로 복귀
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
	// ★ 전체 센서 측정을 위해 로테이션 스캔 모드로 전환
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

	// 원래 설정 모드로 복구
	IR_Sensor.is_calibration = old_calib;
	Button_Wait_Release(&btn_k);
}

void Sensor_Normalize_Printf() {
	Sensor_Start();
	uint8_t i = 0;
	uint32_t sum = 0;
	LCD_Printf(0, 0, "Sensor Normal");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		if (i == 0) {
			sum = 0;
		}
		sum += IR_Sensor.data->normalized[i];
		Sensor_Printf(i, IR_Sensor.data->normalized);
		if (i == 15) {
			LCD_Printf(0, 13, "%-5d", sum);
		}
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
		if (i < LINE_N_SENSORS) {
			LCD_Printf(i, 2, "%c", state);
		}
		if (i == 16) {
			LCD_Printf(0, 3, "%c", state);
		}
		if (i == 17) {
			LCD_Printf(15, 3, "%c", state);
		}
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
		float weighted_sum = 0.0f;
		uint32_t total_weight = 0;

		// LINE_N_SENSORS (16개) 에 대해서만 가중 평균 계산
		for (uint8_t i = 0; i < LINE_N_SENSORS; i++) {
			uint16_t weight = IR_Sensor.data->normalized[i];
			weighted_sum += (float) weight * line_sensor_pos[i];
			total_weight += weight;
		}

		// 센서 측정값의 총합이 최소 기준치(line_lost_sum_min) 이상일 때만 위치 계산
		if (total_weight > IR_Sensor.data->line_lost_sum_min) {
			float position = weighted_sum / (float) total_weight;
			// %+6.3f 포맷: 부호 포함하여 소수점 아래 3자리까지 출력
			LCD_Printf(0, 2, "Pos: %+6.3f", position);
		} else {
			// 라인을 벗어난 경우 (총합이 부족할 때)
			LCD_Printf(0, 2, "Pos: LOST   ");
		}

		// 디버깅을 위해 총 가중치 합(total_weight)도 함께 출력 (선택 사항)
		LCD_Printf(0, 3, "Sum: %-5lu", total_weight);

		// 화면 업데이트 주기를 조절하기 위해 약간의 딜레이 추가
		HAL_Delay(10);
	}

	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void IMU_Test() {
	LCD_Printf(0, 0, "IMU Test");
	uint32_t last_tick = HAL_GetTick();

	LSM6DS3_ReadGyroZ_DMA_Start();

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		uint32_t now = HAL_GetTick();
		float dt = (now - last_tick) / 1000.0f;
		last_tick = now;

		if (imu_gyro_z_ready) {
			imu_gyro_z_ready = 0;
			LSM6DS3_UpdateYaw(&imu_data, dt);
			LSM6DS3_ReadGyroZ_DMA_Start();
		}

		LCD_Printf(0, 1, "%6.3f", imu_data.Yaw_Angle);
	}
	Button_Wait_Release(&btn_k);
}
