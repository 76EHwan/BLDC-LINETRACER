/*
 * sensor.c
 *
 *  Created on: 2026. 5. 2.
 *      Author: kth59
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

#define SENSOR_TRIG_TIM &htim2

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
#define LINE_N_SENSORS      16

#define LINE_WEIGHT(n)      (uint8_t)(n)

static const uint8_t scan_group1[SCAN_GROUP1_LEN] =
		{ 7, 8, 10, 5, 3, 12, 14, 1 };

static const uint8_t scan_group2_candidates[SCAN_GROUP2_CANDIDATE_LEN] = { 9, 6,
		4, 11, 13, 2, 0, 15 };

// 초기화: 캘리브레이션 시작 시 배열에 쓰레기값이 들어가는 것을 방지
static uint8_t scan_group2_active[SCAN_GROUP2_LEN] = { 9, 6, 4, 11, 13, 2 };

// ★ 추가: 스킵된 센서의 인덱스를 기억하기 위한 배열
static uint8_t scan_group2_skipped[2] = { 0, 15 };

#define POS1_EDGE_THRESH  (1.0f / 15.0f)

static void Select_Group2_Active(float position1) {
	uint8_t skip_a, skip_b;
	if (position1 < -POS1_EDGE_THRESH) {
		skip_a = 13;
		skip_b = 15;
	} else if (position1 > POS1_EDGE_THRESH) {
		skip_a = 0;
		skip_b = 2;
	} else {
		skip_a = 0;
		skip_b = 15;
	}

	// ★ 스킵 결정된 물리 센서 번호를 기록 (잔상 제거용)
	scan_group2_skipped[0] = skip_a;
	scan_group2_skipped[1] = skip_b;

	uint8_t w = 0;
	for (int i = 0; i < SCAN_GROUP2_CANDIDATE_LEN && w < SCAN_GROUP2_LEN; i++) {
		uint8_t phys = scan_group2_candidates[i];
		if (phys == skip_a || phys == skip_b)
			continue;
		scan_group2_active[w++] = phys;
	}
}

static float running_num = 0.0f;
static int32_t running_den = 0;
volatile uint8_t sensor_pos_updated = 0;

__STATIC_INLINE uint8_t Scan_Slot_To_Phys(uint8_t slot) {
	if (slot < SCAN_GROUP1_LEN) {
		return scan_group1[slot];
	}
	return scan_group2_active[slot - SCAN_SLOT_GROUP2_START];
}

// @formatter:off

static const float line_sensor_pos[LINE_N_SENSORS] = {
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

// DMA Circular 모드 16-rank 결과 버퍼
__attribute__((section(".ram_d3"), aligned(32))) uint16_t adc3_buffer[16];

volatile SensorDataTypeDef sensorData = {
        .idx = 0,
        .raw = { 0 },
        .blackmax = { 0 },
        .whitemax = { 0 },
        .normalized = { 0 },
        .state = 0,
        .threshold = 100,
        .pos_center_idx = (LINE_N_SENSORS - 1) / 2,
        .cross_left = 0,
        .cross_right = 0,
        .line_w_bandwidth = 10,
        .line_lost_sum_min = 20,
};

volatile Sensor_TypeDef IR_Sensor = {
        .is_calibration = 0,
        .is_lost_position = 0,
        .data = &sensorData,
};

// @formatter:on

volatile uint32_t count_sensor_irq = 0;

void Sensor_Printf(uint8_t idx, volatile uint16_t *sensor_data) {
	LCD_Printf(8 * (idx & 0x1), idx / 2 + 1, "0x%03X", *(sensor_data + idx));
}

void Sensor_Start() {
	IR_Sensor.data->idx = 0;
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	// DMA만 켜고, EOC 강제 활성화 코드는 삭제합니다!
	HAL_StatusTypeDef ret = HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adc3_buffer,
			16);

	if (ret != HAL_OK) {
		LCD_Printf(0, 15, "ADC ERR:%d", ret);
		return;
	}
	HAL_TIM_Base_Start_IT(&htim7);
}

void Sensor_Stop() {
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Stop_IT(&htim7);
	HAL_ADC_Stop_DMA(&hadc3);
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

__STATIC_INLINE void Sensor_Normalize(uint8_t idx) {
	const uint16_t raw = IR_Sensor.data->raw[idx];
	const uint16_t bmax = IR_Sensor.data->blackmax[idx];
	const uint16_t bias = IR_Sensor.data->normalized_coef_bias[idx];
	uint16_t diff = (raw > bmax) ? (raw - bmax) : 0;
	uint32_t result = ((uint32_t) diff * bias) >> 8;
	IR_Sensor.data->normalized[idx] = (result > 255U) ? 255U : (uint8_t) result;
}

float Sensor_Line_Estimate_Pass1(void) {
	count_sensor_irq++;

	const uint8_t line_w_deadband = IR_Sensor.data->line_w_bandwidth;
	const uint8_t line_lost_sum_min = IR_Sensor.data->line_lost_sum_min;

	int peak_idx = IR_Sensor.data->pos_center_idx;
	uint8_t peak_w = 0;
	for (int g = 0; g < SCAN_GROUP1_LEN; g++) {
		uint8_t phys = scan_group1[g];
		uint8_t w = LINE_WEIGHT(IR_Sensor.data->normalized[phys]);
		if (w > peak_w && w >= line_w_deadband) {
			peak_w = w;
			peak_idx = phys;
		}
	}
	IR_Sensor.data->pos_center_idx = (uint8_t) peak_idx;

	int start = peak_idx - POS_WINDOW_HALF;
	int end = peak_idx + (POS_WINDOW_HALF - 1);
	if (start < 0) {
		end -= start;
		start = 0;
	}
	if (end > LINE_N_SENSORS - 1) {
		start -= (end - (LINE_N_SENSORS - 1));
		end = LINE_N_SENSORS - 1;
	}
	if (start < 0)
		start = 0;
	IR_Sensor.data->win_start = (uint8_t) start;
	IR_Sensor.data->win_end = (uint8_t) end;

	float num = 0.0f;
	int32_t den = 0;
	uint8_t cross_left = 0, cross_right = 0;
	for (int g = 0; g < SCAN_GROUP1_LEN; g++) {
		uint8_t phys = scan_group1[g];
		uint8_t w = LINE_WEIGHT(IR_Sensor.data->normalized[phys]);
		if (phys >= start && phys <= end) {
			if (w >= line_w_deadband) {
				num += (float) w * line_sensor_pos[phys];
				den += w;
			}
		} else if (w >= line_w_deadband) {
			if (phys < start)
				cross_left = 1;
			else
				cross_right = 1;
		}
	}
	IR_Sensor.data->cross_left = cross_left;
	IR_Sensor.data->cross_right = cross_right;

	running_num = num;
	running_den = den;

	if (running_den < line_lost_sum_min) {
		IR_Sensor.is_lost_position = 1;
	} else {
		IR_Sensor.is_lost_position = 0;
		IR_Sensor.data->line_position = running_num / (float) running_den;
	}
	IR_Sensor.data->line_position1 = IR_Sensor.data->line_position;

	Select_Group2_Active(IR_Sensor.data->line_position1);

	return IR_Sensor.data->line_position1;
}

float Sensor_Line_Estimate_Pass2(void) {
	const uint8_t line_lost_sum_min = IR_Sensor.data->line_lost_sum_min;

	count_sensor_irq++;
	if (running_den < line_lost_sum_min) {
		IR_Sensor.is_lost_position = 1;
	} else {
		IR_Sensor.is_lost_position = 0;
		IR_Sensor.data->line_position = running_num / (float) running_den;
	}

	return IR_Sensor.data->line_position;
}

void TIM7_IRQ_Handler() {
	uint8_t slot = IR_Sensor.data->idx;

	if (slot == SCAN_SLOT_MARK_L) {
		SENSOR_IR_HIGH();
		SENSOR_PT_HIGH();
	} else if (slot == SCAN_SLOT_MARK_R) {
		// Mux 상태 유지
	} else {
		uint8_t phys = Scan_Slot_To_Phys(slot);
		Set_Mux_Channel_Fast(phys);
		SENSOR_IR_LOW();
		SENSOR_PT_LOW();
	}

	// 다음 변환을 위해 슬롯 미리 증가
	IR_Sensor.data->idx = (slot == SCAN_CYCLE_LEN - 1) ? 0 : (slot + 1);

	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;
	TIM2->SR = ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void ADC3_IRQ_Half_Handler() {
	// Group1 (0~7) 데이터 일괄 처리
	for (int i = 0; i < SCAN_GROUP1_LEN; i++) {
		uint8_t phys = scan_group1[i];
		IR_Sensor.data->raw[phys] = adc3_buffer[i];

		if (IR_Sensor.is_calibration) {
			Sensor_Normalize(phys);
			uint8_t w = LINE_WEIGHT(IR_Sensor.data->normalized[phys]);
			IR_Sensor.data->state &= ~(1u << phys);
			IR_Sensor.data->state |= ((uint32_t) (w > IR_Sensor.data->threshold)
					<< phys);
		}
	}

	// Group1 처리 완료 후 Pass1 실행
	if (IR_Sensor.is_calibration) {
		Sensor_Line_Estimate_Pass1(); // 내부에서 count_sensor_irq++ 실행됨
		sensor_pos_updated = 1;
	}
}

void ADC3_IRQ_Cplt_Handler() {
	uint8_t m_idx[2] = { LEFT_MARK_SENSOR_INDEX, RIGHT_MARK_SENSOR_INDEX };
	for (int i = 0; i < 2; i++) {
		uint8_t mark = m_idx[i];
		IR_Sensor.data->raw[mark] = adc3_buffer[8 + i];
		if (IR_Sensor.is_calibration) {
			Sensor_Normalize(mark);
			uint8_t on = IR_Sensor.data->normalized[mark]
					> IR_Sensor.data->threshold;
			IR_Sensor.data->state &= ~(1u << mark);
			IR_Sensor.data->state |= ((uint32_t) on << mark);
		}
	}

	// Group2 처리 (슬롯 10~15)
	if (IR_Sensor.is_calibration) {
		const uint8_t line_w_deadband = IR_Sensor.data->line_w_bandwidth;
		uint8_t start = IR_Sensor.data->win_start;
		uint8_t end = IR_Sensor.data->win_end;

		// ★ 스캔 제외된 센서 초기화 (잔상 제거 로직)
		for (int i = 0; i < 2; i++) {
			uint8_t phys = scan_group2_skipped[i];
			IR_Sensor.data->raw[phys] = 0;
			IR_Sensor.data->normalized[phys] = 0;
			IR_Sensor.data->state &= ~(1u << phys);
		}

		for (int i = 10; i < 16; i++) {
			uint8_t phys = scan_group2_active[i - 10];
			IR_Sensor.data->raw[phys] = adc3_buffer[i];
			Sensor_Normalize(phys);
			uint8_t w = LINE_WEIGHT(IR_Sensor.data->normalized[phys]);
			IR_Sensor.data->state &= ~(1u << phys);
			IR_Sensor.data->state |= ((uint32_t) (w > IR_Sensor.data->threshold)
					<< phys);

			// Pass2를 위한 누적 연산을 여기서 일괄 처리
			if (phys >= start && phys <= end) {
				if (w >= line_w_deadband) {
					running_den += w;
					running_num += (float) w * line_sensor_pos[phys];
				}
			} else if (w >= line_w_deadband) {
				if (phys < start)
					IR_Sensor.data->cross_left = 1;
				else
					IR_Sensor.data->cross_right = 1;
			}
		}

		// 데이터 수집이 끝났으므로 Pass2 최종 확정
		Sensor_Line_Estimate_Pass2(); // 내부에서 count_sensor_irq++ 실행됨
		sensor_pos_updated = 1;

	} else {
		// 캘리브레이션/Raw 모드 시 Group2 센서값만 저장
		for (int i = 10; i < 16; i++) {
			uint8_t phys = scan_group2_active[i - 10];
			IR_Sensor.data->raw[phys] = adc3_buffer[i];
		}

		// 전체 센서 로테이션 스캔 갱신
		static uint8_t skip_idx = 0;
		uint8_t w = 0;
		for (int i = 0; i < SCAN_GROUP2_CANDIDATE_LEN && w < SCAN_GROUP2_LEN;
				i++) {
			if (i == skip_idx || i == (skip_idx + 1) % SCAN_GROUP2_CANDIDATE_LEN) {
				continue;
			}
			scan_group2_active[w++] = scan_group2_candidates[i];
		}
		skip_idx = (skip_idx + 2) % SCAN_GROUP2_CANDIDATE_LEN;
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

	FRESULT load_res = Sensor_Load_Calibration();
	LCD_Printf(0, 0, "Load:%d", load_res);
	HAL_Delay(500);
	LCD_Clear();

	// ★ 전체 센서 측정을 위해 로테이션 스캔 모드로 강제 전환
	IR_Sensor.is_calibration = 0;

	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "White Max");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 10, "%02d", IR_Sensor.data->idx);
		if (IR_Sensor.data->whitemax[i] < IR_Sensor.data->raw[i]) {
			IR_Sensor.data->whitemax[i] = IR_Sensor.data->raw[i];
		}
		Sensor_Printf(i, IR_Sensor.data->whitemax);
		i = (i + 1) % 18;
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);

	Sensor_Start();
	LCD_Printf(0, 0, "Black Max");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 10, "%02d", IR_Sensor.data->idx);
		if (IR_Sensor.data->blackmax[i] < IR_Sensor.data->raw[i]) {
			IR_Sensor.data->blackmax[i] = IR_Sensor.data->raw[i];
		}
		Sensor_Printf(i, IR_Sensor.data->blackmax);
		i = (i + 1) % 18;
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
		LCD_Printf(0, 13, "%2d", IR_Sensor.data->idx);
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
	LCD_Printf(0, 0, "Sensor Pos");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 1, "%6.3f", IR_Sensor.data->line_position);
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
