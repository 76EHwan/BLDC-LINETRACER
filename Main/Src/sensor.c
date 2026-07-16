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
 *   IR_EN = LOW  -> 좌우 mark OFF,       CD4067 INH=LOW  -> 중앙 mux 통과
 * 두 그룹이 같은 ADC 노드를 공유하지 않더라도 enable이 배타적이라
 * 한 사이클에 한 그룹만 읽는다. PT_EN은 IR_EN과 같은 레벨로 구동.
 */
#define SENSOR_IR_HIGH()  (SENSOR_IR_EN_GPIO_Port->BSRR = SENSOR_IR_EN_Pin)
#define SENSOR_IR_LOW()   (SENSOR_IR_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_IR_EN_Pin << 16U))
#define SENSOR_PT_HIGH()  (SENSOR_PT_EN_GPIO_Port->BSRR = SENSOR_PT_EN_Pin)
#define SENSOR_PT_LOW()   (SENSOR_PT_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_PT_EN_Pin << 16U))

// === 라인 위치 추정 (weighted centroid) ================================
#define LINE_N_SENSORS      16

// 사이클: idx 0..15 중앙, idx 16 좌우 mark
#define PHASE_MARK_IDX      LINE_N_SENSORS
#define CYCLE_LEN           (LINE_N_SENSORS + 1)

#define LINE_WEIGHT(n)      (uint8_t)(n)
#define LINE_W_DEADBAND     30u     // 이하 weight는 배경 residual로 보고 무시
#define LINE_LOST_SUM_MIN   160u    // weight 합이 이하면 라인 상실로 판단

static const float line_sensor_pos[LINE_N_SENSORS] = { -1.0f, -13.0f / 15.0f,
		-11.0f / 15.0f, -9.0f / 15.0f, -7.0f / 15.0f, -5.0f / 15.0f, -3.0f
				/ 15.0f, -1.0f / 15.0f, 1.0f / 15.0f, 3.0f / 15.0f, 5.0f
				/ 15.0f, 7.0f / 15.0f, 9.0f / 15.0f, 11.0f / 15.0f, 13.0f
				/ 15.0f, 1.0f };

__attribute__((section(".ram_d3"), aligned(32)))                uint16_t adc3_buffer[3];

volatile uint32_t tim7_count = 0;
volatile uint32_t adc_count = 0;

volatile Sensor_TypeDef IR_Sensor = { .data = { .idx = 0, .raw = { 0 },
		.blackmax = { 0 }, .whitemax = { 0 }, .normalized = { 0 }, .state = 0,
		.threshold = 50, .pos_center_idx = (LINE_N_SENSORS - 1) / 2, .cross_left = 0,
		.cross_right = 0 }, .is_calibration = 0 };

void Sensor_Printf(uint8_t idx, volatile uint16_t *sensor_data) {
	LCD_Printf(8 * (idx & 0x1), idx / 2 + 1, "0x%03X", *(sensor_data + idx));
}

void Sensor_Start() {
	IR_Sensor.data.idx = 0;
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_StatusTypeDef ret = HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adc3_buffer,
			3);
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
	// MUX0 (Bit 0: 가중치 1)
	if (index & 0x01)
		SENSOR_MUX0_GPIO_Port->BSRR = SENSOR_MUX0_Pin;
	else
		SENSOR_MUX0_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX0_Pin << 16U;

	// MUX1 (Bit 1: 가중치 2)
	if (index & 0x02)
		SENSOR_MUX1_GPIO_Port->BSRR = SENSOR_MUX1_Pin;
	else
		SENSOR_MUX1_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX1_Pin << 16U;

	// MUX2 (Bit 2: 가중치 4)
	if (index & 0x04)
		SENSOR_MUX2_GPIO_Port->BSRR = SENSOR_MUX2_Pin;
	else
		SENSOR_MUX2_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX2_Pin << 16U;

	// MUX3 (Bit 3: 가중치 8)
	if (index & 0x08)
		SENSOR_MUX3_GPIO_Port->BSRR = SENSOR_MUX3_Pin;
	else
		SENSOR_MUX3_GPIO_Port->BSRR = (uint32_t) SENSOR_MUX3_Pin << 16U;
}

__STATIC_INLINE void Sensor_Normalize(uint8_t idx) {
	const uint16_t raw = IR_Sensor.data.raw[idx];
	const uint16_t bmax = IR_Sensor.data.blackmax[idx];
	const uint16_t bias = IR_Sensor.data.normalized_coef_bias[idx];
	uint16_t diff = (raw > bmax) ? (raw - bmax) : 0;
	uint32_t result = ((uint32_t) diff * bias) >> 8;
	IR_Sensor.data.normalized[idx] = (result > 255U) ? 255U : (uint8_t) result;
}

// weighted centroid, but restricted to an 8-sensor window centered on the
// last known line position (pos_center_idx). This keeps outer sensors free
// to act as cross(교차로) marker candidates instead of polluting the centroid.
// 라인 상실(window 내 weight 합 부족) 시 마지막 위치/중심 유지.
float Sensor_Line_Estimate(void) {
	int center = IR_Sensor.data.pos_center_idx;

	int start = center - POS_WINDOW_HALF;
	int end = center + (POS_WINDOW_HALF - 1);   // POS_WINDOW_SIZE(8)개: [center-4, center+3]
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

	float num = 0.0f;
	uint32_t den = 0;
	int peak_idx = center;
	uint8_t peak_w = 0;

	for (int i = start; i <= end; i++) {
		uint8_t w = LINE_WEIGHT(IR_Sensor.data.normalized[i]);
		if (w < LINE_W_DEADBAND) {
			continue;
		}
		num += (float) w * line_sensor_pos[i];
		den += w;
		if (w > peak_w) {
			peak_w = w;
			peak_idx = i;
		}
	}

	// 창 바깥쪽: 좌/우 교차로 마커 후보 검출 (deadband 초과 시 후보로 간주)
	uint8_t cross_left = 0;
	for (int i = 0; i < start; i++) {
		if (LINE_WEIGHT(IR_Sensor.data.normalized[i]) >= LINE_W_DEADBAND) {
			cross_left = 1;
			break;
		}
	}
	uint8_t cross_right = 0;
	for (int i = end + 1; i < LINE_N_SENSORS; i++) {
		if (LINE_WEIGHT(IR_Sensor.data.normalized[i]) >= LINE_W_DEADBAND) {
			cross_right = 1;
			break;
		}
	}
	IR_Sensor.data.cross_left = cross_left;
	IR_Sensor.data.cross_right = cross_right;

	if (den < LINE_LOST_SUM_MIN) {
		IR_Sensor.is_lose_position = 1;
		return IR_Sensor.data.line_position;   // 상실: 마지막 위치/중심 hold
	}
	IR_Sensor.is_lose_position = 0;
	IR_Sensor.data.pos_center_idx = (uint8_t) peak_idx;   // 다음 프레임 창 재중심
	return num / (float) den;
}

/*
 * TIM7 tick 마다 현재 phase를 세팅하고 TIM2(트리거 지연) 킥.
 *   idx 0..15 : 중앙 채널 idx  -> IR_EN LOW  (mux 활성), mux=idx
 *   idx 16    : 좌우 mark      -> IR_EN HIGH (2N7002 ON, mux 차단)
 * TIM2 만료 시 TRGO로 ADC3 변환 시작 -> settling time = TIM2 지연.
 */
void TIM7_IRQ_Handler() {
	tim7_count++;

	const uint8_t idx = IR_Sensor.data.idx;

	if (idx < LINE_N_SENSORS) {
		// 중앙 phase (active low)
		Set_Mux_Channel_Fast(idx);
		SENSOR_IR_LOW();
		SENSOR_PT_LOW();
	} else {
		// 좌우 mark phase (active high)
		SENSOR_IR_HIGH();
		SENSOR_PT_HIGH();
	}

	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;
	TIM2->SR = ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;
}

/*
 * ADC3 EOS. 현재 phase에 유효한 채널만 저장.
 *   buffer[0] = PC3_C = LEFT_MARK  (mark phase에서만 유효)
 *   buffer[1] = PC1   = RIGHT_MARK (mark phase에서만 유효)
 *   buffer[2] = PC2_C = 중앙 mux 출력 (중앙 phase에서만 유효)
 * adc3_buffer는 SRAM4(D3) + MPU non-cacheable이므로 invalidate 불필요.
 */
void ADC3_IRQ_Handler() {
	ADC3->ISR = (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

	const uint8_t idx = IR_Sensor.data.idx;

	if (idx < LINE_N_SENSORS) {
		// === active low: 중앙 채널 idx 저장 + 계산 ===
		IR_Sensor.data.raw[idx] = adc3_buffer[2];

		if (IR_Sensor.is_calibration) {
			Sensor_Normalize(idx);

			// state bit: 이 센서가 라인 위인지. weight > threshold.
			uint8_t w = LINE_WEIGHT(IR_Sensor.data.normalized[idx]);
			IR_Sensor.data.state &= ~(1u << idx);
			IR_Sensor.data.state |= ((uint32_t) (w > IR_Sensor.data.threshold)
					<< idx);

			// 중앙 한 바퀴 끝 -> 라인 위치 갱신
			if (idx == LINE_N_SENSORS - 1) {
				IR_Sensor.data.line_position = Sensor_Line_Estimate();
			}
		}
		IR_Sensor.data.idx = idx + 1;   // -> 다음 중앙 또는 mark(16)

	} else {
		// === active high: 좌우 mark 저장 + 계산 ===
		IR_Sensor.data.raw[LEFT_MARK_SENSOR_INDEX] = adc3_buffer[0];
		IR_Sensor.data.raw[RIGHT_MARK_SENSOR_INDEX] = adc3_buffer[1];

		if (IR_Sensor.is_calibration) {
			Sensor_Normalize(LEFT_MARK_SENSOR_INDEX);
			Sensor_Normalize(RIGHT_MARK_SENSOR_INDEX);

			// 좌우 mark 검출 (level). normalized > threshold 이면 mark 위.
			uint8_t left_on = IR_Sensor.data.normalized[LEFT_MARK_SENSOR_INDEX]
					> IR_Sensor.data.threshold;
			uint8_t right_on =
					IR_Sensor.data.normalized[RIGHT_MARK_SENSOR_INDEX]
							> IR_Sensor.data.threshold;

			// state의 bit 16/17에 저장 (state는 uint32_t 이상이어야 함)
			IR_Sensor.data.state &= ~((1u << LEFT_MARK_SENSOR_INDEX)
					| (1u << RIGHT_MARK_SENSOR_INDEX));
			IR_Sensor.data.state |= ((uint32_t) left_on
					<< LEFT_MARK_SENSOR_INDEX);
			IR_Sensor.data.state |= ((uint32_t) right_on
					<< RIGHT_MARK_SENSOR_INDEX);
		}

		IR_Sensor.data.idx = 0;         // 사이클 재시작
	}
}

// =========================================================
// [SD카드 저장 및 불러오기] - FOC와 동일한 key=value 텍스트 포맷
// =========================================================
// @formatter:off
static const SDCard_ConfigEntry sensor_calib_table[] = {
		{ "whitemax_00",	&IR_Sensor.data.whitemax[0],			SDCFG_UINT16 },
		{ "whitemax_01",	&IR_Sensor.data.whitemax[1],			SDCFG_UINT16 },
		{ "whitemax_02",	&IR_Sensor.data.whitemax[2],			SDCFG_UINT16 },
		{ "whitemax_03",	&IR_Sensor.data.whitemax[3],			SDCFG_UINT16 },
		{ "whitemax_04",	&IR_Sensor.data.whitemax[4],			SDCFG_UINT16 },
		{ "whitemax_05",	&IR_Sensor.data.whitemax[5],			SDCFG_UINT16 },
		{ "whitemax_06",	&IR_Sensor.data.whitemax[6],			SDCFG_UINT16 },
		{ "whitemax_07",	&IR_Sensor.data.whitemax[7],			SDCFG_UINT16 },
		{ "whitemax_08",	&IR_Sensor.data.whitemax[8],			SDCFG_UINT16 },
		{ "whitemax_09",	&IR_Sensor.data.whitemax[9],			SDCFG_UINT16 },
		{ "whitemax_10",	&IR_Sensor.data.whitemax[10],			SDCFG_UINT16 },
		{ "whitemax_11",	&IR_Sensor.data.whitemax[11],			SDCFG_UINT16 },
		{ "whitemax_12",	&IR_Sensor.data.whitemax[12],			SDCFG_UINT16 },
		{ "whitemax_13",	&IR_Sensor.data.whitemax[13],			SDCFG_UINT16 },
		{ "whitemax_14",	&IR_Sensor.data.whitemax[14],			SDCFG_UINT16 },
		{ "whitemax_15",	&IR_Sensor.data.whitemax[15],			SDCFG_UINT16 },
		{ "whitemax_16",	&IR_Sensor.data.whitemax[16],			SDCFG_UINT16 },
		{ "whitemax_17",	&IR_Sensor.data.whitemax[17],			SDCFG_UINT16 },
		{ "blackmax_00",	&IR_Sensor.data.blackmax[0],			SDCFG_UINT16 },
		{ "blackmax_01",	&IR_Sensor.data.blackmax[1],			SDCFG_UINT16 },
		{ "blackmax_02",	&IR_Sensor.data.blackmax[2],			SDCFG_UINT16 },
		{ "blackmax_03",	&IR_Sensor.data.blackmax[3],			SDCFG_UINT16 },
		{ "blackmax_04",	&IR_Sensor.data.blackmax[4],			SDCFG_UINT16 },
		{ "blackmax_05",	&IR_Sensor.data.blackmax[5],			SDCFG_UINT16 },
		{ "blackmax_06",	&IR_Sensor.data.blackmax[6],			SDCFG_UINT16 },
		{ "blackmax_07",	&IR_Sensor.data.blackmax[7],			SDCFG_UINT16 },
		{ "blackmax_08",	&IR_Sensor.data.blackmax[8],			SDCFG_UINT16 },
		{ "blackmax_09",	&IR_Sensor.data.blackmax[9],			SDCFG_UINT16 },
		{ "blackmax_10",	&IR_Sensor.data.blackmax[10],			SDCFG_UINT16 },
		{ "blackmax_11",	&IR_Sensor.data.blackmax[11],			SDCFG_UINT16 },
		{ "blackmax_12",	&IR_Sensor.data.blackmax[12],			SDCFG_UINT16 },
		{ "blackmax_13",	&IR_Sensor.data.blackmax[13],			SDCFG_UINT16 },
		{ "blackmax_14",	&IR_Sensor.data.blackmax[14],			SDCFG_UINT16 },
		{ "blackmax_15",	&IR_Sensor.data.blackmax[15],			SDCFG_UINT16 },
		{ "blackmax_16",	&IR_Sensor.data.blackmax[16],			SDCFG_UINT16 },
		{ "blackmax_17",	&IR_Sensor.data.blackmax[17],			SDCFG_UINT16 },
		{ "coef_bias_00",	&IR_Sensor.data.normalized_coef_bias[0],	SDCFG_UINT16 },
		{ "coef_bias_01",	&IR_Sensor.data.normalized_coef_bias[1],	SDCFG_UINT16 },
		{ "coef_bias_02",	&IR_Sensor.data.normalized_coef_bias[2],	SDCFG_UINT16 },
		{ "coef_bias_03",	&IR_Sensor.data.normalized_coef_bias[3],	SDCFG_UINT16 },
		{ "coef_bias_04",	&IR_Sensor.data.normalized_coef_bias[4],	SDCFG_UINT16 },
		{ "coef_bias_05",	&IR_Sensor.data.normalized_coef_bias[5],	SDCFG_UINT16 },
		{ "coef_bias_06",	&IR_Sensor.data.normalized_coef_bias[6],	SDCFG_UINT16 },
		{ "coef_bias_07",	&IR_Sensor.data.normalized_coef_bias[7],	SDCFG_UINT16 },
		{ "coef_bias_08",	&IR_Sensor.data.normalized_coef_bias[8],	SDCFG_UINT16 },
		{ "coef_bias_09",	&IR_Sensor.data.normalized_coef_bias[9],	SDCFG_UINT16 },
		{ "coef_bias_10",	&IR_Sensor.data.normalized_coef_bias[10],	SDCFG_UINT16 },
		{ "coef_bias_11",	&IR_Sensor.data.normalized_coef_bias[11],	SDCFG_UINT16 },
		{ "coef_bias_12",	&IR_Sensor.data.normalized_coef_bias[12],	SDCFG_UINT16 },
		{ "coef_bias_13",	&IR_Sensor.data.normalized_coef_bias[13],	SDCFG_UINT16 },
		{ "coef_bias_14",	&IR_Sensor.data.normalized_coef_bias[14],	SDCFG_UINT16 },
		{ "coef_bias_15",	&IR_Sensor.data.normalized_coef_bias[15],	SDCFG_UINT16 },
		{ "coef_bias_16",	&IR_Sensor.data.normalized_coef_bias[16],	SDCFG_UINT16 },
		{ "coef_bias_17",	&IR_Sensor.data.normalized_coef_bias[17],	SDCFG_UINT16 },
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
	// 캘리브레이션 시작 시 기존 SD카드 저장값을 먼저 불러온다.
	// (없으면 그냥 무시하고 새로 캘리브레이션 진행)
	FRESULT load_res = Sensor_Load_Calibration();
	LCD_Printf(0, 0, "Load:%d", load_res);
	HAL_Delay(500);
	LCD_Clear();

	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "White Max");
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 10, "%02d", IR_Sensor.data.idx);
		if (IR_Sensor.data.whitemax[i] < IR_Sensor.data.raw[i]) {
			IR_Sensor.data.whitemax[i] = IR_Sensor.data.raw[i];
		}
		Sensor_Printf(i, IR_Sensor.data.whitemax);
		i = (i + 1) % 18;
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
	Sensor_Start();
	LCD_Printf(0, 0, "Black Max");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 10, "%02d", IR_Sensor.data.idx);

		if (IR_Sensor.data.blackmax[i] < IR_Sensor.data.raw[i]) {
			IR_Sensor.data.blackmax[i] = IR_Sensor.data.raw[i];
		}
		Sensor_Printf(i, IR_Sensor.data.blackmax);
		i = (i + 1) % 18;
	}
	for (i = 0; i < 18; i++) {
		const uint16_t wmax = IR_Sensor.data.whitemax[i];
		const uint16_t bmax = IR_Sensor.data.blackmax[i];
		uint16_t range = (wmax > bmax) ? (wmax - bmax) : 0;
		IR_Sensor.data.normalized_coef_bias[i] =
				(range > 0) ? (uint16_t) ((255U << 8) / range) : 0;
	}
	IR_Sensor.is_calibration = 1;

	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);

	// 캘리브레이션 결과를 SD카드에 저장
	FRESULT save_res = Sensor_Save_Calibration();
	LCD_Printf(0, 0, "Save:%d", save_res);
	HAL_Delay(500);
	LCD_Clear();

	// 저장 직후 센서 상태(라인 인식 여부) 확인 화면으로 바로 진입
	Sensor_State_Printf();
}

void Sensor_Raw_Printf() {
	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "Sensor Raw");
	UserInput_t bt;
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		Sensor_Printf(i, IR_Sensor.data.raw);
		LCD_Printf(0, 13, "%2d", IR_Sensor.data.idx);
		i = (i + 1) % 18;
	}

	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void Sensor_Normalize_Printf() {
	Sensor_Start();
	uint8_t i = 0;
	LCD_Printf(0, 0, "Sensor Normal");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		Sensor_Printf(i, IR_Sensor.data.raw);
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
		char state = (IR_Sensor.data.state & 0x01 << i) ? '1' : '0';
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
		LCD_Printf(0, 1, "%6.3f", IR_Sensor.data.line_position);
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void IMU_Test() {
	LCD_Printf(0, 0, "IMU Test");
	uint32_t last_tick = HAL_GetTick();

	LSM6DS3_ReadGyroZ_DMA_Start();   // 최초 요청 kick

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
