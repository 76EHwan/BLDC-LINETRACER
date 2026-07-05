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
#define LINE_LOST_SUM_MIN   200u    // weight 합이 이하면 라인 상실로 판단

static const float line_sensor_pos[LINE_N_SENSORS] = { -1.0f, -13.0f / 15.0f,
		-11.0f / 15.0f, -9.0f / 15.0f, -7.0f / 15.0f, -5.0f / 15.0f, -3.0f
				/ 15.0f, -1.0f / 15.0f, 1.0f / 15.0f, 3.0f / 15.0f, 5.0f
				/ 15.0f, 7.0f / 15.0f, 9.0f / 15.0f, 11.0f / 15.0f, 13.0f
				/ 15.0f, 1.0f };

__attribute__((section(".ram_d3"), aligned(32)))    uint16_t adc3_buffer[3];

volatile uint32_t tim7_count = 0;
volatile uint32_t adc_count = 0;

volatile Sensor_TypeDef IR_Sensor = { .data = { .idx = 0, .raw = { 0 },
		.blackmax = { 0 }, .whitemax = { 0 }, .normalized = { 0 }, .state = 0,
		.threshold = 100 }, .is_calibration = 0 };

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
	(index & 0x01) ?
			(SENSOR_MUX0_GPIO_Port->BSRR = SENSOR_MUX0_Pin) :
			(SENSOR_MUX0_GPIO_Port->BSRR = ((uint32_t) SENSOR_MUX0_Pin << 16U));
	(index & 0x02) ?
			(SENSOR_MUX1_GPIO_Port->BSRR = SENSOR_MUX1_Pin) :
			(SENSOR_MUX1_GPIO_Port->BSRR = ((uint32_t) SENSOR_MUX1_Pin << 16U));
	(index & 0x04) ?
			(SENSOR_MUX2_GPIO_Port->BSRR = SENSOR_MUX2_Pin) :
			(SENSOR_MUX2_GPIO_Port->BSRR = ((uint32_t) SENSOR_MUX2_Pin << 16U));
	(index & 0x08) ?
			(SENSOR_MUX3_GPIO_Port->BSRR = SENSOR_MUX3_Pin) :
			(SENSOR_MUX3_GPIO_Port->BSRR = ((uint32_t) SENSOR_MUX3_Pin << 16U));
}

__STATIC_INLINE void Sensor_Normalize(uint8_t idx) {
	const uint16_t raw = IR_Sensor.data.raw[idx];
	const uint16_t bmax = IR_Sensor.data.blackmax[idx];
	const uint16_t bias = IR_Sensor.data.normalized_coef_bias[idx];
	uint16_t diff = (raw > bmax) ? (raw - bmax) : 0;
	uint32_t result = ((uint32_t) diff * bias) >> 8;
	IR_Sensor.data.normalized[idx] = (result > 255U) ? 255U : (uint8_t) result;
}

// weighted centroid. weight = 라인에서 커지는 값, 위치를 weight로 가중 평균.
// 라인 상실(weight 합 부족) 시 마지막 위치 유지.
float Sensor_Line_Estimate(void) {
	float num = 0.0f;
	uint32_t den = 0;

	for (int i = 0; i < LINE_N_SENSORS; i++) {
		uint8_t w = LINE_WEIGHT(IR_Sensor.data.normalized[i]);
		if (w < LINE_W_DEADBAND) {
			continue;
		}
		num += (float) w * line_sensor_pos[i];
		den += w;
	}

	if (den < LINE_LOST_SUM_MIN) {
		return IR_Sensor.data.line_position;   // 상실: 마지막 위치 hold
	}
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
	adc_count++;
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
			IR_Sensor.data.state |= ((uint32_t)(w > IR_Sensor.data.threshold) << idx);

			// 중앙 한 바퀴 끝 -> 라인 위치 갱신
			if (idx == LINE_N_SENSORS - 1) {
				IR_Sensor.data.line_position = Sensor_Line_Estimate();
			}
		}

		IR_Sensor.data.idx = idx + 1;   // -> 다음 중앙 또는 mark(16)
	} else {
		// === active high: 좌우 mark 저장 + 계산 ===
		IR_Sensor.data.raw[LEFT_MARK_SENSOR_INDEX]  = adc3_buffer[0];
		IR_Sensor.data.raw[RIGHT_MARK_SENSOR_INDEX] = adc3_buffer[1];

		if (IR_Sensor.is_calibration) {
			Sensor_Normalize(LEFT_MARK_SENSOR_INDEX);
			Sensor_Normalize(RIGHT_MARK_SENSOR_INDEX);

			// 좌우 mark 검출 (level). normalized > threshold 이면 mark 위.
			uint8_t left_on  =
					IR_Sensor.data.normalized[LEFT_MARK_SENSOR_INDEX]  > IR_Sensor.data.threshold;
			uint8_t right_on =
					IR_Sensor.data.normalized[RIGHT_MARK_SENSOR_INDEX] > IR_Sensor.data.threshold;

			// state의 bit 16/17에 저장 (state는 uint32_t 이상이어야 함)
			IR_Sensor.data.state &=
					~((1u << LEFT_MARK_SENSOR_INDEX) | (1u << RIGHT_MARK_SENSOR_INDEX));
			IR_Sensor.data.state |= ((uint32_t) left_on  << LEFT_MARK_SENSOR_INDEX);
			IR_Sensor.data.state |= ((uint32_t) right_on << RIGHT_MARK_SENSOR_INDEX);
		}

		IR_Sensor.data.idx = 0;         // 사이클 재시작
	}
}

void Sensor_Calibration() {
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
	Button_Wait_Release(&btn_k);
	Sensor_Stop();
}

void Sensor_Raw_Printf() {
	Sensor_Start();

	uint8_t i = 0;
	LCD_Printf(0, 0, "Sensor Raw");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 10, "%02d", IR_Sensor.data.idx);
		Sensor_Printf(i, IR_Sensor.data.raw);
		i = (i + 1) % 18;
		HAL_GPIO_WritePin(SENSOR_LED_L_GPIO_Port, SENSOR_LED_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SENSOR_LED_R_GPIO_Port, SENSOR_LED_R_Pin, GPIO_PIN_RESET);

		if(Button_Get_Input() == INPUT_CMD_L_HOLD) {
			HAL_GPIO_WritePin(SENSOR_LED_L_GPIO_Port, SENSOR_LED_L_Pin, GPIO_PIN_SET);
		}

		if(Button_Get_Input() == INPUT_CMD_R_HOLD) {
			HAL_GPIO_WritePin(SENSOR_LED_R_GPIO_Port, SENSOR_LED_R_Pin, GPIO_PIN_SET);
		}
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}

void IMU_Test() {
	for (uint16_t a = 0; a < 128; a++) {
		if (HAL_I2C_IsDeviceReady(IMU_I2C, a << 1, 2, 10) == HAL_OK) {
			LCD_Printf(0, 0, "found:%02x", a);
			break;
		}
		LCD_Printf(0, 1, "Not fount 404");
	}
	while (1) {
		LED_Test();
	}
}
