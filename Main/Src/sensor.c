/*
 * sensor.c
 *
 *  Created on: 2026. 5. 2.
 *      Author: kth59
 */
#include "sensor.h"
#include "main.h"
#include "button.h"
#include "st7789_lcd.h"
#include <math.h>
#include "cmsis_gcc.h"  // 또는 arm_acle.h

#define SENSOR_TRIG_TIM &htim2

#define SENSOR_IR_EN_FAST()   (SENSOR_IR_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_IR_EN_Pin << 16U))
#define SENSOR_IR_DIS_FAST()  (SENSOR_IR_EN_GPIO_Port->BSRR = SENSOR_IR_EN_Pin)
#define SENSOR_PT_EN_FAST()   (SENSOR_PT_EN_GPIO_Port->BSRR = ((uint32_t)SENSOR_PT_EN_Pin << 16U))
#define SENSOR_PT_DIS_FAST()  (SENSOR_PT_EN_GPIO_Port->BSRR = SENSOR_PT_EN_Pin)

// === 라인 위치 추정 LUT/버퍼 ============================================
#define LINE_N_SENSORS     16
#define LINE_N_CANDIDATES  200
#define LINE_X_MIN        (-1.1f)
#define LINE_X_MAX        ( 1.1f)
#define LINE_DX           ((LINE_X_MAX - LINE_X_MIN) / (LINE_N_CANDIDATES - 1))

static const float line_sensor_pos[LINE_N_SENSORS] = { -1.0f, -13.0f / 15.0f,
		-11.0f / 15.0f, -9.0f / 15.0f, -7.0f / 15.0f, -5.0f / 15.0f, -3.0f
				/ 15.0f, -1.0f / 15.0f, 1.0f / 15.0f, 3.0f / 15.0f, 5.0f
				/ 15.0f, 7.0f / 15.0f, 9.0f / 15.0f, 11.0f / 15.0f, 13.0f
				/ 15.0f, 1.0f };

static uint8_t line_mu_lut[LINE_N_CANDIDATES][LINE_N_SENSORS] __attribute__((aligned(4)));
static int32_t line_sse_buf[LINE_N_CANDIDATES];

extern uint16_t adc3_buffer[3];

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

// 거리 -> 예상 normalized 값 [0, 1]. 블로그 예제: max(1 - 3|d|, 0).
// 실측 응답 곡선 있으면 이 함수만 교체. LUT 재초기화 필요.
static float Sensor_Line_Mu(float d) {
	float v = 1.0f - 3.0f * fabsf(d);
	return v > 0.0f ? v : 0.0f;
}

void Sensor_Line_LUT_Init(void) {
	for (int j = 0; j < LINE_N_CANDIDATES; j++) {
		float x = LINE_X_MIN + j * LINE_DX;
		for (int i = 0; i < LINE_N_SENSORS; i++) {
			float v = Sensor_Line_Mu(x - line_sensor_pos[i]) * 255.0f;
			line_mu_lut[j][i] = (uint8_t) (v + 0.5f);
		}
	}
}

float Sensor_Line_Estimate(void) {
	// volatile normalized snapshot (volatile read 횟수 축소 + 일관성)
	uint32_t v_packed[LINE_N_SENSORS / 4];
	// volatile snapshot을 4-byte word로 패킹
	for (int i = 0; i < LINE_N_SENSORS / 4; i++) {
		v_packed[i] = ((uint32_t) IR_Sensor.data.normalized[i * 4 + 0])
				| ((uint32_t) IR_Sensor.data.normalized[i * 4 + 1] << 8)
				| ((uint32_t) IR_Sensor.data.normalized[i * 4 + 2] << 16)
				| ((uint32_t) IR_Sensor.data.normalized[i * 4 + 3] << 24);
	}

	int best_j = 0;
	uint32_t best_sad = UINT32_MAX;

	for (int j = 0; j < LINE_N_CANDIDATES; j++) {
		const uint32_t *row = (const uint32_t*) line_mu_lut[j];
		uint32_t sad = 0;
		sad = __USADA8(v_packed[0], row[0], sad);
		sad = __USADA8(v_packed[1], row[1], sad);
		sad = __USADA8(v_packed[2], row[2], sad);
		sad = __USADA8(v_packed[3], row[3], sad);
		line_sse_buf[j] = (int32_t) sad;
		if (sad < best_sad) {
			best_sad = sad;
			best_j = j;
		}
	}

	float x = LINE_X_MIN + best_j * LINE_DX;

	// parabolic sub-pixel refinement
	if (best_j > 0 && best_j < LINE_N_CANDIDATES - 1) {
		int32_t lo = line_sse_buf[best_j - 1];
		int32_t mid = best_sad;
		int32_t hi = line_sse_buf[best_j + 1];
		int32_t denom = lo - 2 * mid + hi;
		if (denom > 0) {
			float delta = 0.5f * (float) (lo - hi) / (float) denom;
			x += delta * LINE_DX;
		}
	}
	return x;
}

void TIM7_IRQ_Handler() {
	tim7_count++;

	Set_Mux_Channel_Fast(IR_Sensor.data.idx);
	SENSOR_IR_EN_FAST();
	SENSOR_PT_EN_FAST();

	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;
	TIM2->SR = ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void ADC3_IRQ_Handler() {
	ADC3->ISR = (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

	const uint8_t idx = IR_Sensor.data.idx;

	IR_Sensor.data.raw[LEFT_MARK_SENSOR_INDEX] = adc3_buffer[0];
	IR_Sensor.data.raw[RIGHT_MARK_SENSOR_INDEX] = adc3_buffer[1];
	IR_Sensor.data.raw[idx] = adc3_buffer[2];

	SENSOR_IR_DIS_FAST();
	SENSOR_PT_DIS_FAST();

	if (IR_Sensor.is_calibration) {
		Sensor_Normalize(idx);
		Sensor_Normalize(LEFT_MARK_SENSOR_INDEX);
		Sensor_Normalize(RIGHT_MARK_SENSOR_INDEX);

		if (idx == LINE_N_SENSORS - 1) {
			IR_Sensor.data.line_position = Sensor_Line_Estimate();
		}
	}

	IR_Sensor.data.idx = (idx + 1) & 0x0F;
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
	Sensor_Line_LUT_Init();           // ← 추가
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
	}
	LCD_Clear();
	Sensor_Stop();
	Button_Wait_Release(&btn_k);
}
