/*
 * sensor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "init.h"
#include "adc.h"
#include "sensor.h"
#include "motor.h"
#include "lcd.h"
#include "tim.h"
#include "lptim.h"
#include "math.h"
#include "switch.h"

#define NUM_ADC_CHANNELS 14

#define ADC_SENSOR_CHANNEL	(&hadc1)
#define ADC_BATTERY_CHANNEL	(&hadc2)
#define ADC_MARKER_CHANNEL	(&hadc3)

#define ADC_SENSOR_TIM		(&hlptim3)
#define ADC_BATTERY_TIM		(&hlptim5)

#define VARIANCE			0.4f
#define PDF_COEFF			(1.f / VARIANCE / M_SQRTPI / M_SQRT2)
#define PDF_EXP(x)			(-powf(x,2.f) / (2.f * powf(VARIANCE, 2.f)))

sensor_t sensor = { { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, 0, 0.f, 0.f, 100, 0.f };

menu_t sensorMenu[] = { { "1.S Raw   ", Sensor_Test_Raw }, { "2.S Norm  ",
		Sensor_Test_Normalized }, { "3.S State ", Sensor_Test_State }, {
		"4.S Pos   ", Sensor_Test_Position },
		{ "5.S Volt", Sensor_Test_Voltage }, { "6.S " }, { "7.S Volt  ",
				Sensor_Test_Voltage }, { "8.OUT     ", } };

float_t positionWeight[14] = { -1.3f, -1.1f, -0.9f, -0.7f, -0.5f, -0.3f, -0.1f,
		0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.1f, 1.3f };

bool is_sensor_start = false;
bool is_marker_start = false;

//volatile uint8_t adc_dma_complete_flag = 1;
//
//uint32_t adc_dma_buffer[NUM_ADC_CHANNELS];
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
//	if(hadc->Instance == ADC1){
//		adc_dma_complete_flag = 1;
//	}
//}

void Sensor_Test_Menu() {
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
//		ADC_Battery_Start();
	Encoder_Start();
	static uint8_t maxMenu = sizeof(sensorMenu) / sizeof(menu_t);
	static uint8_t beforeMenu = 0;
	while (1) {
		uint32_t cnt = ((hlptim1.Instance->CNT + 128) / 256 + beforeMenu)
				% maxMenu;
		Custom_LCD_Printf(0, 0, "Main Menu", hlptim1.Instance->CNT);
		for (uint8_t i = 0; i < maxMenu; i++) {
			Set_Color(cnt, i);
			Custom_LCD_Printf(0, i + 1, "%s", (sensorMenu + i)->name);
		}
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
		//		Show_Remain_Battery();
		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
			while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
				;
			Custom_LCD_Clear();
			Encoder_Stop();
			if (cnt == maxMenu - 1)
				return;
			(sensorMenu + cnt)->func();
			Encoder_Start();
			Custom_LCD_Clear();
			beforeMenu = cnt;
		}
	}
}

void Sensor_Start() {
	is_sensor_start = true;
	is_marker_start = true;
	HAL_LPTIM_Counter_Start_IT(ADC_SENSOR_TIM, 0);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	htim15.Instance->CCR1 = 0;

//	if (HAL_ADC_Start_DMA(ADC_SENSOR_CHANNEL, (uint32_t*) adc_dma_buffer,
//			NUM_ADC_CHANNELS) != HAL_OK) {
//		Error_Handler();
//	}

}

void Sensor_Stop() {
	htim15.Instance->CCR1 = 0;
	HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin, 0);
	HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin, 0);
	HAL_ADC_Stop(ADC_SENSOR_CHANNEL);
	HAL_ADC_Stop(ADC_MARKER_CHANNEL);
	HAL_LPTIM_Counter_Stop_IT(ADC_SENSOR_TIM);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
}

__STATIC_INLINE uint16_t Sensor_ADC_Read(ADC_HandleTypeDef *hadc) {
	static uint16_t raw;
	__disable_irq();
	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
		raw = HAL_ADC_GetValue(hadc);
	}
	__enable_irq();
	return raw;
}

void Sensor_LPTIM3_IRQ() {
	static uint8_t i = 0;
	static uint8_t raw;
	static float pos_sum1 = 0.f;
	static float pos_sum2 = 0.f;

	if (is_sensor_start) {
		i = 0;
		is_sensor_start = false;
	}

	float pos_weight = *(positionWeight + i);
	uint8_t normalize = *(sensor.normalized + i);

	pos_sum1 -= pos_weight * normalize;
	pos_sum2 -= normalize;

	raw = Sensor_ADC_Read(ADC_SENSOR_CHANNEL) >> 4;

	if (raw > *(sensor.whiteMax + i))
		normalize = 0xff;
	else if (raw < *(sensor.blackMax + i))
		normalize = 0;
	else
		normalize = 0xff * (raw - *(sensor.blackMax + i))
				/ *(sensor.normalizeCoef + i);

	sensor.state = (sensor.state & ~(0x01 << i))
			| ((normalize > sensor.threshold) << i);

	pos_sum1 += pos_weight * normalize;
	pos_sum2 += normalize;

	sensor.position = (pos_sum2) ? pos_sum1 / pos_sum2 : 0;
	*(sensor.normalized + i) = normalize;

//	if (adc_dma_complete_flag == 1) {
//		adc_dma_complete_flag = 0; // 플래그 초기화
//
//		// 이제 adc_dma_buffer 배열에 14개 채널의 최신 ADC 값이 랭크 순서대로 저장되어 있습니다.
//		// 이 값들을 활용하여 원하는 로직을 수행합니다.
//		// 예: UART로 값 출력, LCD에 표시 등
//		for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
////			 printf("ADC[%d]: %lu\r\n", i, adc_dma_buffer[i]); // 디버깅용 출력
//			*(sensor.raw + i) = *(adc_dma_buffer + i) >> 4;
//		}
//	}

	*(sensor.raw + i) = raw;

	i = (i + 1) % 14;
}

void Marker_LPTIM3_IRQ() {
	static uint8_t i = 0;
	static uint8_t raw;
	static uint8_t normalize;
	if (is_marker_start) {
		i = 0;
		is_marker_start = false;
	}

	raw = Sensor_ADC_Read(ADC_MARKER_CHANNEL);

	if (raw > *(sensor.whiteMax + i + 14))
		normalize = 0xff;
	else if (raw < *(sensor.blackMax + i + 14))
		normalize = 0;
	else
		normalize = 0xff * (raw - *(sensor.blackMax + i + 14))
				/ *(sensor.normalizeCoef + i + 14);
	sensor.state = (sensor.state & ~(0x01 << (i + 14)))
			| ((normalize > sensor.threshold) << (i + 14));


	HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin,
			(sensor.state >> 14) & 0x01);
	HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin,
			(sensor.state >> 15) & 0x01);
	htim15.Instance->CCR1 = (sensor.state & (0x03 << 14)) ? 5 : 0;

	*(sensor.normalized + i + 14) = normalize;
	*(sensor.raw + i + 14) = raw;

	i = (i + 1) & 0x01;
}

void Sensor_Printf(char *name, int32_t *sensorValue) {
	Custom_LCD_Printf(0, 0, name);
	Custom_LCD_Printf(0, 1, "0/1:%02x %02x", *(sensorValue),
			*(sensorValue + 1));
	Custom_LCD_Printf(0, 2, "2/3:%02x %02x", *(sensorValue + 2),
			*(sensorValue + 3));
	Custom_LCD_Printf(0, 3, "4/5:%02x %02x", *(sensorValue + 4),
			*(sensorValue + 5));
	Custom_LCD_Printf(0, 4, "6/7:%02x %02x", *(sensorValue + 6),
			*(sensorValue + 7));
	Custom_LCD_Printf(0, 5, "8/9:%02x %02x", *(sensorValue + 8),
			*(sensorValue + 9));
	Custom_LCD_Printf(0, 6, "a/b:%02x %02x", *(sensorValue + 10),
			*(sensorValue + 11));
	Custom_LCD_Printf(0, 7, "c/d:%02x %02x", *(sensorValue + 12),
			*(sensorValue + 13));
	Custom_LCD_Printf(0, 8, "left right");
	Custom_LCD_Printf(0, 9, "%02x      %02x", *(sensorValue + 14),
			*(sensorValue + 15));
}

void Sensor_Test_Raw() {
	Sensor_Start();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Sensor_Printf("S Raw ", sensor.raw);
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
	Sensor_Stop();
}

void Sensor_Calibration() {
	Sensor_Start();
	uint8_t i = 0;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Sensor_Printf("Cali WhiteMax", sensor.whiteMax);
		*(sensor.whiteMax + i) = (*(sensor.raw + i) > *(sensor.whiteMax + i)) ?
				*(sensor.raw + i) : *(sensor.whiteMax + i);
		i = (i + 1) & 0x0F;
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		Custom_LCD_Clear();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Sensor_Printf("Cali BlackMax", sensor.blackMax);
		*(sensor.blackMax + i) =
				(*(sensor.raw + i) > *(sensor.blackMax + i)) ?
						*(sensor.raw + i) : *(sensor.blackMax + i);
		i = (i + 1) & 0x0F;
	}
	Sensor_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		Custom_LCD_Clear();

	for (i = 0; i < 16; i++) {
		*(sensor.normalizeCoef + i) = *(sensor.whiteMax + i)
				- *(sensor.blackMax + i);
	}
}

void Sensor_Test_Normalized() {
	Sensor_Start();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Sensor_Printf("S Normalize", sensor.normalized);
	}
	Sensor_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
}

void Sensor_Test_State() {
	Sensor_Start();
	uint8_t i = 0;
	char sensorStateL[7];
	char sensorStateR[7];

	Custom_LCD_Printf(0, 0, "S State");
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		*(sensorStateL + i) = (sensor.state & (0x01 << i)) ? '1' : '0';
		*(sensorStateR + i) = (sensor.state & (0x01 << (i + 7))) ? '1' : '0';
		Custom_LCD_Printf(0, 1, "L:%.7s   ", sensorStateL);
		Custom_LCD_Printf(0, 2, "R:%.7s   ", sensorStateR);
		i = (i + 1) % 7;
	}
	Sensor_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
}

void Sensor_Test_Position() {
	Sensor_Start();
	Custom_LCD_Printf(0, 0, "S Position");
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Custom_LCD_Printf(0, 1, "%.6f ", sensor.position);
	}
	Sensor_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
}

void Sensor_Test_Window() {
	Sensor_Start();
	Custom_LCD_Printf(0, 0, "S Window");
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {

	}
	Sensor_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
}

//__STATIC_INLINE uint16_t Battery_ADC_Read() {
//
//}

void Battery_Start(){
	HAL_LPTIM_Counter_Start_IT(ADC_BATTERY_TIM, 0);
}

void Battery_Stop(){
	HAL_LPTIM_Counter_Stop_IT(ADC_BATTERY_TIM);
}

void ADC_Battery_LPTIM5_IRQ() {
	uint16_t adc_voltage = Sensor_ADC_Read(ADC_BATTERY_CHANNEL);
	HAL_ADC_Stop(ADC_BATTERY_CHANNEL);
	sensor.voltage = (float) (adc_voltage) / 65536.f * 3.3f * 11.f * 4.f;
}

void Sensor_Test_Voltage() {
	Battery_Start();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		Custom_LCD_Printf(0, 0, "%.6f", sensor.voltage);
	}
	Battery_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
		;
}
