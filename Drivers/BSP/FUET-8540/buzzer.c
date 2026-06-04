/*
 * buzzer.c
 *
 *  Created on: 2026. 6. 2.
 *      Author: kth59
 */

#include "buzzer.h"

extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;

/* DMA로 내보낼 2샘플 square wave: 0V / 3.3V */
static uint32_t dac_buf[2] = { 0, 4095 };

void Buzzer_Init(void) {
	/* DAC 출력 0으로 초기화 */
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Buzzer_Start(void) {
	/* TIM6 먼저 시작 → TRGO 공급 */
	HAL_TIM_Base_Start(&htim6);

	/* DAC DMA Circular 시작 */
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, dac_buf, 2,
	DAC_ALIGN_12B_R);
}

void Buzzer_Stop(void) {
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	/* 출력 0으로 고정 → 트랜지스터 OFF */
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

/* 볼륨 조절: level 0~4095
 * DMA Circular 동작 중에도 즉시 반영됨 */
void Buzzer_SetVolume(uint16_t level) {
	if (level > 4095)
		level = 4095;
	dac_buf[1] = level;
}
