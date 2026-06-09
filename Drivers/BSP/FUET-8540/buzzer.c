/*
 * buzzer.c
 *
 *  Created on: 2026. 6. 2.
 *      Author: kth59
 */

#include "buzzer.h"

extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;

static uint32_t dac_buf[2] = { 0, 4095 >> 6 };

void Buzzer_Init(void) {
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Buzzer_Start(void) {
	HAL_TIM_Base_Start(&htim6);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, dac_buf, 2,
	DAC_ALIGN_12B_R);
}

void Buzzer_Stop(void) {
	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Buzzer_SetVolume(uint16_t level) {
	if (level > 4095)
		level = 4095;
	dac_buf[1] = level;
}
