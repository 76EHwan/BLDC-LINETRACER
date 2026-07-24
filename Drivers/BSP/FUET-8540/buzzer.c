/*
 * buzzer.c
 *
 *  Created on: 2026. 6. 2.
 *      Author: kth59
 */

#include "buzzer.h"
#include "dac.h"
#include "tim.h"

#define TIM_BUZZER			(&htim6)
#define DAC_BUZZER			(&hdac1)
#define DAC_BUZZER_CHANNEL	(DAC_CHANNEL_1)

static uint32_t dac_buf[2] = { 0, 4095 };

void Buzzer_Init(void) {
	HAL_DAC_SetValue(DAC_BUZZER, DAC_BUZZER_CHANNEL, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(DAC_BUZZER, DAC_BUZZER_CHANNEL);
}

void Buzzer_Start(void) {
	HAL_TIM_Base_Start(TIM_BUZZER);

	HAL_DAC_Start_DMA(DAC_BUZZER, DAC_BUZZER_CHANNEL, dac_buf, 2,
	DAC_ALIGN_12B_R);
}

void Buzzer_Stop(void) {
	HAL_TIM_Base_Stop(TIM_BUZZER);
	HAL_DAC_Stop_DMA(DAC_BUZZER, DAC_BUZZER_CHANNEL);

	HAL_DAC_SetValue(DAC_BUZZER, DAC_BUZZER_CHANNEL, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(DAC_BUZZER, DAC_BUZZER_CHANNEL);
}

void Buzzer_SetVolume(uint16_t level) {
	if (level > 4095)
		level = 4095;
	dac_buf[1] = level;
}
