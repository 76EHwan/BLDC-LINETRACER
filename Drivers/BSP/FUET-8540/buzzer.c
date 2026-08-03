/*
 * buzzer.c
 *
 *  Created on: 2026. 6. 2.
 *      Author: kth59
 */

#include "buzzer.h"
#include "dac.h"
#include "tim.h"
#include "lptim.h"    // ★ 추가
#include "sensor.h"  // ★ 추가 (buzzer_timer_count 접근용)

#define TIM_BUZZER			(&htim6)
#define Buzzer_LPTIM	    (&hlptim3) // ★ drive.c에서 이동됨

static uint32_t dac_buf[2] = { 0, 4095 };

void Buzzer_Init(void) {
	HAL_DAC_SetValue(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL);
}

void Buzzer_Start(void) {
	HAL_TIM_Base_Start(TIM_BUZZER);

	HAL_DAC_Start_DMA(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL, dac_buf, 2,
	DAC_ALIGN_12B_R);
}

void Buzzer_Stop(void) {
	HAL_TIM_Base_Stop(TIM_BUZZER);
	HAL_DAC_Stop_DMA(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL);

	HAL_DAC_SetValue(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL);
}

void Buzzer_SetVolume(uint16_t level) {
	if (level > 4095)
		level = 4095;
	dac_buf[1] = level;
}

// ==============================================
// ★ drive.c에서 이동된 타이머 및 끄기 기능
// ==============================================

void LPTIM3_IRQ_Handler(void) {
	if (buzzer_timer_count > 0) {
		buzzer_timer_count--;
		if (buzzer_timer_count == 0) {
			Buzzer_Stop();
		}
	}
}

void Buzzer_Discount_Start(void) {
	HAL_LPTIM_Counter_Start_IT(Buzzer_LPTIM, 0);
}

void Buzzer_Discount_Stop(void) {
	HAL_DAC_SetValue(DAC_BUZZER_HANDLER, DAC_BUZZER_CHANNEL, DAC_ALIGN_12B_R, 0);
	HAL_LPTIM_Counter_Stop_IT(Buzzer_LPTIM);
}
