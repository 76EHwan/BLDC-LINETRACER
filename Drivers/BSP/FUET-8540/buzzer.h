/*
 * buzzer.h
 *
 *  Created on: 2026. 6. 2.
 *      Author: kth59
 */

#ifndef BSP_FUET_8540_BUZZER_H_
#define BSP_FUET_8540_BUZZER_H_

#define DAC_BUZZER_HANDLER	(&hdac1)
#define DAC_BUZZER_CHANNEL	(DAC_CHANNEL_1)

#include "main.h"

void Buzzer_Init(void);
void Buzzer_Start(void);
void Buzzer_Stop(void);
void Buzzer_SetVolume(uint16_t level);

// ★ 추가된 함수 원형
void Buzzer_Discount_Start(void);
void Buzzer_Discount_Stop(void);

#endif /* BSP_FUET_8540_BUZZER_H_ */
