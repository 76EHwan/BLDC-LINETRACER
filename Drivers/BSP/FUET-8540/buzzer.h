/*
 * buzzer.h
 *
 *  Created on: 2026. 6. 2.
 *      Author: kth59
 */

#ifndef BSP_FUET_8540_BUZZER_H_
#define BSP_FUET_8540_BUZZER_H_

#include "main.h"

void Buzzer_Init(void);
void Buzzer_Start(void);
void Buzzer_Stop(void);
void Buzzer_SetVolume(uint16_t level);

#endif /* BSP_FUET_8540_BUZZER_H_ */
