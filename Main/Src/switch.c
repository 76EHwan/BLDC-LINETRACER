/*
 * switch.c
 *
 *  Created on: Jul 18, 2025
 *      Author: kth59
 */

#include "switch.h"

#define IDLE		0x00
#define READ		0x01
#define DECISION	0x02

#define MIN_TIME	300

uint8_t state = IDLE;

uint8_t Custom_Switch_Read(void) {
	bool output = KEY_NONE;
	static uint32_t preTick = 0;
	switch (state) {
	case IDLE:
		if (HAL_GetTick() - preTick < MIN_TIME)
			break;
		else if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
			state = READ;
			preTick = HAL_GetTick();
		}
		break;
	case READ:
		if (HAL_GetTick() - preTick < MIN_TIME)
			break;
		else if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
			state = DECISION;
			preTick = HAL_GetTick();
		}
		break;
	case DECISION:
		if (HAL_GetTick() - preTick < MIN_TIME)
			break;
		else {
			output = KEY_PUSH;
			preTick = HAL_GetTick();
		}
		break;
	}
	return output;
}
