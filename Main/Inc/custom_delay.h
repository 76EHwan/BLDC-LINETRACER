/*
 * custom_delay.h
 *
 *  Created on: Jul 23, 2025
 *      Author: kth59
 */

#ifndef INC_CUSTOM_DELAY_H_
#define INC_CUSTOM_DELAY_H_

#include "stm32h7xx_hal.h" // 또는 사용하는 MCU에 맞는 HAL 헤더 파일

// DWT 초기화 함수
void Custom_Delay_Init(void);

// 마이크로초 단위 딜레이 함수
void Custom_Delay_us(uint32_t us);

#endif /* INC_CUSTOM_DELAY_H_ */
