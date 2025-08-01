/*
 * custom_delay.c
 *
 *  Created on: Jul 23, 2025
 *      Author: kth59
 */


#include "custom_delay.h"

/**
  * @brief DWT 사이클 카운터 초기화
  * @param None
  * @retval None
  */
void Custom_Delay_Init(void) {
    // CoreDebug DWT 활성화 (TRCENA 비트 설정)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#ifdef STM32H7 // STM32H7 시리즈의 경우 DWT 레지스터 잠금 해제 필요
    DWT->LAR = 0xC5ACCE55; // DWT Lock Access Register
#endif

    // DWT 사이클 카운터 활성화 (CYCCNTENA 비트 설정)
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 사이클 카운터 리셋
    DWT->CYCCNT = 0;
}

/**
  * @brief 마이크로초(us) 단위 딜레이
  * @param us: 딜레이할 마이크로초 값
  * @retval None
  */
void Custom_Delay_us(uint32_t us) {
    uint32_t start_tick = DWT->CYCCNT;
    // HCLK 주파수를 얻어와서 사이클 수를 계산
    // HAL_RCC_GetHCLKFreq() -> STM32CubeMX에서 설정한 HCLK 주파수를 반환
    uint32_t delay_cycles = (HAL_RCC_GetHCLKFreq() / 500000) * us;

    while ((DWT->CYCCNT - start_tick) < delay_cycles);
}
