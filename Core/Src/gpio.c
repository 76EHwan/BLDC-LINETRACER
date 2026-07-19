/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN (OSC32_IN)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT (OSC32_OUT)   ------> RCC_OSC32_OUT
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SENSOR_LED_R_Pin|SENSOR_PT_EN_Pin|SENSOR_IR_EN_Pin|SENSOR_MUX2_Pin
                          |SENSOR_MUX1_Pin|SENSOR_MUX3_Pin|LCD_CS_Pin|LCD_WR_RS_Pin
                          |SENSOR_LED_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SENSOR_MUX0_Pin|MTR_CS_L_Pin|MTR_CS_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MTR_nSLEEP_L_Pin|MTR_DRVOFF_L_Pin|MTR_PWM_INLX_Pin|F_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MTR_nSLEEP_R_Pin|MTR_DRVOFF_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR_LED_R_Pin E3_Pin SENSOR_PT_EN_Pin SENSOR_IR_EN_Pin
                           SENSOR_MUX2_Pin SENSOR_MUX1_Pin SENSOR_MUX3_Pin LCD_CS_Pin
                           LCD_WR_RS_Pin SENSOR_LED_L_Pin */
  GPIO_InitStruct.Pin = SENSOR_LED_R_Pin|E3_Pin|SENSOR_PT_EN_Pin|SENSOR_IR_EN_Pin
                          |SENSOR_MUX2_Pin|SENSOR_MUX1_Pin|SENSOR_MUX3_Pin|LCD_CS_Pin
                          |LCD_WR_RS_Pin|SENSOR_LED_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DVP_PWDN_Pin */
  GPIO_InitStruct.Pin = DVP_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(DVP_PWDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR_MUX0_Pin MTR_CS_L_Pin MTR_CS_R_Pin */
  GPIO_InitStruct.Pin = SENSOR_MUX0_Pin|MTR_CS_L_Pin|MTR_CS_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_nSLEEP_L_Pin MTR_DRVOFF_L_Pin MTR_PWM_INLX_Pin */
  GPIO_InitStruct.Pin = MTR_nSLEEP_L_Pin|MTR_DRVOFF_L_Pin|MTR_PWM_INLX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_nFAULT_L_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = MTR_nFAULT_L_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_nSLEEP_R_Pin MTR_DRVOFF_R_Pin */
  GPIO_InitStruct.Pin = MTR_nSLEEP_R_Pin|MTR_DRVOFF_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MTR_nFAULT_R_Pin */
  GPIO_InitStruct.Pin = MTR_nFAULT_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MTR_nFAULT_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH_LEFT_Pin SWITCH_UP_Pin SWITCH_DOWN_Pin SWITCH_RIGHT_Pin */
  GPIO_InitStruct.Pin = SWITCH_LEFT_Pin|SWITCH_UP_Pin|SWITCH_DOWN_Pin|SWITCH_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : F_CS_Pin */
  GPIO_InitStruct.Pin = F_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(F_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT1_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
