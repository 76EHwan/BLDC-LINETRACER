/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define LED_ON		HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET)
#define LED_OFF		HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_RESET)
#define LED_TOGGLE	HAL_GPIO_TogglePin(E3_GPIO_Port, E3_Pin)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void LED_Blink(uint32_t delay);
void LED_Test();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CUSTOM_HID_EPOUT_SIZE 0x40U
#define CUSTOM_HID_EPIN_SIZE 0x40U
#define SENSOR_LED_R_Pin GPIO_PIN_2
#define SENSOR_LED_R_GPIO_Port GPIOE
#define E3_Pin GPIO_PIN_3
#define E3_GPIO_Port GPIOE
#define SENSOR_PT_EN_Pin GPIO_PIN_4
#define SENSOR_PT_EN_GPIO_Port GPIOE
#define SENSOR_IR_EN_Pin GPIO_PIN_5
#define SENSOR_IR_EN_GPIO_Port GPIOE
#define FAN_PWM_Pin GPIO_PIN_6
#define FAN_PWM_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define PWR_ADC_BAT_Pin GPIO_PIN_0
#define PWR_ADC_BAT_GPIO_Port GPIOC
#define SENSOR_ADC_IR_IN_R_Pin GPIO_PIN_1
#define SENSOR_ADC_IR_IN_R_GPIO_Port GPIOC
#define SENSOR_ADC_IR_IN_C_Pin GPIO_PIN_2
#define SENSOR_ADC_IR_IN_C_GPIO_Port GPIOC
#define SENSOR_ADC_IR_IN_L_Pin GPIO_PIN_3
#define SENSOR_ADC_IR_IN_L_GPIO_Port GPIOC
#define MTR_PWM_R_Pin GPIO_PIN_0
#define MTR_PWM_R_GPIO_Port GPIOA
#define MTR_PWM_L_Pin GPIO_PIN_1
#define MTR_PWM_L_GPIO_Port GPIOA
#define BUZZER_DAC_IN_Pin GPIO_PIN_4
#define BUZZER_DAC_IN_GPIO_Port GPIOA
#define MTR_DAC_ILIMIT_Pin GPIO_PIN_5
#define MTR_DAC_ILIMIT_GPIO_Port GPIOA
#define MTR_ADC_SOC_R_Pin GPIO_PIN_6
#define MTR_ADC_SOC_R_GPIO_Port GPIOA
#define DVP_PWDN_Pin GPIO_PIN_7
#define DVP_PWDN_GPIO_Port GPIOA
#define MTR_ADC_SOA_R_Pin GPIO_PIN_4
#define MTR_ADC_SOA_R_GPIO_Port GPIOC
#define MTR_ADC_SOC_L_Pin GPIO_PIN_5
#define MTR_ADC_SOC_L_GPIO_Port GPIOC
#define MTR_PWM_W_L_Pin GPIO_PIN_0
#define MTR_PWM_W_L_GPIO_Port GPIOB
#define MTR_ADC_SOA_L_Pin GPIO_PIN_1
#define MTR_ADC_SOA_L_GPIO_Port GPIOB
#define SENSOR_MUX0_Pin GPIO_PIN_2
#define SENSOR_MUX0_GPIO_Port GPIOB
#define SENSOR_MUX2_Pin GPIO_PIN_7
#define SENSOR_MUX2_GPIO_Port GPIOE
#define SENSOR_MUX1_Pin GPIO_PIN_8
#define SENSOR_MUX1_GPIO_Port GPIOE
#define SENSOR_MUX3_Pin GPIO_PIN_9
#define SENSOR_MUX3_GPIO_Port GPIOE
#define LCD_BK_Pin GPIO_PIN_10
#define LCD_BK_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LCD_SCK_Pin GPIO_PIN_12
#define LCD_SCK_GPIO_Port GPIOE
#define LCD_WR_RS_Pin GPIO_PIN_13
#define LCD_WR_RS_GPIO_Port GPIOE
#define LCD_MOSI_Pin GPIO_PIN_14
#define LCD_MOSI_GPIO_Port GPIOE
#define SENSOR_LED_L_Pin GPIO_PIN_15
#define SENSOR_LED_L_GPIO_Port GPIOE
#define ENC_IN1_L_Pin GPIO_PIN_10
#define ENC_IN1_L_GPIO_Port GPIOB
#define MTR_CS_L_Pin GPIO_PIN_12
#define MTR_CS_L_GPIO_Port GPIOB
#define SPI2_MTR_ENC_SCK_Pin GPIO_PIN_13
#define SPI2_MTR_ENC_SCK_GPIO_Port GPIOB
#define SPI2_MTR_ENC_MISO_Pin GPIO_PIN_14
#define SPI2_MTR_ENC_MISO_GPIO_Port GPIOB
#define SPI2_MTR_MOSI_Pin GPIO_PIN_15
#define SPI2_MTR_MOSI_GPIO_Port GPIOB
#define MTR_nSLEEP_L_Pin GPIO_PIN_8
#define MTR_nSLEEP_L_GPIO_Port GPIOD
#define MTR_nFAULT_L_Pin GPIO_PIN_9
#define MTR_nFAULT_L_GPIO_Port GPIOD
#define MTR_DRVOFF_L_Pin GPIO_PIN_10
#define MTR_DRVOFF_L_GPIO_Port GPIOD
#define ENC_IN2_L_Pin GPIO_PIN_11
#define ENC_IN2_L_GPIO_Port GPIOD
#define ENC_IN1_R_Pin GPIO_PIN_12
#define ENC_IN1_R_GPIO_Port GPIOD
#define MTR_PWM_W_R_Pin GPIO_PIN_14
#define MTR_PWM_W_R_GPIO_Port GPIOD
#define MTR_PWM_INLX_Pin GPIO_PIN_15
#define MTR_PWM_INLX_GPIO_Port GPIOD
#define MTR_PWM_U_L_Pin GPIO_PIN_6
#define MTR_PWM_U_L_GPIO_Port GPIOC
#define MTR_PWM_V_L_Pin GPIO_PIN_7
#define MTR_PWM_V_L_GPIO_Port GPIOC
#define MTR_nSLEEP_R_Pin GPIO_PIN_8
#define MTR_nSLEEP_R_GPIO_Port GPIOA
#define MTR_nFAULT_R_Pin GPIO_PIN_9
#define MTR_nFAULT_R_GPIO_Port GPIOA
#define MTR_DRVOFF_R_Pin GPIO_PIN_10
#define MTR_DRVOFF_R_GPIO_Port GPIOA
#define SWITCH_LEFT_Pin GPIO_PIN_0
#define SWITCH_LEFT_GPIO_Port GPIOD
#define SWITCH_UP_Pin GPIO_PIN_1
#define SWITCH_UP_GPIO_Port GPIOD
#define SWITCH_DOWN_Pin GPIO_PIN_3
#define SWITCH_DOWN_GPIO_Port GPIOD
#define SD_DETECT_Pin GPIO_PIN_4
#define SD_DETECT_GPIO_Port GPIOD
#define SWITCH_RIGHT_Pin GPIO_PIN_5
#define SWITCH_RIGHT_GPIO_Port GPIOD
#define F_CS_Pin GPIO_PIN_6
#define F_CS_GPIO_Port GPIOD
#define MTR_CS_R_Pin GPIO_PIN_5
#define MTR_CS_R_GPIO_Port GPIOB
#define MTR_PWM_U_R_Pin GPIO_PIN_6
#define MTR_PWM_U_R_GPIO_Port GPIOB
#define MTR_PWM_V_R_Pin GPIO_PIN_7
#define MTR_PWM_V_R_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_0
#define IMU_INT1_GPIO_Port GPIOE
#define ENC_IN2_R_Pin GPIO_PIN_1
#define ENC_IN2_R_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define MTR_FGOUT_L_GPIO_Port MTR_ADC_SOA_L_GPIO_Port
#define MTR_FGOUT_L_Pin MTR_ADC_SOA_L_Pin
#define MTR_BRAKE_L_GPIO_Port MTR_ADC_SOC_L_GPIO_Port
#define MTR_BRAKE_L_Pin MTR_ADC_SOC_L_Pin
#define MTR_FGOUT_R_GPIO_Port MTR_ADC_SOA_R_GPIO_Port
#define MTR_FGOUT_R_Pin MTR_ADC_SOA_R_Pin
#define MTR_BRAKE_R_GPIO_Port MTR_ADC_SOC_R_GPIO_Port
#define MTR_BRAKE_R_Pin MTR_ADC_SOC_R_Pin

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
