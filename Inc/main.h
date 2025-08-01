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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MARK_R_Pin GPIO_PIN_2
#define MARK_R_GPIO_Port GPIOE
#define E3_Pin GPIO_PIN_3
#define E3_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define MARK_L_Pin GPIO_PIN_8
#define MARK_L_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LCD_WR_RS_Pin GPIO_PIN_13
#define LCD_WR_RS_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Motor_L_nFAULT_Pin GPIO_PIN_10
#define Motor_L_nFAULT_GPIO_Port GPIOD
#define Motor_L_Driveoff_Pin GPIO_PIN_13
#define Motor_L_Driveoff_GPIO_Port GPIOD
#define Motor_L_Dir_Pin GPIO_PIN_14
#define Motor_L_Dir_GPIO_Port GPIOD
#define Motor_L_Brake_Pin GPIO_PIN_15
#define Motor_L_Brake_GPIO_Port GPIOD
#define Motor_L_PWM_Pin GPIO_PIN_6
#define Motor_L_PWM_GPIO_Port GPIOC
#define Motor_R_PWM_Pin GPIO_PIN_7
#define Motor_R_PWM_GPIO_Port GPIOC
#define Motor_R_Driveoff_Pin GPIO_PIN_8
#define Motor_R_Driveoff_GPIO_Port GPIOA
#define Motor_R_Dir_Pin GPIO_PIN_9
#define Motor_R_Dir_GPIO_Port GPIOA
#define Motor_R_Brake_Pin GPIO_PIN_10
#define Motor_R_Brake_GPIO_Port GPIOA
#define Motor_R_nFAULT_Pin GPIO_PIN_15
#define Motor_R_nFAULT_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOD
#define L_SCL_Pin GPIO_PIN_6
#define L_SCL_GPIO_Port GPIOB
#define L_SDA_Pin GPIO_PIN_7
#define L_SDA_GPIO_Port GPIOB
#define R_SCL_Pin GPIO_PIN_8
#define R_SCL_GPIO_Port GPIOB
#define R_SDA_Pin GPIO_PIN_9
#define R_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
