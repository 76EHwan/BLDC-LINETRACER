/*
 * lsm6ds3tr_c.c
 *
 *  Created on: Jul 15, 2025
 *      Author: kth59
 */

#include "lsm6ds3tr_c.h"

uint8_t LSM6DS3TR_C_ReadReg(uint8_t reg_addr) {
	uint8_t tx_byte = reg_addr | 0x80; // Read operation (MSB = 1)
	uint8_t rx_byte;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // CS Low
	HAL_SPI_Transmit(&hspi2, &tx_byte, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, &rx_byte, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);  // CS High

	return rx_byte;
}

void LSM6DS3TR_C_Init(void) {
	// 1. WHO_AM_I 레지스터 읽어서 장치 확인
	while (1) {
		uint8_t who_am_i = LSM6DS3TR_C_ReadReg(LSM6DS3TR_C_WHO_AM_I_REG);
		Custom_LCD_Printf(0, 0, "%x", who_am_i);
	}
}
