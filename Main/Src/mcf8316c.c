/*
 * mcf8316c.c
 *
 *  Created on: Jul 15, 2025
 *      Author: kth59
 */

/*
 * MCF8316C-Q1
 * 	Default Setting
 * 		SPEED_MODE = 01b (PWM mode)
 * 		SLEEP_ENTRY_TIME = 10b 		(20 ms)
 * 		REF_PROFILE_CONFIG = 00b 	(1%)
 * 		MOTOR_RESISTER = 0xBC		(1.92 Ω)
 * 		MOTOR_INDUCTANCE = 0x3D		(0.129 mH)
 * 		MOTOR_BEMF = 0x9B			(70 mV/Hz)
 *
 *
 *
 */

#include "mcf8316c.h"

HAL_StatusTypeDef MCF8316C_WriteSetting(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status;
	uint8_t tx_buffer[7];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_WRITE | CONTROL_CRC | CONTROL_DATA_LEN
			| CONTROL_MEM_SEC;
	tx_buffer[1] = ((MCF8316C_WRITE_READ_ADDR >> 8) & 0xF) | CONTROL_MEM_PAGE;
	tx_buffer[2] = MCF8316C_WRITE_READ_ADDR & 0xFF;
	tx_buffer[3] = MCF8316C_REG_WRITE_BIT & 0xFF;
	tx_buffer[4] = (MCF8316C_REG_WRITE_BIT >> 8) & 0xFF;
	tx_buffer[5] = (MCF8316C_REG_WRITE_BIT >> 16) & 0xFF;
	tx_buffer[6] = (MCF8316C_REG_WRITE_BIT >> 24) & 0xFF;

	status = HAL_I2C_Master_Transmit(hi2c, slave_address_8bit, tx_buffer,
			sizeof(tx_buffer),
			HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MCF8316C_ReadSetting(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status;
	uint8_t tx_buffer[7];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_READ | CONTROL_CRC | CONTROL_DATA_LEN
			| CONTROL_MEM_SEC;
	tx_buffer[1] = ((MCF8316C_WRITE_READ_ADDR >> 8) & 0xF) | CONTROL_MEM_PAGE;
	tx_buffer[2] = MCF8316C_WRITE_READ_ADDR & 0xFF;
	tx_buffer[3] = MCF8316C_REG_READ_BIT & 0xFF;
	tx_buffer[4] = (MCF8316C_REG_READ_BIT >> 8) & 0xFF;
	tx_buffer[5] = (MCF8316C_REG_READ_BIT >> 16) & 0xFF;
	tx_buffer[6] = (MCF8316C_REG_READ_BIT >> 24) & 0xFF;

	status = HAL_I2C_Master_Transmit(hi2c, slave_address_8bit, tx_buffer,
			sizeof(tx_buffer),
			HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MCF8316C_WriteRegister(I2C_HandleTypeDef *hi2c,
		uint16_t reg_addr, uint8_t *pData) {

	HAL_StatusTypeDef status;
	uint8_t tx_buffer[7];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_WRITE | CONTROL_CRC | CONTROL_DATA_LEN
			| CONTROL_MEM_SEC;
	tx_buffer[1] = ((reg_addr >> 8) & 0xF) | CONTROL_MEM_PAGE;
	tx_buffer[2] = reg_addr & 0xFF;
	tx_buffer[3] = *(pData + 0);
	tx_buffer[4] = *(pData + 1);
	tx_buffer[5] = *(pData + 2);
	tx_buffer[6] = *(pData + 3);
	status = HAL_I2C_Master_Transmit(hi2c, slave_address_8bit, tx_buffer,
			sizeof(tx_buffer),
			HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MCF8316C_ReadRegister(I2C_HandleTypeDef *hi2c,
		uint16_t reg_addr, uint8_t *pData, uint16_t Size) {

	HAL_StatusTypeDef status;
	uint8_t tx_buffer[3];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_READ | CONTROL_CRC | CONTROL_DATA_LEN
			| CONTROL_MEM_SEC;
	tx_buffer[1] = ((reg_addr >> 8) & 0xF) | CONTROL_MEM_PAGE;
	tx_buffer[2] = reg_addr & 0xFF;

	status = HAL_I2C_Master_Transmit(hi2c, slave_address_8bit, tx_buffer,
			sizeof(tx_buffer),
			HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_I2C_Master_Receive(hi2c, slave_address_8bit, pData, Size,
	HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}

	return HAL_OK;
}

void MCF8316C_Get_Voltage() {
	uint8_t rx_bufferL[4];
	uint8_t rx_bufferR[4];
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		MCF8316C_ReadRegister(MCF8316C_I2C_LEFT_CHANNEL,
		MCF8316C_VM_ADDR, rx_bufferL, sizeof(rx_bufferL));
		MCF8316C_ReadRegister(MCF8316C_I2C_RIGHT_CHANNEL,
		MCF8316C_VM_ADDR, rx_bufferR, sizeof(rx_bufferR));
		uint32_t voltL = 0;
		uint32_t voltR = 0;
		for (uint8_t i = 0; i < sizeof(rx_bufferL); i++) {
			voltL |= (*(rx_bufferL + i) << (8 * i));
			voltR |= (*(rx_bufferR + i) << (8 * i));
		}
		double vmL = (double) (voltL * VM_COEFF1 / VM_COEFF2);
		double vmR = (double) (voltR * VM_COEFF1 / VM_COEFF2);
		Custom_LCD_Printf(0, 0, "VL:%2.4f", vmL);
		Custom_LCD_Printf(0, 1, "VR:%2.4f", vmR);
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

HAL_StatusTypeDef MCF8316C_Set_Register(I2C_HandleTypeDef *hi2c,
		uint16_t reg_addr, uint32_t pData) {
	HAL_StatusTypeDef status;
	uint8_t tx_buffer[4];

	for (uint8_t i = 0; i < 4; i++) {
		tx_buffer[i] = pData & 0xFF;
		pData = pData >> 8;
	}
	status = MCF8316C_WriteRegister(hi2c, reg_addr, tx_buffer);
	if (status)
		return status;
	return HAL_OK;
}

void MCF8316C_Test_I2C_Addr() {
	HAL_StatusTypeDef status;
	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_LEFT_CHANNEL;
	uint8_t rx_data[4];
	status = MCF8316C_ReadSetting(hi2c);
	if (status)
		Custom_LCD_Printf(0, 0, "RS FAIL");
	else
		Custom_LCD_Printf(0, 0, "RS SUCCESS");
	HAL_Delay(100);
	status = MCF8316C_ReadRegister(hi2c, GD_CONFIG2_ADDR, rx_data,
			sizeof(rx_data));
	if (status)
		Custom_LCD_Printf(0, 1, "RR FAIL");
	else
		Custom_LCD_Printf(0, 1, "RR SUCCESS");
	Custom_LCD_Printf(0, 2, "%02x%02x%02x%02x", rx_data[3], rx_data[2],
			rx_data[1], rx_data[0]);
	HAL_Delay(300);
	status = MCF8316C_Set_Register(hi2c, GD_CONFIG2_ADDR, GD_CONFIG2_DATA);
	if (status)
		Custom_LCD_Printf(0, 3, "SR FAIL");
	else
		Custom_LCD_Printf(0, 3, "SR SUCCESS");
	Custom_LCD_Printf(0, 4, "%08x", GD_CONFIG2_DATA);
	HAL_Delay(300);
	status = MCF8316C_WriteSetting(hi2c);
	if (status)
		Custom_LCD_Printf(0, 5, "WS FAIL");
	else
		Custom_LCD_Printf(0, 5, "WS SUCCESS");

	HAL_Delay(300);
	status = MCF8316C_ReadSetting(hi2c);
	if (status)
		Custom_LCD_Printf(0, 6, "RS FAIL");
	else
		Custom_LCD_Printf(0, 6, "RS SUCCESS");

	HAL_Delay(100);
	status = MCF8316C_ReadRegister(hi2c, GD_CONFIG2_ADDR, rx_data,
			sizeof(rx_data));
	if (status)
		Custom_LCD_Printf(0, 7, "SR FAIL");
	else
		Custom_LCD_Printf(0, 7, "SR SUCCESS");

	Custom_LCD_Printf(0, 8, "%02x%02x%02x%02x", rx_data[3], rx_data[2],
			rx_data[1], rx_data[0]);
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_Set_I2C_Addr() {
	HAL_StatusTypeDef status;
	uint32_t fail_addr = 0;
	uint32_t fail_addrL = 0;
	uint32_t fail_addrR = 0;

	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_LEFT_CHANNEL;
	for (uint8_t i = 0; i < 2; i++) {
		if (i % 2) {
			hi2c = MCF8316C_I2C_RIGHT_CHANNEL;
		} else
			hi2c = MCF8316C_I2C_LEFT_CHANNEL;
		status = MCF8316C_Set_Register(hi2c, ISD_CONFIG_ADDR, ISD_CONFIG_DATA);
		if (status)
			fail_addr |= 0x1 << 0;
		status = MCF8316C_Set_Register(hi2c, REV_DRIVE_CONFIG_ADDR,
		REV_DRIVE_CONFIG_DATA);
		if (status)
			fail_addr |= 0x1 << 1;
		status = MCF8316C_Set_Register(hi2c, MOTOR_STARTUP1_ADDR,
		MOTOR_STARTUP1_DATA);
		if (status)
			fail_addr |= 0x1 << 2;
		status = MCF8316C_Set_Register(hi2c, MOTOR_STARTUP2_ADDR,
		MOTOR_STARTUP2_DATA);
		if (status)
			fail_addr |= 0x1 << 3;
		status = MCF8316C_Set_Register(hi2c, CLOSED_LOOP1_ADDR,
		CLOSED_LOOP1_DATA);
		if (status)
			fail_addr |= 0x1 << 4;
		status = MCF8316C_Set_Register(hi2c, CLOSED_LOOP2_ADDR,
		CLOSED_LOOP2_DATA);
		if (status)
			fail_addr |= 0x1 << 5;
		status = MCF8316C_Set_Register(hi2c, CLOSED_LOOP3_ADDR,
		CLOSED_LOOP3_DATA);
		if (status)
			fail_addr |= 0x1 << 6;
		status = MCF8316C_Set_Register(hi2c, CLOSED_LOOP4_ADDR,
		CLOSED_LOOP4_DATA);
		if (status)
			fail_addr |= 0x1 << 7;
		status = MCF8316C_Set_Register(hi2c, REF_PROFILES1_ADDR,
		REF_PROFILE1_DATA);
		if (status)
			fail_addr |= 0x1 << 8;
		status = MCF8316C_Set_Register(hi2c, REF_PROFILES2_ADDR,
		REF_PROFILE2_DATA);
		if (status)
			fail_addr |= 0x1 << 9;
		status = MCF8316C_Set_Register(hi2c, REF_PROFILES3_ADDR,
		REF_PROFILE3_DATA);
		if (status)
			fail_addr |= 0x1 << 10;
		status = MCF8316C_Set_Register(hi2c, REF_PROFILES4_ADDR,
		REF_PROFILE4_DATA);
		if (status)
			fail_addr |= 0x1 << 11;
		status = MCF8316C_Set_Register(hi2c, REF_PROFILES5_ADDR,
		REF_PROFILE5_DATA);
		if (status)
			fail_addr |= 0x1 << 12;
		status = MCF8316C_Set_Register(hi2c, REF_PROFILES6_ADDR,
		REF_PROFILE6_DATA);
		if (status)
			fail_addr |= 0x1 << 13;
		status = MCF8316C_Set_Register(hi2c, FAULT_CONFIG1_ADDR,
		FAULT_CONFIG1_DATA);
		if (status)
			fail_addr |= 0x1 << 14;
		status = MCF8316C_Set_Register(hi2c, FAULT_CONFIG2_ADDR,
		FAULT_CONFIG2_DATA);
		if (status)
			fail_addr |= 0x1 << 15;
		status = MCF8316C_Set_Register(hi2c, PIN_CONFIG_ADDR, PIN_CONFIG_DATA);
		if (status)
			fail_addr |= 0x1 << 16;
		status = MCF8316C_Set_Register(hi2c, DEVICE_CONFIG1_ADDR,
		DEVICE_CONFIG1_DATA);
		if (status)
			fail_addr |= 0x1 << 17;
		status = MCF8316C_Set_Register(hi2c, DEVICE_CONFIG2_ADDR,
		DEVICE_CONFIG2_DATA);
		if (status)
			fail_addr |= 0x1 << 18;
		status = MCF8316C_Set_Register(hi2c, PERI_CONFIG1_ADDR,
		PERI_CONFIG1_DATA);
		if (status)
			fail_addr |= 0x1 << 19;
		status = MCF8316C_Set_Register(hi2c, GD_CONFIG1_ADDR, GD_CONFIG1_DATA);
		if (status)
			fail_addr |= 0x1 << 20;
		status = MCF8316C_Set_Register(hi2c, GD_CONFIG2_ADDR, GD_CONFIG2_DATA);
		if (status)
			fail_addr |= 0x1 << 21;
		status = MCF8316C_Set_Register(hi2c, INT_ALGO_1_ADDR, INT_ALGO_1_DATA);
		if (status)
			fail_addr |= 0x1 << 22;
		status = MCF8316C_Set_Register(hi2c, INT_ALGO_2_ADDR, INT_ALGO_2_DATA);
		if (status)
			fail_addr |= 0x1 << 23;
		status = MCF8316C_WriteSetting(MCF8316C_I2C_RIGHT_CHANNEL);
		HAL_Delay(300);
		if (status)
			HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
		if (i) {
			fail_addrR = fail_addr;
			if (status) {
				Custom_LCD_Printf(0, 2, "R FAIL");
				Custom_LCD_Printf(0, 3, "%03x", fail_addrR);
			} else
				Custom_LCD_Printf(0, 2, "R SUCCESS");
		} else {
			fail_addrL = fail_addr;
			if (status) {
				Custom_LCD_Printf(0, 0, "L FAIL");
				Custom_LCD_Printf(0, 1, "%03x", fail_addrL);
			} else
				Custom_LCD_Printf(0, 0, "L SUCCESS");
		}
		fail_addr = 0;
	}

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_Check_SAME() {
//for(uint8_t i = 0; i < )
	uint8_t rx_buffer1[4];
	uint8_t rx_buffer2[4];
	MCF8316C_ReadSetting(MCF8316C_I2C_LEFT_CHANNEL);
	HAL_Delay(100);
	MCF8316C_ReadRegister(MCF8316C_I2C_LEFT_CHANNEL, 0xAE, rx_buffer1,
			sizeof(rx_buffer1));
	MCF8316C_ReadSetting(MCF8316C_I2C_RIGHT_CHANNEL);
	HAL_Delay(100);
	MCF8316C_ReadRegister(MCF8316C_I2C_RIGHT_CHANNEL, 0xAE, rx_buffer2,
			sizeof(rx_buffer2));
	Custom_LCD_Printf(0, 0, "%02x%02x%02x%02x  ", rx_buffer1[3], rx_buffer1[2],
			rx_buffer1[1], rx_buffer1[0]);
	Custom_LCD_Printf(0, 1, "%02x%02x%02x%02x  ", rx_buffer2[3], rx_buffer2[2],
			rx_buffer2[1], rx_buffer2[0]);
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_Test_Fault() {
	uint8_t rx_buffer1[4];
	uint8_t rx_buffer2[4];

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		MCF8316C_ReadSetting(MCF8316C_I2C_LEFT_CHANNEL);
		MCF8316C_ReadSetting(MCF8316C_I2C_RIGHT_CHANNEL);
		HAL_Delay(100);
		MCF8316C_ReadRegister(MCF8316C_I2C_LEFT_CHANNEL, 0xE0, rx_buffer1,
				sizeof(rx_buffer1));
		MCF8316C_ReadRegister(MCF8316C_I2C_RIGHT_CHANNEL, 0xE0, rx_buffer2,
				sizeof(rx_buffer2));
		Custom_LCD_Printf(0, 0, "L:%02x%02x%02x%02x", rx_buffer1[3],
				rx_buffer1[2], rx_buffer1[1], rx_buffer1[0]);
		Custom_LCD_Printf(0, 1, "R:%02x%02x%02x%02x", rx_buffer2[3],
				rx_buffer2[2], rx_buffer2[1], rx_buffer2[0]);
		Custom_LCD_Printf(0, 2, "L: %d R: %d",
				HAL_GPIO_ReadPin( Motor_L_nFAULT_GPIO_Port, Motor_L_nFAULT_Pin),
				HAL_GPIO_ReadPin( Motor_R_nFAULT_GPIO_Port,
				Motor_R_nFAULT_Pin));
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_Init() {
	// MOTOR RESISTOR 변화 확인

	// MOTOR INDUCTANCE 변화 확인

	// MOTOR BEMF 변화 확인

	//
}
