/*
 * mcf8316c.c
 *
 *  Created on: Jul 15, 2025
 *      Author: kth59
 */

#include "mcf8316c.h"

HAL_StatusTypeDef Transmit_Set(I2C_HandleTypeDef *hi2c) {
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

HAL_StatusTypeDef Receive_Set(I2C_HandleTypeDef *hi2c, uint8_t size) {
	HAL_StatusTypeDef status;
	uint8_t tx_buffer[7];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_READ | CONTROL_CRC | CONTROL_MEM_SEC;
	switch (size) {
	case 2:
		tx_buffer[0] |= CONTROL_DATA_16BIT;
		break;
	case 8:
		tx_buffer[0] |= CONTROL_DATA_64BIT;
		break;
	default:
		tx_buffer[0] |= CONTROL_DATA_32BIT;
		break;
	}
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

HAL_StatusTypeDef Transmit_Reg(I2C_HandleTypeDef *hi2c, uint16_t reg_addr,
		uint8_t *pData, uint8_t size) {

	HAL_StatusTypeDef status;
	uint8_t tx_buffer[7];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_WRITE | CONTROL_CRC | CONTROL_MEM_SEC;
	switch (size) {
	case 2:
		tx_buffer[0] |= CONTROL_DATA_16BIT;
		break;
	case 8:
		tx_buffer[0] |= CONTROL_DATA_64BIT;
		break;
	default:
		tx_buffer[0] |= CONTROL_DATA_32BIT;
	}
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

HAL_StatusTypeDef Receive_Reg(I2C_HandleTypeDef *hi2c, uint16_t reg_addr,
		uint8_t *pData, uint16_t size) {

	HAL_StatusTypeDef status;
	uint8_t tx_buffer[3];
	uint16_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT << 1;

	tx_buffer[0] = CONTROL_READ | CONTROL_CRC | CONTROL_DATA_LEN
			| CONTROL_MEM_SEC;
	tx_buffer[1] = ((reg_addr >> 8) & 0xF) | CONTROL_MEM_PAGE;
	tx_buffer[2] = reg_addr & 0xFF;

	status = HAL_I2C_Master_Transmit(hi2c, slave_address_8bit, tx_buffer,
			sizeof(tx_buffer), 1);
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_I2C_Master_Receive(hi2c, slave_address_8bit, pData, size, 1);
	if (status != HAL_OK) {
		return status;
	}

	return HAL_OK;
}

//void I2C_TEST1() {
//	HAL_StatusTypeDef status;
//	uint8_t slave_address_8bit = MCF8316C_I2C_ADDRESS_7BIT;
//
//	uint8_t tx_buffer[4] = { 0 };
//	tx_buffer[0] = (I2C_TARGET_ADDR >> 0) & 0xFF;
//	tx_buffer[1] = (I2C_TARGET_ADDR >> 8) & 0xFF;
//	tx_buffer[2] = (I2C_TARGET_ADDR >> 16) & 0xFF;
//	tx_buffer[3] = (I2C_TARGET_ADDR >> 24) & 0xFF;
//
//	while (1) {
//		status = HAL_I2C_IsDeviceReady(MCF8316C_I2C_RIGHT_CHANNEL,
//				slave_address_8bit << 1, 1, HAL_MAX_DELAY);
//		if (status) {
//			HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
//			Custom_LCD_Printf(0, 1, "%d", slave_address_8bit);
//			slave_address_8bit += 1;
//			HAL_Delay(100);
//		} else {
//			status = Transmit_Reg_32BIT(MCF8316C_I2C_RIGHT_CHANNEL, 0xA6,
//					tx_buffer);
//			if (!status)
//				Custom_LCD_Printf(0, 4, "TRANSMIT");
//			else
//				Custom_LCD_Printf(0, 4, "TX FAIL");
//			status = Transmit_Set(MCF8316C_I2C_RIGHT_CHANNEL);
//			if (!status)
//				Custom_LCD_Printf(0, 5, "TRANSMIT");
//			else
//				Custom_LCD_Printf(0, 5, "TX FAIL");
//			HAL_Delay(300);
//			Custom_LCD_Printf(0, 3, "%02x%02x%02x%02x", tx_buffer[3],
//					tx_buffer[2], tx_buffer[1], tx_buffer[0]);
//			Custom_LCD_Printf(0, 0, "Success");
//			Custom_LCD_Printf(0, 2, "%d", slave_address_8bit);
//
//			uint8_t rx_buffer[4];
//			status = Receive_Reg(MCF8316C_I2C_RIGHT_CHANNEL, 0xA6, rx_buffer,
//					sizeof(rx_buffer));
//			Custom_LCD_Printf(0, 6, "%02x%02x%02x%02x", rx_buffer[3],
//					rx_buffer[2], rx_buffer[1], rx_buffer[0]);
//			while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
//				;
//			break;
//		}
//	}
//	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
//		;
//	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
//		;
//}

//void I2C_TEST2() {
//	HAL_StatusTypeDef status = HAL_ERROR;
//
//	static uint8_t tx_buffer[4];
//	tx_buffer[0] = (I2C_TARGET_ADDR >> 16) & 0xFF;
//	tx_buffer[1] = (I2C_TARGET_ADDR >> 16) & 0xFF;
//	tx_buffer[2] = (I2C_TARGET_ADDR >> 16) & 0xFF;
//	tx_buffer[3] = (I2C_TARGET_ADDR >> 16) & 0xFF;
//
//	Custom_LCD_Printf(0, 3, "%02x%02x%02x%02x", tx_buffer[3], tx_buffer[2],
//			tx_buffer[1], tx_buffer[0]);
//
//	uint8_t rx_buffer[4];
//	status = Receive_Set(MCF8316C_I2C_RIGHT_CHANNEL);
//	if (!status)
//		Custom_LCD_Printf(0, 0, "RX SET");
//	else
//		Custom_LCD_Printf(0, 0, "RX FAIL");
//
//	status = Receive_Reg(MCF8316C_I2C_RIGHT_CHANNEL, 0xA6, rx_buffer,
//			sizeof(rx_buffer));
//	if (!status)
//		Custom_LCD_Printf(0, 1, "RECEIVE");
//	else
//		Custom_LCD_Printf(0, 1, "RX FAIL");
//	Custom_LCD_Printf(0, 2, "%02x%02x%02x%02x", rx_buffer[3], rx_buffer[2],
//			rx_buffer[1], rx_buffer[0]);
//
//	status = HAL_I2C_IsDeviceReady(MCF8316C_I2C_RIGHT_CHANNEL,
//	MCF8316C_I2C_ADDRESS_7BIT << 1, 3, HAL_MAX_DELAY);
//	Custom_LCD_Printf(0, 4, "%02x%02x%02x%02x", tx_buffer[3], tx_buffer[2],
//			tx_buffer[1], tx_buffer[0]);
//	if (!status) {
//		status = Transmit_Reg_32BIT(MCF8316C_I2C_RIGHT_CHANNEL, 0xA6,
//				tx_buffer);
//	}
//	if (!status)
//		Custom_LCD_Printf(0, 7, "TRANSMIT");
//	else
//		Custom_LCD_Printf(0, 7, "TX FAIL");
//	status = Transmit_Set(MCF8316C_I2C_RIGHT_CHANNEL);
//	if (!status)
//		Custom_LCD_Printf(0, 8, "TRANSMIT");
//	else
//		Custom_LCD_Printf(0, 8, "TX FAIL");
//	HAL_Delay(300);
//	uint8_t i = 0;
//	status = false;
//	while (!status) {
//		status = HAL_I2C_IsDeviceReady(MCF8316C_I2C_RIGHT_CHANNEL, i << 1, 3,
//		HAL_MAX_DELAY);
//		if (status) {
//			HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
//			Custom_LCD_Printf(0, 9, "F:%d", i);
//			i++;
//			status = false;
//		} else {
//			Custom_LCD_Printf(0, 9, "S:%d", i);
//			break;
//		}
//	}
//
//	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
//		;
//	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
//		;
//}

void MCF8316C_Set_EEPROM() {
	uint8_t tx_buffer[4];
	HAL_StatusTypeDef status;
	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_LEFT_CHANNEL;
	uint8_t i = 0;
	uint32_t err_array = 0;

	for (uint8_t j = 0; j < 2; j++) {
		if (j)
			hi2c = MCF8316C_I2C_RIGHT_CHANNEL;

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (ISD_CONFIG_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, ISD_CONFIG_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 0;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (MOTOR_STARTUP1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, MOTOR_STARTUP1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 1;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (MOTOR_STARTUP2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, MOTOR_STARTUP2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 2;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, CLOSED_LOOP1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 3;
		}

		uint32_t CLOSED_LOOP2_DATA =
				(j) ? CLOSED_LOOP2_DATA_R : CLOSED_LOOP2_DATA_L;
		uint32_t CLOSED_LOOP3_DATA =
				(j) ? CLOSED_LOOP3_DATA_R : CLOSED_LOOP3_DATA_L;
		uint32_t CLOSED_LOOP4_DATA =
				(j) ? CLOSED_LOOP4_DATA_R : CLOSED_LOOP4_DATA_L;

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, CLOSED_LOOP2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 4;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP3_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, CLOSED_LOOP3_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 5;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP4_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, CLOSED_LOOP4_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 6;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, REF_PROFILES1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 7;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, REF_PROFILES2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 8;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE3_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, REF_PROFILES3_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 9;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE4_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, REF_PROFILES4_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 10;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE5_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, REF_PROFILES5_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 11;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE6_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, REF_PROFILES6_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 12;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (FAULT_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, FAULT_CONFIG1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 13;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (FAULT_CONFIG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, FAULT_CONFIG2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 14;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (PIN_CONFIG_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, PIN_CONFIG_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 15;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (DEVICE_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, DEVICE_CONFIG1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 16;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (DEVICE_CONFIG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, DEVICE_CONFIG2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 17;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (PERI_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, PERI_CONFIG1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 18;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (GD_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, GD_CONFIG1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 19;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (GD_CONFIG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, GD_CONFIG2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 20;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (INT_ALGO_1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, INT_ALGO_1_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 21;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (INT_ALGO_2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg(hi2c, INT_ALGO_2_ADDR, tx_buffer,
				sizeof(tx_buffer));
		if (status) {
			err_array |= 0x1 << 22;
		}

		status = Transmit_Set(hi2c);
		if (status) {
			Custom_LCD_Printf(0, 2 * j, "TX ERR");
			I2C_Error();
		} else
			Custom_LCD_Printf(0, 2 * j, "TX SUCCESS");

		Custom_LCD_Printf(0, 2 * j + 1, "%04x", err_array);

	}
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_Get_Fault() {
	uint8_t rx_buffer[4];
	HAL_StatusTypeDef status;

	Receive_Set(MCF8316C_I2C_LEFT_CHANNEL, sizeof(rx_buffer));
	Receive_Set(MCF8316C_I2C_RIGHT_CHANNEL, sizeof(rx_buffer));
	HAL_Delay(100);

	status = Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, DRIVER_FAULT_ADDR,
			rx_buffer, sizeof(rx_buffer));
	if (status)
		Custom_LCD_Printf(0, 0, "L ERROR");
	else
		Custom_LCD_Printf(0, 0, "L %02x%02x%02x%02x", rx_buffer[3],
				rx_buffer[2], rx_buffer[1], rx_buffer[0]);

	status = Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, CONTROLLER_FAULT_ADDR,
			rx_buffer, sizeof(rx_buffer));
	if (status)
		Custom_LCD_Printf(0, 1, "L ERROR");
	else
		Custom_LCD_Printf(0, 1, "L %02x%02x%02x%02x", rx_buffer[3],
				rx_buffer[2], rx_buffer[1], rx_buffer[0]);

	Custom_LCD_Printf(0, 2, "L nFault:%d",
			HAL_GPIO_ReadPin(Motor_L_nFAULT_GPIO_Port, Motor_L_nFAULT_Pin));

	status = Receive_Reg(MCF8316C_I2C_RIGHT_CHANNEL, DRIVER_FAULT_ADDR,
			rx_buffer, sizeof(rx_buffer));

	if (status)
		Custom_LCD_Printf(0, 3, "R ERROR");
	else
		Custom_LCD_Printf(0, 3, "R %02x%02x%02x%02x", rx_buffer[3],
				rx_buffer[2], rx_buffer[1], rx_buffer[0]);

	status = Receive_Reg(MCF8316C_I2C_RIGHT_CHANNEL, CONTROLLER_FAULT_ADDR,
			rx_buffer, sizeof(rx_buffer));
	if (status)
		Custom_LCD_Printf(0, 4, "L ERROR");
	else
		Custom_LCD_Printf(0, 4, "L %02x%02x%02x%02x", rx_buffer[3],
				rx_buffer[2], rx_buffer[1], rx_buffer[0]);

	Custom_LCD_Printf(0, 5, "R nFault:%d",
			HAL_GPIO_ReadPin(Motor_R_nFAULT_GPIO_Port, Motor_R_nFAULT_Pin));

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_Get_Voltage() {
	uint8_t rx_bufferL[4];
	uint8_t rx_bufferR[4];
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL,
		MCF8316C_VM_ADDR, rx_bufferL, sizeof(rx_bufferL));
		Receive_Reg(MCF8316C_I2C_RIGHT_CHANNEL,
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

void MCF8316C_MPET() {
	uint8_t tx_buffer32[4];
	uint8_t rx_buffer32[4];
	uint8_t rx_buffer16[2];
//	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_LEFT_CHANNEL; // 우선 한쪽 채널만 테스트
	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_RIGHT_CHANNEL; // 우선 한쪽 채널만 테스트

//	Transmit_Set(hi2c);
//	HAL_Delay(300);

	for (uint8_t i = 0; i < 4; i++) {
		tx_buffer32[i] = (ALGO_DEBUG2_DATA >> (8 * i)) & 0xFF;
	}
	Custom_LCD_Printf(0, 0, "MPET Start");
//
//	HAL_Delay(300);
//
//	Receive_Set(hi2c, sizeof(rx_buffer16));

//	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin,
//			GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
//			GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
			GPIO_PIN_RESET);

	if (Transmit_Reg(hi2c, ALGO_DEBUG2_ADDR, tx_buffer32, sizeof(tx_buffer32))
			!= HAL_OK) {
		Custom_LCD_Printf(0, 1, "Fail!");
		return;
	}

	HAL_Delay(100);

	Custom_LCD_Clear();

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		if (HAL_I2C_IsDeviceReady(hi2c, MCF8316C_I2C_ADDRESS_7BIT << 1, 1, 0x01)
				== HAL_OK) {
//			Receive_Reg(hi2c, ALGO_STATUS_MPET, rx_buffer32,
//					sizeof(rx_buffer32));
			Receive_Reg(hi2c, ALGORITHM_STATUS_ADDR, rx_buffer16,
					sizeof(rx_buffer16));
			HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
		} else
			HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_RESET);

//		Custom_LCD_Printf(0, 1, "%02x%02x%02x%02x", rx_buffer32[3], rx_buffer32[2], rx_buffer32[1], rx_buffer32[0]);
		Custom_LCD_Printf(0, 1, "%02x%02x", rx_buffer16[1], rx_buffer16[0]);
		HAL_Delay(500);
	}

	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;

//	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
//			GPIO_PIN_SET);

	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
			GPIO_PIN_SET);

	HAL_Delay(500);

	// 측정된 파라미터가 MTR_PARAMS 레지스터에 기록되었는지 확인
	HAL_StatusTypeDef status = Receive_Reg(hi2c, MTR_PARAMS, rx_buffer32,
			sizeof(rx_buffer32));
	if (status == HAL_OK) {
		Custom_LCD_Printf(0, 4, "Read Success!");
		Custom_LCD_Printf(0, 5, "R:%02x L:%02x", rx_buffer32[3],
				rx_buffer32[1]);
		Custom_LCD_Printf(0, 6, "BEMF: %02x", rx_buffer32[2]);
	} else {
		Custom_LCD_Printf(0, 4, "I2C Read Fail!");
	}
	status = Receive_Reg(hi2c, MCF8316C_CURRENT_PI_ADDR, rx_buffer32,
			sizeof(rx_buffer32));
	if (status == HAL_OK) {
		Custom_LCD_Printf(0, 7, "Read Success!");
		Custom_LCD_Printf(0, 8, "I%02x%02xP%02x%02x", rx_buffer32[3],
				rx_buffer32[2], rx_buffer32[1], rx_buffer32[0]);
	} else {
		Custom_LCD_Printf(0, 7, "I2C Read Fail!");
	}
	status = Receive_Reg(hi2c, MCF8316C_SPEED_PI_ADDR, rx_buffer32,
			sizeof(rx_buffer32));
	if (status == HAL_OK) {
		Custom_LCD_Printf(0, 7, "Read Success!");
		Custom_LCD_Printf(0, 9, "I%02x%02xP%02x%02x", rx_buffer32[3],
				rx_buffer32[2], rx_buffer32[1], rx_buffer32[0]);
	} else {
		Custom_LCD_Printf(0, 7, "I2C Read Fail!");
	}

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

/*
 * @brief 현재 섀도우 레지스터에 있는 파라미터 값들을 EEPROM에 영구 저장합니다.
 * @note  이 함수는 반드시 모터가 완전히 멈춘 상태에서만 호출해야 합니다.
 */

void MCF8316C_PID_CONTROL() {
	uint8_t rx_buffer[2];
	uint8_t rx_buffer2[4];
	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_LEFT_CHANNEL;
//	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_RIGHT_CHANNEL;

	Receive_Set(hi2c, sizeof(rx_buffer));

//	HAL_TIM_PWM_Start(MOTOR_L_TIM, MOTOR_L_CHANNEL);
//	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, 0);

//	HAL_TIM_PWM_Start(MOTOR_R_TIM, MOTOR_R_CHANNEL);
//	__HAL_TIM_SET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL, 0);

//	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
//			GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin,
//			GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(Motor_R_Driveoff_GPIO_Port, Motor_R_Driveoff_Pin,
//			GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Motor_R_Brake_GPIO_Port, Motor_R_Brake_Pin,
//			GPIO_PIN_RESET);

	Motor_Start();
	Encoder_Start();

	HAL_Delay(10);

	motor[ML].mps = 2.f;
	motor[MR].mps = 2.f;

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		Receive_Reg(hi2c, MCF8316C_DRIVER_STATE_ADDR, rx_buffer,
				sizeof(rx_buffer));
		Custom_LCD_Printf(0, 0, "%02x%02x", rx_buffer[1], rx_buffer[0]);
		Receive_Reg(hi2c, DRIVER_FAULT_ADDR, rx_buffer2, sizeof(rx_buffer2));
		Custom_LCD_Printf(0, 2, "%02x%02x%02x%02x", rx_buffer2[3],
				rx_buffer2[2], rx_buffer2[1], rx_buffer2[0]);
		Receive_Reg(hi2c, CONTROLLER_FAULT_ADDR, rx_buffer2,
				sizeof(rx_buffer2));
		Custom_LCD_Printf(0, 3, "%02x%02x%02x%02x", rx_buffer2[3],
				rx_buffer2[2], rx_buffer2[1], rx_buffer2[0]);

		if (HAL_GPIO_ReadPin(Motor_L_nFAULT_GPIO_Port, Motor_L_nFAULT_Pin))
			HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin,
				HAL_GPIO_ReadPin(Motor_L_nFAULT_GPIO_Port, Motor_L_nFAULT_Pin));
//		if (HAL_GPIO_ReadPin(Motor_R_nFAULT_GPIO_Port, Motor_R_nFAULT_Pin))
//			HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin,
//				HAL_GPIO_ReadPin(Motor_R_nFAULT_GPIO_Port, Motor_R_nFAULT_Pin));

		Custom_LCD_Printf(0, 4, "terr:%5d", encoder_tick_err_l);
		Custom_LCD_Printf(0, 5, "errP:%.6f", err_p_l);
		Custom_LCD_Printf(0, 6, "errI:%.6f", err_i_l);
		Custom_LCD_Printf(0, 7, "%.6f", iq_ref_l);
		Custom_LCD_Printf(0, 8, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL));
	}
	Motor_Stop();
	Encoder_Stop();
	HAL_Delay(10);
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin));
}

void MCF8316C_SPEED() {
	Motor_Start();

	HAL_Delay(10);

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		motor[ML].mps = 1.f;
		motor[MR].mps = 1.f;
		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin, GPIO_PIN_RESET);
		Custom_LCD_Printf(0, 0, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL));
		Custom_LCD_Printf(0, 1, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL));

		HAL_Delay(2000);

		motor[ML].mps = 0.f;
		motor[MR].mps = 0.f;
		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin, GPIO_PIN_RESET);
		Custom_LCD_Printf(0, 0, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL));
		Custom_LCD_Printf(0, 1, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL));

		HAL_Delay(1000);

		motor[ML].mps = -1.f;
		motor[MR].mps = -1.f;
		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin, GPIO_PIN_SET);
		Custom_LCD_Printf(0, 0, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL));
		Custom_LCD_Printf(0, 1, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL));

		HAL_Delay(2000);

		motor[ML].mps = 0.f;
		motor[MR].mps = 0.f;
		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin, GPIO_PIN_RESET);
		Custom_LCD_Printf(0, 0, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL));
		Custom_LCD_Printf(0, 1, "%5d", __HAL_TIM_GET_COMPARE(MOTOR_R_TIM, MOTOR_R_CHANNEL));

		HAL_Delay(1000);

	}
	Motor_Stop();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}
