/*
 * mcf8316c.c
 *
 *  Created on: Jul 15, 2025
 *      Author: kth59
 */

#include "mcf8316c.h"

HAL_StatusTypeDef Transmit_Set(I2C_HandleTypeDef *hi2c, uint8_t size) {
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
		break;
	}
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

HAL_StatusTypeDef Transmit_Reg_32BIT(I2C_HandleTypeDef *hi2c, uint16_t reg_addr,
		uint8_t *pData) {

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

HAL_StatusTypeDef Receive_Reg(I2C_HandleTypeDef *hi2c, uint16_t reg_addr,
		uint8_t *pData, uint16_t Size) {

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

void I2C_TEST3() {
	uint8_t tx_buffer[4];
	uint8_t rx_buffer[4];
	HAL_StatusTypeDef status;

	I2C_HandleTypeDef *hi2c = MCF8316C_I2C_LEFT_CHANNEL;
	uint16_t reg_addr = ALGO_DEBUG2_ADDR;
	uint32_t reg_data = ALGO_DEBUG2_DATA;

	status = Receive_Set(hi2c, sizeof(rx_buffer));
	if (status) {
		Custom_LCD_Printf(0, 2, "RX ERR");
		I2C_Error();
	}
	status = Receive_Reg(hi2c, reg_addr, rx_buffer, sizeof(rx_buffer));
	if (status) {
		Custom_LCD_Printf(0, 3, "RX ERR");
		I2C_Error();
	}
	Custom_LCD_Printf(0, 4, "%02x%02x%02x%02x", rx_buffer[3], rx_buffer[2],
			rx_buffer[1], rx_buffer[0]);

	for (uint8_t i = 0; i < 4; i++) {
		tx_buffer[i] = (reg_data >> (8 * i)) & 0xFF;
	}
	status = Transmit_Reg_32BIT(hi2c, reg_addr, tx_buffer);
	if (status) {
		Custom_LCD_Printf(0, 0, "TX ERR");
		I2C_Error();
	}

	status = Transmit_Set(hi2c, sizeof(tx_buffer));
	if (status) {
		Custom_LCD_Printf(0, 1, "TX ERR");
		I2C_Error();
	}
	HAL_Delay(100);

	status = Receive_Set(hi2c, sizeof(rx_buffer));
	if (status) {
		Custom_LCD_Printf(0, 2, "RX ERR");
		I2C_Error();
	}
	status = Receive_Reg(hi2c, reg_addr, rx_buffer, sizeof(rx_buffer));
	if (status) {
		Custom_LCD_Printf(0, 3, "RX ERR");
		I2C_Error();
	}
	HAL_Delay(300);

	Custom_LCD_Printf(0, 5, "%02x%02x%02x%02x", tx_buffer[3], tx_buffer[2],
			tx_buffer[1], tx_buffer[0]);
	Custom_LCD_Printf(0, 6, "%02x%02x%02x%02x", rx_buffer[3], rx_buffer[2],
			rx_buffer[1], rx_buffer[0]);

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

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
		status = Transmit_Reg_32BIT(hi2c, ISD_CONFIG_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 0;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (MOTOR_STARTUP1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, MOTOR_STARTUP1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 1;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (MOTOR_STARTUP2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, MOTOR_STARTUP2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 2;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, CLOSED_LOOP1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 3;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, CLOSED_LOOP2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 4;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP3_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, CLOSED_LOOP3_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 5;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (CLOSED_LOOP4_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, CLOSED_LOOP4_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 6;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, REF_PROFILES1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 7;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, REF_PROFILES2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 8;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE3_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, REF_PROFILES3_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 9;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE4_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, REF_PROFILES4_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 10;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE5_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, REF_PROFILES5_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 11;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (REF_PROFILE6_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, REF_PROFILES6_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 12;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (FAULT_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, FAULT_CONFIG1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 13;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (FAULT_CONFIG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, FAULT_CONFIG2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 14;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (PIN_CONFIG_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, PIN_CONFIG_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 15;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (DEVICE_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, DEVICE_CONFIG1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 16;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (DEVICE_CONFIG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, DEVICE_CONFIG2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 17;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (PERI_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, PERI_CONFIG1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 18;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (GD_CONFIG1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, GD_CONFIG1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 19;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (GD_CONFIG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, GD_CONFIG2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 20;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (INT_ALGO_1_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, INT_ALGO_1_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 21;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (INT_ALGO_2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, INT_ALGO_2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 22;
		}

		for (i = 0; i < 4; i++) {
			tx_buffer[i] = (ALGO_DEBUG2_DATA >> (8 * i)) & 0xFF;
		}
		status = Transmit_Reg_32BIT(hi2c, ALGO_DEBUG2_ADDR, tx_buffer);
		if (status) {
			err_array |= 0x1 << 22;
		}

		status = Transmit_Set(hi2c, sizeof(tx_buffer));
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
	uint8_t tx_buffer[4];
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
	tx_buffer[0] = MCF8316C_FAULT_CLEAR & 0xFF;
	tx_buffer[1] = (MCF8316C_FAULT_CLEAR >> 8) & 0xFF;
	tx_buffer[2] = (MCF8316C_FAULT_CLEAR >> 16) & 0xFF;
	tx_buffer[3] = (MCF8316C_FAULT_CLEAR >> 24) & 0xFF;
	status = Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL,
	MCF8316C_WRITE_READ_ADDR, tx_buffer, sizeof(tx_buffer));
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
	uint8_t rx_buffer_16[2];
	uint8_t rx_buffer_32[4];

	HAL_GPIO_WritePin(Motor_L_Brake_GPIO_Port, Motor_L_Brake_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_L_Driveoff_GPIO_Port, Motor_L_Driveoff_Pin,
			GPIO_PIN_RESET);

	HAL_TIM_PWM_Start(MOTOR_L_TIM, MOTOR_L_CHANNEL);
	uint32_t duty = __HAL_TIM_GET_AUTORELOAD(MOTOR_L_TIM) / 4;
	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, duty);
	Custom_LCD_Printf(0, 0, "MTR PARAMS");

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
//		status = Receive_Set(MCF8316C_I2C_LEFT_CHANNEL, sizeof(rx_buffer_16));
//		if(status) break;
//		status = Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, ALGORITHM_STATUS_ADDR,
//				rx_buffer_16, sizeof(rx_buffer_16));
//		if(status) break;
//
//		uint16_t algo_status = (rx_buffer_16[1] << 8) | rx_buffer_16[0];
//		Custom_LCD_Printf(0, 9, "STATUS:%02x", algo_status);

//		if (algo_status == 0x14) {
//			Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, MCF8316C_CURR_PI_ADDR,
//					rx_buffer_16, sizeof(rx_buffer_16));
//			Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, MCF8316C_SPEED_PI_ADDR,
//					rx_buffer_16, sizeof(rx_buffer_16));
//			Custom_LCD_Printf(0, 5, "KI:%02x", rx_buffer_16[1]);
//			Custom_LCD_Printf(0, 6, "KP:%02x", rx_buffer_16[0]);
//
//		}
//
//		if (algo_status == 0x17) {
//			Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, MTR_PARAMS, rx_buffer_32,
//					sizeof(rx_buffer_32));
//			Custom_LCD_Printf(0, 2, "BEMF:%02x", rx_buffer_32[2]);
//		}
//		if (algo_status == 0x17 || algo_status == 0x18)
//			break;
	}
	Motor_Stop();

	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	Receive_Set(MCF8316C_I2C_LEFT_CHANNEL, sizeof(rx_buffer_16));
	HAL_Delay(100);
	Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, MTR_PARAMS, rx_buffer_16,
			sizeof(rx_buffer_16));
	Custom_LCD_Printf(0, 3, "KI:%02x", rx_buffer_16[1]);
	Custom_LCD_Printf(0, 4, "KP:%02x", rx_buffer_16[0]);

	Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, MTR_PARAMS, rx_buffer_16,
			sizeof(rx_buffer_16));

	uint16_t algo_status = (rx_buffer_16[1] << 8) | rx_buffer_16[0];
	Custom_LCD_Printf(0, 9, "STATUS:%02x", algo_status);

	Receive_Set(MCF8316C_I2C_LEFT_CHANNEL, sizeof(rx_buffer_32));
	HAL_Delay(100);
	Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, MTR_PARAMS, rx_buffer_32,
			sizeof(rx_buffer_32));
	Custom_LCD_Printf(0, 1, "R:%02x L:%02x", rx_buffer_32[3], rx_buffer_32[1]);
	Custom_LCD_Printf(0, 2, "BEMF:%02x", rx_buffer_32[2]);

	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}

void MCF8316C_PI_CTRL() {
	Motor_Start();
	Encoder_Start();
	uint8_t rx_buffer_16[2];
	uint32_t duty = __HAL_TIM_GET_AUTORELOAD(MOTOR_L_TIM) / 100 * 15;
	__HAL_TIM_SET_COMPARE(MOTOR_L_TIM, MOTOR_L_CHANNEL, duty);
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		Receive_Set(MCF8316C_I2C_LEFT_CHANNEL, sizeof(rx_buffer_16));
		Receive_Reg(MCF8316C_I2C_LEFT_CHANNEL, ALGORITHM_STATUS_ADDR,
				rx_buffer_16, sizeof(rx_buffer_16));
		uint16_t algo_status = (rx_buffer_16[1] << 8) | rx_buffer_16[0];
		Custom_LCD_Printf(0, 9, "STATUS:%02x", algo_status);
		HAL_GPIO_WritePin(MARK_L_GPIO_Port, MARK_L_Pin,
				!HAL_GPIO_ReadPin(Motor_L_nFAULT_GPIO_Port,
				Motor_L_nFAULT_Pin));
		HAL_GPIO_WritePin(MARK_R_GPIO_Port, MARK_R_Pin,
				!HAL_GPIO_ReadPin(Motor_R_nFAULT_GPIO_Port,
				Motor_R_nFAULT_Pin));
		Custom_LCD_Printf(0, 0, "L: %5d", *(ENCODER_L_TIM.Instance->CNT));
		Custom_LCD_Printf(0, 1, "R: %5d", *(ENCODER_R_TIM.Instance->CNT));
	}
	Motor_Stop();
	Encoder_Stop();

	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		;
}
