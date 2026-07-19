/*
 * mt6701.c
 *
 *  Created on: Jan 24, 2026
 *  Author: kth59
 */

#include "mt6701.h"
#include "lptim.h"
#include "motor.h"

#define ENC_SPI &hspi2

/* USER CODE BEGIN 0 */
MT6701_Data_t encDataL =
		{ .cs_port = ENC_CS_L_GPIO_Port, .cs_pin = ENC_CS_L_Pin };
MT6701_Data_t encDataR =
		{ .cs_port = ENC_CS_R_GPIO_Port, .cs_pin = ENC_CS_R_Pin };

void MT6701_ReconfigPinForCS(MT6701_Data_t *encData, LPTIM_HandleTypeDef *hlptim) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	HAL_LPTIM_Encoder_Stop(hlptim); // 인터럽트 모드였다면 HAL_LPTIM_Encoder_Stop_IT 사용

	HAL_LPTIM_DeInit(hlptim);

	// 2. 기존 LPTIM으로 사용되던 GPIO 핀(PE1 또는 PD11) 초기화
	// Alternate Function으로 묶여있던 핀을 해제합니다.
	HAL_GPIO_DeInit(encData->cs_port, encData->cs_pin);

	// 3. 해당 핀을 GPIO 출력(Output) 모드로 재설정
	GPIO_InitStruct.Pin = encData->cs_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull 모드
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // 풀업/풀다운 없음 (필요시 변경)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 통신용이므로 속도 높음
	HAL_GPIO_Init(encData->cs_port, &GPIO_InitStruct);

	// 4. CS 핀 기본 상태를 HIGH로 설정 (SPI 비활성 상태)
	HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef MT6701_Init(MT6701_Data_t *encData, uint8_t *rxBuffer) {

	MT6701_ReconfigPinForCS(&encDataL, &hlptim2);
	MT6701_ReconfigPinForCS(&encDataR, &hlptim1);

	HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(ENC_SPI, rxBuffer, 3, 10);
	HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_SET);

	// 3. 24ë¹í¸ ë°ì´í°ë¥¼ ë¨¼ì  íëë¡ í©ì¹¨ (íì± ì¤ë¥ ë°©ì§)
	uint32_t rawValue = ((uint32_t) rxBuffer[0] << 16)
			| ((uint32_t) rxBuffer[1] << 8) | rxBuffer[2];

	// 4. í©ì³ì§ ë°ì´í°ìì 4ë¹í¸ Status ì¶ì¶ (ë¹í¸ 9~6 ìë¦¬)
	uint8_t fault_state = (rawValue >> 6) & 0x0F;
	encData->status = fault_state;

	if ((fault_state & 0x08) || ((fault_state & 0x03) == 0x03)) {
		return HAL_ERROR;
	}

	encData->last_raw_angle = (rawValue >> 10) & 0x3FFF;
	encData->motor_elec_angle = 0.0f;
	return HAL_OK;
}

void MT6701_ReadSSI(MT6701_Data_t *encData) {
	uint8_t rxBuffer[3]; // ì´ê¸°í ë¶íì (ìë â)

	HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef spi_status = HAL_SPI_Receive(ENC_SPI, rxBuffer, 3, 2);
	HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_SET);

	if (spi_status == HAL_OK) {
		uint32_t rawValue = ((uint32_t) rxBuffer[0] << 16)
				| ((uint32_t) rxBuffer[1] << 8) | rxBuffer[2];
		uint16_t new_raw = (rawValue >> 10) & 0x3FFF; // 0 ~ 16383

		// 3. Status ìë°ì´í¸ (ì¤ìê° ìë¬ ëª¨ëí°ë§ì©)
		encData->status = (rawValue >> 6) & 0x0F;

		// ë³íë ê³ì° (Delta)
		int32_t diff = (int32_t) new_raw - (int32_t) encData->last_raw_angle;

		// Wrap-around ì²ë¦¬
		if (diff < -ENC_HALF)
			diff += 16384;
		else if (diff > ENC_HALF)
			diff -= 16384;

		// ê°ë ë³í ë° ëì 
		float angle_step = (float) diff * DEG_PER_TICK * GEAR_RATIO * POLE_PAIRS;
		encData->motor_elec_angle += angle_step;

		// 0~360 ë²ì ì ì§
		encData->motor_elec_angle -= 360.0f
				* floorf(encData->motor_elec_angle / 360.0f);

		// ë°ì´í° ê°±ì 
		encData->last_raw_angle = new_raw;
	}
}

void Swap_MT6701_Spi_Mode(){
	hspi2.Init.IOSwap = SPI_IO_SWAP_ENABLE;
	HAL_SPI_Init(&hspi2);
}
