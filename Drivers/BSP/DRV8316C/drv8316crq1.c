/*
 * DRV8316C.c
 *
 * Created on: Nov 10, 2025
 * Author: kth59
 */
#include "drv8316crq1.h"

#ifdef FOC_CONTROL

// --- [CRITICAL] DRV8316C SPI Bit Definitions ---
#define DRV_RW_READ_BIT     (1 << 15)
#define DRV_ADDR_SHIFT      9           // 주소는 9비트 밀어야 함 (Bit 14-9)
#define DRV_PARITY_BIT      (1 << 8)    // 패리티는 Bit 8에 위치
#define DRV_DATA_MASK       0xFF        // 데이터는 하위 8비트

DRV8316C_Handle_t DRV8316C_L;
DRV8316C_Handle_t DRV8316C_R;

/*=======================================================================*/
/* Internal Helper Functions                                             */
/*=======================================================================*/

/**
 * @brief  Calculates the even parity bit.
 */
static uint8_t DRV8316C_CalculateEvenParity(uint16_t data) {
	uint8_t one_count = 0;
	data &= ~DRV_PARITY_BIT; // 패리티 비트 위치 제외하고 계산

	for (int i = 0; i < 16; i++) {
		if ((data >> i) & 0x01) {
			one_count++;
		}
	}
	// DRV8316C는 Odd Parity를 사용합니다 (Total 1s including parity should be odd)
	// 기존 코드의 로직(one_count % 2)을 유지합니다.
	return one_count % 2;
}

/**
 * @brief  Internal helper for SPI Tx/Rx (8-bit x 2 Transfer)
 */
static HAL_StatusTypeDef DRV8316C_SPI_TxRx(DRV8316C_Handle_t *hdrv,
		uint8_t *pTxData, uint8_t *pRxData) {
	HAL_StatusTypeDef status;

	DRV8316C_CS_LOW(hdrv);

	// 8-bit 모드: Size = 2 (8비트 배열 2개 전송)
	status = HAL_SPI_TransmitReceive(hdrv->hspi, pTxData, pRxData, 2, 100);

	DRV8316C_CS_HIGH(hdrv);

	return status;
}

/*=======================================================================*/
/* Public Function Implementations                                       */
/*=======================================================================*/

void DRV8316C_Init(DRV8316C_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, GPIO_TypeDef *nFAULT_Port,
		uint16_t nFAULT_Pin, GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin) {
	hdrv->hspi = hspi;
	hdrv->nCS_Port = nCS_Port;
	hdrv->nCS_Pin = nCS_Pin;

	hdrv->nFAULT_Port = nFAULT_Port;
	hdrv->nFAULT_Pin = nFAULT_Pin;

	hdrv->DRVOFF_Port = DRVOFF_Port;
	hdrv->DRVOFF_Pin = DRVOFF_Pin;

	// 초기 핀 상태 설정
	DRV8316C_CS_HIGH(hdrv);
	DRV8316C_DRVOFF_LOW(hdrv); // DRVOFF=0 (Normal Operation)
}

/**
 * @brief  Writes 16-bit frame with CORRECT Bit Packing (8-bit x 2 Transfer)
 */
HAL_StatusTypeDef DRV8316C_WriteRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t data) {
	uint16_t tx_frame = 0;
	uint8_t tx_data[2] = { 0, };
	uint8_t rx_data[2] = { 0, };

	// 1. 프레임 생성 (Write=0, Address Shift=9, Data)
	tx_frame = ((uint16_t) (regAddr & 0x3F) << DRV_ADDR_SHIFT)
			| (data & DRV_DATA_MASK);

	// 2. 패리티 계산 및 삽입 (Bit 8)
	if (DRV8316C_CalculateEvenParity(tx_frame)) {
		tx_frame |= DRV_PARITY_BIT;
	}

	// =========================================================
	// [FAULT 강제 유발 테스트 코드]
	// 정상적으로 계산된 패리티 비트를 강제로 반전시킵니다.
	// DRV8316-Q1은 이를 SPI 에러로 인식하여 nFAULT를 즉시 출력합니다.
//    tx_frame ^= DRV_PARITY_BIT;
	// =========================================================

	// 3. 16비트 데이터를 8비트 2개로 분할 (MSB First)
	tx_data[0] = (uint8_t) ((tx_frame >> 8) & 0xFF); // 상위 8비트
	tx_data[1] = (uint8_t) (tx_frame & 0xFF);        // 하위 8비트

	// 4. 전송
	return DRV8316C_SPI_TxRx(hdrv, tx_data, rx_data);
}

/**
 * @brief  Reads 16-bit frame with CORRECT Bit Packing (8-bit x 2 Transfer)
 */
HAL_StatusTypeDef DRV8316C_ReadRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t *pData) {
	uint16_t tx_frame = 0;
	uint8_t tx_data[2] = { 0, };
	uint8_t rx_data[2] = { 0, };
	HAL_StatusTypeDef status;

	// 1. Read 프레임 생성 (Read=1, Address Shift=9)
	tx_frame = DRV_RW_READ_BIT
			| ((uint16_t) (regAddr & 0x3F) << DRV_ADDR_SHIFT);

	// 2. 패리티 계산 및 삽입
	if (DRV8316C_CalculateEvenParity(tx_frame)) {
		tx_frame |= DRV_PARITY_BIT;
	}

	// 3. 16비트 데이터를 8비트 2개로 분할 (MSB First)
	tx_data[0] = (uint8_t) ((tx_frame >> 8) & 0xFF); // 상위 8비트
	tx_data[1] = (uint8_t) (tx_frame & 0xFF);        // 하위 8비트

	// 4. 전송 및 수신
	status = DRV8316C_SPI_TxRx(hdrv, tx_data, rx_data);

	if (status == HAL_OK) {
		// 5. 수신된 8비트 2개를 다시 16비트로 조립
		uint16_t rx_frame = ((uint16_t) rx_data[0] << 8) | rx_data[1];

		// 6. 데이터 추출 (하위 8비트)
		*pData = (uint8_t) (rx_frame & DRV_DATA_MASK);
	}

	return status;
}

HAL_StatusTypeDef DRV8316C_UnlockRegister(DRV8316C_Handle_t *hdrv) {
	return DRV8316C_WriteRegister(hdrv, 0x3, 0x3);
}

HAL_StatusTypeDef DRV8316C_LockRegister(DRV8316C_Handle_t *hdrv) {
	return DRV8316C_WriteRegister(hdrv, 0x3, 0x6);
}

/**
 * @brief  Applies a common default configuration to the DRV8316C.
 */
HAL_StatusTypeDef DRV8316C_ApplyDefaultConfig(DRV8316C_Handle_t *hdrv) {
	HAL_StatusTypeDef status;
	uint8_t reg_val;

	// [CTRL 2] 기본 설정: SDO Push-Pull, Slew Rate 125V/us, PWM 3x Mode, Clear Fault
	reg_val = DRV_CTRL2_SDO_MODE_PP | DRV_CTRL2_SLEW_125V_us
			| DRV_CTRL2_PWM_MODE_3X | DRV_CTRL2_CLR_FLT_BIT;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_2, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 3] OVP(과전압 보호) 끄기
	reg_val = DRV_CTRL3_PWM_100_DUTY_40KHZ | DRV_CTRL3_OVP_SEL_22V;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_3, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 4] OCP(과전류 보호) 설정을 "REPORT ONLY"로 변경
	reg_val = DRV_CTRL4_OCP_MODE_REPORT | DRV_CTRL4_OCP_LVL_24A
			| DRV_CTRL4_OCP_DEG_0_6US;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_4, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 5] 전류 센싱 게인 설정 (0.6V/A) + ASR/AAR 켜기
	reg_val = DRV_CTRL5_CSA_GAIN_0_6VA;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_5, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 6] 벅 컨버터 끄기 (사용 안 함)
	reg_val =
			DRV_CTRL6_BUCK_PS_DIS | DRV_CTRL6_BUCK_SEL_5V | DRV_CTRL6_BUCK_DIS;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_6, reg_val);

	return status;
}

HAL_StatusTypeDef DRV8316C_ClearFaults(DRV8316C_Handle_t *hdrv) {
	HAL_GPIO_WritePin(hdrv->nSLEEP_Port, hdrv->nSLEEP_Pin, GPIO_PIN_SET);
	// 레지스터를 통한 Clear
	return DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_2,
			DRV_CTRL2_SDO_MODE_PP | DRV_CTRL2_SLEW_125V_us
					| DRV_CTRL2_PWM_MODE_3X | DRV_CTRL2_CLR_FLT_BIT);
}

DRV8316C_REG_Typedef DRV8316C_VerifyConfig(DRV8316C_Handle_t *hdrv) {
	HAL_StatusTypeDef status;
	uint8_t read_val = 0;
	uint8_t expected_val = 0;

	// CTRL2 확인
	expected_val = DRV_CTRL2_SDO_MODE_PP | DRV_CTRL2_SLEW_125V_us
			| DRV_CTRL2_PWM_MODE_3X;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_2, &read_val);
	if (status != HAL_OK)
		return REG_FAULT_CTRL2;
	if ((read_val & 0xFE) != (expected_val & 0xFE))
		return REG_FAULT_CTRL2;

	// CTRL3 확인
	expected_val = DRV_CTRL3_PWM_100_DUTY_40KHZ | DRV_CTRL3_OVP_SEL_22V;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_3, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL3;

	// CTRL4 확인
	expected_val = DRV_CTRL4_OCP_MODE_REPORT | DRV_CTRL4_OCP_LVL_24A
			| DRV_CTRL4_OCP_DEG_0_6US;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_4, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL4;

	// CTRL5 확인
	expected_val = DRV_CTRL5_CSA_GAIN_0_6VA;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_5, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL5;

	// CTRL6 확인
	expected_val = DRV_CTRL6_BUCK_PS_DIS | DRV_CTRL6_BUCK_SEL_5V
			| DRV_CTRL6_BUCK_DIS;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_6, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL6;

	return REG_OK;
}

#endif
