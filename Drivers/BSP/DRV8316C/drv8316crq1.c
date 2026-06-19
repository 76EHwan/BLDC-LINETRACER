/*
 * DRV8316C.c
 *
 * Created on: Nov 10, 2025
 * Author: kth59
 */
#include "drv8316crq1.h"
#include "dac.h"

#ifdef FOC_CONTROL

// --- [CRITICAL] DRV8316C SPI Bit Definitions ---
#define DRV_RW_READ_BIT     (1 << 15)
#define DRV_ADDR_SHIFT      9           // ì£¼ìë 9ë¹í¸ ë°ì´ì¼ í¨ (Bit 14-9)
#define DRV_PARITY_BIT      (1 << 8)    // í¨ë¦¬í°ë Bit 8ì ìì¹
#define DRV_DATA_MASK       0xFF        // ë°ì´í°ë íì 8ë¹í¸

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
	data &= ~DRV_PARITY_BIT; // í¨ë¦¬í° ë¹í¸ ìì¹ ì ì¸íê³  ê³ì°

	for (int i = 0; i < 16; i++) {
		if ((data >> i) & 0x01) {
			one_count++;
		}
	}
	// DRV8316Cë Odd Parityë¥¼ ì¬ì©í©ëë¤ (Total 1s including parity should be odd)
	// ê¸°ì¡´ ì½ëì ë¡ì§(one_count % 2)ì ì ì§í©ëë¤.
	return one_count % 2;
}

/**
 * @brief  Internal helper for SPI Tx/Rx (8-bit x 2 Transfer)
 */
static HAL_StatusTypeDef DRV8316C_SPI_TxRx(DRV8316C_Handle_t *hdrv,
		uint8_t *pTxData, uint8_t *pRxData) {
	HAL_StatusTypeDef status;

	DRV8316C_CS_LOW(hdrv);

	// 8-bit ëª¨ë: Size = 2 (8ë¹í¸ ë°°ì´ 2ê° ì ì¡)
	status = HAL_SPI_TransmitReceive(hdrv->hspi, pTxData, pRxData, 2, 100);

	DRV8316C_CS_HIGH(hdrv);

	return status;
}

/*=======================================================================*/
/* Public Function Implementations                                       */
/*=======================================================================*/

void DRV8316C_Init(DRV8316C_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, GPIO_TypeDef *nSLEEP_Port,
		uint16_t nSLEEP_Pin, GPIO_TypeDef *nFAULT_Port, uint16_t nFAULT_Pin,
		GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin) {
	hdrv->hspi = hspi;
	hdrv->nCS_Port = nCS_Port;
	hdrv->nCS_Pin = nCS_Pin;

	hdrv->nSLEEP_Port = nSLEEP_Port; // 💡 구조체에 할당 추가
	hdrv->nSLEEP_Pin = nSLEEP_Pin;

	hdrv->nFAULT_Port = nFAULT_Port;
	hdrv->nFAULT_Pin = nFAULT_Pin;

	hdrv->DRVOFF_Port = DRVOFF_Port;
	hdrv->DRVOFF_Pin = DRVOFF_Pin;

	// 초기 핀 상태 설정
	DRV8316C_CS_HIGH(hdrv);
	DRV8316C_DRVOFF_LOW(hdrv);
}
/**
 * @brief  Writes 16-bit frame with CORRECT Bit Packing (8-bit x 2 Transfer)
 */
HAL_StatusTypeDef DRV8316C_WriteRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t data) {
	uint16_t tx_frame = 0;
	uint8_t tx_data[2] = { 0, };
	uint8_t rx_data[2] = { 0, };

	// 1. íë ì ìì± (Write=0, Address Shift=9, Data)
	tx_frame = ((uint16_t) (regAddr & 0x3F) << DRV_ADDR_SHIFT)
			| (data & DRV_DATA_MASK);

	// 2. í¨ë¦¬í° ê³ì° ë° ì½ì (Bit 8)
	if (DRV8316C_CalculateEvenParity(tx_frame)) {
		tx_frame |= DRV_PARITY_BIT;
	}

	// =========================================================
	// [FAULT ê°ì  ì ë° íì¤í¸ ì½ë]
	// ì ìì ì¼ë¡ ê³ì°ë í¨ë¦¬í° ë¹í¸ë¥¼ ê°ì ë¡ ë°ì ìíµëë¤.
	// DRV8316-Q1ì ì´ë¥¼ SPI ìë¬ë¡ ì¸ìíì¬ nFAULTë¥¼ ì¦ì ì¶ë ¥í©ëë¤.
//    tx_frame ^= DRV_PARITY_BIT;
	// =========================================================

	// 3. 16ë¹í¸ ë°ì´í°ë¥¼ 8ë¹í¸ 2ê°ë¡ ë¶í  (MSB First)
	tx_data[0] = (uint8_t) ((tx_frame >> 8) & 0xFF); // ìì 8ë¹í¸
	tx_data[1] = (uint8_t) (tx_frame & 0xFF);        // íì 8ë¹í¸

	// 4. ì ì¡
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

	// 1. Read íë ì ìì± (Read=1, Address Shift=9)
	tx_frame = DRV_RW_READ_BIT
			| ((uint16_t) (regAddr & 0x3F) << DRV_ADDR_SHIFT);

	// 2. í¨ë¦¬í° ê³ì° ë° ì½ì
	if (DRV8316C_CalculateEvenParity(tx_frame)) {
		tx_frame |= DRV_PARITY_BIT;
	}

	// 3. 16ë¹í¸ ë°ì´í°ë¥¼ 8ë¹í¸ 2ê°ë¡ ë¶í  (MSB First)
	tx_data[0] = (uint8_t) ((tx_frame >> 8) & 0xFF); // ìì 8ë¹í¸
	tx_data[1] = (uint8_t) (tx_frame & 0xFF);        // íì 8ë¹í¸

	// 4. ì ì¡ ë° ìì 
	status = DRV8316C_SPI_TxRx(hdrv, tx_data, rx_data);

	if (status == HAL_OK) {
		// 5. ìì ë 8ë¹í¸ 2ê°ë¥¼ ë¤ì 16ë¹í¸ë¡ ì¡°ë¦½
		uint16_t rx_frame = ((uint16_t) rx_data[0] << 8) | rx_data[1];

		// 6. ë°ì´í° ì¶ì¶ (íì 8ë¹í¸)
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

	// [CTRL 2] ê¸°ë³¸ ì¤ì : SDO Push-Pull, Slew Rate 125V/us, PWM 3x Mode, Clear Fault
	reg_val = DRV_CTRL2_SDO_MODE_PP | DRV_CTRL2_SLEW_125V_us
			| DRV_CTRL2_PWM_MODE_3X | DRV_CTRL2_CLR_FLT_BIT;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_2, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 3] OVP(ê³¼ì ì ë³´í¸) ëê¸°
	reg_val = DRV_CTRL3_PWM_100_DUTY_40KHZ | DRV_CTRL3_OVP_SEL_22V;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_3, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 4] OCP(ê³¼ì ë¥ ë³´í¸) ì¤ì ì "REPORT ONLY"ë¡ ë³ê²½
	reg_val = DRV_CTRL4_OCP_MODE_REPORT | DRV_CTRL4_OCP_LVL_24A
			| DRV_CTRL4_OCP_DEG_0_6US;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_4, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 5] ì ë¥ ì¼ì± ê²ì¸ ì¤ì  (0.6V/A) + ASR/AAR ì¼ê¸°
	reg_val = DRV_CTRL5_CSA_GAIN_0_6VA;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_5, reg_val);
	if (status != HAL_OK)
		return status;

	// [CTRL 6] ë² ì»¨ë²í° ëê¸° (ì¬ì© ì í¨)
	reg_val =
	DRV_CTRL6_BUCK_PS_DIS | DRV_CTRL6_BUCK_SEL_5V | DRV_CTRL6_BUCK_DIS;
	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_6, reg_val);

	return status;
}

HAL_StatusTypeDef DRV8316C_ClearFaults(DRV8316C_Handle_t *hdrv) {
	HAL_GPIO_WritePin(hdrv->nSLEEP_Port, hdrv->nSLEEP_Pin, GPIO_PIN_SET);
	// ë ì§ì¤í°ë¥¼ íµí Clear
	return DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_2,
			DRV_CTRL2_SDO_MODE_PP | DRV_CTRL2_SLEW_125V_us
					| DRV_CTRL2_PWM_MODE_3X | DRV_CTRL2_CLR_FLT_BIT);
}

DRV8316C_REG_Typedef DRV8316C_VerifyConfig(DRV8316C_Handle_t *hdrv) {
	HAL_StatusTypeDef status;
	uint8_t read_val = 0;
	uint8_t expected_val = 0;

	// CTRL2 íì¸
	expected_val = DRV_CTRL2_SDO_MODE_PP | DRV_CTRL2_SLEW_125V_us
			| DRV_CTRL2_PWM_MODE_3X;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_2, &read_val);
	if (status != HAL_OK)
		return REG_FAULT_CTRL2;
	if ((read_val & 0xFE) != (expected_val & 0xFE))
		return REG_FAULT_CTRL2;

	// CTRL3 íì¸
	expected_val = DRV_CTRL3_PWM_100_DUTY_40KHZ | DRV_CTRL3_OVP_SEL_22V;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_3, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL3;

	// CTRL4 íì¸
	expected_val = DRV_CTRL4_OCP_MODE_REPORT | DRV_CTRL4_OCP_LVL_24A
			| DRV_CTRL4_OCP_DEG_0_6US;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_4, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL4;

	// CTRL5 íì¸
	expected_val = DRV_CTRL5_CSA_GAIN_0_6VA;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_5, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL5;

	// CTRL6 íì¸
	expected_val = DRV_CTRL6_BUCK_PS_DIS | DRV_CTRL6_BUCK_SEL_5V
			| DRV_CTRL6_BUCK_DIS;
	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_6, &read_val);
	if (status != HAL_OK || read_val != expected_val)
		return REG_FAULT_CTRL6;

	return REG_OK;
}

void MX_DRV8316C_Init() {
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0; // 초기 Duty Ratio 0% (안전)
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	/* --- Left Motor (TIM3) 25kHz PWM 강제 설정 및 시작 --- */
	htim3.Init.Prescaler = 0;             // PSC = 0
	htim3.Init.Period = 9600 - 1;         // ARR = 9599 (240MHz 클럭 기준 25kHz)
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	/* --- Right Motor (TIM4) 25kHz PWM 강제 설정 및 시작 --- */
	htim4.Init.Prescaler = 0;             // PSC = 0
	htim4.Init.Period = 9600 - 1;         // ARR = 9599 (240MHz 클럭 기준 25kHz)
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = MTR_PWM_L_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MTR_PWM_L_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = MTR_PWM_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MTR_PWM_R_GPIO_Port, &GPIO_InitStruct);

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0xFFF);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	/* --- Left motor driver --- */
	DRV8316C_Init(&DRV8316C_L, &hspi2,
	MTR_CS_L_GPIO_Port, MTR_CS_L_Pin,
	MTR_nSLEEP_L_GPIO_Port, MTR_nSLEEP_L_Pin,
	MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin,
	MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin);

	/* --- Right motor driver --- */
	DRV8316C_Init(&DRV8316C_R, &hspi2,
	MTR_CS_R_GPIO_Port, MTR_CS_R_Pin,
	MTR_nSLEEP_R_GPIO_Port, MTR_nSLEEP_R_Pin,
	MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin,
	MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin);

	/* Wake Left driver */
	DRV8316C_WAKEUP(&DRV8316C_L);
	HAL_Delay(5);

	/* Apply configuration to left driver */
	DRV8316C_UnlockRegister(&DRV8316C_L);
	DRV8316C_ApplyDefaultConfig(&DRV8316C_L);
	DRV8316C_VerifyConfig(&DRV8316C_L);
	DRV8316C_LockRegister(&DRV8316C_L);

	/* Wake Right driver */
	DRV8316C_WAKEUP(&DRV8316C_R);
	HAL_Delay(5);

	/* Apply configuration to right driver */
	DRV8316C_UnlockRegister(&DRV8316C_R);
	DRV8316C_ApplyDefaultConfig(&DRV8316C_R);
	DRV8316C_VerifyConfig(&DRV8316C_R);
	DRV8316C_LockRegister(&DRV8316C_R);
}

#endif

