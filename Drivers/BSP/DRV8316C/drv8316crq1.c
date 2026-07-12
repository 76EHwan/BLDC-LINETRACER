#include "drv8316crq1.h"
#include "dac.h"
#include "foc.h"

#define DRV_RW_READ_BIT     (1 << 15)
#define DRV_ADDR_SHIFT      9
#define DRV_PARITY_BIT      (1 << 8)
#define DRV_DATA_MASK       0xFF

// @formatter:off

/* ===================================================================== */
/* Control Register 2 (Offset = 4h) : PWM Mode, Slew Rate, SDO           */
/* ===================================================================== */
static const uint8_t DRV_DEFAULT_CTRL2 =
// #7~6 Reserved
// #5   SDO_MODE - SPI 데이터 출력 핀 모드
//      DRV_CTRL2_SDO_MODE_OD |     // Open drain 방식
        DRV_CTRL2_SDO_MODE_PP |     // Push pull 방식
// #4~3 SLEW - 스위칭 슬루율 (Slew Rate)
        DRV_CTRL2_SLEW_25V_us |     // 25 V/μs (노이즈 최소화)
//      DRV_CTRL2_SLEW_50V_us |     // 50 V/μs
//      DRV_CTRL2_SLEW_125V_us |    // 125 V/μs
//      DRV_CTRL2_SLEW_200V_us |    // 200 V/μs
// #2~1 PWM_MODE - PWM 입력 모드
//      DRV_CTRL2_PWM_MODE_6X;      // 6 PWM 방식 (전류제한 X)
//      DRV_CTRL2_PWM_MODE_6X_CL;   // 6 PWM 방식 (전류제한 O)
        DRV_CTRL2_PWM_MODE_3X;      // 3 PWM 방식 (전류제한 X)
//      DRV_CTRL2_PWM_MODE_3X_CL;   // 3 PWM 방식 (전류제한 O)
// #0   CLR_FLT - 폴트 초기화 (초기화 시에만 1을 씀, 기본 설정에선 생략)


/* ===================================================================== */
/* Control Register 3 (Offset = 5h) : OVP, PWM Duty, Fault Reporting     */
/* ===================================================================== */
static const uint8_t DRV_DEFAULT_CTRL3 =
// #7~5 Reserved
// #4   PWM_100_DUTY_SEL - 100% 듀티 시 차지펌프 동작 주파수
//      DRV_CTRL3_PWM_100_DUTY_SEL_20KHZ |
        DRV_CTRL3_PWM_100_DUTY_SEL_40KHZ |
// #3   OVP_SEL - 과전압 보호 기준치
//      DRV_CTRL3_OVP_SEL_34V |
        DRV_CTRL3_OVP_SEL_22V |
// #2   OVP_EN - 과전압 보호 활성화
//      DRV_CTRL3_OVP_EN_DIS |      // OVP 끄기
        DRV_CTRL3_OVP_EN_EN |       // OVP 켜기
// #1   SPI_FLT_REP - SPI 에러 보고 활성화
//      DRV_CTRL3_SPI_FLT_REP_DIS |
        DRV_CTRL3_SPI_FLT_REP_EN |
// #0   OTW_REP - 과열 경고 보고 활성화
//      DRV_CTRL3_OTW_REP_DIS;
        DRV_CTRL3_OTW_REP_EN;


/* ===================================================================== */
/* Control Register 4 (Offset = 6h) : OCP (Overcurrent Protection)       */
/* ===================================================================== */
static const uint8_t DRV_DEFAULT_CTRL4 =
// #7(6) DRVOFF - 드라이버 강제 종료 대기 모드 (헤더 매크로 베이스 기준)
//      DRV_CTRL4_DRVOFF_NO_ACTION |// 기본 동작 (모터 구동 가능)
        DRV_CTRL4_DRVOFF_STANDBY |  // 대기 상태 (Hi-Z로 모든 출력 차단)
// #6(5) OCP_CBC - 사이클 단위 과전류 해제 (Cycle-by-Cycle)
        DRV_CTRL4_OCP_CBC_DIS |     // 비활성화
//      DRV_CTRL4_OCP_CBC_EN |      // 활성화
// #5~4 OCP_DEG - 과전류 인식 지연 시간 (Deglitch Time)
//      DRV_CTRL4_OCP_DEG_0_2US |
        DRV_CTRL4_OCP_DEG_0_6US |   // 0.6 µs (기본값 추천)
//      DRV_CTRL4_OCP_DEG_1_25US |
//      DRV_CTRL4_OCP_DEG_1_6US |
// #3   OCP_RETRY - 과전류 발생 후 재시도 시간 (Auto-retry 모드 시)
        DRV_CTRL4_OCP_RETRY_5MS |
//      DRV_CTRL4_OCP_RETRY_500MS |
// #2   OCP_LVL - 과전류 판단 기준치
//      DRV_CTRL4_OCP_LVL_16A |
        DRV_CTRL4_OCP_LVL_24A |
// #1~0 OCP_MODE - 과전류 발생 시 대처 방법
//      DRV_CTRL4_OCP_MODE_LATCH;   // 즉시 정지 및 잠금 (가장 안전)
//      DRV_CTRL4_OCP_MODE_RETRY;   // 일정 시간 후 자동 재시도
        DRV_CTRL4_OCP_MODE_REPORT;  // 정지 안함, 폴트만 띄움 (디버깅용)
//      DRV_CTRL4_OCP_MODE_DISABLED;// OCP 기능 끄기


/* ===================================================================== */
/* Control Register 5 (Offset = 7h) : Current Sense & Rectification      */
/* ===================================================================== */
static const uint8_t DRV_DEFAULT_CTRL5 =
// #7   Reserved
// #6   ILIM_RECIR - 전류 제한 시 잉여 전류 순환 방식
        DRV_CTRL5_ILIM_RECIR_BRAKE |// Brake 모드 (FET 켬 - 정밀 제어 유리)
//      DRV_CTRL5_ILIM_RECIR_COAST |// Coast 모드 (바디 다이오드 사용)
// #5~4 Reserved
// #3   EN_AAR - 비동기 능동 정류 활성화
        DRV_CTRL5_EN_AAR_DIS |      // AAR 비활성화
//      DRV_CTRL5_EN_AAR_EN |       // AAR 활성화 (발열 감소 추천)
// #2   EN_ASR - 동기 능동 정류 활성화
        DRV_CTRL5_EN_ASR_DIS |      // ASR 비활성화
//      DRV_CTRL5_EN_ASR_EN |       // ASR 활성화 (발열 감소 추천)
// #1~0 CSA_GAIN - 전류 센싱 증폭기(CSA) 게인 (V/A)
#if CURRENT_CSA_GAIN_MA == 150
      DRV_CTRL5_CSA_GAIN_0_15VA;  // 0.15 V/A (대전류용)
#endif
#if CURRENT_CSA_GAIN_MA == 300
      	DRV_CTRL5_CSA_GAIN_0_3VA;   // 0.3 V/A
#endif
#if CURRENT_CSA_GAIN_MA == 600
      DRV_CTRL5_CSA_GAIN_0_6VA;   // 0.6 V/A (정밀 제어용 추천)
#endif
#if CURRENT_CSA_GAIN_MA == 1200
      DRV_CTRL5_CSA_GAIN_1_2VA;   // 1.2 V/A
#endif

/* ===================================================================== */
/* Control Register 6 (Offset = 8h) : Buck Regulator Configuration       */
/* ===================================================================== */
static const uint8_t DRV_DEFAULT_CTRL6 =
// #7~5 Reserved
// #4   BUCK_PS - 벅 컨버터 파워 시퀀싱
//      DRV_CTRL6_BUCK_PS_EN |
        DRV_CTRL6_BUCK_PS_DIS |     // 시퀀싱 비활성화
// #3   BUCK_CL - 벅 컨버터 전류 제한 (생략 가능, 기본값 600mA)
// #2~1 BUCK_SEL - 벅 컨버터 출력 전압 설정
//      DRV_CTRL6_BUCK_SEL_3V3 |    // 3.3V
        DRV_CTRL6_BUCK_SEL_5V |     // 5.0V
//      DRV_CTRL6_BUCK_SEL_4V |     // 4.0V
//      DRV_CTRL6_BUCK_SEL_5V7 |    // 5.7V
// #0   BUCK_EN - 벅 컨버터 활성화 여부
//      DRV_CTRL6_BUCK_EN;          // 컨버터 켜기
        DRV_CTRL6_BUCK_DIS;         // 컨버터 끄기 (외부 전원 사용 시)


/* ===================================================================== */
/* Control Register 10 (Offset = Ch) : Delay Compensation                */
/* ===================================================================== */
static const uint8_t DRV_DEFAULT_CTRL10 =
// #7~5 Reserved
// #4   DLYCMP_EN - 드라이버 지연 보상 기능
        DRV_CTRL10_DLYCMP_DIS |     // 보상 기능 끄기
//      DRV_CTRL10_DLYCMP_EN |      // 보상 기능 켜기
// #3~0 DLY_TARGET - 목표 지연 시간 설정
        DRV_CTRL10_DLY_TARGET_0US;  // 0 µs
//      DRV_CTRL10_DLY_TARGET_0_4US;// 0.4 µs
//      DRV_CTRL10_DLY_TARGET_0_8US;// 0.8 µs
//      DRV_CTRL10_DLY_TARGET_1_2US;// 1.2 µs
//		DRV_CTRL10_DLY_TARGET_1_4US;// 1.4 µs
//		DRV_CTRL10_DLY_TARGET_1_6US;// 1.6 µs
//		DRV_CTRL10_DLY_TARGET_1_8US;// 1.8 µs
//		DRV_CTRL10_DLY_TARGET_2US;	// 2 µs
//		DRV_CTRL10_DLY_TARGET_2_2US;// 2.2 µs
//		DRV_CTRL10_DLY_TARGET_2_4US;// 2.4 µs
//		DRV_CTRL10_DLY_TARGET_2_6US;// 2.6 µs
//		DRV_CTRL10_DLY_TARGET_2_8US;// 2.8 µs
//		DRV_CTRL10_DLY_TARGET_3US;	// 3 µs
//		DRV_CTRL10_DLY_TARGET_3_2US;// 3.2 µs


// @formatter:on

DRV8316C_Handle_t DRV8316C_L;
DRV8316C_Handle_t DRV8316C_R;

static uint8_t DRV8316C_CalculateEvenParity(uint16_t data) {
	uint8_t one_count = 0;
	data &= ~DRV_PARITY_BIT;

	for (int i = 0; i < 16; i++) {
		if ((data >> i) & 0x01) {
			one_count++;
		}
	}
	return one_count % 2;
}

static HAL_StatusTypeDef DRV8316C_SPI_TxRx(DRV8316C_Handle_t *hdrv,
		uint8_t *pTxData, uint8_t *pRxData) {
	HAL_StatusTypeDef status;

	DRV8316C_CS_LOW(hdrv);

	status = HAL_SPI_TransmitReceive(hdrv->hspi, pTxData, pRxData, 2, 100);

	DRV8316C_CS_HIGH(hdrv);

	return status;
}

void DRV8316C_Init(DRV8316C_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, GPIO_TypeDef *nSLEEP_Port,
		uint16_t nSLEEP_Pin, GPIO_TypeDef *nFAULT_Port, uint16_t nFAULT_Pin,
		GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin) {
	hdrv->hspi = hspi;
	hdrv->nCS_Port = nCS_Port;
	hdrv->nCS_Pin = nCS_Pin;

	hdrv->nSLEEP_Port = nSLEEP_Port;
	hdrv->nSLEEP_Pin = nSLEEP_Pin;

	hdrv->nFAULT_Port = nFAULT_Port;
	hdrv->nFAULT_Pin = nFAULT_Pin;

	hdrv->DRVOFF_Port = DRVOFF_Port;
	hdrv->DRVOFF_Pin = DRVOFF_Pin;

	DRV8316C_CS_HIGH(hdrv);
	DRV8316C_DRVOFF_LOW(hdrv);
}

HAL_StatusTypeDef DRV8316C_WriteRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t data) {
	uint16_t tx_frame = 0;
	uint8_t tx_data[2] = { 0, };
	uint8_t rx_data[2] = { 0, };

	tx_frame = ((uint16_t) (regAddr & 0x3F) << DRV_ADDR_SHIFT)
			| (data & DRV_DATA_MASK);

	if (DRV8316C_CalculateEvenParity(tx_frame)) {
		tx_frame |= DRV_PARITY_BIT;
	}

	tx_data[0] = (uint8_t) ((tx_frame >> 8) & 0xFF);
	tx_data[1] = (uint8_t) (tx_frame & 0xFF);

	return DRV8316C_SPI_TxRx(hdrv, tx_data, rx_data);
}

HAL_StatusTypeDef DRV8316C_ReadRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t *pData) {
	uint16_t tx_frame = 0;
	uint8_t tx_data[2] = { 0, };
	uint8_t rx_data[2] = { 0, };
	HAL_StatusTypeDef status;

	tx_frame = DRV_RW_READ_BIT
			| ((uint16_t) (regAddr & 0x3F) << DRV_ADDR_SHIFT);

	if (DRV8316C_CalculateEvenParity(tx_frame)) {
		tx_frame |= DRV_PARITY_BIT;
	}

	tx_data[0] = (uint8_t) ((tx_frame >> 8) & 0xFF);
	tx_data[1] = (uint8_t) (tx_frame & 0xFF);

	status = DRV8316C_SPI_TxRx(hdrv, tx_data, rx_data);

	if (status == HAL_OK) {
		uint16_t rx_frame = ((uint16_t) rx_data[0] << 8) | rx_data[1];
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

HAL_StatusTypeDef DRV8316C_ApplyDefaultConfig(DRV8316C_Handle_t *hdrv) {
	HAL_StatusTypeDef status;

	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_2,
			DRV_DEFAULT_CTRL2 | DRV_CTRL2_CLR_FLT_BIT);
	if (status != HAL_OK)
		return status;

	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_3, DRV_DEFAULT_CTRL3);
	if (status != HAL_OK)
		return status;

	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_4, DRV_DEFAULT_CTRL4);
	if (status != HAL_OK)
		return status;

	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_5, DRV_DEFAULT_CTRL5);
	if (status != HAL_OK)
		return status;

	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_6, DRV_DEFAULT_CTRL6);
	if (status != HAL_OK)
		return status;

	status = DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_10, DRV_DEFAULT_CTRL10);
	if (status != HAL_OK)
		return status;

	return status;
}

HAL_StatusTypeDef DRV8316C_ClearFaults(DRV8316C_Handle_t *hdrv) {
	HAL_GPIO_WritePin(hdrv->nSLEEP_Port, hdrv->nSLEEP_Pin, GPIO_PIN_SET);
	return DRV8316C_WriteRegister(hdrv, DRV_REG_CTRL_2,
			DRV_DEFAULT_CTRL2 | DRV_CTRL2_CLR_FLT_BIT);
}

DRV8316C_REG_Typedef DRV8316C_VerifyConfig(DRV8316C_Handle_t *hdrv) {
	HAL_StatusTypeDef status;
	uint8_t read_val = 0;

	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_2, &read_val);
	if (status != HAL_OK)
		return REG_FAULT_CTRL2;
	if ((read_val & 0xFE) != (DRV_DEFAULT_CTRL2 & 0xFE))
		return REG_FAULT_CTRL2;

	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_3, &read_val);
	if (status != HAL_OK || read_val != DRV_DEFAULT_CTRL3)
		return REG_FAULT_CTRL3;

	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_4, &read_val);
	if (status != HAL_OK || read_val != DRV_DEFAULT_CTRL4)
		return REG_FAULT_CTRL4;

	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_5, &read_val);
	if (status != HAL_OK || read_val != DRV_DEFAULT_CTRL5)
		return REG_FAULT_CTRL5;

	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_6, &read_val);
	if (status != HAL_OK || read_val != DRV_DEFAULT_CTRL6)
		return REG_FAULT_CTRL6;

	status = DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_10, &read_val);
	if (status != HAL_OK || read_val != DRV_DEFAULT_CTRL10)
		return REG_FAULT_CTRL10;

	return REG_OK;
}

static void MTR_TIM_HallToPWM(TIM_HandleTypeDef *htim) {
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_OC_InitTypeDef sConfigOC4 = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	__HAL_TIM_DISABLE(htim);
	htim->Instance->SMCR = 0;
	htim->Instance->CR2 &= ~TIM_CR2_TI1S;
	htim->Instance->CCMR1 = 0;
	htim->Instance->CCMR2 = 0;
	htim->Instance->CCER = 0;

	htim->Init.Prescaler = 0;
	htim->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim->Init.Period = 4800;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(htim) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);

	sConfigOC4.OCMode = TIM_OCMODE_PWM2;
	sConfigOC4.Pulse = 4800 - ADC_READ_TIMING;
	sConfigOC4.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC4.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC4, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
}

void MX_DRV8316C_Init() {
	MTR_TIM_HallToPWM(&htim3);
	MTR_TIM_HallToPWM(&htim4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM4->CR1 &= ~TIM_CR1_CEN;
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 4800);
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;

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

	DRV8316C_Init(&DRV8316C_L, &hspi2,
	MTR_CS_L_GPIO_Port, MTR_CS_L_Pin,
	MTR_nSLEEP_L_GPIO_Port, MTR_nSLEEP_L_Pin,
	MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin,
	MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin);

	DRV8316C_Init(&DRV8316C_R, &hspi2,
	MTR_CS_R_GPIO_Port, MTR_CS_R_Pin,
	MTR_nSLEEP_R_GPIO_Port, MTR_nSLEEP_R_Pin,
	MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin,
	MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin);

	DRV8316C_FOC_PWM_DIS();

	DRV8316C_WAKEUP(&DRV8316C_L);

	HAL_Delay(1);

	DRV8316C_UnlockRegister(&DRV8316C_L);
	DRV8316C_ApplyDefaultConfig(&DRV8316C_L);
	DRV8316C_VerifyConfig(&DRV8316C_L);
	DRV8316C_LockRegister(&DRV8316C_L);

	DRV8316C_WAKEUP(&DRV8316C_R);
	HAL_Delay(1);

	DRV8316C_UnlockRegister(&DRV8316C_R);
	DRV8316C_ApplyDefaultConfig(&DRV8316C_R);
	DRV8316C_VerifyConfig(&DRV8316C_R);
	DRV8316C_LockRegister(&DRV8316C_R);
}
