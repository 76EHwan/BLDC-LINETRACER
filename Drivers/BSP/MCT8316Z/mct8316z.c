/*
 * mct8316z.c
 *
 * Created on: 2026. 4. 16.
 * Author: kth59
 */
#include "main.h"
#include "dac.h"
#include "mct8316z.h"


#ifdef SENSOR_TRAP_CONTROL

/*=======================================================================*/
/* Internal SPI frame bit definitions                                    */
/*=======================================================================*/
#define MCT_RW_READ_BIT     (1U << 15)
#define MCT_ADDR_SHIFT      (9U)
#define MCT_PARITY_BIT      (1U << 8)
#define MCT_DATA_MASK       (0xFFU)

// @formatter:off

/* ===================================================================== */
/* Control Register 2 (Offset = 4h) : PWM Mode, Slew Rate, SDO           */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL2 =
// #7~6 Reserved
// #5   SDO_MODE - SPI 데이터 출력 핀 모드
//      MCT_CTRL2_SDO_MODE_OD |     // Open drain 방식
        MCT_CTRL2_SDO_MODE_PP |     // Push pull 방식
// #4~3 SLEW - 스위칭 슬루율 (Slew Rate)
        MCT_CTRL2_SLEW_25V_us |     // 25 V/μs (노이즈 최소화)
//      MCT_CTRL2_SLEW_50V_us |     // 50 V/μs
//      	MCT_CTRL2_SLEW_125V_us |    // 125 V/μs
//      MCT_CTRL2_SLEW_200V_us |    // 200 V/μs
// #2~1 PWM_MODE - PWM 입력 모드
//		MCT_CTRL2_PWM_MODE_ASYN_ANALOG |
//		MCT_CTRL2_PWM_MODE_ASYN_DIGITAL |
//		MCT_CTRL2_PWM_MODE_SYN_ANALOG |
		MCT_CTRL2_PWM_MODE_ASYN_DIGITAL;
// #0   CLR_FLT - 폴트 초기화 (초기화 시에만 1을 씀, 기본 설정에선 생략)

/* ===================================================================== */
/* Control Register 3 (Offset = 5h) : OVP, PWM Duty, Fault Reporting     */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL3 =
// #7~5 Reserved
// #4   PWM_100_DUTY_SEL - 100% 듀티 시 차지펌프 동작 주파수
//		MCT_CTRL3_PWM_100_DUTY_20KHZ |
		MCT_CTRL3_PWM_100_DUTY_40KHZ |
// #3   OVP_SEL - 과전압 보호 기준치
//      MCT_CTRL3_OVP_SEL_34V |
        MCT_CTRL3_OVP_SEL_22V |
// #2   OVP_EN - 과전압 보호 활성화
//      MCT_CTRL3_OVP_EN_DIS |      // OVP 끄기
        MCT_CTRL3_OVP_EN_EN |       // OVP 켜기
// #1   SPI_FLT_REP - SPI 에러 보고 활성화
//      MCT_CTRL3_SPI_FLT_REP_DIS |
        MCT_CTRL3_SPI_FLT_REP_EN |
// #0   OTW_REP - 과열 경고 보고 활성화
//      MCT_CTRL3_OTW_REP_DIS;
        MCT_CTRL3_OTW_REP_EN;

/* ===================================================================== */
/* Control Register 4 (Offset = 6h) : OCP (Overcurrent Protection)       */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL4 =
// #7   Reserved
// #6	DRVOFF - 드라이버 강제 종료 대기 모드 (헤더 매크로 베이스 기준)
//      MCT_CTRL4_DRVOFF_NO_ACTION |// 기본 동작 (모터 구동 가능)
        MCT_CTRL4_DRVOFF_STANDBY |  // 대기 상태 (Hi-Z로 모든 출력 차단)
// #5	OCP_CBC - 사이클 단위 과전류 해제 (Cycle-by-Cycle)
        MCT_CTRL4_OCP_CBC_DIS |     // 비활성화
//      MCT_CTRL4_OCP_CBC_EN |      // 활성화
// #4~3	OCP_DEG - 과전류 인식 지연 시간 (Deglitch Time)
//      MCT_CTRL4_OCP_DEG_0_2US |
        MCT_CTRL4_OCP_DEG_0_6US |   // 0.6 µs (기본값 추천)
//      MCT_CTRL4_OCP_DEG_1_25US |
//      MCT_CTRL4_OCP_DEG_1_6US |
// #2   OCP_RETRY - 과전류 발생 후 재시도 시간 (Auto-retry 모드 시)
        MCT_CTRL4_OCP_RETRY_5MS |
//      MCT_CTRL4_OCP_RETRY_500MS |
// #1   OCP_LVL - 과전류 판단 기준치
//      MCT_CTRL4_OCP_LVL_16A |
        MCT_CTRL4_OCP_LVL_24A |
// #0	OCP_MODE - 과전류 발생 시 대처 방법
//      MCT_CTRL4_OCP_MODE_LATCH;   // 즉시 정지 및 잠금 (가장 안전)
		MCT_CTRL4_OCP_MODE_RETRY;   // 일정 시간 후 자동 재시도
        


/* ===================================================================== */
/* Control Register 5 (Offset = 7h) : Current Sense & Rectification      */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL5 =
// #7   Reserved
// #6   ILIM_RECIR - 전류 제한 시 잉여 전류 순환 방식
        MCT_CTRL5_ILIM_RECIR_BRAKE |// Brake 모드 (FET 켬 - 정밀 제어 유리)
//      MCT_CTRL5_ILIM_RECIR_COAST |// Coast 모드 (바디 다이오드 사용)
// #5~4 Reserved
// #3   EN_AAR - 비동기 능동 정류 활성화
        MCT_CTRL5_EN_AAR_DIS |      // AAR 비활성화
//      MCT_CTRL5_EN_AAR_EN |       // AAR 활성화 (발열 감소 추천)
// #2   EN_ASR - 동기 능동 정류 활성화
        MCT_CTRL5_EN_ASR_DIS |      // ASR 비활성화
//      MCT_CTRL5_EN_ASR_EN |       // ASR 활성화 (발열 감소 추천)
// #1~0 CSA_GAIN - 전류 센싱 증폭기(CSA) 게인 (V/A)
#if CURRENT_CSA_GAIN_MA == 150
      	MCT_CTRL5_CSA_GAIN_0_15VA;  // 0.15 V/A (대전류용)
#endif
#if CURRENT_CSA_GAIN_MA == 300
      	MCT_CTRL5_CSA_GAIN_0_3VA;   // 0.3 V/A
#endif
#if CURRENT_CSA_GAIN_MA == 600
      	MCT_CTRL5_CSA_GAIN_0_6VA;   // 0.6 V/A (정밀 제어용 추천)
#endif
#if CURRENT_CSA_GAIN_MA == 1200
      	MCT_CTRL5_CSA_GAIN_1_2VA;   // 1.2 V/A
#endif

/* ===================================================================== */
/* Control Register 6 (Offset = 8h) : Buck Regulator Configuration       */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL6 =
// #7~5 Reserved
// #4   BUCK_PS - 벅 컨버터 파워 시퀀싱
//      MCT_CTRL6_BUCK_PS_EN |
        MCT_CTRL6_BUCK_PS_DIS |     // 시퀀싱 비활성화
// #3   BUCK_CL - 벅 컨버터 전류 제한 (생략 가능, 기본값 600mA)
// #2~1 BUCK_SEL - 벅 컨버터 출력 전압 설정
//      MCT_CTRL6_BUCK_SEL_3V3 |    // 3.3V
        MCT_CTRL6_BUCK_SEL_5V |     // 5.0V
//      MCT_CTRL6_BUCK_SEL_4V |     // 4.0V
//      MCT_CTRL6_BUCK_SEL_5V7 |    // 5.7V
// #0   BUCK_EN - 벅 컨버터 활성화 여부
//      MCT_CTRL6_BUCK_EN;          // 컨버터 켜기
        MCT_CTRL6_BUCK_DIS;         // 컨버터 끄기 (외부 전원 사용 시)


/* ===================================================================== */
/* Control Register 7 (Offset = 9h) :        */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL7 =
// #7~5 Reserved
// #4   HALL_HYS - 
//      MCT_CTRL7_HALL_HYS_5MV |
        MCT_CTRL7_HALL_HYS_50MV |     // 시퀀싱 비활성화
// #3   BRAKE_MODE - 
		MCT_CTRL7_BRAKE_MODE_BRAKE |
//		MCT_CTRL7_BRAKE_MODE_COAST |
// #2   COAST - 
		MCT_CTRL7_COAST_DIS |
//		MCT_CTRL7_COAST_EN | 
// #1   BRAKE - 
		MCT_CTRL7_BRAKE_DIS |
//		MCT_CTRL7_BRAKE_EN | 
// #0   DIR - 
		MCT_CTRL7_DIR_FWD;
//		MCT_CTRL7_DIR_REV;


/* ===================================================================== */
/* Control Register 8 (Offset = Ah) :        */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL8 =
// #7~6	FGOUT_SEL - 
//      MCT_CTRL8_FGOUT_SEL_3X |
        MCT_CTRL8_FGOUT_SEL_1X |
        MCT_CTRL8_FGOUT_SEL_0_5X |
        MCT_CTRL8_FGOUT_SEL_0_25X |
// #4   MTR_LOCK_RETRY - 
		MCT_CTRL8_MTR_LOCK_RETRY_500MS |
//		MCT_CTRL8_MTR_LOCK_RETRY_5000MS |
// #3~2 MTR_LOCK_TDET - 
		MCT_CTRL8_MTR_LOCK_TDET_300MS |
//		MCT_CTRL8_MTR_LOCK_TDET_500MS |
//		MCT_CTRL8_MTR_LOCK_TDET_1000MS |
//		MCT_CTRL8_MTR_LOCK_TDET_5000MS |
// #1~0 DIR - 
//		MCT_CTRL8_MTR_LOCK_MODE_LATCH;
//		MCT_CTRL8_MTR_LOCK_MODE_RETRY;
		MCT_CTRL8_MTR_LOCK_MODE_REPORT;
//		MCT_CTRL8_MTR_LOCK_MODE_NO_ACT;

/* ===================================================================== */
/* Control Register 9 (Offset = Bh) :        */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL9 =
// #7~3 Reserved
// #0~2 MTR_ADVANCE_LVL - 
		MCT_CTRL9_MTR_ADVANCE_LVL_0DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_4DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_7DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_11DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_15DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_20DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_25DEG;
//		MCT_CTRL9_MTR_ADVANCE_LVL_30DEG;


/* ===================================================================== */
/* Control Register 10 (Offset = Ch) : Delay Compensation                */
/* ===================================================================== */
static const uint8_t MCT_DEFAULT_CTRL10 =
// #7~5 Reserved
// #4   DLYCMP_EN - 드라이버 지연 보상 기능
//    	MCT_CTRL10_DLYCMP_DIS |     // 보상 기능 끄기
		MCT_CTRL10_DLYCMP_EN |      // 보상 기능 켜기
// #3~0 DLY_TARGET - 목표 지연 시간 설정
//      MCT_CTRL10_DLY_TARGET_0US;  // 0 µs
//      MCT_CTRL10_DLY_TARGET_0_4US;// 0.4 µs
//      MCT_CTRL10_DLY_TARGET_0_8US;// 0.8 µs
//      MCT_CTRL10_DLY_TARGET_1_2US;// 1.2 µs
//		MCT_CTRL10_DLY_TARGET_1_4US;// 1.4 µs
//		MCT_CTRL10_DLY_TARGET_1_6US;// 1.6 µs
//		MCT_CTRL10_DLY_TARGET_1_8US;// 1.8 µs
//		MCT_CTRL10_DLY_TARGET_2US;	// 2 µs
//		MCT_CTRL10_DLY_TARGET_2_2US;// 2.2 µs
//		MCT_CTRL10_DLY_TARGET_2_4US;// 2.4 µs
//		MCT_CTRL10_DLY_TARGET_2_6US;// 2.6 µs
//		MCT_CTRL10_DLY_TARGET_2_8US;// 2.8 µs
//		MCT_CTRL10_DLY_TARGET_3US;	// 3 µs
		MCT_CTRL10_DLY_TARGET_3_2US;// 3.2 µs


/*=======================================================================*/
/* Global Handle Instances                                               */
/*=======================================================================*/
MCT8316Z_Handle_t MCT8316Z_L;
MCT8316Z_Handle_t MCT8316Z_R;

/*=======================================================================*/
/* Internal Helper Functions                                             */
/*=======================================================================*/

static uint8_t MCT8316Z_CalcParity(uint16_t frame) {
	uint8_t ones = 0U;
	frame &= ~MCT_PARITY_BIT;

	for (int i = 0; i < 16; i++) {
		if ((frame >> i) & 0x01U) {
			ones++;
		}
	}
	return (ones % 2U);
}

static HAL_StatusTypeDef MCT8316Z_SPI_TxRx(MCT8316Z_Handle_t *hdrv,
		uint8_t *pTx, uint8_t *pRx) {
	HAL_StatusTypeDef status;

	MCT8316Z_CS_LOW(hdrv);
	status = HAL_SPI_TransmitReceive(hdrv->hspi, pTx, pRx, 2U, 100U);
	MCT8316Z_CS_HIGH(hdrv);

	return status;
}

static void MCT8316Z_BuildFrame(uint16_t raw_frame, uint8_t *pTx) {
	if (MCT8316Z_CalcParity(raw_frame)) {
		raw_frame |= MCT_PARITY_BIT;
	}
	pTx[0] = (uint8_t) ((raw_frame >> 8U) & 0xFFU);
	pTx[1] = (uint8_t) (raw_frame & 0xFFU);
}

/*=======================================================================*/
/* Public API Implementations                                            */
/*=======================================================================*/

void MCT8316Z_Init(MCT8316Z_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
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

	MCT8316Z_CS_HIGH(hdrv);
	MCT8316Z_DRVOFF_LOW(hdrv);
}

HAL_StatusTypeDef MCT8316Z_WriteRegister(MCT8316Z_Handle_t *hdrv,
		uint8_t regAddr, uint8_t data) {
	uint8_t tx[2], rx[2];
	uint16_t frame;

	frame = ((uint16_t) (regAddr & 0x3FU) << MCT_ADDR_SHIFT)
			| (data & MCT_DATA_MASK);

	MCT8316Z_BuildFrame(frame, tx);
	return MCT8316Z_SPI_TxRx(hdrv, tx, rx);
}

HAL_StatusTypeDef MCT8316Z_ReadRegister(MCT8316Z_Handle_t *hdrv,
		uint8_t regAddr, uint8_t *pData) {
	uint8_t tx[2], rx[2] = { 0U, 0U };
	uint16_t frame;
	HAL_StatusTypeDef status;

	frame = MCT_RW_READ_BIT | ((uint16_t) (regAddr & 0x3FU) << MCT_ADDR_SHIFT);

	MCT8316Z_BuildFrame(frame, tx);
	status = MCT8316Z_SPI_TxRx(hdrv, tx, rx);

	if (status == HAL_OK) {
		uint16_t rx_frame = ((uint16_t) rx[0] << 8U) | rx[1];
		*pData = (uint8_t) (rx_frame & MCT_DATA_MASK);
	}

	return status;
}

HAL_StatusTypeDef MCT8316Z_UnlockRegister(MCT8316Z_Handle_t *hdrv) {
	return MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_1, 0x03U);
}

HAL_StatusTypeDef MCT8316Z_LockRegister(MCT8316Z_Handle_t *hdrv) {
	return MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_1, 0x06U);
}

HAL_StatusTypeDef MCT8316Z_ApplyDefaultConfig(MCT8316Z_Handle_t *hdrv) {
	HAL_StatusTypeDef status;

	/* CTRL2: SDO push-pull, slew 125V/us, Sync Digital mode, clear faults */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_2, MCT_DEFAULT_CTRL2);
	if (status != HAL_OK)
		return status;

	/* CTRL3: 100% duty at 40kHz support, OVP sel 22V, OVP disabled */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_3, MCT_DEFAULT_CTRL3);
	if (status != HAL_OK)
		return status;

	/* CTRL4: OCP latched, 24A threshold, 0.6us deglitch */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_4, MCT_DEFAULT_CTRL4);
	if (status != HAL_OK)
		return status;

	/* CTRL5: CSA gain 0.6V/A, ASR/AAR disabled */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_5, MCT_DEFAULT_CTRL5);
	if (status != HAL_OK)
		return status;

	/* CTRL6: Buck disabled (not used on this board) */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_6, MCT_DEFAULT_CTRL6);
	if (status != HAL_OK)
		return status;

	/* CTRL7: Hall hysteresis 5mV, brake mode = brake, forward direction */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_7, MCT_DEFAULT_CTRL7);
	if (status != HAL_OK)
		return status;

	/* CTRL8: FG output 1x, motor lock retry 500ms, tdet 500ms, report-only */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_8, MCT_DEFAULT_CTRL8);
	if (status != HAL_OK)
		return status;

	/* CTRL9: No commutation advance */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_9, MCT_DEFAULT_CTRL9);
	if (status != HAL_OK)
		return status;

	/* CTRL10: No commutation advance */
	status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_10, MCT_DEFAULT_CTRL10);
	
	return status;
}

HAL_StatusTypeDef MCT8316Z_ClearFaults(MCT8316Z_Handle_t *hdrv) {
	MCT8316Z_WAKEUP(hdrv);
	HAL_Delay(1U);

	uint8_t reg_val = MCT_DEFAULT_CTRL2 | MCT_CTRL2_CLR_FLT_BIT;

	return MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_2, reg_val);
}

MCT8316Z_REG_Typedef MCT8316Z_VerifyConfig(MCT8316Z_Handle_t *hdrv) {
	HAL_StatusTypeDef status;
	uint8_t read_val = 0U;

	/* --- CTRL2 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_2, &read_val);
	if (status != HAL_OK)
		return MCT_REG_FAULT_CTRL2;
	if ((read_val & 0xFEU) != (MCT_DEFAULT_CTRL2 & 0xFEU))
		return MCT_REG_FAULT_CTRL2;

	/* --- CTRL3 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_3, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL3)
		return MCT_REG_FAULT_CTRL3;

	/* --- CTRL4 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_4, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL4)
		return MCT_REG_FAULT_CTRL4;

	/* --- CTRL5 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_5, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL5)
		return MCT_REG_FAULT_CTRL5;

	/* --- CTRL6 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_6, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL6)
		return MCT_REG_FAULT_CTRL6;

	/* --- CTRL7 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_7, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL7)
		return MCT_REG_FAULT_CTRL7;

	/* --- CTRL8 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_8, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL8)
		return MCT_REG_FAULT_CTRL8;

	/* --- CTRL9 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_9, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL9)
		return MCT_REG_FAULT_CTRL9;

	/* --- CTRL10 --- */
	status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_10, &read_val);
	if (status != HAL_OK || read_val != MCT_DEFAULT_CTRL10)
		return MCT_REG_FAULT_CTRL10;

	return MCT_REG_OK;
}

void MX_MCT8316Z_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = MTR_FGOUT_L_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MTR_FGOUT_L_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	GPIO_InitStruct.Pin = MTR_BRAKE_L_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(MTR_BRAKE_L_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = MTR_FGOUT_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MTR_FGOUT_R_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	GPIO_InitStruct.Pin = MTR_BRAKE_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(MTR_BRAKE_R_GPIO_Port, &GPIO_InitStruct);

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1550);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	/* --- Left motor driver --- */
	MCT8316Z_Init(&MCT8316Z_L, MCT8316Z_SPI,
	MTR_CS_L_GPIO_Port, MTR_CS_L_Pin,
	MTR_nSLEEP_L_GPIO_Port, MTR_nSLEEP_L_Pin,
	MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin,
	MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin);

	/* --- Right motor driver --- */
	MCT8316Z_Init(&MCT8316Z_R, MCT8316Z_SPI,
	MTR_CS_R_GPIO_Port, MTR_CS_R_Pin,
	MTR_nSLEEP_R_GPIO_Port, MTR_nSLEEP_R_Pin,
	MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin,
	MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin);


	/* Wake both drivers*/
	MCT8316Z_WAKEUP(&MCT8316Z_L);
	HAL_Delay(1U);

	/* Apply configuration to left driver */
	MCT8316Z_UnlockRegister(&MCT8316Z_L);
	MCT8316Z_ApplyDefaultConfig(&MCT8316Z_L);
	MCT8316Z_VerifyConfig(&MCT8316Z_L);
	MCT8316Z_LockRegister(&MCT8316Z_L);

	MCT8316Z_WAKEUP(&MCT8316Z_R);
	HAL_Delay(1U);

	/* Apply configuration to right driver */
	MCT8316Z_UnlockRegister(&MCT8316Z_R);
	MCT8316Z_ApplyDefaultConfig(&MCT8316Z_R);
	MCT8316Z_VerifyConfig(&MCT8316Z_R);
	MCT8316Z_LockRegister(&MCT8316Z_R);
}

#endif /* SENSOR_TRAP_CONTROL */
