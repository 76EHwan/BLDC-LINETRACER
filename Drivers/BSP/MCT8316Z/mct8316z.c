/*
 * mct8316z.c
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */
#include "main.h"
#include "mct8316z.h"

#ifdef SENSOR_TRAP_CONTROL

/*=======================================================================*/
/* Internal SPI frame bit definitions                                    */
/*=======================================================================*/
#define MCT_RW_READ_BIT     (1U << 15)
#define MCT_ADDR_SHIFT      (9U)
#define MCT_PARITY_BIT      (1U << 8)
#define MCT_DATA_MASK       (0xFFU)

/*=======================================================================*/
/* Global Handle Instances                                               */
/*=======================================================================*/
MCT8316Z_Handle_t MCT8316Z_L;
MCT8316Z_Handle_t MCT8316Z_R;

/*=======================================================================*/
/* Internal Helper Functions                                             */
/*=======================================================================*/

/**
 * @brief  Calculates even parity over bits [15:0] excluding bit 8.
 *         DRV/MCT series requires the total 1-count (excl. parity bit) to
 *         be even → parity bit set when count is odd.
 */
static uint8_t MCT8316Z_CalcParity(uint16_t frame)
{
    uint8_t ones = 0U;
    frame &= ~MCT_PARITY_BIT; /* exclude parity bit position */

    for (int i = 0; i < 16; i++) {
        if ((frame >> i) & 0x01U) {
            ones++;
        }
    }
    return (ones % 2U); /* 1 → set parity bit to make total even */
}

/**
 * @brief  Performs a 16-bit SPI transaction (2 × 8-bit transfers, MSB first).
 */
static HAL_StatusTypeDef MCT8316Z_SPI_TxRx(MCT8316Z_Handle_t *hdrv,
                                             uint8_t *pTx, uint8_t *pRx)
{
    HAL_StatusTypeDef status;

    MCT8316Z_CS_LOW(hdrv);
    status = HAL_SPI_TransmitReceive(hdrv->hspi, pTx, pRx, 2U, 100U);
    MCT8316Z_CS_HIGH(hdrv);

    return status;
}

/**
 * @brief  Builds a 16-bit SPI frame, appends parity, and splits into 2 bytes.
 */
static void MCT8316Z_BuildFrame(uint16_t raw_frame, uint8_t *pTx)
{
    if (MCT8316Z_CalcParity(raw_frame)) {
        raw_frame |= MCT_PARITY_BIT;
    }
    pTx[0] = (uint8_t)((raw_frame >> 8U) & 0xFFU); /* MSB */
    pTx[1] = (uint8_t)(raw_frame & 0xFFU);          /* LSB */
}

/*=======================================================================*/
/* Public API Implementations                                            */
/*=======================================================================*/

/**
 * @brief  Initializes MCT8316Z handle and sets safe default pin states.
 */
void MCT8316Z_Init(MCT8316Z_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin,
                   GPIO_TypeDef *nSLEEP_Port, uint16_t nSLEEP_Pin,
                   GPIO_TypeDef *nFAULT_Port, uint16_t nFAULT_Pin,
                   GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin)
{
    hdrv->hspi        = hspi;
    hdrv->nCS_Port    = nCS_Port;
    hdrv->nCS_Pin     = nCS_Pin;
    hdrv->nSLEEP_Port = nSLEEP_Port;
    hdrv->nSLEEP_Pin  = nSLEEP_Pin;
    hdrv->nFAULT_Port = nFAULT_Port;
    hdrv->nFAULT_Pin  = nFAULT_Pin;
    hdrv->DRVOFF_Port = DRVOFF_Port;
    hdrv->DRVOFF_Pin  = DRVOFF_Pin;

    MCT8316Z_CS_HIGH(hdrv);    /* deassert chip select */
    MCT8316Z_DRVOFF_LOW(hdrv); /* DRVOFF=0 → normal gate drive operation */
}

/**
 * @brief  Writes 8-bit data to the specified register.
 *         Frame: RW=0 | ADDR[5:0] | P | DATA[7:0]
 */
HAL_StatusTypeDef MCT8316Z_WriteRegister(MCT8316Z_Handle_t *hdrv,
                                          uint8_t regAddr, uint8_t data)
{
    uint8_t tx[2], rx[2];
    uint16_t frame;

    frame = ((uint16_t)(regAddr & 0x3FU) << MCT_ADDR_SHIFT)
            | (data & MCT_DATA_MASK);

    MCT8316Z_BuildFrame(frame, tx);
    return MCT8316Z_SPI_TxRx(hdrv, tx, rx);
}

/**
 * @brief  Reads 8-bit data from the specified register.
 *         Frame: RW=1 | ADDR[5:0] | P | 0x00
 *         Response data is in the lower 8 bits of the reply frame.
 */
HAL_StatusTypeDef MCT8316Z_ReadRegister(MCT8316Z_Handle_t *hdrv,
                                         uint8_t regAddr, uint8_t *pData)
{
    uint8_t tx[2], rx[2] = {0U, 0U};
    uint16_t frame;
    HAL_StatusTypeDef status;

    frame = MCT_RW_READ_BIT
            | ((uint16_t)(regAddr & 0x3FU) << MCT_ADDR_SHIFT);

    MCT8316Z_BuildFrame(frame, tx);
    status = MCT8316Z_SPI_TxRx(hdrv, tx, rx);

    if (status == HAL_OK) {
        uint16_t rx_frame = ((uint16_t)rx[0] << 8U) | rx[1];
        *pData = (uint8_t)(rx_frame & MCT_DATA_MASK);
    }

    return status;
}

/**
 * @brief  Unlocks control registers (CTRL1 = 0x03: unlock key).
 */
HAL_StatusTypeDef MCT8316Z_UnlockRegister(MCT8316Z_Handle_t *hdrv)
{
    return MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_1, 0x03U);
}

/**
 * @brief  Locks control registers (CTRL1 = 0x06: lock key).
 */
HAL_StatusTypeDef MCT8316Z_LockRegister(MCT8316Z_Handle_t *hdrv)
{
    return MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_1, 0x06U);
}

/**
 * @brief  Applies the default configuration for trapezoidal control.
 *         Call after wakeup and register unlock.
 */
HAL_StatusTypeDef MCT8316Z_ApplyDefaultConfig(MCT8316Z_Handle_t *hdrv)
{
    HAL_StatusTypeDef status;
    uint8_t reg_val;

    /* CTRL2: SDO push-pull, slew 125V/us, 3x PWM mode, clear faults */
    reg_val = MCT_CTRL2_SDO_MODE_PP
            | MCT_CTRL2_SLEW_125V_US
            | MCT_CTRL2_PWM_SYN_DIGITAL
            | MCT_CTRL2_CLR_FLT_BIT;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_2, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL3: 100% duty at 40kHz support, OVP sel 22V, OVP disabled */
    reg_val = MCT_CTRL3_PWM_100_DUTY_40KHZ | MCT_CTRL3_OVP_SEL_22V;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_3, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL4: OCP report-only, 24A threshold, 0.6us deglitch */
    reg_val = MCT_CTRL4_OCP_MODE_REPORT
            | MCT_CTRL4_OCP_LVL_24A
            | MCT_CTRL4_OCP_DEG_0_6US;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_4, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL5: CSA gain 0.6V/A, ASR/AAR enabled */
    reg_val = MCT_CTRL5_CSA_GAIN_0_6VA
            | MCT_CTRL5_EN_ASR_DIS
            | MCT_CTRL5_EN_AAR_DIS;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_5, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL6: Buck disabled (not used on this board) */
    reg_val = MCT_CTRL6_BUCK_PS_DIS
            | MCT_CTRL6_BUCK_SEL_5V
            | MCT_CTRL6_BUCK_DIS;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_6, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL7: Hall hysteresis 5mV, brake mode = brake, forward direction */
    reg_val = MCT_CTRL7_HALL_HYS_5MV
            | MCT_CTRL7_BRAKE_MODE_BRAKE
            | MCT_CTRL7_COAST_DIS
            | MCT_CTRL7_BRAKE_DIS
            | MCT_CTRL7_DIR_FWD;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_7, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL8: FG output 1x, motor lock retry 500ms, tdet 500ms, report-only */
    reg_val = MCT_CTRL8_FGOUT_SEL_1X
            | MCT_CTRL8_MTR_LOCK_RETRY_500MS
            | MCT_CTRL8_MTR_LOCK_TDET_500MS
            | MCT_CTRL8_MTR_LOCK_MODE_REPORT;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_8, reg_val);
    if (status != HAL_OK) return status;

    /* CTRL9: No commutation advance */
    reg_val = MCT_CTRL9_MTR_ADVANCE_LVL_0DEG;
    status = MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_9, reg_val);

    return status;
}

/**
 * @brief  Clears all latched fault flags via CLR_FLT bit in CTRL2.
 *         Also re-asserts nSLEEP to exit any sleep state first.
 */
HAL_StatusTypeDef MCT8316Z_ClearFaults(MCT8316Z_Handle_t *hdrv)
{
    /* Ensure device is awake before clearing */
    MCT8316Z_WAKEUP(hdrv);
    HAL_Delay(1U); /* t_wake: 1ms margin */

    uint8_t reg_val = MCT_CTRL2_SDO_MODE_PP
                    | MCT_CTRL2_SLEW_125V_US
                    | MCT_CTRL2_PWM_MODE_3X
                    | MCT_CTRL2_CLR_FLT_BIT;

    return MCT8316Z_WriteRegister(hdrv, MCT_REG_CTRL_2, reg_val);
}

/**
 * @brief  Reads back all configured registers and verifies against expected values.
 * @return MCT_REG_OK if all match, otherwise the faulting register identifier.
 * @note   CLR_FLT (bit 0 of CTRL2) is self-clearing, so it is masked out
 *         when comparing CTRL2 read-back.
 */
MCT8316Z_REG_Typedef MCT8316Z_VerifyConfig(MCT8316Z_Handle_t *hdrv)
{
    HAL_StatusTypeDef status;
    uint8_t read_val  = 0U;
    uint8_t expected  = 0U;

    /* --- CTRL2 --- */
    expected = MCT_CTRL2_SDO_MODE_PP | MCT_CTRL2_SLEW_125V_US | MCT_CTRL2_PWM_MODE_3X;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_2, &read_val);
    if (status != HAL_OK) return MCT_REG_FAULT_CTRL2;
    if ((read_val & 0xFEU) != (expected & 0xFEU)) return MCT_REG_FAULT_CTRL2; /* mask CLR_FLT */

    /* --- CTRL3 --- */
    expected = MCT_CTRL3_PWM_100_DUTY_40KHZ | MCT_CTRL3_OVP_SEL_22V;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_3, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL3;

    /* --- CTRL4 --- */
    expected = MCT_CTRL4_OCP_MODE_REPORT | MCT_CTRL4_OCP_LVL_24A | MCT_CTRL4_OCP_DEG_0_6US;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_4, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL4;

    /* --- CTRL5 --- */
    expected = MCT_CTRL5_CSA_GAIN_0_6VA | MCT_CTRL5_EN_ASR_EN | MCT_CTRL5_EN_AAR_EN;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_5, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL5;

    /* --- CTRL6 --- */
    expected = MCT_CTRL6_BUCK_PS_DIS | MCT_CTRL6_BUCK_SEL_5V | MCT_CTRL6_BUCK_DIS;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_6, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL6;

    /* --- CTRL7 --- */
    expected = MCT_CTRL7_HALL_HYS_5MV | MCT_CTRL7_BRAKE_MODE_BRAKE
             | MCT_CTRL7_COAST_DIS | MCT_CTRL7_BRAKE_DIS | MCT_CTRL7_DIR_FWD;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_7, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL7;

    /* --- CTRL8 --- */
    expected = MCT_CTRL8_FGOUT_SEL_1X | MCT_CTRL8_MTR_LOCK_RETRY_500MS
             | MCT_CTRL8_MTR_LOCK_TDET_500MS | MCT_CTRL8_MTR_LOCK_MODE_REPORT;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_8, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL8;

    /* --- CTRL9 --- */
    expected = MCT_CTRL9_MTR_ADVANCE_LVL_0DEG;
    status = MCT8316Z_ReadRegister(hdrv, MCT_REG_CTRL_9, &read_val);
    if (status != HAL_OK || read_val != expected) return MCT_REG_FAULT_CTRL9;

    return MCT_REG_OK;
}

/**
 * @brief  Top-level init: creates both L/R handle instances, wakes the device,
 *         unlocks registers, applies config, verifies, then locks.
 * @note   GPIO and SPI handles must be initialized by CubeMX before calling this.
 */
void MX_MCT8316Z_Init(void)
{
    /* --- Left motor driver --- */
    MCT8316Z_Init(&MCT8316Z_L, MCT8316Z_SPI,
                  MTR_CS_L_GPIO_Port, MTR_CS_R,
                  MTR_nSLEEP_GPIO_Port,   MTR_nSLEEP_Pin,
                  MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin,
                  MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin);

    /* --- Right motor driver --- */
    MCT8316Z_Init(&MCT8316Z_R, MCT8316Z_SPI,
                  MTR_CS_R_GPIO_Port,    MTR_CS_R_Pin,
                  MTR_nSLEEP_GPIO_Port,   MTR_nSLEEP_Pin,
                  MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin,
                  MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin);

    /* Wake both drivers (shared nSLEEP line) */
    MCT8316Z_WAKEUP(&MCT8316Z_L);
    HAL_Delay(1U); /* t_wake */

    /* Apply configuration to left driver */
    MCT8316Z_UnlockRegister(&MCT8316Z_L);
    MCT8316Z_ApplyDefaultConfig(&MCT8316Z_L);
    MCT8316Z_VerifyConfig(&MCT8316Z_L);
    MCT8316Z_LockRegister(&MCT8316Z_L);

    /* Apply configuration to right driver */
    MCT8316Z_UnlockRegister(&MCT8316Z_R);
    MCT8316Z_ApplyDefaultConfig(&MCT8316Z_R);
    MCT8316Z_VerifyConfig(&MCT8316Z_R);
    MCT8316Z_LockRegister(&MCT8316Z_R);
}

#endif /* SENSOR_TRAP_CONTROL */
