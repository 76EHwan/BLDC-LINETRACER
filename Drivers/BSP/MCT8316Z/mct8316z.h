/*
 * mct8316z.h
 *
 * Created on: 2026. 4. 16.
 * Author: kth59
 */

#include "user_init.h"

#ifndef BSP_MCT8316Z_MCT8316Z_H_
#define BSP_MCT8316Z_MCT8316Z_H_

#ifdef SENSOR_TRAP_CONTROL

#include "main.h"
#include "spi.h"
#include "tim.h"

#define MCT8316Z_SPI    &hspi2
#define MCT8316Z_L_TIM  &htim3
#define MCT8316Z_R_TIM  &htim8

/* Macros for manual nCS pin control */
#define MCT8316Z_CS_LOW(hdrv)      HAL_GPIO_WritePin((hdrv)->nCS_Port, (hdrv)->nCS_Pin, GPIO_PIN_RESET)
#define MCT8316Z_CS_HIGH(hdrv)     HAL_GPIO_WritePin((hdrv)->nCS_Port, (hdrv)->nCS_Pin, GPIO_PIN_SET)

/* Macros for manual nSLEEP pin control */
#define MCT8316Z_WAKEUP(hdrv)      HAL_GPIO_WritePin((hdrv)->nSLEEP_Port, (hdrv)->nSLEEP_Pin, GPIO_PIN_SET)
#define MCT8316Z_SLEEP(hdrv)       HAL_GPIO_WritePin((hdrv)->nSLEEP_Port, (hdrv)->nSLEEP_Pin, GPIO_PIN_RESET)

/* Macros for manual DRVOFF pin control */
#define MCT8316Z_DRVOFF_LOW(hdrv)  HAL_GPIO_WritePin((hdrv)->DRVOFF_Port, (hdrv)->DRVOFF_Pin, GPIO_PIN_RESET)
#define MCT8316Z_DRVOFF_HIGH(hdrv) HAL_GPIO_WritePin((hdrv)->DRVOFF_Port, (hdrv)->DRVOFF_Pin, GPIO_PIN_SET)

/*=======================================================================*/
/* SPI Frame Definition                                                  */
/* 16-bit Frame: [RW(B15)] | [A5-A0(B14-B9)] | [P(B8)] | [D7-D0(B7-B0)] */
/*=======================================================================*/
#define MCT_SPI_READ_BIT        (1U << 15)
#define MCT_SPI_ADDR_SHIFT      (9U)
#define MCT_SPI_ADDR_MASK       (0x3FU << MCT_SPI_ADDR_SHIFT)
#define MCT_SPI_PARITY_BIT      (1U << 8)
#define MCT_SPI_DATA_MASK       (0x00FFU)

/*=======================================================================*/
/* MCT8316Z Register Address Map                                         */
/*=======================================================================*/
#define MCT_REG_IC_STATUS   (0x00U)
#define MCT_REG_STATUS_1    (0x01U)
#define MCT_REG_STATUS_2    (0x02U)
#define MCT_REG_CTRL_1      (0x03U)
#define MCT_REG_CTRL_2      (0x04U)
#define MCT_REG_CTRL_3      (0x05U)
#define MCT_REG_CTRL_4      (0x06U)
#define MCT_REG_CTRL_5      (0x07U)
#define MCT_REG_CTRL_6      (0x08U)
#define MCT_REG_CTRL_7      (0x09U)
#define MCT_REG_CTRL_8      (0x0AU)
#define MCT_REG_CTRL_9      (0x0BU)
#define MCT_REG_CTRL_10     (0x0CU)

/*=======================================================================*/
/* IC Status Register (Offset = 0h)                                      */
/*=======================================================================*/
#define MCT_IC_STATUS_MTR_LOCK  (1U << 7)
#define MCT_IC_STATUS_BK_FLT    (1U << 6)
#define MCT_IC_STATUS_SPI_FLT   (1U << 5)
#define MCT_IC_STATUS_OCP       (1U << 4)
#define MCT_IC_STATUS_NPOR      (1U << 3)
#define MCT_IC_STATUS_OVP       (1U << 2)
#define MCT_IC_STATUS_OT        (1U << 1)
#define MCT_IC_STATUS_FAULT     (1U << 0)

/*=======================================================================*/
/* Status Register 1 (Offset = 1h)                                       */
/*=======================================================================*/
#define MCT_STATUS1_OTW     (1U << 7)
#define MCT_STATUS1_OTS     (1U << 6)
#define MCT_STATUS1_OCP_HC  (1U << 5)
#define MCT_STATUS1_OCP_LC  (1U << 4)
#define MCT_STATUS1_OCP_HB  (1U << 3)
#define MCT_STATUS1_OCP_LB  (1U << 2)
#define MCT_STATUS1_OCP_HA  (1U << 1)
#define MCT_STATUS1_OCP_LA  (1U << 0)

/*=======================================================================*/
/* Status Register 2 (Offset = 2h)                                       */
/*=======================================================================*/
#define MCT_STATUS2_OTP_ERR         (1U << 6)
#define MCT_STATUS2_BUCK_OCP        (1U << 5)
#define MCT_STATUS2_BUCK_UV         (1U << 4)
#define MCT_STATUS2_VCP_UV          (1U << 3)
#define MCT_STATUS2_SPI_PARITY      (1U << 2)
#define MCT_STATUS2_SPI_SCLK_FLT    (1U << 1)
#define MCT_STATUS2_SPI_ADDR_FLT    (1U << 0)

/*=======================================================================*/
/* Control Register 2 (Offset = 4h)                                      */
/*=======================================================================*/
#define MCT_CTRL2_SDO_BASE          	(5U)
#define MCT_CTRL2_SDO_MODE_OD       	(0U << MCT_CTRL2_SDO_BASE)
#define MCT_CTRL2_SDO_MODE_PP       	(1U << MCT_CTRL2_SDO_BASE)
#define MCT_CTRL2_SLEW_BASE         	(3U)
#define MCT_CTRL2_SLEW_25V_US       	(0U << MCT_CTRL2_SLEW_BASE)
#define MCT_CTRL2_SLEW_50V_US       	(1U << MCT_CTRL2_SLEW_BASE)
#define MCT_CTRL2_SLEW_125V_US      	(2U << MCT_CTRL2_SLEW_BASE)
#define MCT_CTRL2_SLEW_200V_US      	(3U << MCT_CTRL2_SLEW_BASE)
#define MCT_CTRL2_PWM_MODE_BASE     	(1U)
#define MCT_CTRL2_PWM_MODE_ASYN_ANALOG	(0U << MCT_CTRL2_PWM_MODE_BASE)
#define MCT_CTRL2_PWM_MODE_ASYN_DIGITAL (1U << MCT_CTRL2_PWM_MODE_BASE)
#define MCT_CTRL2_PWM_MODE_SYN_ANALOG	(2U << MCT_CTRL2_PWM_MODE_BASE)
#define MCT_CTRL2_PWM_MODE_SYN_DIGITAL	(3U << MCT_CTRL2_PWM_MODE_BASE)
#define MCT_CTRL2_CLR_FLT_BIT       	(1U << 0)

/*=======================================================================*/
/* Control Register 3 (Offset = 5h)                                      */
/*=======================================================================*/
#define MCT_CTRL3_PWM_100_DUTY_40KHZ    (1U << 4)
#define MCT_CTRL3_OVP_SEL_22V           (1U << 3)
#define MCT_CTRL3_OVP_EN                (1U << 2)
#define MCT_CTRL3_SPI_FLT_REP           (1U << 1)
#define MCT_CTRL3_OTW_REP               (1U << 0)

/*=======================================================================*/
/* Control Register 4 (Offset = 6h)                                      */
/*=======================================================================*/
#define MCT_CTRL4_DRVOFF_BASE           (6U)
#define MCT_CTRL4_DRVOFF_NO_ACTION      (0U << MCT_CTRL4_DRVOFF_BASE)
#define MCT_CTRL4_DRVOFF_STANDBY        (1U << MCT_CTRL4_DRVOFF_BASE)
#define MCT_CTRL4_OCP_CBC_BASE          (5U)
#define MCT_CTRL4_OCP_CBC_DIS           (0U << MCT_CTRL4_OCP_CBC_BASE)
#define MCT_CTRL4_OCP_CBC_EN            (1U << MCT_CTRL4_OCP_CBC_BASE)
#define MCT_CTRL4_OCP_DEG_BASE          (3U)
#define MCT_CTRL4_OCP_DEG_0_2US         (0U << MCT_CTRL4_OCP_DEG_BASE)
#define MCT_CTRL4_OCP_DEG_0_6US         (1U << MCT_CTRL4_OCP_DEG_BASE)
#define MCT_CTRL4_OCP_DEG_1_25US        (2U << MCT_CTRL4_OCP_DEG_BASE)
#define MCT_CTRL4_OCP_DEG_1_6US         (3U << MCT_CTRL4_OCP_DEG_BASE)
#define MCT_CTRL4_OCP_RETRY_BASE        (2U)
#define MCT_CTRL4_OCP_RETRY_5MS         (0U << MCT_CTRL4_OCP_RETRY_BASE)
#define MCT_CTRL4_OCP_RETRY_500MS       (1U << MCT_CTRL4_OCP_RETRY_BASE)
#define MCT_CTRL4_OCP_LVL_BASE          (1U)
#define MCT_CTRL4_OCP_LVL_16A           (0U << MCT_CTRL4_OCP_LVL_BASE)
#define MCT_CTRL4_OCP_LVL_24A           (1U << MCT_CTRL4_OCP_LVL_BASE)
#define MCT_CTRL4_OCP_MODE_BASE         (0U)
#define MCT_CTRL4_OCP_MODE_LATCH        (0U << MCT_CTRL4_OCP_MODE_BASE)
#define MCT_CTRL4_OCP_MODE_RETRY        (1U << MCT_CTRL4_OCP_MODE_BASE)

/*=======================================================================*/
/* Control Register 5 (Offset = 7h)                                      */
/*=======================================================================*/
#define MCT_CTRL5_ILIM_RECIR_BASE       (6U)
#define MCT_CTRL5_ILIM_RECIR_BRAKE      (0U << MCT_CTRL5_ILIM_RECIR_BASE)
#define MCT_CTRL5_ILIM_RECIR_COAST      (1U << MCT_CTRL5_ILIM_RECIR_BASE)
#define MCT_CTRL5_EN_AAR_BASE           (3U)
#define MCT_CTRL5_EN_AAR_DIS            (0U << MCT_CTRL5_EN_AAR_BASE)
#define MCT_CTRL5_EN_AAR_EN             (1U << MCT_CTRL5_EN_AAR_BASE)
#define MCT_CTRL5_EN_ASR_BASE           (2U)
#define MCT_CTRL5_EN_ASR_DIS            (0U << MCT_CTRL5_EN_ASR_BASE)
#define MCT_CTRL5_EN_ASR_EN             (1U << MCT_CTRL5_EN_ASR_BASE)
#define MCT_CTRL5_CSA_GAIN_BASE         (0U)
#define MCT_CTRL5_CSA_GAIN_0_15VA       (0U << MCT_CTRL5_CSA_GAIN_BASE)
#define MCT_CTRL5_CSA_GAIN_0_3VA        (1U << MCT_CTRL5_CSA_GAIN_BASE)
#define MCT_CTRL5_CSA_GAIN_0_6VA        (2U << MCT_CTRL5_CSA_GAIN_BASE)
#define MCT_CTRL5_CSA_GAIN_1_2VA        (3U << MCT_CTRL5_CSA_GAIN_BASE)

/*=======================================================================*/
/* Control Register 6 (Offset = 8h)                                      */
/*=======================================================================*/
#define MCT_CTRL6_BUCK_PS_BASE      (4U)
#define MCT_CTRL6_BUCK_PS_EN        (0U << MCT_CTRL6_BUCK_PS_BASE)
#define MCT_CTRL6_BUCK_PS_DIS       (1U << MCT_CTRL6_BUCK_PS_BASE)
#define MCT_CTRL6_BUCK_CL_BASE      (3U)
#define MCT_CTRL6_BUCK_CL_600MA     (0U << MCT_CTRL6_BUCK_CL_BASE)
#define MCT_CTRL6_BUCK_CL_150MA     (1U << MCT_CTRL6_BUCK_CL_BASE)
#define MCT_CTRL6_BUCK_SEL_BASE     (1U)
#define MCT_CTRL6_BUCK_SEL_3V3      (0U << MCT_CTRL6_BUCK_SEL_BASE)
#define MCT_CTRL6_BUCK_SEL_5V       (1U << MCT_CTRL6_BUCK_SEL_BASE)
#define MCT_CTRL6_BUCK_SEL_4V       (2U << MCT_CTRL6_BUCK_SEL_BASE)
#define MCT_CTRL6_BUCK_SEL_5V7      (3U << MCT_CTRL6_BUCK_SEL_BASE)
#define MCT_CTRL6_BUCK_BASE         (0U)
#define MCT_CTRL6_BUCK_EN           (0U << MCT_CTRL6_BUCK_BASE)
#define MCT_CTRL6_BUCK_DIS          (1U << MCT_CTRL6_BUCK_BASE)

/*=======================================================================*/
/* Control Register 7 (Offset = 9h)                                      */
/*=======================================================================*/
#define MCT_CTRL7_HALL_HYS_BASE         (4U)
#define MCT_CTRL7_HALL_HYS_5MV          (0U << MCT_CTRL7_HALL_HYS_BASE)
#define MCT_CTRL7_HALL_HYS_50MV         (1U << MCT_CTRL7_HALL_HYS_BASE)
#define MCT_CTRL7_BRAKE_MODE_BASE       (3U)
#define MCT_CTRL7_BRAKE_MODE_BRAKE      (0U << MCT_CTRL7_BRAKE_MODE_BASE)
#define MCT_CTRL7_BRAKE_MODE_COAST      (1U << MCT_CTRL7_BRAKE_MODE_BASE)
#define MCT_CTRL7_COAST_BASE            (2U)
#define MCT_CTRL7_COAST_DIS             (0U << MCT_CTRL7_COAST_BASE)
#define MCT_CTRL7_COAST_EN              (1U << MCT_CTRL7_COAST_BASE)
#define MCT_CTRL7_BRAKE_BASE            (1U)
#define MCT_CTRL7_BRAKE_DIS             (0U << MCT_CTRL7_BRAKE_BASE)
#define MCT_CTRL7_BRAKE_EN              (1U << MCT_CTRL7_BRAKE_BASE)
#define MCT_CTRL7_DIR_BASE              (0U)
#define MCT_CTRL7_DIR_FWD               (0U << MCT_CTRL7_DIR_BASE)
#define MCT_CTRL7_DIR_REV               (1U << MCT_CTRL7_DIR_BASE)

/*=======================================================================*/
/* Control Register 8 (Offset = Ah)                                      */
/*=======================================================================*/
#define MCT_CTRL8_FGOUT_SEL_BASE        (6U)
#define MCT_CTRL8_FGOUT_SEL_3X          (0U << MCT_CTRL8_FGOUT_SEL_BASE)
#define MCT_CTRL8_FGOUT_SEL_1X          (1U << MCT_CTRL8_FGOUT_SEL_BASE)
#define MCT_CTRL8_FGOUT_SEL_0_5X        (2U << MCT_CTRL8_FGOUT_SEL_BASE)
#define MCT_CTRL8_FGOUT_SEL_0_25X       (3U << MCT_CTRL8_FGOUT_SEL_BASE)
#define MCT_CTRL8_MTR_LOCK_RETRY_BASE   (4U)
#define MCT_CTRL8_MTR_LOCK_RETRY_500MS  (0U << MCT_CTRL8_MTR_LOCK_RETRY_BASE)
#define MCT_CTRL8_MTR_LOCK_RETRY_5000MS (1U << MCT_CTRL8_MTR_LOCK_RETRY_BASE)
#define MCT_CTRL8_MTR_LOCK_TDET_BASE    (2U)
#define MCT_CTRL8_MTR_LOCK_TDET_300MS   (0U << MCT_CTRL8_MTR_LOCK_TDET_BASE)
#define MCT_CTRL8_MTR_LOCK_TDET_500MS   (1U << MCT_CTRL8_MTR_LOCK_TDET_BASE)
#define MCT_CTRL8_MTR_LOCK_TDET_1000MS  (2U << MCT_CTRL8_MTR_LOCK_TDET_BASE)
#define MCT_CTRL8_MTR_LOCK_TDET_5000MS  (3U << MCT_CTRL8_MTR_LOCK_TDET_BASE)
#define MCT_CTRL8_MTR_LOCK_MODE_BASE    (0U)
#define MCT_CTRL8_MTR_LOCK_MODE_LATCH   (0U << MCT_CTRL8_MTR_LOCK_MODE_BASE)
#define MCT_CTRL8_MTR_LOCK_MODE_RETRY   (1U << MCT_CTRL8_MTR_LOCK_MODE_BASE)
#define MCT_CTRL8_MTR_LOCK_MODE_REPORT  (2U << MCT_CTRL8_MTR_LOCK_MODE_BASE)
#define MCT_CTRL8_MTR_LOCK_MODE_NO_ACT  (3U << MCT_CTRL8_MTR_LOCK_MODE_BASE)

/*=======================================================================*/
/* Control Register 9 (Offset = Bh)                                      */
/*=======================================================================*/
#define MCT_CTRL9_MTR_ADVANCE_LVL_BASE      (0U)
#define MCT_CTRL9_MTR_ADVANCE_LVL_0DEG      (0U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_4DEG      (1U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_7DEG      (2U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_11DEG     (3U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_15DEG     (4U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_20DEG     (5U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_25DEG     (6U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)
#define MCT_CTRL9_MTR_ADVANCE_LVL_30DEG     (7U << MCT_CTRL9_MTR_ADVANCE_LVL_BASE)

/*=======================================================================*/
/* Control Register 10 (Offset = Ch)                                     */
/*=======================================================================*/
#define MCT_CTRL10_DLYCMP_BASE          (4U)
#define MCT_CTRL10_DLYCMP_DIS           (0U << MCT_CTRL10_DLYCMP_BASE)
#define MCT_CTRL10_DLYCMP_EN            (1U << MCT_CTRL10_DLYCMP_BASE)
#define MCT_CTRL10_DLY_TARGET_BASE      (0U)
#define MCT_CTRL10_DLY_TARGET_0US       (0x0U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_0_4US     (0x1U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_0_6US     (0x2U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_0_8US     (0x3U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_1US       (0x4U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_1_2US     (0x5U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_1_4US     (0x6U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_1_6US     (0x7U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_1_8US     (0x8U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_2US       (0x9U << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_2_2US     (0xAU << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_2_4US     (0xBU << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_2_6US     (0xCU << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_2_8US     (0xDU << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_3US       (0xEU << MCT_CTRL10_DLY_TARGET_BASE)
#define MCT_CTRL10_DLY_TARGET_3_2US     (0xFU << MCT_CTRL10_DLY_TARGET_BASE)

/*=======================================================================*/
/* Driver Handle Structure                                               */
/*=======================================================================*/
typedef enum {
    MCT_REG_OK,
    MCT_REG_FAULT_CTRL1,
    MCT_REG_FAULT_CTRL2,
    MCT_REG_FAULT_CTRL3,
    MCT_REG_FAULT_CTRL4,
    MCT_REG_FAULT_CTRL5,
    MCT_REG_FAULT_CTRL6,
    MCT_REG_FAULT_CTRL7,
    MCT_REG_FAULT_CTRL8,
    MCT_REG_FAULT_CTRL9,
    MCT_REG_FAULT_CTRL10
} MCT8316Z_REG_Typedef;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *nCS_Port;
    uint16_t           nCS_Pin;
    GPIO_TypeDef      *nSLEEP_Port;
    uint16_t           nSLEEP_Pin;
    GPIO_TypeDef      *DRVOFF_Port;
    uint16_t           DRVOFF_Pin;
    GPIO_TypeDef      *nFAULT_Port;
    uint16_t           nFAULT_Pin;
} MCT8316Z_Handle_t;

extern MCT8316Z_Handle_t MCT8316Z_L;
extern MCT8316Z_Handle_t MCT8316Z_R;

/*=======================================================================*/
/* Function Prototypes                                                   */
/*=======================================================================*/
void MCT8316Z_Init(MCT8316Z_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin,
                   GPIO_TypeDef *nSLEEP_Port, uint16_t nSLEEP_Pin,
                   GPIO_TypeDef *nFAULT_Port, uint16_t nFAULT_Pin,
                   GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin);

HAL_StatusTypeDef MCT8316Z_WriteRegister(MCT8316Z_Handle_t *hdrv, uint8_t regAddr, uint8_t data);
HAL_StatusTypeDef MCT8316Z_ReadRegister(MCT8316Z_Handle_t *hdrv, uint8_t regAddr, uint8_t *pData);
HAL_StatusTypeDef MCT8316Z_UnlockRegister(MCT8316Z_Handle_t *hdrv);
HAL_StatusTypeDef MCT8316Z_LockRegister(MCT8316Z_Handle_t *hdrv);
HAL_StatusTypeDef MCT8316Z_ApplyDefaultConfig(MCT8316Z_Handle_t *hdrv);
HAL_StatusTypeDef MCT8316Z_ClearFaults(MCT8316Z_Handle_t *hdrv);
MCT8316Z_REG_Typedef MCT8316Z_VerifyConfig(MCT8316Z_Handle_t *hdrv);

void MX_MCT8316Z_Init(void);

#endif /* SENSOR_TRAP_CONTROL */
#endif /* BSP_MCT8316Z_MCT8316Z_H_ */
