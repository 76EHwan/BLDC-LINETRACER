/*
 * drv8316crq1.h
 *
 * Created on: Nov 10, 2025
 * Author: kth59
 */
#include "user_init.h"

#ifndef DEVICES_INC_DRV8316CRQ1_H_
#define DEVICES_INC_DRV8316CRQ1_H_

#ifdef FOC_CONTROL

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "adc.h"
#include "lptim.h"

#define DRV8316C_SPI &hspi2
#define DRV8316C_L_TIM &htim3
#define DRV8316C_L_ADC &hadc2
#define DRV8316C_L_ENC &lptim2
#define DRV8316C_R_TIM &htim4
#define DRV8316C_R_ADC &hadc1
#define DRV8316C_R_ENC &lptim1

#define ADC_READ_TIMING	4

#define DRV8316C_CS_LOW(hdrv)      	HAL_GPIO_WritePin((hdrv)->nCS_Port, (hdrv)->nCS_Pin, GPIO_PIN_RESET)
#define DRV8316C_CS_HIGH(hdrv)     	HAL_GPIO_WritePin((hdrv)->nCS_Port, (hdrv)->nCS_Pin, GPIO_PIN_SET)

#define DRV8316C_WAKEUP(hdrv)      	HAL_GPIO_WritePin((hdrv)->nSLEEP_Port, (hdrv)->nSLEEP_Pin, GPIO_PIN_SET)
#define DRV8316C_SLEEP(hdrv)       	HAL_GPIO_WritePin((hdrv)->nSLEEP_Port, (hdrv)->nSLEEP_Pin, GPIO_PIN_RESET)

#define DRV8316C_DRVOFF_LOW(hdrv)  	HAL_GPIO_WritePin((hdrv)->DRVOFF_Port, (hdrv)->DRVOFF_Pin, GPIO_PIN_RESET)
#define DRV8316C_DRVOFF_HIGH(hdrv) 	HAL_GPIO_WritePin((hdrv)->DRVOFF_Port, (hdrv)->DRVOFF_Pin, GPIO_PIN_SET)

#define DRV8316C_FOC_PWM_EN()		HAL_GPIO_WritePin(MTR_PWM_INLX_GPIO_Port, MTR_PWM_INLX_Pin, GPIO_PIN_SET);
#define DRV8316C_FOC_PWM_DIS()		HAL_GPIO_WritePin(MTR_PWM_INLX_GPIO_Port, MTR_PWM_INLX_Pin, GPIO_PIN_RESET);

#define DRV_SPI_READ_MASK       (0x8000)
#define DRV_SPI_WRITE_MASK      (0x0000)
#define DRV_SPI_ADDR_SHIFT      (9)
#define DRV_SPI_ADDR_MASK       (0x3F << DRV_SPI_ADDR_SHIFT)
#define DRV_SPI_PARITY_BIT      (1 << 8)
#define DRV_SPI_DATA_MASK       (0x00FF)

#define DRV_REG_IC_STATUS       (0x00)
#define DRV_REG_STATUS_1        (0x01)
#define DRV_REG_STATUS_2        (0x02)

#define DRV_REG_CTRL_1          (0x03)
#define DRV_REG_CTRL_2          (0x04)
#define DRV_REG_CTRL_3          (0x05)
#define DRV_REG_CTRL_4          (0x06)
#define DRV_REG_CTRL_5          (0x07)
#define DRV_REG_CTRL_6          (0x08)
#define DRV_REG_CTRL_10         (0x0C)

#define DRV_IC_STATUS_BK_FLT    (1 << 6)
#define DRV_IC_STATUS_SPI_FLT   (1 << 5)
#define DRV_IC_STATUS_OCP       (1 << 4)
#define DRV_IC_STATUS_NPOR      (1 << 3)
#define DRV_IC_STATUS_OVP       (1 << 2)
#define DRV_IC_STATUS_OT        (1 << 1)
#define DRV_IC_STATUS_FAULT     (1 << 0)

#define DRV_STATUS1_OTW         (1 << 7)
#define DRV_STATUS1_OTS         (1 << 6)
#define DRV_STATUS1_OCP_HC      (1 << 5)
#define DRV_STATUS1_OCP_LC      (1 << 4)
#define DRV_STATUS1_OCP_HB      (1 << 3)
#define DRV_STATUS1_OCP_LB      (1 << 2)
#define DRV_STATUS1_OCP_HA      (1 << 1)
#define DRV_STATUS1_OCP_LA      (1 << 0)

#define DRV_STATUS2_OTP_ERR     	(1 << 6)
#define DRV_STATUS2_BUCK_OCP    	(1 << 5)
#define DRV_STATUS2_BUCK_UV     	(1 << 4)
#define DRV_STATUS2_VCP_UV      	(1 << 3)
#define DRV_STATUS2_SPI_PARITY  	(1 << 2)
#define DRV_STATUS2_SPI_SCLK_FLT 	(1 << 1)
#define DRV_STATUS2_SPI_ADDR_FLT 	(1 << 0)

#define DRV_cTRL2_SDO_BASE			(5)
#define DRV_CTRL2_SDO_MODE_OD   	(0 << DRV_cTRL2_SDO_BASE)
#define DRV_CTRL2_SDO_MODE_PP   	(1 << DRV_cTRL2_SDO_BASE)
#define DRV_CTRL2_SLEW_BASE			(3)
#define DRV_CTRL2_SLEW_25V_us   	(0 << DRV_CTRL2_SLEW_BASE)
#define DRV_CTRL2_SLEW_50V_us   	(1 << DRV_CTRL2_SLEW_BASE)
#define DRV_CTRL2_SLEW_125V_us  	(2 << DRV_CTRL2_SLEW_BASE)
#define DRV_CTRL2_SLEW_200V_us  	(3 << DRV_CTRL2_SLEW_BASE)
#define DRV_CTRL2_PWM_MODE_BASE 	(1)
#define DRV_CTRL2_PWM_MODE_6X   	(0 << DRV_CTRL2_PWM_MODE_BASE)
#define DRV_CTRL2_PWM_MODE_6X_CL	(1 << DRV_CTRL2_PWM_MODE_BASE)
#define DRV_CTRL2_PWM_MODE_3X   	(2 << DRV_CTRL2_PWM_MODE_BASE)
#define DRV_CTRL2_PWM_MODE_3X_CL 	(3 << DRV_CTRL2_PWM_MODE_BASE)
#define DRV_CTRL2_CLR_FLT_BASE		(0)
#define DRV_CTRL2_CLR_FLT_BIT   	(1 << DRV_CTRL2_CLR_FLT_BASE)

#define DRV_CTRL3_PWM_100_DUTY_SEL_BASE		(4)
#define DRV_CTRL3_PWM_100_DUTY_SEL_20KHZ  	(0 << DRV_CTRL3_PWM_100_DUTY_SEL_BASE)
#define DRV_CTRL3_PWM_100_DUTY_SEL_40KHZ  	(1 << DRV_CTRL3_PWM_100_DUTY_SEL_BASE)
#define DRV_CTRL3_OVP_SEL_BASE   			(3)
#define DRV_CTRL3_OVP_SEL_34V   			(0 << DRV_CTRL3_OVP_SEL_BASE)
#define DRV_CTRL3_OVP_SEL_22V   			(1 << DRV_CTRL3_OVP_SEL_BASE)
#define DRV_CTRL3_OVP_EN_BASE        		(2)
#define DRV_CTRL3_OVP_EN_DIS        		(0 << DRV_CTRL3_OVP_EN_BASE)
#define DRV_CTRL3_OVP_EN_EN	        		(1 << DRV_CTRL3_OVP_EN_BASE)
#define DRV_CTRL3_SPI_FLT_REP_BASE 			(1)
#define DRV_CTRL3_SPI_FLT_REP_EN   			(0 << DRV_CTRL3_SPI_FLT_REP_BASE)
#define DRV_CTRL3_SPI_FLT_REP_DIS  			(1 << DRV_CTRL3_SPI_FLT_REP_BASE)
#define DRV_CTRL3_OTW_REP_BASE     			(0)
#define DRV_CTRL3_OTW_REP_DIS      			(0 << DRV_CTRL3_OTW_REP_BASE)
#define DRV_CTRL3_OTW_REP_EN	   			(1 << DRV_CTRL3_OTW_REP_BASE)

#define DRV_CTRL4_DRVOFF_BASE		(6)
#define DRV_CTRL4_DRVOFF_NO_ACTION	(0 << DRV_CTRL4_DRVOFF_BASE)
#define DRV_CTRL4_DRVOFF_STANDBY	(1 << DRV_CTRL4_DRVOFF_BASE)
#define DRV_CTRL4_OCP_CBC_BASE		(5)
#define DRV_CTRL4_OCP_CBC_DIS		(0 << DRV_CTRL4_OCP_CBC_BASE)
#define DRV_CTRL4_OCP_CBC_EN		(1 << DRV_CTRL4_OCP_CBC_BASE)
#define DRV_CTRL4_OCP_DEG_BASE		(4)
#define DRV_CTRL4_OCP_DEG_0_2US 	(0 << DRV_CTRL4_OCP_DEG_BASE)
#define DRV_CTRL4_OCP_DEG_0_6US 	(1 << DRV_CTRL4_OCP_DEG_BASE)
#define DRV_CTRL4_OCP_DEG_1_25US 	(2 << DRV_CTRL4_OCP_DEG_BASE)
#define DRV_CTRL4_OCP_DEG_1_6US 	(3 << DRV_CTRL4_OCP_DEG_BASE)
#define DRV_CTRL4_OCP_RETRY_BASE	(3)
#define DRV_CTRL4_OCP_RETRY_5MS 	(0 << DRV_CTRL4_OCP_RETRY_BASE)
#define DRV_CTRL4_OCP_RETRY_500MS 	(1 << DRV_CTRL4_OCP_RETRY_BASE)
#define DRV_CTRL4_OCP_LVL_BASE		(2)
#define DRV_CTRL4_OCP_LVL_16A   	(0 << DRV_CTRL4_OCP_LVL_BASE)
#define DRV_CTRL4_OCP_LVL_24A   	(1 << DRV_CTRL4_OCP_LVL_BASE)
#define DRV_CTRL4_OCP_MODE_BASE		(0)
#define DRV_CTRL4_OCP_MODE_LATCH 	(0 << DRV_CTRL4_OCP_MODE_BASE)
#define DRV_CTRL4_OCP_MODE_RETRY 	(1 << DRV_CTRL4_OCP_MODE_BASE)
#define DRV_CTRL4_OCP_MODE_REPORT	(2 << DRV_CTRL4_OCP_MODE_BASE)
#define DRV_CTRL4_OCP_MODE_DISABLED (3 << DRV_CTRL4_OCP_MODE_BASE)

#define DRV_CTRL5_ILIM_RECIR_BASE	(6)
#define DRV_CTRL5_ILIM_RECIR_BRAKE	(0 << DRV_CTRL5_ILIM_RECIR_BASE)
#define DRV_CTRL5_ILIM_RECIR_COAST	(1 << DRV_CTRL5_ILIM_RECIR_BASE)
#define DRV_CTRL5_EN_AAR_BASE		(3)
#define DRV_CTRL5_EN_AAR_DIS    	(0 << DRV_CTRL5_EN_AAR_BASE)
#define DRV_CTRL5_EN_AAR_EN	    	(1 << DRV_CTRL5_EN_AAR_BASE)
#define DRV_CTRL5_EN_ASR_BASE		(2)
#define DRV_CTRL5_EN_ASR_DIS    	(0 << DRV_CTRL5_EN_ASR_BASE)
#define DRV_CTRL5_EN_ASR_EN	    	(1 << DRV_CTRL5_EN_ASR_BASE)
#define DRV_CTRL5_CSA_GAIN_BASE		(0)
#define DRV_CTRL5_CSA_GAIN_0_15VA 	(0 << DRV_CTRL5_CSA_GAIN_BASE)
#define DRV_CTRL5_CSA_GAIN_0_3VA  	(1 << DRV_CTRL5_CSA_GAIN_BASE)
#define DRV_CTRL5_CSA_GAIN_0_6VA  	(2 << DRV_CTRL5_CSA_GAIN_BASE)
#define DRV_CTRL5_CSA_GAIN_1_2VA  	(3 << DRV_CTRL5_CSA_GAIN_BASE)

#define DRV_CTRL6_BUCK_PS_BASE	(4)
#define DRV_CTRL6_BUCK_PS_EN   	(0 << DRV_CTRL6_BUCK_PS_BASE)
#define DRV_CTRL6_BUCK_PS_DIS   (1 << DRV_CTRL6_BUCK_PS_BASE)
#define DRV_CTRL6_BUCK_CL_BASE  (3)
#define DRV_CTRL6_BUCK_CL_600MA (0 << DRV_CTRL6_BUCK_CL_BASE)
#define DRV_CTRL6_BUCK_CL_150MA (0 << DRV_CTRL6_BUCK_CL_BASE)
#define DRV_CTRL6_BUCK_SEL_BASE (1)
#define DRV_CTRL6_BUCK_SEL_3V3  (0 << DRV_CTRL6_BUCK_SEL_BASE)
#define DRV_CTRL6_BUCK_SEL_5V   (1 << DRV_CTRL6_BUCK_SEL_BASE)
#define DRV_CTRL6_BUCK_SEL_4V   (2 << DRV_CTRL6_BUCK_SEL_BASE)
#define DRV_CTRL6_BUCK_SEL_5V7  (3 << DRV_CTRL6_BUCK_SEL_BASE)
#define DRV_CTRL6_BUCK_BASE 	(0)
#define DRV_CTRL6_BUCK_EN       (0 << DRV_CTRL6_BUCK_BASE)
#define DRV_CTRL6_BUCK_DIS      (1 << DRV_CTRL6_BUCK_BASE)

#define DRV_CTRL10_DLYCMP_BASE		(4)
#define DRV_CTRL10_DLYCMP_DIS		(0 << DRV_CTRL10_DLYCMP_BASE)
#define DRV_CTRL10_DLYCMP_EN		(1 << DRV_CTRL10_DLYCMP_BASE)
#define DRV_CTRL10_DLY_TARGET_BASE	(0)
#define DRV_CTRL10_DLY_TARGET_0US	(0 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_0_4US	(1 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_0_6US	(2 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_0_8US	(3 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_1US	(4 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_1_2US	(5 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_1_4US	(6 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_1_6US	(7 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_1_8US	(8 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_2US	(9 << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_2_2US	(A << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_2_4US	(B << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_2_6US	(C << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_2_8US	(D << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_3US	(E << DRV_CTRL10_DLY_TARGET_BASE)
#define DRV_CTRL10_DLY_TARGET_3_2US	(F << DRV_CTRL10_DLY_TARGET_BASE)

typedef enum {
	REG_OK,
	REG_FAULT_CTRL1,
	REG_FAULT_CTRL2,
	REG_FAULT_CTRL3,
	REG_FAULT_CTRL4,
	REG_FAULT_CTRL5,
	REG_FAULT_CTRL6,
	REG_FAULT_CTRL10
} DRV8316C_REG_Typedef;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *nCS_Port;
	uint16_t nCS_Pin;
	GPIO_TypeDef *nSLEEP_Port;
	uint16_t nSLEEP_Pin;

	GPIO_TypeDef *DRVOFF_Port;
	uint16_t DRVOFF_Pin;

	GPIO_TypeDef *nFAULT_Port;
	uint16_t nFAULT_Pin;

} DRV8316C_Handle_t;

extern DRV8316C_Handle_t DRV8316C_L;
extern DRV8316C_Handle_t DRV8316C_R;

void DRV8316C_Init(DRV8316C_Handle_t *hdrv, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, GPIO_TypeDef *nSLEEP_Port,
		uint16_t nSLEEP_Pin, GPIO_TypeDef *nFAULT_Port, uint16_t nFAULT_Pin,
		GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin);

HAL_StatusTypeDef DRV8316C_WriteRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t data);

HAL_StatusTypeDef DRV8316C_ReadRegister(DRV8316C_Handle_t *hdrv,
		uint8_t regAddr, uint8_t *pData);

HAL_StatusTypeDef DRV8316C_UnlockRegister(DRV8316C_Handle_t *hdrv);

HAL_StatusTypeDef DRV8316C_LockRegister(DRV8316C_Handle_t *hdrv);

HAL_StatusTypeDef DRV8316C_ApplyDefaultConfig(DRV8316C_Handle_t *hdrv);

HAL_StatusTypeDef DRV8316C_ClearFaults(DRV8316C_Handle_t *hdrv);

DRV8316C_REG_Typedef DRV8316C_VerifyConfig(DRV8316C_Handle_t *hdrv);

void MX_DRV8316C_Init(void);

void Test_DRV8316C_Read_Status(DRV8316C_Handle_t *hdrv);

#endif

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_INC_DRV8316CRQ1_H_ */
