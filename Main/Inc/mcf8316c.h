/*
 * mcf8316.h
 *
 *  Created on: Jul 15, 2025
 *      Author: kth59
 */

#ifndef INC_MCF8316C_H_
#define INC_MCF8316C_H_

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "lcd.h"
#include "motor.h"
#include <stdbool.h>
#include <stdlib.h>

#define MCF8316C_I2C_ADDRESS_7BIT	0x01

/*
 *  Control Word
 */

#define CONTROL_READ 			(0x1 << 7)
#define CONTROL_WRITE			(0x0 << 7)

#define CONTROL_CRC_ENABLE		(0X1 << 6)
#define CONTROL_CRC_DISABLE		(0x0 << 6)
#define CONTROL_CRC				CONTROL_CRC_DISABLE

#define CONTROL_DATA_16BIT		(0x0 << 4)
#define CONTROL_DATA_32BIT		(0x1 << 4)
#define CONTROL_DATA_64BIT		(0x2 << 4)
#define CONTROL_DATA_LEN		CONTROL_DATA_32BIT

#define CONTROL_MEM_SEC			0x00
#define CONTROL_MEM_PAGE		0x00

#define MCF8316C_REG_MOTOR_STOP		0x80000000
#define MCF8316C_FAULT_CLEAR		0x30000000
#define MCF8316C_REG_WRITE_BIT		0x8A500000
#define MCF8316C_REG_READ_BIT		0x40000000

#define MCF8316C_I2C_LEFT_CHANNEL	(&hi2c1)
#define MCF8316C_I2C_RIGHT_CHANNEL	(&hi2c4)

#define MOTOR_L_RES
#define MOTOR_L_IND
#define MOTOR_L_BEMF

#define MOTOR_L_CURR_KI
#define MOTOR_L_CURR_KP
#define MOTOR_L_SPEED_KI
#define MOTOR_L_SPEED_KP

#define MOTOR_L_RES
#define MOTOR_L_IND
#define MOTOR_L_BEMF

#define MOTOR_R_CURR_KI
#define MOTOR_R_CURR_KP
#define MOTOR_R_SPEED_KI
#define MOTOR_R_SPEED_KP

/*
 * EEPROM ADDRESS
 */

#define ISD_CONFIG_ADDR				0x80
#define REV_DRIVE_CONFIG_ADDR		0x82
#define MOTOR_STARTUP1_ADDR			0x84
#define MOTOR_STARTUP2_ADDR			0x86
#define CLOSED_LOOP1_ADDR			0x88
#define CLOSED_LOOP2_ADDR			0x8A
#define CLOSED_LOOP3_ADDR			0x8C
#define CLOSED_LOOP4_ADDR			0x8E
#define REF_PROFILES1_ADDR			0x94
#define REF_PROFILES2_ADDR			0x96
#define REF_PROFILES3_ADDR			0x98
#define REF_PROFILES4_ADDR			0x9A
#define REF_PROFILES5_ADDR			0x9C
#define REF_PROFILES6_ADDR			0x9E
#define FAULT_CONFIG1_ADDR			0x90
#define FAULT_CONFIG2_ADDR			0x92
#define PIN_CONFIG_ADDR				0xA4
#define DEVICE_CONFIG1_ADDR			0xA6
#define DEVICE_CONFIG2_ADDR			0xA8
#define PERI_CONFIG1_ADDR			0xAA
#define GD_CONFIG1_ADDR				0xAC
#define GD_CONFIG2_ADDR				0xAE
#define INT_ALGO_1_ADDR				0xA0
#define INT_ALGO_2_ADDR				0xA2
#define ALGO_CTRL1_ADDR				0xEA
#define ALGO_DEBUG1_ADDR			0xEC
#define ALGO_DEBUG2_ADDR			0xEE

/* RAM ADDRESS */
#define DRIVER_FAULT_ADDR			0xE0
#define CONTROLLER_FAULT_ADDR		0xE2
#define MTR_PARAMS					0xE6
#define MCF8316C_WRITE_READ_ADDR	ALGO_CTRL1_ADDR
#define MCF8316C_MOTOR_STOP_ADDR	ALGO_DEBUG1_ADDR
#define MCF8316C_DRIVER_STATE_ADDR	0x190
#define MCF8316C_CURR_PI_ADDR		0xF0
#define MCF8316C_SPEED_PI_ADDR		0xF2
#define ALGORITHM_STATUS_ADDR		0x190
#define MCF8316C_BUS_CURRENT_ADDR	0x410
#define MCF8316C_VM_ADDR			0x476

/* Coef */
#define VM_COEFF1					60.f
#define VM_COEFF2					134217728.f

/*
 * EEPROM DATA
 */

#define EEPROM_SIZE					0x04
#define VM_SIZE_32BIT				0x04

/*
 * ISD_CONFIG
 */

/* ISD (Initial Speed Detect)*/

#define ISD_ENABLE					(0x1 << 30)
#define ISD_DISABLE					(0x0 << 30)
#define ISD_EN						ISD_ENABLE

/* BRAKE (Brake) */
#define BRAKE_ENABLE				(0x1 << 29)
#define BRAKE_DISABLE				(0x0 << 29)
#define BRAKE_EN					BRAKE_ENABLE

/* HIZ (Hi-Z) */
#define HIZ_ENABLE					(0x1 << 28)
#define HIZ_DISABLE					(0x0 << 28)
#define HIZ_EN						HIZ_DISABLE

/* RVS (Reserse Drive) */
#define RVS_DR_ENABLE				(0x1 << 27)
#define RVS_DR_DISABLE				(0x0 << 27)
#define RVS_DR_EN					RVS_DR_DISABLE

/* RESYNC (Resynchronization) */
#define RESYNC_ENABLE				(0x1 << 26)
#define RESYNC_DISABLE				(0x0 << 26)
#define RESYNC_EN					RESYNC_ENABLE

/* FOWARD DRIVE  */
#define FW_DRV_RESYN_5PER			(0x0 << 22)
#define FW_DRV_RESYN_10PER			(0x1 << 22)
#define FW_DRV_RESYN_15PER			(0x2 << 22)
#define FW_DRV_RESYN_20PER			(0x3 << 22)
#define FW_DRV_RESYN_25PER			(0x4 << 22)
#define FW_DRV_RESYN_30PER			(0x5 << 22)
#define FW_DRV_RESYN_35PER			(0x6 << 22)
#define FW_DRV_RESYN_40PER			(0x7 << 22)
#define FW_DRV_RESYN_45PER			(0x8 << 22)
#define FW_DRV_RESYN_50PER			(0x9 << 22)
#define FW_DRV_RESYN_55PER			(0xA << 22)
#define FW_DRV_RESYN_60PER			(0xB << 22)
#define FW_DRV_RESYN_70PER			(0xC << 22)
#define FW_DRV_RESYN_80PER			(0xD << 22)
#define FW_DRV_RESYN_90PER			(0xE << 22)
#define FW_DRV_RESYN_100PER			(0xF << 22)
#define FW_DRV_RESYN_THR			FW_DRV_RESYN_10PER

/* BRK MODE (Brake Mode) */
#define BRK_MODE_LOWSIDE			(0x1 << 21)
#define BRK_MODE_HIGHSIDE			(0x0 << 21)
#define BRK_MODE					BRK_MODE_LOWSIDE

/* BRK CONFIG (Brake Configuration) */
#define BRK_CONFIG_ENABLE			(0x1 << 20)
#define BRK_CONFIG_DISABLE			(0x0 << 20)
#define BRK_CONFIG					BRK_CONFIG_ENABLE

/* BRK CURR TH (Brake Current Threshold */
#define BRK_CURR_THR_0A1			(0x0 << 17)
#define BRK_CURR_THR_0A2			(0x1 << 17)
#define BRK_CURR_THR_0A3			(0x2 << 17)
#define BRK_CURR_THR_0A5			(0x3 << 17)
#define BRK_CURR_THR_1A				(0x4 << 17)
#define BRK_CURR_THR_2A				(0x5 << 17)
#define BRK_CURR_THR_4A				(0x6 << 17)
#define BRK_CURR_THR_8A				(0x7 << 17)
#define BRK_CURR_THR				BRK_CURR_THR_0A2

/* BRK TIME (Brake Time) */
#define BRK_TIME_10MS				(0x0 << 13)
#define BRK_TIME_50MS				(0x1 << 13)
#define BRK_TIME_100MS				(0x2 << 13)
#define BRK_TIME_200MS				(0x3 << 13)
#define BRK_TIME_300MS				(0x4 << 13)
#define BRK_TIME_400MS				(0x5 << 13)
#define BRK_TIME_500MS				(0x6 << 13)
#define BRK_TIME_750MS				(0x7 << 13)
#define BRK_TIME_1S					(0x8 << 13)
#define BRK_TIME_2S					(0x9 << 13)
#define BRK_TIME_3S					(0xA << 13)
#define BRK_TIME_4S					(0xB << 13)
#define BRK_TIME_5S					(0xC << 13)
#define BRK_TIME_7S5				(0xD << 13)
#define BRK_TIME_10S				(0xE << 13)
#define BRK_TIME_15S				(0xF << 13)
#define BRK_TIME					BRK_TIME_5S

/* HIZ TIME (Hi-Z Time) */
#define HIZ_TIME_10MS				(0x0 << 9)
#define HIZ_TIME_50MS				(0x1 << 9)
#define HIZ_TIME_100MS				(0x2 << 9)
#define HIZ_TIME_200MS				(0x3 << 9)
#define HIZ_TIME_300MS				(0x4 << 9)
#define HIZ_TIME_400MS				(0x5 << 9)
#define HIZ_TIME_500MS				(0x6 << 9)
#define HIZ_TIME_750MS				(0x7 << 9)
#define HIZ_TIME_1S					(0x8 << 9)
#define HIZ_TIME_2S					(0x9 << 9)
#define HIZ_TIME_3S					(0xA << 9)
#define HIZ_TIME_4S					(0xB << 9)
#define HIZ_TIME_5S					(0xC << 9)
#define HIZ_TIME_7S5				(0xD << 9)
#define HIZ_TIME_10S				(0xE << 9)
#define HIZ_TIME_15S				(0xF << 9)
#define HIZ_TIME					HIZ_TIME_500MS

/* STAT DETECT THR (BEMF Threshold */
#define STAT_DETECT_THR_50MV		(0x0 << 6)
#define STAT_DETECT_THR_75MV		(0x1 << 6)
#define STAT_DETECT_THR_100MV		(0x2 << 6)
#define STAT_DETECT_THR_250MV		(0x3 << 6)
#define STAT_DETECT_THR_500MV		(0x4 << 6)
#define STAT_DETECT_THR_750MV		(0x5 << 6)
#define STAT_DETECT_THR_1V			(0x6 << 6)
#define STAT_DETECT_THR_1V5			(0x7 << 6)
#define STAT_DETECT_THR				STAT_DETECT_THR_100MV

/* REV DRV HANDOFF THR (Reverse Drive Speed Threshold) */
#define REV_DRV_HANDOFF_2PER5		(0x0 << 2)
#define REV_DRV_HANDOFF_5PER		(0x1 << 2)
#define REV_DRV_HANDOFF_7PER5		(0x2 << 2)
#define REV_DRV_HANDOFF_10PER		(0x3 << 2)
#define REV_DRV_HANDOFF_12PER5		(0x4 << 2)
#define REV_DRV_HANDOFF_15PER		(0x5 << 2)
#define REV_DRV_HANDOFF_20PER		(0x6 << 2)
#define REV_DRV_HANDOFF_25PER		(0x7 << 2)
#define REV_DRV_HANDOFF_30PER		(0x8 << 2)
#define REV_DRV_HANDOFF_40PER		(0x9 << 2)
#define REV_DRV_HANDOFF_50PER		(0xA << 2)
#define REV_DRV_HANDOFF_60PER		(0xB << 2)
#define REV_DRV_HANDOFF_70PER		(0xC << 2)
#define REV_DRV_HANDOFF_80PER		(0xD << 2)
#define REV_DRV_HANDOFF_90PER		(0xE << 2)
#define REV_DRV_HANDOFF_100PER		(0xF << 2)
#define REV_DRV_HANDOFF_THR			REV_DRV_HANDOFF_30PER

/* REV DRV OPEN LOOP CURRENT (Reverse Drive Current Limit) */
#define REV_DRV_OPEN_LOOP_1A5		(0x0 << 0)
#define REV_DRV_OPEN_LOOP_2A5		(0x1 << 0)
#define REV_DRV_OPEN_LOOP_3A5		(0x2 << 0)
#define REV_DRV_OPEN_LOOP_5A		(0x3 << 0)
#define REV_DRV_OPEN_LOOP_CURRENT	REV_DRV_OPEN_LOOP_1A5

/* ISD_CONFIG REGISTER DATA*/
#define ISD_CONFIG_DATA				(ISD_EN | BRAKE_EN | HIZ_EN | RVS_DR_EN | RESYNC_EN | FW_DRV_RESYN_THR | BRK_MODE | BRK_CONFIG | BRK_CURR_THR | BRK_TIME | HIZ_TIME | STAT_DETECT_THR | REV_DRV_HANDOFF_THR | REV_DRV_OPEN_LOOP_CURRENT)

/*
 * REV_DRIVE_CONFIG
 */

/* REV DRV OPEN LOOP ACCEL A1 (Reverse Drive Open Loop Accelation Coefficient A1) */
#define REV_DRV_OPEN_LOOP_COEF1_10MHZ	(0x0 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_50MHZ	(0x1 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_1HZ		(0x2 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_2HZ5	(0x3 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_5HZ		(0x4 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_10HZ	(0x5 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_25HZ	(0x6 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_50HZ	(0x7 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_75HZ	(0x8 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_100HZ	(0x9 << 27)
#define REV_DRV_OPEN_LOOP_COEF1_250HZ	(0xA << 27)
#define REV_DRV_OPEN_LOOP_COEF1_500HZ	(0xB << 27)
#define REV_DRV_OPEN_LOOP_COEF1_750HZ	(0xC << 27)
#define REV_DRV_OPEN_LOOP_COEF1_1KHZ	(0xD << 27)
#define REV_DRV_OPEN_LOOP_COEF1_5KHZ	(0xE << 27)
#define REV_DRV_OPEN_LOOP_COEF1_10KHZ	(0xF << 27)
#define REV_DRV_OPEN_LOOP_ACCEL_A1		REV_DRV_OPEN_LOOP_COEF1_10HZ

/* REV DRV OPEN LOOP ACCEL A2 (Reverse Drive Open Loop Accelation Coefficient A2) */
#define REV_DRV_OPEN_LOOP_COEF2_0HZ		(0x0 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_50MHZ	(0x1 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_1HZ		(0x2 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_2HZ5	(0x3 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_5HZ		(0x4 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_10HZ	(0x5 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_25HZ	(0x6 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_50HZ	(0x7 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_75HZ	(0x8 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_100HZ	(0x9 << 23)
#define REV_DRV_OPEN_LOOP_COEF2_250HZ	(0xA << 23)
#define REV_DRV_OPEN_LOOP_COEF2_500HZ	(0xB << 23)
#define REV_DRV_OPEN_LOOP_COEF2_750HZ	(0xC << 23)
#define REV_DRV_OPEN_LOOP_COEF2_1KHZ	(0xD << 23)
#define REV_DRV_OPEN_LOOP_COEF2_5KHZ	(0xE << 23)
#define REV_DRV_OPEN_LOOP_COEF2_10KHZ	(0xF << 23)
#define REV_DRV_OPEN_LOOP_ACCEL_A2		REV_DRV_OPEN_LOOP_COEF2_0HZ

/* ACTIVE BRAKE CURRENT LIMIT (Reverse Drive Bus Current Limit) */
#define ACTIVE_BRAKE_CURRENT_0A5		(0x0 << 20)
#define ACTIVE_BRAKE_CURRENT_1A			(0x1 << 20)
#define ACTIVE_BRAKE_CURRENT_2A			(0x2 << 20)
#define ACTIVE_BRAKE_CURRENT_3A			(0x3 << 20)
#define ACTIVE_BRAKE_CURRENT_4A			(0x4 << 20)
#define ACTIVE_BRAKE_CURRENT_5A			(0x5 << 20)
#define ACTIVE_BRAKE_CURRENT_6A			(0x6 << 20)
#define ACTIVE_BRAKE_CURRENT_7A			(0x7 << 20)
#define ACTIVE_BRAKE_CURRENT_LIMIT		ACTIVE_BRAKE_CURRENT_2A

/* ACTIVE BRAKE KP (Active Braking Loop Kp) */
#define ACTIVE_BRAKE_KP_BASE			0.f
#define ACTIVE_BRAKE_KP_CALC			((uint16_t) (ACTIVE_BRAKE_KP_BASE * 128.f) & 0x3FF)
#define ACTIVE_BRAKE_KP					(ACTIVE_BRAKE_KP_CALC << 10)

/* ACTIVE BRAKE KI (Active Braking Loop Ki) */
#define ACTIVE_BRAKE_KI_BASE			0.f
#define ACTIVE_BRAKE_KI_CALC			((uint16_t) (ACTIVE_BRAKE_KI_BASE * 512.f) & 0x3FF)
#define ACTIVE_BRAKE_KI					ACTIVE_BRAKE_KI_CALC

/* REV_DRVIVE_CONFIG REGISTER DATA */
#define REV_DRIVE_CONFIG_DATA			(REV_DRV_OPEN_LOOP_ACCEL_A1 | REV_DRV_OPEN_LOOP_ACCEL_A2 | ACTIVE_BRAKE_CURRENT_LIMIT | ACTIVE_BRAKE_KP | ACTIVE_BRAKE_KI)

/*
 * MOTOR_STARTUP1
 */

/* MTR STARTUP (Motor Start-up Method) */
#define MTR_STARTUP_ALIGN				(0x0 << 29)
#define MTR_STARTUP_DOUBLE_ALIGN		(0x1 << 29)
#define MTR_STARTUP_IPD					(0x2 << 29)
#define MTR_STARTUP_SLOW_FIRST			(0x3 << 29)
#define MTR_STARTUP						MTR_STARTUP_ALIGN

/* ALIGN SLOW RAMP RATE (Align Slow First Cycle) */
#define ALIGN_SLOW_RAMP_0A1				(0x0 << 25)
#define ALIGN_SLOW_RAMP_1A				(0x1 << 25)
#define ALIGN_SLOW_RAMP_5A				(0x2 << 25)
#define ALIGN_SLOW_RAMP_10A				(0x3 << 25)
#define ALIGN_SLOW_RAMP_15A				(0x4 << 25)
#define ALIGN_SLOW_RAMP_25A				(0x5 << 25)
#define ALIGN_SLOW_RAMP_50A				(0x6 << 25)
#define ALIGN_SLOW_RAMP_100A			(0x7 << 25)
#define ALIGN_SLOW_RAMP_150A			(0x8 << 25)
#define ALIGN_SLOW_RAMP_200A			(0x9 << 25)
#define ALIGN_SLOW_RAMP_250A			(0xA << 25)
#define ALIGN_SLOW_RAMP_500A			(0xB << 25)
#define ALIGN_SLOW_RAMP_1KA				(0xC << 25)
#define ALIGN_SLOW_RAMP_2KA				(0xD << 25)
#define ALIGN_SLOW_RAMP_5KA				(0xE << 25)
#define ALIGN_SLOW_RAMP_NO_LIMIT		(0xF << 25)
#define ALIGN_SLOW_RAMP_RATE			ALIGN_SLOW_RAMP_25A

/* ALIGN TIME (Align Time */
#define ALIGN_TIME_10MS					(0x0 << 21)
#define ALIGN_TIME_50MS					(0x1 << 21)
#define ALIGN_TIME_100MS				(0x2 << 21)
#define ALIGN_TIME_200MS				(0x3 << 21)
#define ALIGN_TIME_300MS				(0x4 << 21)
#define ALIGN_TIME_400MS				(0x5 << 21)
#define ALIGN_TIME_500MS				(0x6 << 21)
#define ALIGN_TIME_750MS				(0x7 << 21)
#define ALIGN_TIME_1S					(0x8 << 21)
#define ALIGN_TIME_1S5					(0x9 << 21)
#define ALIGN_TIME_2S					(0xA << 21)
#define ALIGN_TIME_3S					(0xB << 21)
#define ALIGN_TIME_4S					(0xC << 21)
#define ALIGN_TIME_5S					(0xD << 21)
#define ALIGN_TIME_7S5					(0xE << 21)
#define ALIGN_TIME_10S					(0xF << 21)
#define ALIGN_TIME						ALIGN_TIME_3S

/* ALIGN OR SLOW CURRENT ILIMIT (Align or Slow First Cycle Current Limit) */
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_0A125		(0x0 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_0A25		(0x1 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_0A5		(0x2 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_1A			(0x3 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_1A5		(0x4 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_2A			(0x5 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_2A5		(0x6 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_3A			(0x7 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_3A5		(0x8 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_4A			(0x9 << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_4A5		(0xA << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_5A			(0xB << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_5A5		(0xC << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_6A			(0xD << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_7A			(0xE << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT_8A			(0xF << 17)
#define ALIGN_OR_SLOW_CURRENT_ILIMIT			ALIGN_OR_SLOW_CURRENT_ILIMIT_1A5

/* IPD CLK FREQ (Initial Position Detection Clock Frequency) */
#define IPD_CLK_FREQ_50HZ		(0x0 << 14)
#define IPD_CLK_FREQ_100HZ		(0x1 << 14)
#define IPD_CLK_FREQ_250HZ		(0x2 << 14)
#define IPD_CLK_FREQ_500HZ		(0x3 << 14)
#define IPD_CLK_FREQ_1KHZ		(0x4 << 14)
#define IPD_CLK_FREQ_2KHZ		(0x5 << 14)
#define IPD_CLK_FREQ_5KHZ		(0x6 << 14)
#define IPD_CLK_FREQ_10KHZ		(0x7 << 14)
#define IPD_CLK_FREQ			IPD_CLK_FREQ_1KHZ

/* IPD_CURR_THR (Initial Position Detection Threshold */
#define IPD_CURR_0A25			(0x00 << 9)
#define IPD_CURR_0A5			(0x01 << 9)
#define IPD_CURR_0A75			(0x02 << 9)
#define IPD_CURR_1A				(0x03 << 9)
#define IPD_CURR_1A25			(0x04 << 9)
#define IPD_CURR_1A5			(0x05 << 9)
#define IPD_CURR_2A				(0x06 << 9)
#define IPD_CURR_2A5			(0x07 << 9)
#define IPD_CURR_3A				(0x08 << 9)
#define IPD_CURR_3A667			(0x09 << 9)
#define IPD_CURR_4A				(0x0A << 9)
#define IPD_CURR_4A667			(0x0B << 9)
#define IPD_CURR_5A				(0x0C << 9)
#define IPD_CURR_5A333			(0x0D << 9)
#define IPD_CURR_6A				(0x0E << 9)
#define IPD_CURR_6A667			(0x0F << 9)
#define IPD_CURR_7A333			(0x10 << 9)
#define IPD_CURR_8A				(0x11 << 9)
#define IPD_CURR_THR			IPD_CURR_0A5

/* IPD RLS MODE (Initial Position Detection Release Mode) */
#define IPD_RLS_BRAKE			(0x0 << 8)
#define IPD_RLS_TRISTATE		(0x1 << 8)
#define IPD_RLS_MODE			IPD_RLS_BRAKE

/* IPD ADV ANGLE (Initial Position Detection Advance Angle */
#define IPD_ADV_DEGREE_0		(0x0 << 6)
#define IPD_ADV_DEGREE_30		(0x1 << 6)
#define IPD_ADV_DEGREE_60		(0x2 << 6)
#define IPD_ADV_DEGREE_90		(0x3 << 6)
#define IPD_ADV_ANGLE			IPD_ADV_DEGREE_0

/* IPD REPEAT (Number of Times IPD Executed)*/
#define IPD_REPEAT_1TIME		(0x0 << 4)
#define IPD_REPEAT_2TIME		(0x1 << 4)
#define IPD_REPEAT_3TIME		(0x2 << 4)
#define IPD_REPEAT_4TIME		(0x3 << 4)
#define IPD_REPEAT				IPD_REPEAT_1TIME

/* OL ILIMIT CONFIG (Open Loop Current Limit Configuration)*/
#define OL_LIMIT_CONFIG_OLILIMIT	(0x0 << 3)
#define OL_LIMIT_CONFIG_ILIMIT		(0x1 << 3)
#define OL_LIMIT_CONFIG				OL_LIMIT_CONFIG_OLILIMIT

/* IQ RAMP EN (IQ Ramp Down for Transition) */
#define IQ_RAMP_ENABLE				(0x1 << 2)
#define IQ_RAMP_DISABLE				(0x0 << 2)
#define IQ_RAMP_EN					IQ_RAMP_DISABLE

/* ACTIVE BRAKE EN (Active Braking During Decelation */
#define ACTIVE_BRAKE_ENABLE			(0x1 << 1)
#define ACTIVE_BRAKE_DISABLE		(0x0 << 1)
#define ACTIVE_BRAKE_EN				ACTIVE_BRAKE_DISABLE

/* REV_DRV_COFIG (Forward and Reverse Setting for Reverse Drive) */
#define REV_DRV_CONFIG_FORWARD		(0x0)
#define REV_DRV_CONFIG_REVERSE		(0x1)
#define REV_DRV_CONFIG				REV_DRV_CONFIG_FORWARD

/* MOTOR_STARTUP1 Register Data */
#define MOTOR_STARTUP1_DATA			(MTR_STARTUP | ALIGN_SLOW_RAMP_RATE | ALIGN_TIME | ALIGN_OR_SLOW_CURRENT_ILIMIT | IPD_CLK_FREQ | IPD_CURR_THR | IPD_RLS_MODE | IPD_ADV_ANGLE | IPD_REPEAT | OL_LIMIT_CONFIG | IQ_RAMP_EN | ACTIVE_BRAKE_EN | REV_DRV_CONFIG)

/*
 * MOTOR STARTUP2
 */

/* OL LIMIT (Open Loop Current Limit) */
#define OL_ILIMIT_0A125		(0x0 << 27)
#define OL_ILIMIT_0A25		(0x1 << 27)
#define OL_ILIMIT_0A5		(0x2 << 27)
#define OL_ILIMIT_1A		(0x3 << 27)
#define OL_ILIMIT_1A5		(0x4 << 27)
#define OL_ILIMIT_2A		(0x5 << 27)
#define OL_ILIMIT_2A5		(0x6 << 27)
#define OL_ILIMIT_3A		(0x7 << 27)
#define OL_ILIMIT_3A5		(0x8 << 27)
#define OL_ILIMIT_4A		(0x9 << 27)
#define OL_ILIMIT_4A5		(0xA << 27)
#define OL_ILIMIT_5A		(0xB << 27)
#define OL_ILIMIT_5A5		(0xC << 27)
#define OL_ILIMIT_6A		(0xD << 27)
#define OL_ILIMIT_7A		(0xE << 27)
#define OL_ILIMIT_8A		(0xF << 27)
#define OL_ILIMIT			OL_ILIMIT_2A5

/* OL ACC A1 (Open Loop Acceleration Coefficient A1) */
#define OL_ACC_A1_0HZ01		(0x0 << 23)
#define OL_ACC_A1_0HZ05		(0x1 << 23)
#define OL_ACC_A1_1HZ		(0x2 << 23)
#define OL_ACC_A1_2HZ5		(0x3 << 23)
#define OL_ACC_A1_5HZ		(0x4 << 23)
#define OL_ACC_A1_10HZ		(0x5 << 23)
#define OL_ACC_A1_25HZ		(0x6 << 23)
#define OL_ACC_A1_50HZ		(0x7 << 23)
#define OL_ACC_A1_75HZ		(0x8 << 23)
#define OL_ACC_A1_100HZ		(0x9 << 23)
#define OL_ACC_A1_250HZ		(0xA << 23)
#define OL_ACC_A1_500HZ		(0xB << 23)
#define OL_ACC_A1_750HZ		(0xC << 23)
#define OL_ACC_A1_1KHZ		(0xD << 23)
#define OL_ACC_A1_5KHZ		(0xE << 23)
#define OL_ACC_A1_10KHZ		(0xF << 23)
#define OL_ACC_A1			OL_ACC_A1_25HZ

/* OL ACC A2 (Open Loop Acceleration Coefficient A2) */
#define OL_ACC_A2_0HZ		(0x0 << 19)
#define OL_ACC_A2_0HZ05		(0x1 << 19)
#define OL_ACC_A2_1HZ		(0x2 << 19)
#define OL_ACC_A2_2HZ5		(0x3 << 19)
#define OL_ACC_A2_5HZ		(0x4 << 19)
#define OL_ACC_A2_10HZ		(0x5 << 19)
#define OL_ACC_A2_25HZ		(0x6 << 19)
#define OL_ACC_A2_50HZ		(0x7 << 19)
#define OL_ACC_A2_75HZ		(0x8 << 19)
#define OL_ACC_A2_100HZ		(0x9 << 19)
#define OL_ACC_A2_250HZ		(0xA << 19)
#define OL_ACC_A2_500HZ		(0xB << 19)
#define OL_ACC_A2_750HZ		(0xC << 19)
#define OL_ACC_A2_1KHZ		(0xD << 19)
#define OL_ACC_A2_5KHZ		(0xE << 19)
#define OL_ACC_A2_10KHZ		(0xF << 19)
#define OL_ACC_A2			OL_ACC_A2_0HZ

/* AUTO HANDOFF EN (Auto Handoff Enable) */
#define AUTO_HANDOFF_ENABLE		(0x1 << 18)
#define AUTO_HANDOFF_DISABLE	(0x0 << 18)
#define AUTO_HANDOFF_EN			AUTO_HANDOFF_ENABLE

/* OPN CL HANDOFF THR (Open to Closed Loop Handoff Threshold) */
#define OPN_CL_HANDOFF_1PER			(0x00 << 13)
#define OPN_CL_HANDOFF_2PER			(0x01 << 13)
#define OPN_CL_HANDOFF_3PER			(0x02 << 13)
#define OPN_CL_HANDOFF_4PER			(0x03 << 13)
#define OPN_CL_HANDOFF_5PER			(0x04 << 13)
#define OPN_CL_HANDOFF_6PER			(0x05 << 13)
#define OPN_CL_HANDOFF_7PER			(0x06 << 13)
#define OPN_CL_HANDOFF_8PER			(0x07 << 13)
#define OPN_CL_HANDOFF_9PER			(0x08 << 13)
#define OPN_CL_HANDOFF_10PER		(0x09 << 13)
#define OPN_CL_HANDOFF_11PER		(0x0A << 13)
#define OPN_CL_HANDOFF_12PER		(0x0B << 13)
#define OPN_CL_HANDOFF_13PER		(0x0C << 13)
#define OPN_CL_HANDOFF_14PER		(0x0D << 13)
#define OPN_CL_HANDOFF_15PER		(0x0E << 13)
#define OPN_CL_HANDOFF_16PER		(0x0F << 13)
#define OPN_CL_HANDOFF_17PER		(0x10 << 13)
#define OPN_CL_HANDOFF_18PER		(0x11 << 13)
#define OPN_CL_HANDOFF_19PER		(0x12 << 13)
#define OPN_CL_HANDOFF_20PER		(0x13 << 13)
#define OPN_CL_HANDOFF_22PER5		(0x14 << 13)
#define OPN_CL_HANDOFF_25PER		(0x15 << 13)
#define OPN_CL_HANDOFF_27PER5		(0x16 << 13)
#define OPN_CL_HANDOFF_30PER		(0x17 << 13)
#define OPN_CL_HANDOFF_32PER5		(0x18 << 13)
#define OPN_CL_HANDOFF_35PER		(0x19 << 13)
#define OPN_CL_HANDOFF_37PER5		(0x1A << 13)
#define OPN_CL_HANDOFF_40PER		(0x1B << 13)
#define OPN_CL_HANDOFF_42PER5		(0x1C << 13)
#define OPN_CL_HANDOFF_45PER		(0x1D << 13)
#define OPN_CL_HANDOFF_47PER5		(0x1E << 13)
#define OPN_CL_HANDOFF_50PER		(0x1F << 13)
#define OPN_CL_HANDOFF_THR			OPN_CL_HANDOFF_20PER

/* ALIGN ANGLE (Align Angle) */
#define ALIGN_0DEG		(0x00 << 8)
#define ALIGN_10DEG		(0x01 << 8)
#define ALIGN_20DEG		(0x02 << 8)
#define ALIGN_30DEG		(0x03 << 8)
#define ALIGN_45DEG		(0x04 << 8)
#define ALIGN_60DEG		(0x05 << 8)
#define ALIGN_70DEG		(0x06 << 8)
#define ALIGN_80DEG		(0x07 << 8)
#define ALIGN_90DEG		(0x08 << 8)
#define ALIGN_110DEG	(0x09 << 8)
#define ALIGN_120DEG	(0x0A << 8)
#define ALIGN_135DEG	(0x0B << 8)
#define ALIGN_150DEG	(0x0C << 8)
#define ALIGN_160DEG	(0x0D << 8)
#define ALIGN_170DEG	(0x0E << 8)
#define ALIGN_180DEG	(0x0F << 8)
#define ALIGN_190DEG	(0x10 << 8)
#define ALIGN_210DEG	(0x11 << 8)
#define ALIGN_225DEG	(0x12 << 8)
#define ALIGN_240DEG	(0x13 << 8)
#define ALIGN_250DEG	(0x14 << 8)
#define ALIGN_260DEG	(0x15 << 8)
#define ALIGN_270DEG	(0x16 << 8)
#define ALIGN_280DEG	(0x17 << 8)
#define ALIGN_290DEG	(0x18 << 8)
#define ALIGN_315DEG	(0x19 << 8)
#define ALIGN_330DEG	(0x1A << 8)
#define ALIGN_340DEG	(0x1B << 8)
#define ALIGN_350DEG	(0x1C << 8)
#define ALIGN_ANGLE		ALIGN_0DEG

/* SLOW FIRST CYCLE FREQ (First Cycle in Start-up Frequency) */
#define SLOW_FIRST_CYC_FREQ_1PER		(0x0 << 4)
#define SLOW_FIRST_CYC_FREQ_2PER		(0x1 << 4)
#define SLOW_FIRST_CYC_FREQ_3PER		(0x2 << 4)
#define SLOW_FIRST_CYC_FREQ_5PER		(0x3 << 4)
#define SLOW_FIRST_CYC_FREQ_7PER5		(0x4 << 4)
#define SLOW_FIRST_CYC_FREQ_10PER		(0x5 << 4)
#define SLOW_FIRST_CYC_FREQ_12PER5		(0x6 << 4)
#define SLOW_FIRST_CYC_FREQ_15PER		(0x7 << 4)
#define SLOW_FIRST_CYC_FREQ_17PER5		(0x8 << 4)
#define SLOW_FIRST_CYC_FREQ_20PER		(0x9 << 4)
#define SLOW_FIRST_CYC_FREQ_25PER		(0xA << 4)
#define SLOW_FIRST_CYC_FREQ_30PER		(0xB << 4)
#define SLOW_FIRST_CYC_FREQ_35PER		(0xC << 4)
#define SLOW_FIRST_CYC_FREQ_40PER		(0xD << 4)
#define SLOW_FIRST_CYC_FREQ_45PER		(0xE << 4)
#define SLOW_FIRST_CYC_FREQ_50PER		(0xF << 4)
#define SLOW_FIRST_CYC_FREQ				SLOW_FIRST_CYC_FREQ_1PER

/* FIRST CYCLE FREQ SEL (First Cycle in Open Loop Start-up) */
#define FIRST_CYCLE_FREQ_SEL_0HZ		(0x0 << 3)
#define FIRST_CYCLE_FREQ_EQUAL_SLOW		(0x1 << 3)
#define FIRST_CYCLE_FREQ_SEL			FIRST_CYCLE_FREQ_SEL_0HZ
/* THETA ERROR RAMP RATE (Ramp Rate Reducing Difference) */
#define THETA_ERROR_RAMP_RATE_0DEG01	0x0
#define THETA_ERROR_RAMP_RATE_0DEG05	0x1
#define THETA_ERROR_RAMP_RATE_0DEG1		0x2
#define THETA_ERROR_RAMP_RATE_0DEG15	0x3
#define THETA_ERROR_RAMP_RATE_0DEG2		0x4
#define THETA_ERROR_RAMP_RATE_0DEG5		0x5
#define THETA_ERROR_RAMP_RATE_1DEG		0x6
#define THETA_ERROR_RAMP_RATE_2DEG		0x7
#define THETA_ERROR_RAMP_RATE			THETA_ERROR_RAMP_RATE_0DEG2

/* MOTOR_STARTUP2 Register Data */
#define MOTOR_STARTUP2_DATA 			(OL_ILIMIT | OL_ACC_A1 | OL_ACC_A2 | AUTO_HANDOFF_EN | OPN_CL_HANDOFF_THR | ALIGN_ANGLE | SLOW_FIRST_CYC_FREQ | FIRST_CYCLE_FREQ_SEL | THETA_ERROR_RAMP_RATE)

/*
 * CLOSED LOOP1
 */

/* OVERMODULATION ENABLE (Overmodulation Enable) */
#define OVERMODULATION_ENABLE			(0x1 << 30)
#define OVERMODULATION_DISABLE			(0x0 << 30)
#define OVERMODULATION_EN				OVERMODULATION_DISABLE

/* CL ACC (Closed Loop Acceleration) */
#define CL_ACC_0HZ5				(0x00 << 25)
#define CL_ACC_1HZ				(0x01 << 25)
#define CL_ACC_2HZ5				(0x02 << 25)
#define CL_ACC_5HZ				(0x03 << 25)
#define CL_ACC_7HZ5				(0x04 << 25)
#define CL_ACC_10HZ				(0x05 << 25)
#define CL_ACC_20HZ				(0x06 << 25)
#define CL_ACC_40HZ				(0x07 << 25)
#define CL_ACC_60HZ				(0x08 << 25)
#define CL_ACC_80HZ				(0x09 << 25)
#define CL_ACC_100HZ			(0x0A << 25)
#define CL_ACC_200HZ			(0x0B << 25)
#define CL_ACC_300HZ			(0x0C << 25)
#define CL_ACC_400HZ			(0x0D << 25)
#define CL_ACC_500HZ			(0x0E << 25)
#define CL_ACC_600HZ			(0x0F << 25)
#define CL_ACC_700HZ			(0x10 << 25)
#define CL_ACC_800HZ			(0x11 << 25)
#define CL_ACC_900HZ			(0x12 << 25)
#define CL_ACC_1KHZ				(0x13 << 25)
#define CL_ACC_2KHZ				(0x14 << 25)
#define CL_ACC_4KHZ				(0x15 << 25)
#define CL_ACC_6KHZ				(0x16 << 25)
#define CL_ACC_8KHZ				(0x17 << 25)
#define CL_ACC_10KHZ			(0x18 << 25)
#define CL_ACC_20KHZ			(0x19 << 25)
#define CL_ACC_30KHZ			(0x1A << 25)
#define CL_ACC_40KHZ			(0x1B << 25)
#define CL_ACC_50KHZ			(0x1C << 25)
#define CL_ACC_60KHZ			(0x1D << 25)
#define CL_ACC_70KHZ			(0x1E << 25)
#define CL_ACC_NO_LIMIT			(0x1F << 25)
#define CL_ACC					CL_ACC_60HZ

/* CL DEC CONFIG (Closed Loop Deceleration Configuration) */
#define CL_DEC_CONFIG_BY_DEC	(0x0 << 24)
#define CL_DEC_CONFIG_BY_ACC	(0x1 << 24)
#define CL_DEC_CONFIG			CL_DEC_CONFIG_BY_ACC

/* CL DEC (Closed Loop Deceleration) */
#define CL_DEC_0HZ5			(0x00 << 19)
#define CL_DEC_1HZ			(0x01 << 19)
#define CL_DEC_2HZ5			(0x02 << 19)
#define CL_DEC_5HZ			(0x03 << 19)
#define CL_DEC_7HZ5			(0x04 << 19)
#define CL_DEC_10HZ			(0x05 << 19)
#define CL_DEC_20HZ			(0x06 << 19)
#define CL_DEC_40HZ			(0x07 << 19)
#define CL_DEC_60HZ			(0x08 << 19)
#define CL_DEC_80HZ			(0x09 << 19)
#define CL_DEC_100HZ		(0x0A << 19)
#define CL_DEC_200HZ		(0x0B << 19)
#define CL_DEC_300HZ		(0x0C << 19)
#define CL_DEC_400HZ		(0x0D << 19)
#define CL_DEC_500HZ		(0x0E << 19)
#define CL_DEC_600HZ		(0x0F << 19)
#define CL_DEC_700HZ		(0x10 << 19)
#define CL_DEC_800HZ		(0x11 << 19)
#define CL_DEC_900HZ		(0x12 << 19)
#define CL_DEC_1KHZ			(0x13 << 19)
#define CL_DEC_2KHZ			(0x14 << 19)
#define CL_DEC_4KHZ			(0x15 << 19)
#define CL_DEC_6KHZ			(0x16 << 19)
#define CL_DEC_8KHZ			(0x17 << 19)
#define CL_DEC_10KHZ		(0x18 << 19)
#define CL_DEC_20KHZ		(0x19 << 19)
#define CL_DEC_30KHZ		(0x1A << 19)
#define CL_DEC_40KHZ		(0x1B << 19)
#define CL_DEC_50KHZ		(0x1C << 19)
#define CL_DEC_6OKHZ		(0x1D << 19)
#define CL_DEC_70KHZ		(0x1E << 19)
#define CL_DEC_NO_LIMIT		(0x1F << 19)
#define CL_DEC				CL_DEC_20HZ

/* PWM FREQ OUT (PWM Output Frequency)*/
#define PWM_FREQ_OUT_10KHZ	(0x0 << 15)
#define PWM_FREQ_OUT_15KHZ	(0x1 << 15)
#define PWM_FREQ_OUT_20KHZ	(0x2 << 15)
#define PWM_FREQ_OUT_25KHZ	(0x3 << 15)
#define PWM_FREQ_OUT_30KHZ	(0x4 << 15)
#define PWM_FREQ_OUT_35KHZ	(0x5 << 15)
#define PWM_FREQ_OUT_40KHZ	(0x6 << 15)
#define PWM_FREQ_OUT_45KHZ	(0x7 << 15)
#define PWM_FREQ_OUT_50KHZ	(0x8 << 15)
#define PWM_FREQ_OUT_55KHZ	(0x9 << 15)
#define PWM_FREQ_OUT_60KHZ	(0xA << 15)
#define PWM_FREQ_OUT		PWM_FREQ_OUT_25KHZ

/* PWM MODE (PWM Modulation) */
#define PWM_CONTINUOUS_SPACE_MODE		(0x0 << 14)
#define PWM_DISCONTINUOUS_SPACE_MODE	(0x1 << 14)
#define PWM_MODE						PWM_CONTINUOUS_SPACE_MODE

/* FG SEL (Frequency Generator Select) */
#define FG_SEL_OL_CL		(0x0 << 12)
#define FG_SEL_CL			(0x1 << 12)
#define FG_SEL_OL			(0x2 << 12)
#define FG_SEL_DISABLE		(0x3 << 12)
#define FG_SEL				FG_SEL_OL_CL

/* FG DIV (Frequency Generator Division Factor) */
#define FG_DIV_1			(0x0 << 8)
#define FG_DIV_2			(0x2 << 8)
#define FG_DIV_3			(0x3 << 8)
#define FG_DIV_4			(0x4 << 8)
#define FG_DIV_5			(0x5 << 8)
#define FG_DIV_6			(0x6 << 8)
#define FG_DIV_7			(0x7 << 8)
#define FG_DIV_8			(0x8 << 8)
#define FG_DIV_9			(0x9 << 8)
#define FG_DIV_10			(0xA << 8)
#define FG_DIV_11			(0xB << 8)
#define FG_DIV_12			(0xC << 8)
#define FG_DIV_13			(0xD << 8)
#define FG_DIV_14			(0xE << 8)
#define FG_DIV_15			(0xF << 8)
#define FG_DIV				FG_DIV_15

/* FG CONFIG (Frequency Generator Configuration) */
#define FG_CONFIG_ACTIVE_DRIVING				(0x0 << 7)
#define FG_CONFIG_ACTIVE_UNDER_FG_BEMF_THR		(0x1 << 7)
#define FG_CONFIG								FG_CONFIG_ACTIVE_UNDER_FG_BEMF_THR

/* FG BEMF THR (Frequency Generator BEMF Threshold */
#define FG_BEMF_1MV			(0x0 << 4)
#define FG_BEMF_2MV			(0x1 << 4)
#define FG_BEMF_5MV			(0x2 << 4)
#define FG_BEMF_10MV		(0x3 << 4)
#define FG_BEMF_20MV		(0x4 << 4)
#define FG_BEMF_30MV		(0x5 << 4)
#define FG_BEMF_THR			FG_BEMF_10MV

/* AVS EN (Anti Voltage Surge Enable) */
#define AVS_ENABLE			(0x1 << 3)
#define AVS_DISABLE			(0x0 << 3)
#define AVS_EN				AVS_ENABLE

/* DEADTIME COMP EN	(Deadtime Compensation Enable) */
#define DEADTIME_COMP_ENABLE		(0x1 << 2)
#define DEADTIME_COMP_DISABLE		(0x0 << 2)
#define DEADTIME_COMP_EN			DEADTIME_COMP_DISABLE

/* SPEED LOOP DIS (Speed Loop Disable or Torque Mode Enable) */
#define SPEED_LOOP_ENABLE			(0x0 << 1)
#define SPEED_LOOP_DISABLE			(0x1 << 1)
#define SPEED_LOOP_DIS				SPEED_LOOP_ENABLE

/* CLOSED_LOOP1 Register Data */
#define CLOSED_LOOP1_DATA			(OVERMODULATION_ENABLE | CL_ACC | CL_DEC_CONFIG | CL_DEC | PWM_FREQ_OUT | PWM_MODE | FG_SEL | FG_DIV | FG_CONFIG | FG_BEMF_THR | AVS_EN | DEADTIME_COMP_EN | SPEED_LOOP_DIS)

/*
 * CLOSED LOOP2
 */

/* MTR STOP (Motor Stop Options) */
#define MTR_STOP_HIZ					(0x0 << 28)
#define MTR_STOP_NO_APPLICABLE			(0x1 << 28)
#define MTR_STOP_LOW_SIDE_BRAKE			(0x2 << 28)
#define MTR_STOP_HIGH_SIDE_BRAKE		(0x3 << 28)
#define MTR_STOP_ACTIVE_SPIN_DOWN		(0x4 << 28)
#define MTR_STOP_ALIGN_BRAKE			(0x5 << 28)
#define MTR_STOP						MTR_STOP_HIZ

/* MTR STOP BRK TIME */
#define MTR_STOP_BRK_TIME_1MS			(0x0 << 24)
#define MTR_STOP_BRK_TIME_5MS			(0x5 << 24)
#define MTR_STOP_BRK_TIME_10MS			(0x6 << 24)
#define MTR_STOP_BRK_TIME_50MS			(0x7 << 24)
#define MTR_STOP_BRK_TIME_100MS			(0x8 << 24)
#define MTR_STOP_BRK_TIME_250MS			(0x9 << 24)
#define MTR_STOP_BRK_TIME_500MS			(0xA << 24)
#define MTR_STOP_BRK_TIME_1S			(0xB << 24)
#define MTR_STOP_BRK_TIME_2S5			(0xC << 24)
#define MTR_STOP_BRK_TIME_5S			(0xD << 24)
#define MTR_STOP_BRK_TIME_10S			(0xE << 24)
#define MTR_STOP_BRK_TIME_15S			(0xF << 24)
#define MTR_STOP_BRK_TIME				MTR_STOP_BRK_TIME_1S

/* ACT SPIN THR (Active Spin Threshold) */
#define ACT_SPIN_100PER					(0x0 << 20)
#define ACT_SPIN_90PER					(0x1 << 20)
#define ACT_SPIN_80PER					(0x2 << 20)
#define ACT_SPIN_70PER					(0x3 << 20)
#define ACT_SPIN_60PER					(0x4 << 20)
#define ACT_SPIN_50PER					(0x5 << 20)
#define ACT_SPIN_45PER					(0x6 << 20)
#define ACT_SPIN_40PER					(0x7 << 20)
#define ACT_SPIN_35PER					(0x8 << 20)
#define ACT_SPIN_30PER					(0x9 << 20)
#define ACT_SPIN_25PER					(0xA << 20)
#define ACT_SPIN_20PER					(0xB << 20)
#define ACT_SPIN_15PER					(0xC << 20)
#define ACT_SPIN_10PER					(0xD << 20)
#define ACT_SPIN_5PER					(0xE << 20)
#define ACT_SPIN_2PER5					(0xF << 20)
#define ACT_SPIN_THR					ACT_SPIN_25PER

/* BRAKE SPEED THRESHOLD (Speed Treshold for BRAKE Pin and Motor Stop Options) */
#define BRAKE_SPEED_100PER				(0x0 << 16)
#define BRAKE_SPEED_90PER				(0x1 << 16)
#define BRAKE_SPEED_80PER				(0x2 << 16)
#define BRAKE_SPEED_70PER				(0x3 << 16)
#define BRAKE_SPEED_60PER				(0x4 << 16)
#define BRAKE_SPEED_50PER				(0x5 << 16)
#define BRAKE_SPEED_45PER				(0x6 << 16)
#define BRAKE_SPEED_40PER				(0x7 << 16)
#define BRAKE_SPEED_35PER				(0x8 << 16)
#define BRAKE_SPEED_30PER				(0x9 << 16)
#define BRAKE_SPEED_25PER				(0xA << 16)
#define BRAKE_SPEED_20PER				(0xB << 16)
#define BRAKE_SPEED_15PER				(0xC << 16)
#define BRAKE_SPEED_10PER				(0xD << 16)
#define BRAKE_SPEED_5PER				(0xE << 16)
#define BRAKE_SPEED_2PER5				(0xF << 16)
#define BRAKE_SPEED_THRESHOLD			BRAKE_SPEED_10PER

/* MOTOR RES (Motor Resistance) */
#define MOTOR_RES_SELF					(0x00 << 8)
#define MOTOR_RES						MOTOR_RES_SELF

/* MOTOR IND (Motor Inductance) */
#define MOTOR_IND_SELF					0x00
#define MOTOR_IND						MOTOR_IND_SELF

/* CLOSED LOOP2 Register Data */
#define CLOSED_LOOP2_DATA				(MTR_STOP | MTR_STOP_BRK_TIME | ACT_SPIN_THR | BRAKE_SPEED_THRESHOLD | MOTOR_RES | MOTOR_IND)

/*
 * CLOSED LOOP3
 */
/* MOTOR_BEMF_CONST */
#define MOTOR_BEMF_SELF					(0x00 << 23)
#define MOTOR_BEMF_CONST				MOTOR_BEMF_SELF

/* CURR LOOP KP (Current iq and id Loop Kp = Value / 10 ^ SCALE) */
#define CURR_LOOP_KP_SCALE_0			(0x0 << 21)
#define CURR_LOOP_KP_SCALE_1			(0x1 << 21)
#define CURR_LOOP_KP_SCALE_2			(0x2 << 21)
#define CURR_LOOP_KP_SCALE_3			(0x3 << 21)
#define CURR_LOOP_KP_SCALE				CURR_LOOP_KP_SCALE_0	// 0x10
#define CURR_LOOP_KP_VALUE				(0x04 << 13)			// 0x2D
#define CURR_LOOP_KP					(CURR_LOOP_KP_SCALE | CURR_LOOP_KI_VALUE)

/* CURR LOOP KI (Current iq and id Loop Ki = 1000 * Value / 10 ^ SCALE) */
#define CURR_LOOP_KI_SCALE_0			(0x0 << 11)
#define CURR_LOOP_KI_SCALE_1			(0x1 << 11)
#define CURR_LOOP_KI_SCALE_2			(0x2 << 11)
#define CURR_LOOP_KI_SCALE_3			(0x3 << 11)
#define CURR_LOOP_KI_SCALE				CURR_LOOP_KI_SCALE_0	// 0x01
#define CURR_LOOP_KI_VALUE				(0x0D << 3)				// 0x52
#define CURR_LOOP_KI					(CURR_LOOP_KI_SCALE | CURR_LOOP_KI_VALUE)

/* SPD LOOP KP (Speed Loop Kp = 0.01 * Value / 10 ^ SCALE) */
#define SPD_LOOP_KP_SCALE_0				(0x0 << 9)
#define SPD_LOOP_KP_SCALE_1				(0x1 << 9)
#define SPD_LOOP_KP_SCALE_2				(0x2 << 9)
#define SPD_LOOP_KP_SCALE_3				(0x3 << 9)
#define SPD_LOOP_KP_SCALE				SPD_LOOP_KP_SCALE_0
#define SPD_LOOP_KP_VALUE				0x0D
#define SPD_LOOP_KP_1					((SPD_LOOP_KP_SCALE | SPD_LOOP_KP_VALUE) >> 7)
#define SPD_LOOP_KP_2					(((SPD_LOOP_KP_SCALE | SPD_LOOP_KP_VALUE) & 0x7F) << 24)

/* CLOSED LOOP3 Register Data */
#define CLOSED_LOOP3_DATA				(MOTOR_BEMF_CONST | CURR_LOOP_KP | CURR_LOOP_KI | SPD_LOOP_KP_1)

/*
 * CLOSED LOOP4
 */
/* SPD LOOP KI (Speed Loop Ki = 0.1 * Value / 10 ^ SCALE) */
#define SPD_LOOP_KI_SCALE_0				(0x0 << 22)
#define SPD_LOOP_KI_SCALE_1				(0x1 << 22)
#define SPD_LOOP_KI_SCALE_2				(0x2 << 22)
#define SPD_LOOP_KI_SCALE_3				(0x3 << 22)
#define SPD_LOOP_KI_SCALE				SPD_LOOP_KI_SCALE_0
#define SPD_LOOP_KI_VALUE				(0x0D << 14)
#define SPD_LOOP_KI						(SPD_LOOP_KI_SCALE | SPD_LOOP_KI_VALUE)

/* MAX SPEED (Maximum value of Speed in Hz) */
#define MAX_SPEED_VALUE					0x32A						/* ECX SPEED 16M 36V 48600 RPM = 810 Hz = 0x32A*/
#define MAX_SPEED						(MAX_SPEED_VALUE * 6)		/* MAX_SPEED = 0x32A * 6 = 0x12FC */

/* CLOSED LOOP4  */
#define CLOSED_LOOP4_DATA 				(SPD_LOOP_KP_2 | SPD_LOOP_KI | MAX_SPEED)

/*
 * REF PROFILES1
 */
/* REF PROFILE CONFIG (Reference Profile Configuration) */
#define REF_PROFILE_REFERENCE			(0x0 << 29)
#define REF_PROFILE_LINEAR				(0x1 << 29)
#define REF_PROFILE_STAIRCASE			(0x2 << 29)
#define REF_PROFILE_FOWARD_REVERSE		(0x3 << 29)
#define REF_PROFILE_CONFIG				REF_PROFILE_REFERENCE

/* DUTY ON1 (Turn On Duty Cycle) */
#define DUTY_ON_CYCLE					0x1
#define DUTY_ON1						(DUTY_ON_CYCLE << 21)

/* DUTY OFF1 (Turn Off Duty Cycle) */
#define DUTY_OFF_CYCLE					0x1
#define DUTY_OFF1						(DUTY_OFF_CYCLE << 0x13)

/* DUTY CLAMP (Duty Cycle for Clamping Duty Input) */
#define DUTY_CLAMP_CYCLE				0x2
#define DUTY_CLAMP						(DUTY_CLAMP_CYCLE << 5)

/* DUTY A (Duty A Setting) */
#define DUTY_A_CYCLE					0x20
#define DUTY_A1							(DUTY_A_CYCLE >> 3)
#define DUTY_A2							((DUTY_A_CYCLE & 0x7) << 28)

/* REF PROFILE1 DATA */
#define REF_PROFILE1_DATA				(REF_PROFILE_CONFIG | DUTY_ON1 | DUTY_OFF1 | DUTY_CLAMP | DUTY_A1)

/*
 * REF PROFILE2
 */
/* DUTY B (Duty B Setting) */
#define DUTY_B_CYCLE					0x40
#define DUTY_B							(DUTY_B_CYCLE << 20)

/* DUTY C (Duty C Setting) */
#define DUTY_C_CYCLE					0x60
#define DUTY_C							(DUTY_C_CYCLE << 12)

/* DUTY D (Duty D Setting) */
#define DUTY_D_CYCLE					0x80
#define DUTY_D							(DUTY_D_CYCLE << 4)

/* DUTY E (Duty E Setting) */
#define DUTY_E_CYCLE					0xA0
#define DUTY_E1							(DUTY_E_CYCLE >> 4)
#define DUTY_E2							((DUTY_E_CYCLE & 0xf) << 27)

/* REF PROFILE2 Register Data */
#define REF_PROFILE2_DATA				(DUTY_A2 | DUTY_B | DUTY_C | DUTY_D | DUTY_E1)

/*
 * REF PROFILE3
 */
/* DUTY ON2 */
#define DUTY_ON2_CYCLE					0xE0
#define DUTY_ON2						(DUTY_ON2_CYCLE << 19)

/* DUTY OFF2 */
#define DUTY_OFF2_CYCLE					0xF0
#define DUTY_OFF2						(DUTY_OFF2_CYCLE << 11)

/* DUTY CLAMP2 */
#define DUTY_CLAMP2_CYCLE				0xC0
#define DUTY_CLAMP2						(DUTY_CLAMP2_CYCLE << 3)

/* DUTY_HYS */
#define DUTY_HYS_0PER					0x0
#define DUTY_HYS_0PER5					0x1
#define DUTY_HYS_1PER					0x2
#define DUTY_HYS_2PER					0x3
#define DUTY_HYS						(DUTY_HYS_0PER << 1)

/* REF PROFILE3 Register Data */
#define REF_PROFILE3_DATA				(DUTY_E2 | DUTY_ON2 | DUTY_OFF2 | DUTY_CLAMP2 | DUTY_HYS)

/*
 * REF PROFILE4
 */
/* REF OFF1 */
#define REF_OFF1_VALUE					0x01
#define REF_OFF1						(REF_OFF1_VALUE << 23)

/* REF CLAMP */
#define REF_CLAMP1_VALUE				0x01
#define REF_CLAMP1						(REF_CLAMP1_VALUE << 15)

/* REF A */
#define REF_A_VALUE						0x20
#define REF_A							(REF_A_VALUE << 7)

/* REF B */
#define REF_B_VALUE						0x40
#define REF_B1							(REF_B_VALUE >> 1)
#define REF_B2							((REF_B_VALUE & 0x1) << 31)

/* REF PROFILE4 Register Data */
#define REF_PROFILE4_DATA				(REF_OFF1 | REF_CLAMP1 | REF_A | REF_B1)

/*
 * REF PROFILE5
 */
/* REF C */
#define REF_C_VALUE						0x60
#define REF_C							(REF_C_VALUE << 22)

/* REF D */
#define REF_D_VALUE						0x80
#define REF_D							(REF_D_VALUE << 14)

/* REF E */
#define REF_E_VALUE						0xA0
#define REF_E							(REF_E_VALUE << 6)

/* REF PROFILE5 Register Data */
#define REF_PROFILE5_DATA				(REF_B2 | REF_C | REF_D | REF_E)

/*
 * REF PROFILE6
 */
/* REF OFF2 */
#define REF_OFF2_VALUE					0x00
#define REF_OFF2						(REF_OFF2_VALUE << 23)

/* REF CLAMP2 */
#define REF_CLAMP2_VALUE				0xC0
#define	REF_CLAMP2						(REF_CLAMP2_VALUE << 15)

/* REF PROFILE6 Register Data*/
#define REF_PROFILE6_DATA				(REF_OFF2 | REF_CLAMP2)

/*
 * FAULT CONFIG1
 */
/* ILIMIT (Current Limit for Iq in Closed Loop) */
#define ILIMIT_0A125		(0x0 << 27)
#define ILIMIT_0A25			(0x1 << 27)
#define ILIMIT_0A5			(0x2 << 27)
#define ILIMIT_1A			(0x3 << 27)
#define ILIMIT_1A5			(0x4 << 27)
#define ILIMIT_2A			(0x5 << 27)
#define ILIMIT_2A5			(0x6 << 27)
#define ILIMIT_3A			(0x7 << 27)
#define ILIMIT_3A5			(0x8 << 27)
#define ILIMIT_4A			(0x9 << 27)
#define ILIMIT_4A5			(0xA << 27)
#define ILIMIT_5A			(0xB << 27)
#define ILIMIT_5A5			(0xC << 27)
#define ILIMIT_6A			(0xD << 27)
#define ILIMIT_7A			(0xE << 27)
#define ILIMIT_8A			(0xF << 27)
#define ILIMIT				ILIMIT_3A

/* HW LOCK LIMIT (Comparator Based Lock Detection Current Threshold) */
#define HW_LOCK_LIMIT_0A125			(0x0 << 23)
#define HW_LOCK_LIMIT_0A25			(0x1 << 23)
#define HW_LOCK_LIMIT_0A5			(0x2 << 23)
#define HW_LOCK_LIMIT_1A			(0x3 << 23)
#define HW_LOCK_LIMIT_1A5			(0x4 << 23)
#define HW_LOCK_LIMIT_2A			(0x5 << 23)
#define HW_LOCK_LIMIT_2A5			(0x6 << 23)
#define HW_LOCK_LIMIT_3A			(0x7 << 23)
#define HW_LOCK_LIMIT_3A5			(0x8 << 23)
#define HW_LOCK_LIMIT_4A			(0x9 << 23)
#define HW_LOCK_LIMIT_4A5			(0xA << 23)
#define HW_LOCK_LIMIT_5A			(0xB << 23)
#define HW_LOCK_LIMIT_5A5			(0xC << 23)
#define HW_LOCK_LIMIT_6A			(0xD << 23)
#define HW_LOCK_LIMIT_7A			(0xE << 23)
#define HW_LOCK_LIMIT_8A			(0xF << 23)
#define HW_LOCK_LIMIT				HW_LOCK_LIMIT_6A

/* LOCK ILIMIT (ADC Based Lock Detection Current Threshold) */
#define LOCK_ILIMIT_0A125			(0x0 << 19)
#define LOCK_ILIMIT_0A25			(0x1 << 19)
#define LOCK_ILIMIT_0A5				(0x2 << 19)
#define LOCK_ILIMIT_1A				(0x3 << 19)
#define LOCK_ILIMIT_1A5				(0x4 << 19)
#define LOCK_ILIMIT_2A				(0x5 << 19)
#define LOCK_ILIMIT_2A5				(0x6 << 19)
#define LOCK_ILIMIT_3A				(0x7 << 19)
#define LOCK_ILIMIT_3A5				(0x8 << 19)
#define LOCK_ILIMIT_4A				(0x9 << 19)
#define LOCK_ILIMIT_4A5				(0xA << 19)
#define LOCK_ILIMIT_5A				(0xB << 19)
#define LOCK_ILIMIT_5A5				(0xC << 19)
#define LOCK_ILIMIT_6A				(0xD << 19)
#define LOCK_ILIMIT_7A				(0xE << 19)
#define LOCK_ILIMIT_8A				(0xF << 19)
#define LOCK_ILIMIT					LOCK_ILIMIT_5A

/* LOCK ILIMIT MODE (Lock Current Limit Mode) */
#define LOCK_ILIMIT_MODE_0					(0x0 << 15)
#define LOCK_ILIMIT_MODE_1					(0x1 << 15)
#define LOCK_ILIMIT_MODE_2					(0x2 << 15)
#define LOCK_ILIMIT_MODE_3					(0x3 << 15)
#define LOCK_ILIMIT_MODE_4					(0x4 << 15)
#define LOCK_ILIMIT_MODE_5					(0x5 << 15)
#define LOCK_ILIMIT_MODE_6					(0x6 << 15)
#define LOCK_ILIMIT_MODE_7					(0x7 << 15)
#define LOCK_ILIMIT_MODE_8					(0x8 << 15)
#define LOCK_ILIMIT_MODE_DISABLE			(0x9 << 15)
#define LOCK_ILIMIT_MODE					(LOCK_ILIMIT_MODE_4)

/* LOCK ILIMIT DEG (Lock Protection Current Limit Deglitch Time) */
#define LOCK_ILIMIT_DEG_DISABLE				(0x0 << 11)
#define LOCK_ILIMIT_DEG_0MS2				(0x2 << 11)
#define LOCK_ILIMIT_DEG_0MS5				(0x3 << 11)
#define LOCK_ILIMIT_DEG_1MS					(0x4 << 11)
#define LOCK_ILIMIT_DEG_2MS5				(0x5 << 11)
#define LOCK_ILIMIT_DEG_5MS					(0x6 << 11)
#define LOCK_ILIMIT_DEG_7MS5				(0x7 << 11)
#define LOCK_ILIMIT_DEG_10MS				(0x8 << 11)
#define LOCK_ILIMIT_DEG_25MS				(0x9 << 11)
#define LOCK_ILIMIT_DEG_50MS				(0xA << 11)
#define LOCK_ILIMIT_DEG_75MS				(0xB << 11)
#define LOCK_ILIMIT_DEG_100MS				(0xC << 11)
#define LOCK_ILIMIT_DEG_200MS				(0xD << 11)
#define LOCK_ILIMIT_DEG_500MS				(0xE << 11)
#define LOCK_ILIMIT_DEG_1S					(0xF << 11)
#define LOCK_ILIMIT_DEG						LOCK_ILIMIT_DEG_5MS

/* LCK RETRY (Lock Deetection Retry Time) */
#define LCK_RETRY_0S3						(0x0 << 7)
#define LCK_RETRY_0S5						(0x1 << 7)
#define LCK_RETRY_1S						(0x2 << 7)
#define LCK_RETRY_2S						(0x3 << 7)
#define LCK_RETRY_3S						(0x4 << 7)
#define LCK_RETRY_4S						(0x5 << 7)
#define LCK_RETRY_5S						(0x6 << 7)
#define LCK_RETRY_6S						(0x7 << 7)
#define LCK_RETRY_7S						(0x8 << 7)
#define LCK_RETRY_8S						(0x9 << 7)
#define LCK_RETRY_9S						(0xA << 7)
#define LCK_RETRY_10S						(0xB << 7)
#define LCK_RETRY_11S						(0xC << 7)
#define LCK_RETRY_12S						(0xD << 7)
#define LCK_RETRY_13S						(0xE << 7)
#define LCK_RETRY_14S						(0xF << 7)
#define LCK_RETRY							LCK_RETRY_0S5

/* MTR LCK MODE (Motor Lock Mode) */
#define MTR_LCK_MODE_0						(0x0 << 3)
#define MTR_LCK_MODE_1						(0x1 << 3)
#define MTR_LCK_MODE_2						(0x2 << 3)
#define MTR_LCK_MODE_3						(0x3 << 3)
#define MTR_LCK_MODE_4						(0x4 << 3)
#define MTR_LCK_MODE_5						(0x5 << 3)
#define MTR_LCK_MODE_6						(0x6 << 3)
#define MTR_LCK_MODE_7						(0x7 << 3)
#define MTR_LCK_MODE_8						(0x8 << 3)
#define MTR_LCK_MODE_DISABLE				(0x9 << 3)
#define MTR_LCK_MODE						MTR_LCK_MODE_4

/* IPD TIMEOUT FAULT EN (IPD Timeout Fault Enable */
#define IPD_TIMEOUT_FAULT_ENABLE			(0x1 << 2)
#define IPD_TIMEOUT_FAULT_DISABLE			(0x0 << 2)
#define IPD_TIMEOUT_FAULT_EN				IPD_TIMEOUT_FAULT_DISABLE

/* IPD FREQ FAULT EN (IPD Frequency Fault Enable) */
#define IPD_FREQUENCY_FAULT_ENABLE			(0x1 << 1)
#define IPD_FREQUENCY_FAULT_DISABLE			(0x0 << 1)
#define IPD_FREQUENCY_FAULT_EN				IPD_FREQUENCY_FAULT_ENABLE

/* SATURATION FLAGS EN (Indication of Current Loop and Speed Loop Saturation Enable) */
#define SATURATION_FLAGS_ENABLE				0x1
#define SATURATION_FLAGS_DISABLE			0x0
#define SATURATION_FLAGS_EN					SATURATION_FLAGS_DISABLE

/* FAULT CONFIG1 Register Data */
#define FAULT_CONFIG1_DATA					(ILIMIT | HW_LOCK_LIMIT | LOCK_ILIMIT | LOCK_ILIMIT_MODE | LOCK_ILIMIT_DEG | LCK_RETRY | MTR_LCK_MODE | IPD_TIMEOUT_FAULT_EN | IPD_FREQUENCY_FAULT_EN | SATURATION_FLAGS_EN)

/*
 * FAULT CONFIG2
 */
/* LOCK1 EN (Abnormal Speed Enable) */
#define LOCK1_ENABLE						(0x1 << 30)
#define LOCK1_DISABLE						(0x0 << 30)
#define LOCK1_EN							LOCK1_ENABLE

/* LOCK2 EN (Abnormal BEMF Enable) */
#define LOCK2_ENABLE						(0x1 << 29)
#define LOCK2_DISABLE						(0x0 << 29)
#define LOCK2_EN							LOCK2_ENABLE

/* LOCK3 EN (Abnormal No Motor Enable) */
#define LOCK3_ENABLE						(0x1 << 28)
#define LOCK3_DISABLE						(0x0 << 28)
#define LOCK3_EN							LOCK3_ENABLE

/* LOCK ABN SPEED (Abnormal Speed Lock Threshold) */
#define LOCK_ABN_SPEED_130PER				(0x0 << 25)
#define LOCK_ABN_SPEED_140PER				(0x1 << 25)
#define LOCK_ABN_SPEED_150PER				(0x2 << 25)
#define LOCK_ABN_SPEED_160PER				(0x3 << 25)
#define LOCK_ABN_SPEED_170PER				(0x4 << 25)
#define LOCK_ABN_SPEED_180PER				(0x5 << 25)
#define LOCK_ABN_SPEED_190PER				(0x6 << 25)
#define LOCK_ABN_SPEED_200PER				(0x7 << 25)
#define LOCK_ABN_SPEED						LOCK_ABN_SPEED_130PER

/* ABNORMAL BEMF THR (Abnormal BEMF Lock Threshold) */
#define ABNORMAL_BEMF_THR_40PER				(0x0 << 22)
#define ABNORMAL_BEMF_THR_45PER				(0x1 << 22)
#define ABNORMAL_BEMF_THR_50PER				(0x2 << 22)
#define ABNORMAL_BEMF_THR_55PER				(0x3 << 22)
#define ABNORMAL_BEMF_THR_60PER				(0x4 << 22)
#define ABNORMAL_BEMF_THR_65PER				(0x5 << 22)
#define ABNORMAL_BEMF_THR_67PER5			(0x6 << 22)
#define ABNORMAL_BEMF_THR_70PER				(0x7 << 22)
#define ABNORMAL_BEMF_THR					ABNORMAL_BEMF_THR_65PER

/* NO MTR THR (No Motor Lock Threshold) */
#define NO_MTR_THR_0A075					(0x0 << 19)
#define NO_MTR_THR_0A1						(0x2 << 19)
#define NO_MTR_THR_0A125					(0x3 << 19)
#define NO_MTR_THR_0A25						(0x4 << 19)
#define NO_MTR_THR_0A5						(0x5 << 19)
#define NO_MTR_THR_0A75						(0x6 << 19)
#define NO_MTR_THR_1A						(0x7 << 19)
#define NO_MTR_THR							NO_MTR_THR_0A1

/* HW LOCK ILIMIT MODE (Hardware Lock Detection Current Mode) */
#define HW_LOCK_ILIMIT_MODE_0				(0x0 << 15)
#define HW_LOCK_ILIMIT_MODE_1				(0x1 << 15)
#define HW_LOCK_ILIMIT_MODE_2				(0x2 << 15)
#define HW_LOCK_ILIMIT_MODE_3				(0x3 << 15)
#define HW_LOCK_ILIMIT_MODE_4				(0x4 << 15)
#define HW_LOCK_ILIMIT_MODE_5				(0x5 << 15)
#define HW_LOCK_ILIMIT_MODE_6				(0x6 << 15)
#define HW_LOCK_ILIMIT_MODE_7				(0x7 << 15)
#define HW_LOCK_ILIMIT_MODE_8				(0x8 << 15)
#define HW_LOCK_ILIMIT_MODE_9				(0x9 << 15)
#define HW_LOCK_ILIMIT_MODE					HW_LOCK_ILIMIT_MODE_4

/* HW LOCK ILIMIT DEG (Hardware Lock Detection Current Limit Deglitch Time) */
#define HW_LOCK_ILIMIT_NO_DEG				(0x0 << 12)
#define HW_LOCK_ILIMIT_DEG_1US				(0x1 << 12)
#define HW_LOCK_ILIMIT_DEG_2US				(0x2 << 12)
#define HW_LOCK_ILIMIT_DEG_3US				(0x3 << 12)
#define HW_LOCK_ILIMIT_DEG_4US				(0x4 << 12)
#define HW_LOCK_ILIMIT_DEG_5US				(0x5 << 12)
#define HW_LOCK_ILIMIT_DEG_6US				(0x6 << 12)
#define HW_LOCK_ILIMIT_DEG_7IS				(0x7 << 12)
#define HW_LOCK_ILIMIT_DEG					HW_LOCK_ILIMIT_DEG_2US

/* MIN_VM_MOTOR (Minimum DC Bus Voltage for Running Motor) */
#define MIN_VM_MOTOR_NO_LIMIT				(0x0 << 8)
#define MIN_VM_MOTOR_4V5					(0x1 << 8)
#define MIN_VM_MOTOR_5V						(0x2 << 8)
#define MIN_VM_MOTOR_5V5					(0x3 << 8)
#define MIN_VM_MOTOR_6V						(0x4 << 8)
#define MIN_VM_MOTOR_7V5					(0x5 << 8)
#define MIN_VM_MOTOR_10V					(0x6 << 8)
#define MIN_VM_MOTOR_12V5					(0x7 << 8)
#define MIN_VM_MOTOR						MIN_VM_MOTOR_NO_LIMIT

/* MIN VM MODE (DC Bus Undervoltage Fault Recovery Mode) */
#define MIN_VM_MODE_LATCH					(0x0 << 7)
#define MIN_VM_MODE_AUTO					(0x1 << 7)
#define MIN_VM_MODE							MIN_VM_MODE_AUTO

/* MAX VM MOTOR (Maximum DC Bus Voltage for Running Motor) */
#define MAX_VM_MOTOR_NO_LIMIT				(0x0 << 4)
#define MAX_VM_MOTOR_20V					(0x1 << 4)
#define MAX_VM_MOTOR_22V5					(0x2 << 4)
#define MAX_VM_MOTOR_25V					(0x3 << 4)
#define MAX_VM_MOTOR_27V5					(0x4 << 4)
#define MAX_VM_MOTOR_30V					(0x5 << 4)
#define MAX_VM_MOTOR_32V5					(0x6 << 4)
#define MAX_VM_MOTOR_35V					(0x7 << 4)
#define MAX_VM_MOTOR						MAX_VM_MOTOR_NO_LIMIT

/* MAX VM MODE */
#define MAX_VM_MODE_LATCH					(0x0 << 3)
#define MAX_VM_MODE_AUTO					(0x1 << 3)
#define MAX_VM_MODE							MAX_VM_MODE_AUTO

/* AUTO RETRY TIMES (Automatic Retry Attemps) */
#define AUTO_RETRY_TIMES_NO_LIMIT			0x0
#define AUTO_RETRY_TIMES_2					0x1
#define AUTO_RETRY_TIMES_3					0x2
#define AUTO_RETRY_TIMES_5					0x3
#define AUTO_RETRY_TIMES_7					0x4
#define AUTO_RETRY_TIMES_10					0x5
#define AUTO_RETRY_TIMES_15					0x6
#define AUTO_RETRY_TIMES_20					0x7
#define AUTO_RETRY_TIMES					AUTO_RETRY_TIMES_NO_LIMIT

/* FAULT CONFIG2 Register Data */
#define FAULT_CONFIG2_DATA					(LOCK1_EN | LOCK2_EN | LOCK3_EN | LOCK_ABN_SPEED | ABNORMAL_BEMF_THR | NO_MTR_THR | HW_LOCK_ILIMIT_MODE | HW_LOCK_ILIMIT_DEG | MIN_VM_MOTOR | MIN_VM_MODE | MAX_VM_MOTOR | MAX_VM_MODE | AUTO_RETRY_TIMES)

/*
 * PIN CONFIG
 */
/* VDC FILTER DISABLE (VM Filter Disable) */
#define VDC_FILTER_DISABLE					(0x0 << 27)
#define VDC_FILTER_ENABLE					(0x1 << 27)
#define VDC_FILTER_DIS						VDC_FILTER_DISABLE

/* FG IDLE CONFIG (FG Configuration During Stop) */
#define FG_IDLE_CONFIG_FG_CONFIG			(0x0 << 9)
#define FG_IDLE_CONFIG_PULLED_HIGH			(0x1 << 9)
#define FG_IDLE_CONFIG_PULLED_LOW			(0x2 << 9)
#define FG_IDLE_CONFIG						FG_IDLE_CONFIG_PULLED_HIGH

/* FG FAULT CONFIG (FG Configuration During Fault */
#define FG_FAULT_CONFIG_LAST_SIGNAL			(0x0 << 7)
#define FG_FAULT_CONFIG_PULLED_HIGH			(0x1 << 7)
#define FG_FAULT_CONFIG_PULLED_LOW			(0x2 << 7)
#define FG_FAULT_CONFIG_FG_CONFIG			(0x3 << 7)
#define FG_FAULT_CONFIG						FG_FAULT_CONFIG_PULLED_LOW

/* ALARM PIN EN (Alarm Pin Enable) */
#define ALARM_PIN_ENABLE					(0x1 << 6)
#define ALARM_PIN_DISABLE					(0x0 << 6)
#define ALARM_PIN_EN						ALARM_PIN_DISABLE

/* BRAKE PIN MODE (Brake Pin Mode) */
#define BRAKE_PIN_LOW_SIDE					(0x0 << 5)
#define BRAKE_PIN_ALIGN						(0x1 << 5)
#define BRAKE_PIN_MODE						BRAKE_PIN_LOW_SIDE

/* ALIGN BRAKE ANGLE SEL (Align Brake Angle Select) */
#define ALIGN_BRAKE_ANGLE_LAST				(0x0 << 4)
#define ALIGN_BRAKE_ANGLE_ALIGN				(0x1 << 4)
#define ALIGN_BRAKE_ANGLE_SEL				ALIGN_BRAKE_ANGLE_LAST

/* BRAKE INPUT (Brake Input Override) */
#define BRAKE_INPUT_HW_PIN					(0x0 << 2)
#define BRAKE_INPUT_BRAKE_PIN_MODE			(0x1 << 2)
#define BRAKE_INPUT_DIGITAL_SPEED_CTRL		(0x2 << 2)
#define BRAKE_INPUT_DIGITAL_SPEED_FREQ		(0x3 << 2)
#define BRAKE_INPUT							BRAKE_INPUT_DIGITAL_SPEED_CTRL

/* SPEED MODE (Configure Motor Control Input Source)*/
#define SPEED_MODE_ANAL_SPEED				0x0
#define SPEED_MODE_PWM_SPEED				0x1
#define SPEED_MODE_DIGITAL_SPEED			0x2
#define SPEED_MODE_FREQ_SPEED				0x3
#define SPEED_MODE							SPEED_MODE_PWM_SPEED

/* PIN_CONFIG Register Data */
#define PIN_CONFIG_DATA						(VDC_FILTER_DIS | FG_IDLE_CONFIG | FG_FAULT_CONFIG | ALARM_PIN_EN | BRAKE_PIN_MODE | ALIGN_BRAKE_ANGLE_SEL | BRAKE_INPUT | SPEED_MODE)

/*
 * DEVICE_CONFIG1
 */
/* DAC_SOx_SEL (Select Between DAC2 and SOx Channels) */
#define DAC_SOx_SEL_DACOUT2					(0x0 << 28)
#define DAC_SOx_SEL_SOA						(0x1 << 28)
#define DAC_SOx_SEL_SOB						(0x2 << 28)
#define DAC_SOx_SEL_SOC						(0x3 << 28)
#define DAC_SOx_SEL							DAC_SOx_SEL_DACOUT2

/* DAC_ENABLE (DAC1 and DAC2 Enable) */
#define DAC_ENABLE							(0x1 << 27)
#define DAC_DISABLE							(0X0 << 27)
#define DAC_EN								DAC_DISABLE

/* I2C TARGET ADDR (I2C Target Address) */
#define I2C_TARGET_ADDR						(0x01 << 20)

/* SLEW_RATE_I2C_PINS (Slew Rate Control for I2C Pins) */
#define SLEW_RATE_I2C_PINS_4MA8				(0x0 << 3)
#define SLEW_RATE_I2C_PINS_3MA9				(0x1 << 3)
#define SLEW_RATE_I2C_PINS_1MA86			(0x2 << 3)
#define SLEW_RATE_I2C_PINS_30MA8			(0x3 << 3)
#define SLEW_RATE_I2C_PINS					SLEW_RATE_I2C_PINS_4MA8

/* PULLUP ENABLE (PULL-UP Enable for nFAULT and FG Pins) */
#define PULLUP_ENABLE						(0x1 << 2)
#define PULLUP_DISABLE						(0x0 << 2)
#define PULLUP_EN							PULLUP_DISABLE

/* BUS VOLT (Maximum DC Bus Voltage Configuration) */
#define BUS_VOLT_15V						0x0
#define BUS_VOLT_30V						0x1
#define BUS_VOLT_60V						0x2
#define BUS_VOLT_NOT_DEFINED				0x3
#define BUS_VOLT							BUS_VOLT_60V

/* DEVICE CONFIG1 Register Data */
#define DEVICE_CONFIG1_DATA					(DAC_SOx_SEL | DAC_ENABLE | I2C_TARGET_ADDR | SLEW_RATE_I2C_PINS | PULLUP_ENABLE | BUS_VOLT)

/*
 * DEVICE CONFIG2
 */
/* INPUT_MAXIMUM_FREQ (Input Frequency on Speed Pin)*/
#define INPUT_MAXIMUM_FREQ					(0x61A8 << 16)	// 9600

/* SLEEP ENTRY TIME (Device Enters Sleep Mode Threshold) */
#define SLEEP_ENTRY_TIME_50US				(0x0 << 14)
#define SLEEP_ENTRY_TIME_200US				(0x1 << 14)
#define SLEEP_ENTRY_TIME_20MS				(0x2 << 14)
#define SLEEP_ENTRY_TIME_200MS				(0x3 << 14)
#define SLEEP_ENTRY_TIME					SLEEP_ENTRY_TIME_20MS

/* DYNAMIC_CSA_GAIN_EN (Adjust CSA gain Dynamically) */
#define DYNAMIC_CSA_GAIN_ENABLE				(0x1 << 13)
#define DYNAMIC_CSA_GAIN_DISABLE			(0x0 << 13)
#define DYNAMIC_CSA_GAIN_EN					DYNAMIC_CSA_GAIN_ENABLE

/* DYNAMIC_VOLTAGE_GAIN_EN (Adjust Voltage Gain Dynamically) */
#define DYNAMIC_VOLTAGE_GAIN_ENABLE			(0x1 << 12)
#define DYNAMIC_VOLTAGE_GAIN_DISABLE		(0x0 << 12)
#define DYNAMIC_VOLTAGE_GAIN_EN				DYNAMIC_VOLTAGE_GAIN_DISABLE

/* DEV MODE (Device Mode Select) */
#define DEV_MODE_STANDBY					(0x0 << 11)
#define DEV_MODE_SLEEP						(0x1 << 11)
#define DEV_MODE							DEV_MODE_STANDBY

/* CLK SEL (Clock Source) */
#define CLK_SEL_INT_OSCILLATOR				(0x0 << 9)
#define CLK_SEL_WDT							(0x1 << 9)
#define CLK_SEL_NOT_APPLICABLE				(0x2 << 9)
#define CLK_SEL_EXT_CLOCK_INPUT				(0x3 << 9)
#define CLK_SEL								CLK_SEL_INT_OSCILLATOR

/* EXT CLK EN (Enable External Clock Mode) */
#define EXT_CLK_ENABLE						(0x0 << 8)
#define EXT_CLK_DISABLE						(0x1 << 8)
#define EXT_CLK_EN							EXT_CLK_DISABLE

/* EXT CLK CONFIG (External Clock Configuration) */
#define EXT_CLK_CONFIG_8K					(0x0 << 5)
#define EXT_CLK_CONFIG_16K					(0x1 << 5)
#define EXT_CLK_CONFIG_32K					(0x2 << 5)
#define EXT_CLK_CONFIG_64K					(0x3 << 5)
#define EXT_CLK_CONFIG_128K					(0x4 << 5)
#define EXT_CLK_CONFIG_256K					(0x5 << 5)
#define EXT_CLK_CONFIG_512K					(0x6 << 5)
#define EXT_CLK_CONFIG_1024K				(0x7 << 5)
#define EXT_CLK_CONFIG						EXT_CLK_CONFIG_8K

/* EXT WDT EN (Enable External Watchdog) */
#define EXT_WDT_ENABLE						(0x1 << 4)
#define EXT_WDT_DISABLE						(0x0 << 4)
#define EXT_WDT_EN							EXT_WDT_DISABLE

/* EXT WDT CONFIG (Time Between Watchdog Tickles) */
#define EXT_WDT_CONFIG_100MS				(0x0 << 2)
#define EXT_WDT_CONFIG_200MS				(0x1 << 2)
#define EXT_WDT_CONFIG_500MS				(0x2 << 2)
#define EXT_WDT_CONFIG_1S					(0x3 << 2)
#define EXT_WDT_CONFIG						EXT_WDT_CONFIG_100MS

/* EXT WDT INPUT MODE (External Watchdog Input Mode) */
#define EXT_WDT_INPUT_MODE_I2C				(0x0 << 1)
#define EXT_WDT_INPUT_MODE_GPIO				(0x1 << 1)
#define EXT_WDT_INPUT_MODE					EXT_WDT_INPUT_MODE_I2C

/* EXT WDT FAULT MODE (External Watchdog Fault Mode) */
#define EXT_WDT_FAULT_MODE_REPORT_ONLY		0x0
#define EXT_WDT_FAULT_MODE_LATCH_HIZ		0x1
#define EXT_WDT_FAULT_MODE					EXT_WDT_FAULT_MODE_REPORT_ONLY

/* DEVICE CONFIG2 Register Data */
#define DEVICE_CONFIG2_DATA					(INPUT_MAXIMUM_FREQ | SLEEP_ENTRY_TIME | DYNAMIC_CSA_GAIN_EN | DYNAMIC_VOLTAGE_GAIN_EN | DEV_MODE | CLK_SEL | EXT_CLK_EN | EXT_CLK_CONFIG | EXT_WDT_EN | EXT_WDT_CONFIG | EXT_WDT_INPUT_MODE | EXT_WDT_FAULT_MODE)

/*
 * PERI CONFIG1
 */
/* SPREAD SPECTRUM MODULATION DIS (Spread Spectrum Modulation Disable) */
#define SPREAD_SPECTRUM_MODULATION_ENABLE		(0x0 << 30)
#define SPREAD_SPECTRUM_MODULATION_DISABLE		(0X1 << 30)
#define SPREAD_SPECTRUM_MODULATION_DIS			SPREAD_SPECTRUM_MODULATION_DISABLE

/* BUS CURRENT LIMIT (Bus Current Limit) */
#define BUS_CURRENT_LIMIT_0A125					(0x0 << 22)
#define BUS_CURRENT_LIMIT_0A25					(0x1 << 22)
#define BUS_CURRENT_LIMIT_0A5					(0x2 << 22)
#define BUS_CURRENT_LIMIT_1A					(0x3 << 22)
#define BUS_CURRENT_LIMIT_1A5					(0x4 << 22)
#define BUS_CURRENT_LIMIT_2A					(0x5 << 22)
#define BUS_CURRENT_LIMIT_2A5					(0x6 << 22)
#define BUS_CURRENT_LIMIT_3A					(0x7 << 22)
#define BUS_CURRENT_LIMIT_3A5					(0x8 << 22)
#define BUS_CURRENT_LIMIT_4A					(0x9 << 22)
#define BUS_CURRENT_LIMIT_4A5					(0xA << 22)
#define BUS_CURRENT_LIMIT_5A					(0xB << 22)
#define BUS_CURRENT_LIMIT_5A5					(0xC << 22)
#define BUS_CURRENT_LIMIT_6A					(0xD << 22)
#define BUS_CURRENT_LIMIT_7A					(0xE << 22)
#define BUS_CURRENT_LIMIT_8A					(0xF << 22)
#define BUS_CURRENT_LIMIT						BUS_CURRENT_LIMIT_3A

/* BUS CURRENT LIMIT ENABLE (Bus Current Limit Enable) */
#define BUS_CURRENT_LIMIT_ENABLE				(0x1 << 21)
#define BUS_CURRENT_LIMIT_DISABLE				(0x0 << 21)
#define BUS_CURRENT_LIMIT_EN					BUS_CURRENT_LIMIT_DISABLE

/* DIR INPUT (DIR Pin Override) */
#define DIR_INPUT_HW							(0x0 << 19)
#define DIR_INPUT_OVERRIDE_OUTA_B_C				(0x1 << 19)
#define DIR_INPUT_OVERRIDE_OUTA_C_B				(0x2 << 19)
#define DIR_INPUT								DIR_INPUT_HW

/* DIR_CHANGE_MODE (Response to Change of DIR) */
#define DIR_CHANGE_MODE_STOP_DRIVE				(0x0 << 18)
#define DIR_CHANGE_MODE_REVERSE_DRIVE			(0x1 << 18)
#define DIR_CHANGE_MODE							DIR_CHANGE_MODE_REVERSE_DRIVE

/* SELF TEST ENABLE (Enable Self-test on Power UP) */
#define SELF_TEST_ENABLE						(0x1 << 17)
#define SELF_TEST_DISABLE						(0x0 << 17)
#define SELF_TEST_EN							SELF_TEST_DISABLE

/* ACTIVE BRAKE SPEED DELTA LIMIT ENTRY (Difference Between Final Speed and Present Speed) */
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_NO			(0x0 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_5PER		(0x1 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_10PER		(0x2 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_15PER		(0x3 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_20PER		(0x4 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_25PER		(0x5 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_30PER		(0x6 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_35PER		(0x7 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_40PER		(0x8 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_45PER		(0x9 << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_50PER		(0xA << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_60PER		(0xB << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_70PER		(0xC << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_80PER		(0xD << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_90PER		(0xE << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_100PER		(0xF << 13)
#define ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY			ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY_10PER

/* ACTIVE BRAKE MODE INDEX LIMIT (Modulation Index Limit) */
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_0PER			(0x0 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_40PER			(0x1 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_50PER			(0x2 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_60PER			(0x3 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_70PER			(0x4 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_80PER			(0x5 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_90PER			(0x6 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT_100PER			(0x7 << 10)
#define ACTIVE_BRAKE_MOD_INDEX_LIMIT				ACTIVE_BRAKE_MOD_INDEX_LIMIT_100PER

/* SPEED RANGE SEL */
#define SPEED_RANGE_SEL_HIGH						(0x0 << 9)
#define SPEED_RANGE_SEL_LOW							(0x1 << 9)
#define SPEED_RANGE_SEL								SPEED_RANGE_SEL_HIGH

/* PERI_CONFIG1 Register Data*/
#define PERI_CONFIG1_DATA		(SPREAD_SPECTRUM_MODULATION_DIS | BUS_CURRENT_LIMIT | BUS_CURRENT_LIMIT_EN | DIR_INPUT | DIR_CHANGE_MODE | SELF_TEST_EN | ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY | ACTIVE_BRAKE_MOD_INDEX_LIMIT | SPEED_RANGE_SEL)

/*
 * GD_CONFIG1
 */
/* PARITY */
#define PARITY					(0x0 << 31)

/* RESERVED */
#define RESERVED_PARITY			0x10228000

/* SLEW RATE (Slew Rate) */
#define SLEW_RATE_NO_APPLICABLE			(0x0 << 26)
#define SLEW_RATE_125V					(0x2 << 26)
#define SLEW_RATE_200V					(0x3 << 26)
#define SLEW_RATE						SLEW_RATE_200V

/* OVP_SEL (Overvoltage Level) */
#define OVP_34V							(0x0 << 19)
#define OVP_22V							(0x1 << 19)
#define OVP_SEL							OVP_34V

/* OVP EN (Overvoltage Enable) */
#define OVP_ENABLE						(0x1 << 18)
#define OVP_DISABLE						(0x0 << 18)
#define OVP_EN							OVP_DISABLE

/* OTW REP (Overtemperature Warning Enable) */
#define OTW_REP_ENABLE					(0x1 << 17)
#define OTW_REP_DISABLE					(0x0 << 17)
#define OTW_REP							OTW_REP_ENABLE

/* OCP DEG (OCP Deglitch Time Setting) */
#define OCP_DEG_0US2					(0x0 << 12)
#define OCP_DEG_0US6					(0x1 << 12)
#define OCP_DEG_1US1					(0x2 << 12)
#define OCP_DEG_1US6					(0x3 << 12)
#define OCP_DEG							OCP_DEG_0US6

/* OCP LVL (Overcurrent Level Setting) */
#define OCP_LVL_16A						(0x0 << 10)
#define OCP_LVL_24A						(0x1 << 10)
#define OCP_LVL							OCP_LVL_16A

/* OCP MODE (OCP Fault Mode) */
#define OCP_MODE_LATCH					(0x0 << 8)
#define OCP_MODE_RETRY_500MS			(0x1 << 8)
#define OCP_MODE_NOT_APPLICABLE			(0x2 << 8)
#define OCP_MODE						OCP_MODE_RETRY_500MS

/* CSA GAIN (Current Sense Amplifier Gain) */
#define CSA_GAIN_0V15					0x0
#define CSA_GAIN_0V3					0x1
#define CSA_GAIN_0V6					0x2
#define CSA_GAIN_1V2					0x3
#define CSA_GAIN						CSA_GAIN_0V15

/* GD CONFIG1 Register Data */
#define GD_CONFIG1_DATA					(PARITY | RESERVED_PARITY | SLEW_RATE | OVP_SEL | OVP_EN | OTW_REP | OCP_DEG | OCP_LVL | OCP_MODE | CSA_GAIN)

/*
 * GD CONFIG2
 */
/* BUCK PS DIS (Buck Power Sequencing Disable) */
#define BUCK_PS_ENABLE					(0x0 << 24)
#define BUCK_PS_DISABLE					(0x1 << 24)
#define BUCK_PS_DIS						BUCK_PS_DISABLE

/* BUCK CL (Buck Current Limit) */
#define BUCK_CL_600MA					(0x0 << 23)
#define BUCK_CL_150MA					(0x1 << 23)
#define BUCK_CL							BUCK_CL_150MA

/* BUCK SEL (Buck Voltage) */
#define BUCK_SEL_3V3					(0x0 << 21)
#define BUCK_SEL_5V						(0x1 << 21)
#define BUCK_SEL_4V						(0x2 << 21)
#define BUCK_SEL_5V7					(0x3 << 21)
#define BUCK_SEL						BUCK_SEL_3V3

/* MIN ON TIME (Minimum ON Time for Low Side MOSFET) */
#define MIN_ON_TIME_OUS					(0x0 << 17)
#define MIN_ON_TIME_AUTO				(0x1 << 17)
#define MIN_ON_TIME_0US5				(0x2 << 17)
#define MIN_ON_TIME_0US75				(0x3 << 17)
#define MIN_ON_TIME_1US					(0x4 << 17)
#define MIN_ON_TIME_1US25				(0x5 << 17)
#define MIN_ON_TIME_1US5				(0x6 << 17)
#define MIN_ON_TIME_2US					(0x7 << 17)
#define MIN_ON_TIME						MIN_ON_TIME_0US5

/* GD CONFIG1 Register Data */
#define GD_CONFIG2_DATA					(PARITY | BUCK_PS_DIS | BUCK_SEL |  BUCK_CL | MIN_ON_TIME)

/*
 * INT ALGO 1
 */
/* ACTIVE BRAKE SPEED DELTA LIMIT EXIT (Difference between final speed and present speed)*/
#define ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT_2PER5			(0x0 << 29)
#define ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT_5PER			(0x1 << 29)
#define ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT_7PER5			(0x2 << 29)
#define ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT_10PER			(0x3 << 29)
#define ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT				ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT_2PER5

/* SPEED PIN GLITCH FILTER */
#define SPEED_PIN_GLITCH_FILTER_NO_FILTER					(0x0 << 27)
#define SPEED_PIN_GLITCH_FILTER_0US2						(0x1 << 27)
#define SPEED_PIN_GLITCH_FILTER_0US5						(0x2 << 27)
#define SPEED_PIN_GLITCH_FILTER_1US							(0x3 << 27)
#define SPEED_PIN_GLITCH_FILTER								SPEED_PIN_GLITCH_FILTER_0US2

/* FAST ISD EN (Enable Fast Speed Durign ISD) */
#define FAST_ISD_ENABLE										(0x1 << 26)
#define FAST_ISD_DISABLE									(0x0 << 26)
#define FAST_ISD_EN											FAST_ISD_ENABLE

/* ISD STOP TIME */
#define ISD_STOP_TIME_1MS									(0x0 << 24)
#define ISD_STOP_TIME_5MS									(0x1 << 24)
#define ISD_STOP_TIME_50MS									(0x2 << 24)
#define ISD_STOP_TIME_100MS									(0x3 << 24)
#define ISD_STOP_TIME										ISD_STOP_TIME_5MS

/* ISD RUN TIME */
#define ISD_RUN_TIME_1MS									(0x0 << 22)
#define ISD_RUN_TIME_5MS									(0x1 << 22)
#define ISD_RUN_TIME_50MS									(0x2 << 22)
#define ISD_RUN_TIME_100MS									(0x << 22)
#define ISD_RUN_TIME										ISD_RUN_TIME_50MS

/* ISD TIMEOUT */
#define ISD_TIMEOUT_500MS									(0x0 << 20)
#define ISD_TIMEOUT_750MS									(0x1 << 20)
#define ISD_TIMEOUT_1S										(0x2 << 20)
#define ISD_TIMEOUT_2S										(0x3 << 20)
#define ISD_TIMEOUT											ISD_TIMEOUT_500MS

/* AUTO HANDOFF MIN BEMF */
#define AUTO_HANDOFF_MIN_BEMF_0MV							(0x0 << 17)
#define AUTO_HANDOFF_MIN_BEMF_50MV							(0x1 << 17)
#define AUTO_HANDOFF_MIN_BEMF_100MV							(0x2 << 17)
#define AUTO_HANDOFF_MIN_BEMF_250MV							(0x3 << 17)
#define AUTO_HANDOFF_MIN_BEMF_500MV							(0x4 << 17)
#define AUTO_HANDOFF_MIN_BEMF_1000MV						(0x5 << 17)
#define AUTO_HANDOFF_MIN_BEMF_1250MV						(0x6 << 17)
#define AUTO_HANDOFF_MIN_BEMF_2500MV						(0x7 << 17)
#define AUTO_HANDOFF_MIN_BEMF								AUTO_HANDOFF_MIN_BEMF_50MV

/* BRAKE CURRENT PERSIST */
#define BRAKE_CURRENT_PERSIST_50MS							(0x0 << 15)
#define BRAKE_CURRENT_PERSIST_100MS							(0x1 << 15)
#define BRAKE_CURRENT_PERSIST_250MS							(0x2 << 15)
#define BRAKE_CURRENT_PERSIST_500MS							(0x3 << 15)
#define BRAKE_CURRENT_PERSIST								BRAKE_CURRENT_PERSIST_100MS

/* MPET IPD CURRENT LIMIT */
#define MPET_IPD_CURRENT_LIMIT_0A1							(0x0 << 13)
#define MPET_IPD_CURRENT_LIMIT_0A5							(0x1 << 13)
#define MPET_IPD_CURRENT_LIMIT_1A							(0x2 << 13)
#define MPET_IPD_CURRENT_LIMIT_2A							(0x3 << 13)
#define MPET_IPD_CURRENT_LIMIT								MPET_IPD_CURRENT_LIMIT_2A

/* MPET IPD FREQ */
#define MPET_IPD_FREQ_1										(0x0 << 11)
#define MPET_IPD_FREQ_2										(0x1 << 11)
#define MPET_IPD_FREQ_4										(0x2 << 11)
#define MPET_IPD_FREQ_8										(0x3 << 11)
#define MPET_IPD_FREQ										MPET_IPD_FREQ_1

/* MPET OPEN LOOP CURRENT */
#define MPET_OPEN_LOOP_CURRENT_REF_1A						(0x0 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_2A						(0x1 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_3A						(0x2 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_4A						(0x3 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_5A						(0x4 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_6A						(0x5 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_7A						(0x6 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF_8A						(0x7 < 8)
#define MPET_OPEN_LOOP_CURRENT_REF							MPET_OPEN_LOOP_CURRENT_REF_4A

/* MPET OPEN LOOP SPEED REF*/
#define MPET_OPEN_LOOP_SPEED_REF_15PER						(0x0 << 6)
#define MPET_OPEN_LOOP_SPEED_REF_25PER						(0x1 << 6)
#define MPET_OPEN_LOOP_SPEED_REF_35PER						(0x2 << 6)
#define MPET_OPEN_LOOP_SPEED_REF_50PER						(0x3 << 6)
#define MPET_OPEN_LOOP_SPEED_REF							MPET_OPEN_LOOP_SPEED_REF_15PER

/* MPET OPEN LOOP SLEW RATE */
#define MPET_OPEN_LOOP_SLEW_RATE_0HZ1						(0x0 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_0HZ5						(0x1 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_1HZ						(0x2 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_2HZ						(0x3 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_3HZ						(0x4 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_5HZ						(0x5 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_10HZ						(0x6 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE_20HZ						(0x7 << 3)
#define MPET_OPEN_LOOP_SLEW_RATE							MPET_OPEN_LOOP_SLEW_RATE_20HZ

/* REV DRV OPEN LOOP DEC */
#define REV_DRV_OPEN_LOOP_DEC_50PER							0x0
#define REV_DRV_OPEN_LOOP_DEC_60PER							0x1
#define REV_DRV_OPEN_LOOP_DEC_70PER							0x2
#define REV_DRV_OPEN_LOOP_DEC_80PER							0x3
#define REV_DRV_OPEN_LOOP_DEC_90PER							0x4
#define REV_DRV_OPEN_LOOP_DEC_100PER						0x5
#define REV_DRV_OPEN_LOOP_DEC_125PER						0x6
#define REV_DRV_OPEN_LOOP_DEC_150PER						0x7
#define REV_DRV_OPEN_LOOP_DEC								REV_DRV_OPEN_LOOP_DEC_100PER

/* INT_ALGO_1 Register Data */
#define INT_ALGO_1_DATA				(ACTIVE_BRAKE_SPEED__DELTA_LIMIT_EXIT | SPEED_PIN_GLITCH_FILTER_NO_FILTER | FAST_ISD_EN | ISD_STOP_TIME | ISD_TIMEOUT | AUTO_HANDOFF_MIN_BEMF | BRAKE_CURRENT_PERSIST | MPET_IPD_CURRENT_LIMIT | MPET_IPD_CURRENT_LIMIT | MPET_IPD_FREQ | MPET_OPEN_LOOP_CURRENT_REF | MPET_OPEN_LOOP_SPEED_REF | MPET_OPEN_LOOP_SLEW_RATE | REV_DRV_OPEN_LOOP_DEC)

/*
 * INT ALGO 2
 */
/* CL SLOW ACC */
#define CL_SLOW_ACC_0HZ1					(0x0 << 6)
#define CL_SLOW_ACC_1HZ						(0x1 << 6)
#define CL_SLOW_ACC_2HZ						(0x2 << 6)
#define CL_SLOW_ACC_3HZ						(0x3 << 6)
#define CL_SLOW_ACC_5HZ						(0x4 << 6)
#define CL_SLOW_ACC_10HZ					(0x5 << 6)
#define CL_SLOW_ACC_20HZ					(0x6 << 6)
#define CL_SLOW_ACC_30HZ					(0x7 << 6)
#define CL_SLOW_ACC_40HZ					(0x8 << 6)
#define CL_SLOW_ACC_50HZ					(0x9 << 6)
#define CL_SLOW_ACC_100HZ					(0xA << 6)
#define CL_SLOW_ACC_200HZ					(0xB << 6)
#define CL_SLOW_ACC_500HZ					(0xC << 6)
#define CL_SLOW_ACC_750HZ					(0xD << 6)
#define CL_SLOW_ACC_1KHZ					(0xE << 6)
#define CL_SLOW_ACC_2KHZ					(0xF << 6)
#define CL_SLOW_ACC							CL_SLOW_ACC_20HZ

/* ACTIVE BRAKE BUS CURRENT SLEW RATE */
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_10A			(0x0 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_50A			(0x1 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_100A			(0x2 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_250A			(0x3 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_500A			(0x4 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_1KA			(0x5 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_5KA			(0x6 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_NO_LIMIT		(0x7 << 3)
#define ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE				ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE_500A

/* MPET IPD SELECT */
#define MPET_IPD_SELECT_NORMAL				(0x0 << 2)
#define MPET_IPD_SELECT_SPECIFIC			(0x1 << 2)
#define MPET_IPD_SELECT						MPET_IPD_SELECT_SPECIFIC

/* MPET KE MEAS PARAMETER SELECT */
#define MPET_KE_MEAS_PARAMETER_SELECT_NORMAL			(0x0 << 1)
#define MPET_KE_MEAS_PARAMETER_SELECT_SPECIFIC			(0x1 << 1)
#define MPET_KE_MEAS_PARAMETER_SELECT					MPET_KE_MEAS_PARAMETER_SELECT_SPECIFIC

/* IPD HIGH RESOLUTION EN */
#define IPD_HIGH_RESOLUTION_ENABLE			0x1
#define IPD_HIGH_RESOLUTION_DISABLE			0x0
#define IPD_HIGH_RESOLUTION_EN				IPD_HIGH_RESOLUTION_ENABLE

/* INT ALGO 2 Register Data */
#define INT_ALGO_2_DATA						(CL_SLOW_ACC | ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE | MPET_IPD_SELECT | MPET_KE_MEAS_PARAMETER_SELECT | IPD_HIGH_RESOLUTION_EN)

/*
 * ALGO_DEBUG1
 */
/* OVERRIDE */
#define OVERRIDE_SPEED_CMD_ANAL_PWM			(0x1 << 31)
#define OVERRIDE_SPEED_CMD_DIGITAL			(0x0 << 31)
#define OVERRIDE							OVERRIDE_SPEED_CMD_ANAL_PWM

/* DIGITAL SPEED CTRL */
#define DIGITAL_SPEED_CTRL					(0x00 << 16)

/* CLOSDE LOOP DIS */
#define CLOSED_LOOP_ENABLE					(0x0 << 15)
#define CLOSED_LOOP_DISALE					(0x1 << 15)
#define CLOSED_LOOP_DIS						CLOSED_LOOP_ENABLE

/* FORCE ALIGN EN */
#define FORCE_ALIGN_ENABLE					(0x1 << 14)
#define FORCE_ALIGN_DISABLE					(0x0 << 14)
#define FORCE_ALIGN_EN						FORCE_ALIGN_DISABLE

/* FORCE SLOW FIRST CYCLE EN */
#define FORCE_SLOW_FIRST_CYCLE_ENABLE		(0x1 << 13)
#define FORCE_SLOW_FIRST_CYCLE_DISABLE		(0x0 << 13)
#define FORCE_SLOW_FIRST_CYCLE_EN			FORCE_SLOW_FIRST_CYCLE_ENABLE

/* FORCE IPD EN */
#define FORCE_IPD_ENABLE					(0x1 << 12)
#define FORCE_IPD_DISABLE					(0x0 << 12)
#define FORCE_IPD_EN						FORCE_IPD_DISABLE

/* FORCE ISD EN */
#define FORCE_ISD_ENABLE					(0x1 << 11)
#define FORCE_ISD_DISABLE					(0x0 << 11)
#define FORCE_ISD_EN						FORCE_ISD_DISABLE

/* FORCE ALIGN ANGLE SRC SEL */
#define FORCE_ALIGN_ANGLE_SRC_SEL_ALIGN		(0x1 << 10)
#define FORCE_ALIGN_ANGLE_SRC_SEL_FORCE		(0x0 << 10)
#define FORCE_ALIGN_ANGLE_SRC_SEL			FORCE_ALIGN_ANGLE_SRC_SEL_ALIGN

/* FORCE ALIGN ANGLE SRC SPEED LOOP DIS */
#define FORCE_ALIGN_ANGLE_SRC_SPEED_LOOP_DIS	0x0

/* ALGO_DEBUG1 */
#define ALGO_DEBUG1_DATA					(OVERRIDE | DIGITAL_SPEED_CTRL | CLOSED_LOOP_DIS | FORCE_ALIGN_EN | FORCE_SLOW_FIRST_CYCLE_EN | FORCE_IPD_EN | FORCE_ISD_EN | FORCE_ALIGN_ANGLE_SRC_SEL | FORCE_ALIGN_ANGLE_SRC_SPEED_LOOP_DIS)

/*
 * ALGO_DEBUG2
 */
/* FORCE RECIRCULATE STOP SECTOR */
#define FORCE_RECIRCULATE_STOP_SECTOR_LAST		(0x0 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR_1			(0x1 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR_2			(0x2 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR_3			(0x3 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR_4			(0x4 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR_5			(0x5 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR_6			(0x6 << 28)
#define FORCE_RECIRCULATE_STOP_SECTOR			FORCE_RECIRCULATE_STOP_SECTOR_LAST

/* FORCE RECIRCULATE STOP EN */
#define FORCE_RECIRCULATE_STOP_ENABLE			(0x0 << 27)
#define FORCE_RECIRCULATE_STOP_DISABLE			(0x1 << 27)
#define FORCE_RECIRCULATE_STOP_EN				FORCE_RECIRCULATE_STOP_ENABLE

/* CURRENT LOOP DIS */
#define CURRENT_LOOP_DISABLE					(0x1 << 0)
#define CURRENT_LOOP_ENABLE						(0x0 << 0)
#define CURRENT_LOOP_DIS						CURRENT_LOOP_DISABLE

/* FORCE VD CURRENT LOOP */
#define FORCE_VD_CURRENT_LOOP_DIS				(0x0 << 16)

/* FORCE VQ CURRENT LOOP */
#define FORCE_VQ_CURRENT_LOOP_DIS				(0xFF << 6)

/* MPET CMD */
#define MPET_CMD_ENABLE						(0x1 << 5)
#define MPET_CMD_DISABLE					(0x0 << 5)
#define MPET_CMD							MPET_CMD_ENABLE

/* MPET R */
#define MPET_R_ENABLE						(0x1 << 4)
#define MPET_R_DISABLE						(0x0 << 4)
#define MPET_R								MPET_R_ENABLE

/* MPET L */
#define MPET_L_ENABLE						(0x1 << 3)
#define MPET_L_DISABLE						(0x0 << 3)
#define MPET_L								MPET_L_ENABLE

/* MPET KE */
#define MPET_KE_ENABLE						(0x1 << 2)
#define MPET_KE_DISABLE						(0x0 << 2)
#define MPET_KE								MPET_KE_ENABLE

/* MPET MECH */
#define MPET_MECH_ENABLE					(0x1 << 1)
#define MPET_MECH_DISABLE					(0x0 << 1)
#define MPET_MECH							MPET_MECH_ENABLE

/* MPET WRITE SHADOW */
#define MPET_WRITE_SHADOW_ENABLE			0x1
#define MPET_WRITE_SHADOW_DISABLE			0x0
#define MPET_WRITE_SHADOW					MPET_WRITE_SHADOW_ENABLE

/* ALGO DEBUG2 Register Data */
#define ALGO_DEBUG2_DATA					(FORCE_RECIRCULATE_STOP_SECTOR | FORCE_RECIRCULATE_STOP_EN | CURRENT_LOOP_DIS | FORCE_VD_CURRENT_LOOP_DIS | FORCE_VQ_CURRENT_LOOP_DIS | MPET_CMD | MPET_R | MPET_L | MPET_KE | MPET_MECH | MPET_WRITE_SHADOW)

/*
 * MCF8316C-Q1 I2C FUNCTION
 */
void I2C_TEST1();

void I2C_TEST2();

void MCF8316C_Set_EEPROM();

void MCF8316C_Get_Voltage();

void MCF8316C_Get_Fault();

void MCF8316C_MPET();

#endif /* INC_MCF8316C_H_ */
