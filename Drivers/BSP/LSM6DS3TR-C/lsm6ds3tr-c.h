/*
 * lsm6ds3tr-c.h
 *
 * Universal ST LSM6DS-Series High-Performance IMU Driver
 * Compatible with:
 *   - LSM6DS3       (WHO_AM_I = 0x69)
 *   - LSM6DS3TR-C   (WHO_AM_I = 0x6A)
 *   - LSM6DSR / DSRX/ ISM330DHCX (WHO_AM_I = 0x6B)
 *   - LSM6DSO / DSOX/ ISM330DLC  (WHO_AM_I = 0x6C)
 *   - LSM6DST / DSTX(WHO_AM_I = 0x6D)
 *   - LSM6DSV / V16X/ V16B       (WHO_AM_I = 0x70)
 *   - LSM6DSV32X    (WHO_AM_I = 0x71)
 */

#ifndef INC_LSM6DS3TR_H_
#define INC_LSM6DS3TR_H_

#include "main.h"
#include "i2c.h"
#include "spi.h"
#include <string.h>
#include <math.h>

#define IMU_I2C                 (&hi2c1)
#define IMU_SPI                 (&hspi1)
#define IMU_SPI_TIMEOUT_MS      5
#define IMU_I2C_TIMEOUT_MS      20

/* I2C Device Addresses (7-bit 0x6A << 1 = 0xD4, 0x6B << 1 = 0xD6) */
#define LSM6DS3_ADDR            0xD4
#define LSM6DS3_ADDR_ALT        0xD6

/* SysTick (1 kHz) ticks between Gyro Z DMA reads in I2C mode: 1 = every 1 ms */
#define IMU_READ_DIV            1

/* Supported Models & WHO_AM_I IDs */
#define WHO_AM_I_LSM6DS3        0x69
#define WHO_AM_I_LSM6DS3TR_C    0x6A /* LSM6DS3TR, LSM6DS3TR-C, LSM6DSL */
#define WHO_AM_I_LSM6DSR        0x6B /* LSM6DSR, LSM6DSRX, ISM330DHCX */
#define WHO_AM_I_LSM6DSO        0x6C /* LSM6DSO, LSM6DSOX, ISM330DLC */
#define WHO_AM_I_LSM6DST        0x6D /* LSM6DST, LSM6DSTX */
#define WHO_AM_I_LSM6DSV        0x70 /* LSM6DSV, LSM6DSV16X, LSM6DSV16B */
#define WHO_AM_I_LSM6DSV32X     0x71 /* LSM6DSV32X */

/* Common Register Map */
#define REG_IF_CFG              0x03 /* LSM6DSV: I2C_I3C_disable bit 0 */
#define REG_INT1_CTRL           0x0D /* INT1 control: bit 1 = DRDY_G */
#define REG_WHO_AM_I            0x0F /* Device identification */
#define REG_CTRL1_XL            0x10 /* Accel control */
#define REG_CTRL2_G             0x11 /* Gyro control */
#define REG_CTRL3_C             0x12 /* Control 3: BDU (bit 6), IF_INC (bit 2), SW_RESET (bit 0) */
#define REG_CTRL4_C             0x13 /* Control 4: I2C_disable (bit 2 in Gen2) */
#define REG_CTRL6_C             0x15 /* LSM6DSV: Gyro Full-scale FS_G[3:0] */
#define REG_CTRL8_XL            0x17 /* LSM6DSV: Accel Full-scale FS_XL[1:0] */
#define REG_STATUS_REG          0x1E /* Status register */
#define REG_OUTX_L_G            0x22 /* Gyro X Low */
#define REG_OUTX_H_G            0x23 /* Gyro X High */
#define REG_OUTY_L_G            0x24 /* Gyro Y Low */
#define REG_OUTY_H_G            0x25 /* Gyro Y High */
#define REG_OUTZ_L_G            0x26 /* Gyro Z Low */
#define REG_OUTZ_H_G            0x27 /* Gyro Z High */
#define REG_OUTX_L_A            0x28 /* Accel X Low */
#define REG_OUTX_H_A            0x29 /* Accel X High */
#define REG_OUTY_L_A            0x2A /* Accel Y Low */
#define REG_OUTY_H_A            0x2B /* Accel Y High */
#define REG_OUTZ_L_A            0x2C /* Accel Z Low */
#define REG_OUTZ_H_A            0x2D /* Accel Z High */
#define REG_OUTX_L_XL           REG_OUTX_L_A

/* Sensitivity for FS = ±2000 dps: 70 mdps/LSB (0.070 dps/LSB) across all models */
#define GYRO_SENSITIVITY_2000DPS   0.070f
#define GYRO_SENSITIVITY           70.0f

/* Sensitivity for Accel FS = ±4g: 0.122 mg/LSB (0.000122 g/LSB) */
#define ACCEL_SENSITIVITY_4G       0.000122f
#define ACCEL_SENSITIVITY          0.122f

/* Deadband for Gyro Z noise filter in dps */
#define GYRO_DEADBAND              0.25f

typedef enum {
    LSM6DS_MODEL_UNKNOWN = 0,
    LSM6DS_MODEL_LSM6DS3,
    LSM6DS_MODEL_LSM6DS3TR_C,
    LSM6DS_MODEL_LSM6DSR,
    LSM6DS_MODEL_LSM6DSO,
    LSM6DS_MODEL_LSM6DST,
    LSM6DS_MODEL_LSM6DSV,
    LSM6DS_MODEL_LSM6DSV32X,
} LSM6DS_Model_t;

typedef struct {
    int16_t Accel_X_Raw;
    int16_t Accel_Y_Raw;
    int16_t Accel_Z_Raw;
    float_t Accel_X;
    float_t Accel_Y;
    float_t Accel_Z;

    int16_t Gyro_X_Raw;
    int16_t Gyro_Y_Raw;
    int16_t Gyro_Z_Raw;
    float_t Gyro_X;
    float_t Gyro_Y;
    float_t Gyro_Z;

    float_t Gyro_X_Offset;
    float_t Gyro_Y_Offset;
    float_t Gyro_Z_Offset;
    float_t Yaw_Angle;
    uint8_t Chip_ID;
    LSM6DS_Model_t Model;
    char    Model_Name[16];
} LSM6DS_Data_t;

/* Backward compatibility typedef & extern */
typedef LSM6DS_Data_t LSM6DS3_Data_t;
extern volatile LSM6DS_Data_t imu_data;
extern volatile uint8_t imu_dma_busy;
extern volatile uint8_t imu_gyro_z_ready;

/* Core Function Declarations */
HAL_StatusTypeDef LSM6DS_Init(void);
void LSM6DS_INT1_IRQHandler(void);
void LSM6DS_ReadGyro_Z_Only(volatile LSM6DS_Data_t *data);
void LSM6DS_ReadAccel(volatile LSM6DS_Data_t *data);
void LSM6DS_ReadAll(volatile LSM6DS_Data_t *data);
void LSM6DS_Gyro_Calibrate_Z_Only(void);
void LSM6DS_UpdateYaw(volatile LSM6DS_Data_t *data, float_t dt);
void LSM6DS_Reset_Yaw(void);
const char* LSM6DS_GetModelName(void);

void LSM6DS3_ReadGyroZ_DMA_Start(void);
HAL_StatusTypeDef LSM6DS3_ReadGyroZ_DMA_Wait(uint32_t timeout_ms);

/* Backward compatibility aliases */
#define LSM6DS3_Init                  LSM6DS_Init
#define LSM6DS3_ReadGyro_Z_Only       LSM6DS_ReadGyro_Z_Only
#define LSM6DS3_ReadAccel             LSM6DS_ReadAccel
#define LSM6DS3_ReadAll               LSM6DS_ReadAll
#define LSM6DS3_Gyro_Calibrate_Z_Only LSM6DS_Gyro_Calibrate_Z_Only
#define LSM6DS3_UpdateYaw             LSM6DS_UpdateYaw
#define LSM6DS3_Reset_Yaw             LSM6DS_Reset_Yaw

#endif /* INC_LSM6DS3TR_H_ */
