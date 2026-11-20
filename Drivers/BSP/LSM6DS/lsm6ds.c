/*
 * lsm6ds.c
 *
 * Universal ST LSM6DS-Series High-Performance IMU Driver
 * Supports LSM6DS3, LSM6DS3TR-C, LSM6DSR, LSM6DSO, LSM6DST, LSM6DSV, LSM6DSV32X
 */

#include "lsm6ds.h"

volatile LSM6DS_Data_t imu_data;
volatile uint8_t imu_dma_busy = 0;
volatile uint8_t imu_gyro_z_ready = 0;
volatile uint8_t imu_straight_bias_track = 0;
volatile uint8_t imu_accel_capture_en = 0;

static float_t s_gyro_z_cal_offset = 0.0f;
static float_t s_gyro_z_lpf = 0.0f;
static float_t s_straight_hold_s = 0.0f;
static uint32_t s_spi_last_cyccnt = 0;
static uint32_t s_i2c_last_cyccnt = 0;

#ifndef IMU_INT_Pin
#ifdef IMU_INT1_Pin
#define IMU_INT_Pin IMU_INT1_Pin
#endif
#endif

static LSM6DS_Model_t LSM6DS_Identify(uint8_t id) {
    switch (id) {
    case WHO_AM_I_LSM6DS3:
        strcpy((char*)imu_data.Model_Name, "LSM6DS3");
        return LSM6DS_MODEL_LSM6DS3;
    case WHO_AM_I_LSM6DS3TR_C:
        strcpy((char*)imu_data.Model_Name, "LSM6DS3TR-C");
        return LSM6DS_MODEL_LSM6DS3TR_C;
    case WHO_AM_I_LSM6DSR:
        strcpy((char*)imu_data.Model_Name, "LSM6DSR");
        return LSM6DS_MODEL_LSM6DSR;
    case WHO_AM_I_LSM6DSO:
        strcpy((char*)imu_data.Model_Name, "LSM6DSO");
        return LSM6DS_MODEL_LSM6DSO;
    case WHO_AM_I_LSM6DST:
        strcpy((char*)imu_data.Model_Name, "LSM6DST");
        return LSM6DS_MODEL_LSM6DST;
    case WHO_AM_I_LSM6DSV:
        strcpy((char*)imu_data.Model_Name, "LSM6DSV");
        return LSM6DS_MODEL_LSM6DSV;
    case WHO_AM_I_LSM6DSV32X:
        strcpy((char*)imu_data.Model_Name, "LSM6DSV32X");
        return LSM6DS_MODEL_LSM6DSV32X;
    default:
        strcpy((char*)imu_data.Model_Name, "LSM6DS-Gen");
        return LSM6DS_MODEL_UNKNOWN;
    }
}

#ifdef IMU_CS_Pin
/* ============================================================================
 * SPI Mode (when IMU_CS_Pin is defined)
 * ============================================================================ */
static inline void IMU_CS_LOW(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static inline void IMU_CS_HIGH(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef LSM6DS_Bus_Read(uint8_t reg, uint8_t *pData, uint16_t len) {
    uint8_t tx_buf[16] = { 0 };
    uint8_t rx_buf[16] = { 0 };
    if (len == 0 || len > 14) return HAL_ERROR;

    tx_buf[0] = reg | 0x80; // MSB = 1 for READ

    IMU_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(IMU_SPI, tx_buf, rx_buf, len + 1, IMU_SPI_TIMEOUT_MS);
    IMU_CS_HIGH();

    if (status == HAL_OK) {
        memcpy(pData, &rx_buf[1], len);
    }
    return status;
}

static HAL_StatusTypeDef LSM6DS_Bus_Write(uint8_t reg, uint8_t val) {
    uint8_t tx_buf[2];
    tx_buf[0] = reg & 0x7F; // MSB = 0 for WRITE
    tx_buf[1] = val;

    IMU_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(IMU_SPI, tx_buf, 2, IMU_SPI_TIMEOUT_MS);
    IMU_CS_HIGH();

    return status;
}

#else
/* ============================================================================
 * I2C Mode (when IMU_CS_Pin is not defined — v2/v3 board with DRV8316C)
 * ============================================================================ */
static uint16_t lsm6ds_i2c_addr = LSM6DS3_ADDR;

/* OUTZ_G (0x26..0x27), then OUTX_A / OUTY_A (0x28..0x2B) in the same burst when accel capture is on */
__attribute__((section(".ram_d2_nocache"), aligned(32)))
static uint8_t gyro_z_dma_buf[6];
static volatile uint16_t s_dma_len = 2;

static HAL_StatusTypeDef LSM6DS_Bus_Read(uint8_t reg, uint8_t *pData, uint16_t len) {
    return HAL_I2C_Mem_Read(IMU_I2C, lsm6ds_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, pData, len, IMU_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef LSM6DS_Bus_Write(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(IMU_I2C, lsm6ds_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_I2C_TIMEOUT_MS);
}
#endif

HAL_StatusTypeDef LSM6DS_Init(void) {
    // Prevent background DMA / IRQ from accessing IMU during initialization
    imu_data.Model = LSM6DS_MODEL_UNKNOWN;
    imu_dma_busy = 0;
    imu_gyro_z_ready = 0;

    // 1. Enable DWT cycle counter for sub-microsecond dt calculation
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#ifdef IMU_CS_Pin
    // 2. Hardware CS pulse sequence to force SPI mode
    IMU_CS_HIGH();
    HAL_Delay(5);
    IMU_CS_LOW();
    HAL_Delay(1);
    IMU_CS_HIGH();
    HAL_Delay(5);
#endif

    // 3. Read WHO_AM_I with retry
    uint8_t chipID = 0;
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int retry = 0; retry < 5; retry++) {
#ifndef IMU_CS_Pin
        lsm6ds_i2c_addr = LSM6DS3_ADDR;
        status = LSM6DS_Bus_Read(REG_WHO_AM_I, &chipID, 1);
        if (status != HAL_OK || chipID == 0x00 || chipID == 0xFF) {
            lsm6ds_i2c_addr = LSM6DS3_ADDR_ALT;
            status = LSM6DS_Bus_Read(REG_WHO_AM_I, &chipID, 1);
        }
#else
        status = LSM6DS_Bus_Read(REG_WHO_AM_I, &chipID, 1);
#endif
        if (status == HAL_OK && chipID != 0x00 && chipID != 0xFF) {
            break;
        }
        HAL_Delay(5);
    }

    if (status != HAL_OK || chipID == 0x00 || chipID == 0xFF) {
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
        imu_data.Model = LSM6DS_MODEL_UNKNOWN;
        imu_data.Chip_ID = 0;
        imu_data.Gyro_Z = 0.0f;
        imu_data.Gyro_Z_Offset = 0.0f;
        imu_data.Yaw_Angle = 0.0f;
        strcpy((char*)imu_data.Model_Name, "NONE");
        return HAL_ERROR;
    }

    imu_data.Chip_ID = chipID;
    LSM6DS_Model_t detected_model = LSM6DS_Identify(chipID);

    // 4. Software Reset (CTRL3_C / CTRL3 at 0x12)
    LSM6DS_Bus_Write(REG_CTRL3_C, 0x01); // SW_RESET
    HAL_Delay(15);

#ifdef IMU_CS_Pin
    // 5. Interface specific overrides after reset (SPI mode)
    if (detected_model == LSM6DS_MODEL_LSM6DSV || detected_model == LSM6DS_MODEL_LSM6DSV32X) {
        LSM6DS_Bus_Write(REG_IF_CFG, 0x01);
    } else if (detected_model == LSM6DS_MODEL_LSM6DSO || detected_model == LSM6DS_MODEL_LSM6DSR || detected_model == LSM6DS_MODEL_LSM6DST) {
        LSM6DS_Bus_Write(REG_CTRL4_C, 0x04);
    }
#endif

    // 6. Set BDU (Block Data Update) and IF_INC (Auto increment)
    // 0x44: BDU=1 (bit 6), IF_INC=1 (bit 2)
    LSM6DS_Bus_Write(REG_CTRL3_C, 0x44);

    // 7. Configure Gyroscope & Accelerometer
    if (detected_model == LSM6DS_MODEL_LSM6DSV || detected_model == LSM6DS_MODEL_LSM6DSV32X) {
        // LSM6DSV Generation:
        // CTRL1 (0x10): ODR_XL = 960 Hz (0x70) High-Performance
        LSM6DS_Bus_Write(REG_CTRL1_XL, 0x70);
        // CTRL8_XL (0x17): FS_XL = ±4g (0x02)
        LSM6DS_Bus_Write(REG_CTRL8_XL, 0x02);
        // CTRL2 (0x11): ODR_G = 960 Hz (0x70) High-Performance
        LSM6DS_Bus_Write(REG_CTRL2_G, 0x70);
        // CTRL6_C (0x15): FS_G = ±2000 dps (0x04)
        LSM6DS_Bus_Write(REG_CTRL6_C, 0x04);
    } else {
        // LSM6DS3, LSM6DS3TR-C, LSM6DSO, LSM6DSR, LSM6DST Generation:
        // CTRL1_XL (0x10): ODR_XL = 833 Hz, FS_XL = ±4g (0x78)
        LSM6DS_Bus_Write(REG_CTRL1_XL, 0x78);
        // CTRL2_G (0x11): ODR_G = 1.66 kHz, FS_G = ±2000 dps (0x8C)
        LSM6DS_Bus_Write(REG_CTRL2_G, 0x8C);
    }

#ifdef IMU_CS_Pin
    // 8. Route Gyro Data-Ready to physical pin INT1 (0x0D): bit 1 = INT1_DRDY_G
    LSM6DS_Bus_Write(REG_INT1_CTRL, 0x02);

    // 9. Initial dummy read of gyro outputs to clear any pending DRDY flags
    uint8_t dummy[6];
    LSM6DS_Bus_Read(REG_OUTX_L_G, dummy, 6);

    // 10. Enable EXTI0 Interrupt in NVIC for PE0 (IMU_INT)
    __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT_Pin);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#else
    // In I2C mode, SysTick triggers non-blocking I2C DMA reads; disable unused EXTI0
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif

    imu_dma_busy = 0;
    imu_gyro_z_ready = 0;
    imu_straight_bias_track = 0;
    s_gyro_z_cal_offset = 0.0f;
    s_gyro_z_lpf = 0.0f;
    s_straight_hold_s = 0.0f;
    s_spi_last_cyccnt = 0;
    s_i2c_last_cyccnt = 0;
    imu_data.Model = detected_model;
    return HAL_OK;
}

/* Interrupt Service Routine called from EXTI0_IRQHandler on rising edge of PE0 */
void LSM6DS_INT1_IRQHandler(void) {
    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN) {
        return;
    }
#ifdef IMU_CS_Pin
    uint8_t raw[12];
    const uint16_t n_read = imu_accel_capture_en ? 12 : 6;
    if (LSM6DS_Bus_Read(REG_OUTX_L_G, raw, n_read) != HAL_OK) {
        return;
    }
    if (n_read == 12) {
        imu_data.Accel_X_Raw = (int16_t)((raw[7] << 8) | raw[6]);
        imu_data.Accel_Y_Raw = (int16_t)((raw[9] << 8) | raw[8]);
        imu_data.Accel_Z_Raw = (int16_t)((raw[11] << 8) | raw[10]);
        imu_data.Accel_X = imu_data.Accel_X_Raw * ACCEL_SENSITIVITY_4G;
        imu_data.Accel_Y = imu_data.Accel_Y_Raw * ACCEL_SENSITIVITY_4G;
        imu_data.Accel_Z = imu_data.Accel_Z_Raw * ACCEL_SENSITIVITY_4G;
    }

    imu_data.Gyro_X_Raw = (int16_t)((raw[1] << 8) | raw[0]);
    imu_data.Gyro_Y_Raw = (int16_t)((raw[3] << 8) | raw[2]);
    imu_data.Gyro_Z_Raw = (int16_t)((raw[5] << 8) | raw[4]);

    imu_data.Gyro_X = imu_data.Gyro_X_Raw * GYRO_SENSITIVITY_2000DPS;
    imu_data.Gyro_Y = imu_data.Gyro_Y_Raw * GYRO_SENSITIVITY_2000DPS;
    imu_data.Gyro_Z = imu_data.Gyro_Z_Raw * GYRO_SENSITIVITY_2000DPS;

    uint32_t now_cyccnt = DWT->CYCCNT;

    if (s_spi_last_cyccnt != 0) {
        uint32_t diff = now_cyccnt - s_spi_last_cyccnt;
        float_t dt = (float_t)diff / (float_t)SystemCoreClock;

        if (dt > 0.0002f && dt < 0.05f) {
            LSM6DS_UpdateYaw(&imu_data, dt);
        }
    }
    s_spi_last_cyccnt = now_cyccnt;

    imu_gyro_z_ready = 1;
#endif
}

void LSM6DS3_ReadGyroZ_DMA_Start(void) {
#ifndef IMU_CS_Pin
    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN || imu_dma_busy) {
        return;
    }
    imu_dma_busy = 1;
    s_dma_len = imu_accel_capture_en ? 6 : 2;

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(
        IMU_I2C, lsm6ds_i2c_addr, REG_OUTZ_L_G,
        I2C_MEMADD_SIZE_8BIT, gyro_z_dma_buf, s_dma_len);

    if (ret != HAL_OK) {
        imu_dma_busy = 0;
    }
#endif
}

HAL_StatusTypeDef LSM6DS3_ReadGyroZ_DMA_Wait(uint32_t timeout_ms) {
#ifndef IMU_CS_Pin
    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN) {
        return HAL_ERROR;
    }
    LSM6DS3_ReadGyroZ_DMA_Start();

    uint32_t start = HAL_GetTick();
    while (!imu_gyro_z_ready) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    imu_gyro_z_ready = 0;
#else
    (void)timeout_ms;
#endif
    return HAL_OK;
}

void LSM6DS_UpdateYaw(volatile LSM6DS_Data_t *data, float_t dt) {
    float_t gyro_z_corrected = data->Gyro_Z - data->Gyro_Z_Offset;

    /* Low-pass gate (~62 ms) prevents high-frequency gyro noise from resetting straight hold */
    float_t lpf_alpha = dt * 16.0f;
    if (lpf_alpha > 1.0f) lpf_alpha = 1.0f;
    s_gyro_z_lpf += lpf_alpha * (gyro_z_corrected - s_gyro_z_lpf);

    /* Online residual gyro bias observer active only when caller confirms mapped straight (Drive_Fourth) */
    if (imu_straight_bias_track &&
        s_gyro_z_lpf > -3.5f && s_gyro_z_lpf < 3.5f &&
        gyro_z_corrected > -20.0f && gyro_z_corrected < 20.0f) {
        s_straight_hold_s += dt;
        if (s_straight_hold_s >= 0.08f) {
            float_t alpha = dt * 0.50f; /* tau = 2.0 s, sample-rate invariant */
            if (alpha > 0.005f) alpha = 0.005f;
            float_t next_off = data->Gyro_Z_Offset + alpha * gyro_z_corrected;
            float_t min_off = s_gyro_z_cal_offset - GYRO_BIAS_TRACK_MAX_DELTA;
            float_t max_off = s_gyro_z_cal_offset + GYRO_BIAS_TRACK_MAX_DELTA;
            if (next_off < min_off) next_off = min_off;
            else if (next_off > max_off) next_off = max_off;
            data->Gyro_Z_Offset = next_off;
            gyro_z_corrected = data->Gyro_Z - data->Gyro_Z_Offset;
        }
    } else {
        s_straight_hold_s = 0.0f;
    }

    data->Yaw_Angle += gyro_z_corrected * dt;

    if (data->Yaw_Angle > 180.0f) {
        data->Yaw_Angle -= 360.0f;
    } else if (data->Yaw_Angle < -180.0f) {
        data->Yaw_Angle += 360.0f;
    }
}

void LSM6DS_Reset_Yaw(void) {
    imu_data.Yaw_Angle = 0.0f;
    imu_straight_bias_track = 0;
    s_gyro_z_lpf = 0.0f;
    s_straight_hold_s = 0.0f;
}

void LSM6DS_Gyro_Calibrate_Z_Only(void) {
    imu_straight_bias_track = 0;
    s_gyro_z_cal_offset = 0.0f;
    s_gyro_z_lpf = 0.0f;
    s_straight_hold_s = 0.0f;

    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN) {
        imu_data.Gyro_Z_Offset = 0.0f;
        imu_data.Yaw_Angle = 0.0f;
        return;
    }

    double sum = 0.0;
    double sum_ax = 0.0, sum_ay = 0.0;
    const uint8_t with_accel = imu_accel_capture_en;
    const int warmup_samples = 50;
    const int sample_count = 2000;
    int skipped_samples = 0;
    int valid_samples = 0;

    imu_data.Gyro_Z_Offset = 0.0f;
    imu_gyro_z_ready = 0;

    uint32_t start_tick = HAL_GetTick();
    while (valid_samples < sample_count) {
        if (imu_gyro_z_ready) {
            imu_gyro_z_ready = 0;
            if (skipped_samples < warmup_samples) {
                skipped_samples++;
            } else {
                sum += (double)imu_data.Gyro_Z;
                sum_ax += (double)imu_data.Accel_X;
                sum_ay += (double)imu_data.Accel_Y;
                valid_samples++;
            }
        }
        // Timeout protection (max 3500 ms)
        if ((HAL_GetTick() - start_tick) > 3500) {
            break;
        }
    }

    if (valid_samples > 0) {
        imu_data.Gyro_Z_Offset = (float_t)(sum / (double)valid_samples);
    }
    if (with_accel && valid_samples > 0) {
        imu_data.Accel_X_Offset = (float_t)(sum_ax / (double)valid_samples);
        imu_data.Accel_Y_Offset = (float_t)(sum_ay / (double)valid_samples);
    } else {
        imu_data.Accel_X_Offset = 0.0f;
        imu_data.Accel_Y_Offset = 0.0f;
    }
    s_gyro_z_cal_offset = imu_data.Gyro_Z_Offset;
    imu_data.Yaw_Angle = 0.0f;
}

void LSM6DS_ReadGyro_Z_Only(volatile LSM6DS_Data_t *data) {
    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN) {
        return;
    }
    uint8_t raw[2];
#ifdef IMU_CS_Pin
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
    HAL_StatusTypeDef status = LSM6DS_Bus_Read(REG_OUTZ_L_G, raw, 2);
#ifdef IMU_CS_Pin
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#endif

    if (status == HAL_OK) {
        data->Gyro_Z_Raw = (int16_t)((raw[1] << 8) | raw[0]);
        data->Gyro_Z = data->Gyro_Z_Raw * GYRO_SENSITIVITY_2000DPS;
    }
}

void LSM6DS_ReadAccel(volatile LSM6DS_Data_t *data) {
    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN) {
        return;
    }
    uint8_t raw[6];
#ifdef IMU_CS_Pin
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
    HAL_StatusTypeDef status = LSM6DS_Bus_Read(REG_OUTX_L_A, raw, 6);
#ifdef IMU_CS_Pin
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#endif

    if (status == HAL_OK) {
        data->Accel_X_Raw = (int16_t)((raw[1] << 8) | raw[0]);
        data->Accel_Y_Raw = (int16_t)((raw[3] << 8) | raw[2]);
        data->Accel_Z_Raw = (int16_t)((raw[5] << 8) | raw[4]);

        data->Accel_X = data->Accel_X_Raw * ACCEL_SENSITIVITY_4G;
        data->Accel_Y = data->Accel_Y_Raw * ACCEL_SENSITIVITY_4G;
        data->Accel_Z = data->Accel_Z_Raw * ACCEL_SENSITIVITY_4G;
    }
}

void LSM6DS_ReadAll(volatile LSM6DS_Data_t *data) {
    if (imu_data.Model == LSM6DS_MODEL_UNKNOWN) {
        return;
    }
    uint8_t raw[12];
#ifdef IMU_CS_Pin
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
    HAL_StatusTypeDef status = LSM6DS_Bus_Read(REG_OUTX_L_G, raw, 12);
#ifdef IMU_CS_Pin
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#endif

    if (status == HAL_OK) {
        data->Gyro_X_Raw = (int16_t)((raw[1] << 8) | raw[0]);
        data->Gyro_Y_Raw = (int16_t)((raw[3] << 8) | raw[2]);
        data->Gyro_Z_Raw = (int16_t)((raw[5] << 8) | raw[4]);

        data->Accel_X_Raw = (int16_t)((raw[7] << 8) | raw[6]);
        data->Accel_Y_Raw = (int16_t)((raw[9] << 8) | raw[8]);
        data->Accel_Z_Raw = (int16_t)((raw[11] << 8) | raw[10]);

        data->Gyro_X = data->Gyro_X_Raw * GYRO_SENSITIVITY_2000DPS;
        data->Gyro_Y = data->Gyro_Y_Raw * GYRO_SENSITIVITY_2000DPS;
        data->Gyro_Z = data->Gyro_Z_Raw * GYRO_SENSITIVITY_2000DPS;

        data->Accel_X = data->Accel_X_Raw * ACCEL_SENSITIVITY_4G;
        data->Accel_Y = data->Accel_Y_Raw * ACCEL_SENSITIVITY_4G;
        data->Accel_Z = data->Accel_Z_Raw * ACCEL_SENSITIVITY_4G;
    }
}

const char* LSM6DS_GetModelName(void) {
    return (const char*)imu_data.Model_Name;
}

#ifndef IMU_CS_Pin
/* ===================== HAL I2C DMA 콜백 ===================== */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != hi2c1.Instance) {
        return;
    }

    imu_data.Gyro_Z_Raw = (int16_t)((gyro_z_dma_buf[1] << 8) | gyro_z_dma_buf[0]);
    imu_data.Gyro_Z = imu_data.Gyro_Z_Raw * GYRO_SENSITIVITY_2000DPS;
    if (s_dma_len == 6) {
        imu_data.Accel_X_Raw = (int16_t)((gyro_z_dma_buf[3] << 8) | gyro_z_dma_buf[2]);
        imu_data.Accel_Y_Raw = (int16_t)((gyro_z_dma_buf[5] << 8) | gyro_z_dma_buf[4]);
        imu_data.Accel_X = imu_data.Accel_X_Raw * ACCEL_SENSITIVITY_4G;
        imu_data.Accel_Y = imu_data.Accel_Y_Raw * ACCEL_SENSITIVITY_4G;
    }

    uint32_t now_cyccnt = DWT->CYCCNT;

    if (s_i2c_last_cyccnt != 0) {
        uint32_t diff = now_cyccnt - s_i2c_last_cyccnt;
        float_t dt = (float_t)diff / (float_t)SystemCoreClock;

        if (dt > 0.0001f && dt < 0.1f) {
            LSM6DS_UpdateYaw(&imu_data, dt);
        }
    }
    s_i2c_last_cyccnt = now_cyccnt;

    imu_gyro_z_ready = 1;
    imu_dma_busy = 0;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != hi2c1.Instance) {
        return;
    }
    imu_dma_busy = 0;
}
#endif
