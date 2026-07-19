/*
 * LSM6DS3TR.c
 *
 * Integrated Fixes:
 * 1. Added missing bias variables (bias_gx_rad, etc.)
 * 2. Fixed Complementary Filter discontinuity issue (wrapping logic)
 * 3. Uses -180 ~ +180 degree range for robust calculation
 * 4. Gyro Z-axis read converted to DMA (non-blocking) — polling I2C 제거
 */

#include "lsm6ds3tr-c.h"

LSM6DS3_Data_t imu_data;

/* ===================== DMA 관련 상태 ===================== */
volatile uint8_t imu_dma_busy = 0;      // DMA 전송 진행 중 플래그
volatile uint8_t imu_gyro_z_ready = 0;  // 새 Gyro_Z 값이 갱신되었는지 플래그

/*
 * DMA 수신 버퍼.
 * RAM_D2_NC(.ram_d2_nocache) 섹션에 배치 -> 캐시 무효화(invalidate) 불필요.
 * (RAM_D3_NC는 ADC3/BDMA 전용이므로 일반 DMA1/DMA2를 쓰는 I2C 버퍼는 여기 두지 않음)
 */
__attribute__((section(".ram_d2_nocache"), aligned(32)))
static uint8_t gyro_z_dma_buf[2];

/**
  * @brief  LSM6DS3 초기화 함수
  * @retval HAL_OK: 성공, HAL_ERROR: 실패 (ID 불일치 또는 통신 에러)
  */
HAL_StatusTypeDef LSM6DS3_Init() {
    uint8_t chipID;
    uint8_t data;

    // 1. WHO_AM_I 레지스터 확인
    if (HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &chipID, 1, 100) != HAL_OK) {
        return HAL_ERROR; // 통신 에러
    }

    if (chipID != WHO_AM_I_ID) {
        return HAL_ERROR; // ID 불일치
    }

    // 2. 가속도계 설정 (CTRL1_XL)
    // ODR = 104Hz, FS = 2g, BW = 400Hz
    // 0100 (104Hz) 00 (2g) 0 (400Hz) 0 -> 0x40
    data = 0x40;
    HAL_I2C_Mem_Write(IMU_I2C, LSM6DS3_ADDR, REG_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 3. 자이로스코프 설정 (CTRL2_G)
    // ODR = 104Hz, FS = 2000dps
    // 0100 (104Hz) 11 (2000dps) 00 -> 0x4C
    data = 0x4C;
    HAL_I2C_Mem_Write(IMU_I2C, LSM6DS3_ADDR, REG_CTRL2_G, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    imu_dma_busy = 0;
    imu_gyro_z_ready = 0;

    return HAL_OK; // 성공
}

/**
  * @brief  가속도 데이터 읽기 및 변환 (polling, 유지)
  */
__STATIC_INLINE void LSM6DS3_ReadAccel(LSM6DS3_Data_t *data) {
    uint8_t rawData[6];

    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_XL, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

    data->Accel_X_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Accel_Y_Raw = (int16_t)((rawData[3] << 8) | rawData[2]);
    data->Accel_Z_Raw = (int16_t)((rawData[5] << 8) | rawData[4]);

    data->Accel_X = (data->Accel_X_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Y = (data->Accel_Y_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Z = (data->Accel_Z_Raw * ACCEL_SENSITIVITY) / 1000.0f;
}

/**
  * @brief  자이로 데이터 읽기 및 변환 (polling, 유지 — 전체 축 필요할 때만 사용)
  */
__STATIC_INLINE void LSM6DS3_ReadGyro(LSM6DS3_Data_t *data) {
    uint8_t rawData[6];

    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

    data->Gyro_X_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Gyro_Y_Raw = (int16_t)((rawData[3] << 8) | rawData[2]);
    data->Gyro_Z_Raw = (int16_t)((rawData[5] << 8) | rawData[4]);

    data->Gyro_X = (data->Gyro_X_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Y = (data->Gyro_Y_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

/**
  * @brief  Z축 자이로 읽기 (polling 버전) — 캘리브레이션 등 blocking이 허용되는 곳에서만 사용
  */
void LSM6DS3_ReadGyro_Z_Only(LSM6DS3_Data_t *data) {
    uint8_t rawData[2];

    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTZ_L_G, I2C_MEMADD_SIZE_8BIT, rawData, 2, 100);

    data->Gyro_Z_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

/**
  * @brief  Z축 자이로 DMA 읽기 시작 (non-blocking)
  * @note   이전 전송이 아직 안 끝났으면 아무것도 하지 않고 리턴 (오버런 방지)
  */
void LSM6DS3_ReadGyroZ_DMA_Start(void) {
    if (imu_dma_busy) {
        return;
    }
    imu_dma_busy = 1;

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(
        IMU_I2C, LSM6DS3_ADDR, REG_OUTZ_L_G,
        I2C_MEMADD_SIZE_8BIT, gyro_z_dma_buf, 2);

    if (ret != HAL_OK) {
        imu_dma_busy = 0; // 시작 자체가 실패하면 즉시 플래그 해제 (다음 tick에 재시도)
    }
}

/**
  * @brief  Z축 자이로 DMA 읽기 완료까지 blocking 대기 (캘리브레이션 등에서 사용)
  * @param  timeout_ms: 최대 대기 시간
  * @retval HAL_OK: 성공, HAL_TIMEOUT: 타임아웃
  */
HAL_StatusTypeDef LSM6DS3_ReadGyroZ_DMA_Wait(uint32_t timeout_ms) {
    LSM6DS3_ReadGyroZ_DMA_Start();

    uint32_t start = HAL_GetTick();
    while (!imu_gyro_z_ready) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    imu_gyro_z_ready = 0;
    return HAL_OK;
}

/**
  * @brief  모든 데이터(Accel+Gyro) 한 번에 읽기 (polling, 유지)
  * @note   레지스터 맵 상 Gyro가 Accel보다 먼저 위치하므로 12바이트를 한 번에 읽음
  */
void LSM6DS3_ReadAll(LSM6DS3_Data_t *data) {
    uint8_t rawData[12];

    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, rawData, 12, 100);

    data->Gyro_X_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Gyro_Y_Raw = (int16_t)((rawData[3] << 8) | rawData[2]);
    data->Gyro_Z_Raw = (int16_t)((rawData[5] << 8) | rawData[4]);

    data->Accel_X_Raw = (int16_t)((rawData[7] << 8) | rawData[6]);
    data->Accel_Y_Raw = (int16_t)((rawData[9] << 8) | rawData[8]);
    data->Accel_Z_Raw = (int16_t)((rawData[11] << 8) | rawData[10]);

    data->Accel_X = (data->Accel_X_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Y = (data->Accel_Y_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Z = (data->Accel_Z_Raw * ACCEL_SENSITIVITY) / 1000.0f;

    data->Gyro_X = (data->Gyro_X_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Y = (data->Gyro_Y_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

/**
  * @brief  Z축 자이로 오프셋(bias) 캘리브레이션
  * @note   DMA 완료를 기다리며 샘플링 -> HAL_Delay 폴링 대신 DMA wait 사용
  */
void LSM6DS3_Gyro_Calibrate_Z_Only(void) {
    float sum = 0.0f;
    const int sample_count = 1000; // 샘플링 횟수 (약 1초 상당)

    for (int i = 0; i < sample_count; i++) {
        if (LSM6DS3_ReadGyroZ_DMA_Wait(50) != HAL_OK) {
            continue; // 타임아웃 시 해당 샘플 스킵
        }
        sum += imu_data.Gyro_Z;
        HAL_Delay(1); // ODR(104Hz) 대비 샘플 간격 확보
    }

    // 평균값(Offset) 계산
    imu_data.Gyro_Z_Offset = sum / (float)sample_count;
}

void LSM6DS3_UpdateYaw(LSM6DS3_Data_t *data, float dt) {
    // 1. 오프셋(bias) 보정된 각속도 (dps)
    float gyro_z_corrected = data->Gyro_Z - data->Gyro_Z_Offset;

    // 2. 각속도 적분 -> 각도 변화량 (dps * s = degree)
    data->Yaw_Angle += gyro_z_corrected * dt;

    // 3. -180 ~ +180 범위로 래핑 (불연속 처리)
    if (data->Yaw_Angle > 180.0f) {
        data->Yaw_Angle -= 360.0f;
    } else if (data->Yaw_Angle < -180.0f) {
        data->Yaw_Angle += 360.0f;
    }
}

/* ===================== HAL 콜백 ===================== */

/**
  * @brief  I2C DMA 수신 완료 콜백
  * @note   같은 I2C 핸들을 다른 주변장치도 DMA로 쓴다면 여기서 구분 로직 추가 필요
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != hi2c1.Instance) {
        return;
    }

    imu_data.Gyro_Z_Raw = (int16_t)((gyro_z_dma_buf[1] << 8) | gyro_z_dma_buf[0]);
    imu_data.Gyro_Z = (imu_data.Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;

    imu_gyro_z_ready = 1;
    imu_dma_busy = 0;
}

/**
  * @brief  I2C 에러 콜백 — busy 플래그 미해제로 인한 hang 방지
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != hi2c1.Instance) {
        return;
    }
    imu_dma_busy = 0;
}
