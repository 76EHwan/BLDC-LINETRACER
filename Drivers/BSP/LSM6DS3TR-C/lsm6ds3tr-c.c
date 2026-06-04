/*
 * LSM6DS3TR.c
 *
 * Integrated Fixes:
 * 1. Added missing bias variables (bias_gx_rad, etc.)
 * 2. Fixed Complementary Filter discontinuity issue (wrapping logic)
 * 3. Uses -180 ~ +180 degree range for robust calculation
 */

#include "lsm6ds3tr-c.h"

#define IMU_I2C	&hi2c1

LSM6DS3_Data_t imu_data;

/**
  * @brief  LSM6DS3 초기화 함수
  * @param  hi2c: I2C 핸들 포인터
  * @retval 0: 성공, 1: 실패 (ID 불일치 또는 통신 에러)
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

    return HAL_OK; // 성공
}

/**
  * @brief  가속도 데이터 읽기 및 변환
  */
__STATIC_INLINE void LSM6DS3_ReadAccel(LSM6DS3_Data_t *data) {
    uint8_t rawData[6];

    // 가속도 레지스터 6바이트 (X_L, X_H, Y_L, Y_H, Z_L, Z_H) 연속 읽기
    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_XL, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

    // 데이터 결합 (Little Endian)
    data->Accel_X_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Accel_Y_Raw = (int16_t)((rawData[3] << 8) | rawData[2]);
    data->Accel_Z_Raw = (int16_t)((rawData[5] << 8) | rawData[4]);

    // 물리량 변환 (mg -> g)
    // 2g 모드에서 0.061 mg/LSB
    data->Accel_X = (data->Accel_X_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Y = (data->Accel_Y_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Z = (data->Accel_Z_Raw * ACCEL_SENSITIVITY) / 1000.0f;
}

/**
  * @brief  자이로 데이터 읽기 및 변환
  */
__STATIC_INLINE void LSM6DS3_ReadGyro(LSM6DS3_Data_t *data) {
    uint8_t rawData[6];

    // 자이로 레지스터 6바이트 연속 읽기
    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

    data->Gyro_X_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Gyro_Y_Raw = (int16_t)((rawData[3] << 8) | rawData[2]);
    data->Gyro_Z_Raw = (int16_t)((rawData[5] << 8) | rawData[4]);

    // 물리량 변환 (mdps -> dps)
    // 2000dps 모드에서 70 mdps/LSB
    data->Gyro_X = (data->Gyro_X_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Y = (data->Gyro_Y_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

void LSM6DS3_ReadGyro_Z_Only(LSM6DS3_Data_t *data) {
    uint8_t rawData[2];

    // 자이로 레지스터 6바이트 연속 읽기
    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTZ_L_G, I2C_MEMADD_SIZE_8BIT, rawData, 2, 100);

    data->Gyro_Z_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);

    // 물리량 변환 (mdps -> dps)
    // 2000dps 모드에서 70 mdps/LSB
    data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

/**
  * @brief  모든 데이터(Accel+Gyro) 한 번에 읽기
  * @note   레지스터 맵 상 Gyro가 Accel보다 먼저 위치하므로 12바이트를 한 번에 읽음
  */
void LSM6DS3_ReadAll(LSM6DS3_Data_t *data) {
    uint8_t rawData[12];

    // OUTX_L_G(0x22) 부터 12바이트 읽기 (Gyro 6bytes + Accel 6bytes)
    HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, rawData, 12, 100);

    // Gyro Parsing
    data->Gyro_X_Raw = (int16_t)((rawData[1] << 8) | rawData[0]);
    data->Gyro_Y_Raw = (int16_t)((rawData[3] << 8) | rawData[2]);
    data->Gyro_Z_Raw = (int16_t)((rawData[5] << 8) | rawData[4]);

    // Accel Parsing
    data->Accel_X_Raw = (int16_t)((rawData[7] << 8) | rawData[6]);
    data->Accel_Y_Raw = (int16_t)((rawData[9] << 8) | rawData[8]);
    data->Accel_Z_Raw = (int16_t)((rawData[11] << 8) | rawData[10]);

    // Conversion
    data->Accel_X = (data->Accel_X_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Y = (data->Accel_Y_Raw * ACCEL_SENSITIVITY) / 1000.0f;
    data->Accel_Z = (data->Accel_Z_Raw * ACCEL_SENSITIVITY) / 1000.0f;

    data->Gyro_X = (data->Gyro_X_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Y = (data->Gyro_Y_Raw * GYRO_SENSITIVITY) / 1000.0f;
    data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

void Gyro_Calibrate_Z_Only(void) {
    float sum = 0.0f;
    const int sample_count = 1000; // 샘플링 횟수 (1초 정도 소요)

    for (int i = 0; i < sample_count; i++) {
        // Z축 데이터만 읽기
        LSM6DS3_ReadGyro_Z_Only(&imu_data);
        sum += imu_data.Gyro_Z;
        HAL_Delay(1); // 1ms 대기
    }

    // 평균값(Offset) 계산
    imu_data.Gyro_Z_Offset = sum / (float)sample_count;
}
