/*
 * LSM6DS3TR.c
 *
 * Integrated Fixes:
 * 1. Automatic DMA trigger via EXTI (INT1)
 * 2. Automatic Yaw integration inside DMA Complete Callback
 */
#include "lsm6ds3tr-c.h"

LSM6DS3_Data_t imu_data;

volatile uint8_t imu_dma_busy = 0;
volatile uint8_t imu_gyro_z_ready = 0;

__attribute__((section(".ram_d2_nocache"), aligned(32)))
   static uint8_t gyro_z_dma_buf[2];

HAL_StatusTypeDef LSM6DS3_Init() {
	uint8_t chipID;
	uint8_t data;

	// ★ DWT 사이클 카운터 활성화 (초정밀 시간 측정용)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55; // 이 마법의 키(Key)를 넣어야 카운터가 움직입니다!
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	// 1. WHO_AM_I 레지스터 확인
	if (HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_WHO_AM_I,
	I2C_MEMADD_SIZE_8BIT, &chipID, 1, 100) != HAL_OK) {
		return HAL_ERROR;
	}

	if (chipID != WHO_AM_I_ID) {
		return HAL_ERROR;
	}

	// 2. 가속도계 설정 (CTRL1_XL)
	data = 0x40;
	HAL_I2C_Mem_Write(IMU_I2C, LSM6DS3_ADDR, REG_CTRL1_XL, I2C_MEMADD_SIZE_8BIT,
			&data, 1, 100);

	// 3. 자이로스코프 설정 (CTRL2_G)
	data = 0x8C;
	HAL_I2C_Mem_Write(IMU_I2C, LSM6DS3_ADDR, REG_CTRL2_G, I2C_MEMADD_SIZE_8BIT,
			&data, 1, 100);

	// ★ 수정: 하드웨어 인터럽트 핀을 사용하지 않으므로 INT1 매핑 설정 삭제
	/*
	 data = 0x02;
	 HAL_I2C_Mem_Write(IMU_I2C, LSM6DS3_ADDR, REG_INT1_CTRL,
	 I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	 */

	imu_dma_busy = 0;
	imu_gyro_z_ready = 0;

	return HAL_OK;
}

__STATIC_INLINE void LSM6DS3_ReadAccel(LSM6DS3_Data_t *data) {
	uint8_t rawData[6];
	HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_XL, I2C_MEMADD_SIZE_8BIT,
			rawData, 6, 100);

	data->Accel_X_Raw = (int16_t) ((rawData[1] << 8) | rawData[0]);
	data->Accel_Y_Raw = (int16_t) ((rawData[3] << 8) | rawData[2]);
	data->Accel_Z_Raw = (int16_t) ((rawData[5] << 8) | rawData[4]);

	data->Accel_X = (data->Accel_X_Raw * ACCEL_SENSITIVITY) / 1000.0f;
	data->Accel_Y = (data->Accel_Y_Raw * ACCEL_SENSITIVITY) / 1000.0f;
	data->Accel_Z = (data->Accel_Z_Raw * ACCEL_SENSITIVITY) / 1000.0f;
}

__STATIC_INLINE void LSM6DS3_ReadGyro(LSM6DS3_Data_t *data) {
	uint8_t rawData[6];
	HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT,
			rawData, 6, 100);

	data->Gyro_X_Raw = (int16_t) ((rawData[1] << 8) | rawData[0]);
	data->Gyro_Y_Raw = (int16_t) ((rawData[3] << 8) | rawData[2]);
	data->Gyro_Z_Raw = (int16_t) ((rawData[5] << 8) | rawData[4]);

	data->Gyro_X = (data->Gyro_X_Raw * GYRO_SENSITIVITY) / 1000.0f;
	data->Gyro_Y = (data->Gyro_Y_Raw * GYRO_SENSITIVITY) / 1000.0f;
	data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

void LSM6DS3_ReadGyro_Z_Only(LSM6DS3_Data_t *data) {
	uint8_t rawData[2];
	HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTZ_L_G, I2C_MEMADD_SIZE_8BIT,
			rawData, 2, 100);

	data->Gyro_Z_Raw = (int16_t) ((rawData[1] << 8) | rawData[0]);
	data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

void LSM6DS3_ReadGyroZ_DMA_Start(void) {
	if (imu_dma_busy) {
		return;
	}
	imu_dma_busy = 1;

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(
	IMU_I2C, LSM6DS3_ADDR, REG_OUTZ_L_G,
	I2C_MEMADD_SIZE_8BIT, gyro_z_dma_buf, 2);

	if (ret != HAL_OK) {
		imu_dma_busy = 0;
	}
}

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

void LSM6DS3_ReadAll(LSM6DS3_Data_t *data) {
	uint8_t rawData[12];
	HAL_I2C_Mem_Read(IMU_I2C, LSM6DS3_ADDR, REG_OUTX_L_G, I2C_MEMADD_SIZE_8BIT,
			rawData, 12, 100);

	data->Gyro_X_Raw = (int16_t) ((rawData[1] << 8) | rawData[0]);
	data->Gyro_Y_Raw = (int16_t) ((rawData[3] << 8) | rawData[2]);
	data->Gyro_Z_Raw = (int16_t) ((rawData[5] << 8) | rawData[4]);

	data->Accel_X_Raw = (int16_t) ((rawData[7] << 8) | rawData[6]);
	data->Accel_Y_Raw = (int16_t) ((rawData[9] << 8) | rawData[8]);
	data->Accel_Z_Raw = (int16_t) ((rawData[11] << 8) | rawData[10]);

	data->Accel_X = (data->Accel_X_Raw * ACCEL_SENSITIVITY) / 1000.0f;
	data->Accel_Y = (data->Accel_Y_Raw * ACCEL_SENSITIVITY) / 1000.0f;
	data->Accel_Z = (data->Accel_Z_Raw * ACCEL_SENSITIVITY) / 1000.0f;

	data->Gyro_X = (data->Gyro_X_Raw * GYRO_SENSITIVITY) / 1000.0f;
	data->Gyro_Y = (data->Gyro_Y_Raw * GYRO_SENSITIVITY) / 1000.0f;
	data->Gyro_Z = (data->Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;
}

void LSM6DS3_Gyro_Calibrate_Z_Only(void) {
	float_t sum = 0.0f;
	const int sample_count = 1000;

	for (int i = 0; i < sample_count; i++) {
		if (LSM6DS3_ReadGyroZ_DMA_Wait(50) != HAL_OK) {
			continue;
		}
		sum += imu_data.Gyro_Z;
		HAL_Delay(1);
	}
	imu_data.Gyro_Z_Offset = sum / (float_t) sample_count;
}

// 데드밴드 임계값 (단위: dps)
// 이 값보다 작은 회전은 진동(노이즈)으로 간주하고 무시합니다.
#define GYRO_DEADBAND  0.25f

void LSM6DS3_UpdateYaw(LSM6DS3_Data_t *data, float_t dt) {
	// 1. 정적 오프셋 제거 (주행 전 캘리브레이션으로 구한 평균 노이즈)
	float_t gyro_z_corrected = data->Gyro_Z - data->Gyro_Z_Offset;

	// 2. 데드밴드(Deadband) 적용
	// 이 값보다 작은 미세한 떨림은 회전이 아닌 노이즈로 보고 0으로 만듭니다.
	if (gyro_z_corrected > -GYRO_DEADBAND && gyro_z_corrected < GYRO_DEADBAND) {
		gyro_z_corrected = 0.0f;
	}

	// ★ HPF 삭제: 보정된 각속도를 "있는 그대로" 적분해야 절대 각도가 유지됩니다.
	data->Yaw_Angle += gyro_z_corrected * dt;

	// 3. -180 ~ +180 범위 래핑
	if (data->Yaw_Angle > 180.0f) {
		data->Yaw_Angle -= 360.0f;
	} else if (data->Yaw_Angle < -180.0f) {
		data->Yaw_Angle += 360.0f;
	}
}

void LSM6DS3_Reset_Yaw() {
	imu_data.Yaw_Angle = 0.f;
}

/* ===================== HAL 콜백 ===================== */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance != hi2c1.Instance) {
		return;
	}

	// 1. Z축 자이로 원시 데이터 파싱
	imu_data.Gyro_Z_Raw = (int16_t) ((gyro_z_dma_buf[1] << 8)
			| gyro_z_dma_buf[0]);
	imu_data.Gyro_Z = (imu_data.Gyro_Z_Raw * GYRO_SENSITIVITY) / 1000.0f;

	// ★ 2. DWT 사이클 카운터를 이용한 초정밀 dt 계산
	static uint32_t last_cyccnt = 0;
	uint32_t now_cyccnt = DWT->CYCCNT; // 현재 CPU 사이클 카운트 읽기

	if (last_cyccnt != 0) {
		// 오버플로우가 발생하더라도 unsigned 32-bit 연산 특성상 차이값은 정상적으로 계산됨
		uint32_t diff = now_cyccnt - last_cyccnt;

		// SystemCoreClock(예: 400000000)으로 나누면 완벽한 초(Seconds) 단위의 dt가 도출됨
		float_t dt = (float_t) diff / (float_t) SystemCoreClock;

		// 예외 처리: 만약 인터럽트가 너무 빨리 걸려서 dt가 극단적으로 작거나 무시할 수준이면 방어
		if (dt > 0.0001f && dt < 0.1f) {
			LSM6DS3_UpdateYaw(&imu_data, dt);
		}
	}
	last_cyccnt = now_cyccnt;

	imu_gyro_z_ready = 1;
	imu_dma_busy = 0;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance != hi2c1.Instance) {
		return;
	}
	imu_dma_busy = 0;
}
