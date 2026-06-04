/*
 * mt6701.c
 *
 *  Created on: Jan 24, 2026
 *  Author: kth59
 */

#include "mt6701.h"

#ifdef MAGNETIC_ENCODER

#define ENC_SPI &hspi2

/* USER CODE BEGIN 0 */
MT6701_Data_t encDataL = { .cs_port = ENC_L_CS_GPIO_Port, .cs_pin = ENC_L_CS_Pin };
MT6701_Data_t encDataR = { .cs_port = ENC_R_CS_GPIO_Port, .cs_pin = ENC_R_CS_Pin };

// 초기화 함수
HAL_StatusTypeDef MT6701_Init(MT6701_Data_t *encData, uint8_t *rxBuffer) {

    HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(ENC_SPI, rxBuffer, 3, 10);
    HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_SET);

    // 3. 24비트 데이터를 먼저 하나로 합침 (파싱 오류 방지)
    uint32_t rawValue = ((uint32_t)rxBuffer[0] << 16) | ((uint32_t)rxBuffer[1] << 8) | rxBuffer[2];

    // 4. 합쳐진 데이터에서 4비트 Status 추출 (비트 9~6 자리)
    uint8_t fault_state = (rawValue >> 6) & 0x0F;
    encData->status = fault_state;

    // 5. 에러 조건 검사 로직 개선
    // - fault_state & 0x08 (Bit 9가 1이면 Loss of Track 에러)
    // - fault_state & 0x03 == 0x03 (정의되지 않은 Mg 상태 에러)
    if ((fault_state & 0x08) || ((fault_state & 0x03) == 0x03)) {
        return HAL_ERROR;
    }

    encData->last_raw_angle = (rawValue >> 10) & 0x3FFF;
    encData->motor_elec_angle = 0.0f;
    return HAL_OK;
}

void MT6701_ReadSSI(MT6701_Data_t *encData) {
    uint8_t rxBuffer[3]; // 초기화 불필요 (속도 ↑)

    HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef spi_status = HAL_SPI_Receive(ENC_SPI, rxBuffer, 3, 2);
    HAL_GPIO_WritePin(encData->cs_port, encData->cs_pin, GPIO_PIN_SET);

    if (spi_status == HAL_OK) {
        uint32_t rawValue = ((uint32_t)rxBuffer[0] << 16) | ((uint32_t)rxBuffer[1] << 8) | rxBuffer[2];
        uint16_t new_raw = (rawValue >> 10) & 0x3FFF; // 0 ~ 16383

        // 3. Status 업데이트 (실시간 에러 모니터링용)
        encData->status = (rawValue >> 6) & 0x0F;

        // 변화량 계산 (Delta)
        int32_t diff = (int32_t)new_raw - (int32_t)encData->last_raw_angle;

        // Wrap-around 처리
        if (diff < -ENC_HALF) diff += 16384;
        else if (diff > ENC_HALF) diff -= 16384;

        // 각도 변환 및 누적
        float angle_step = (float)diff * DEG_PER_TICK * GEAR_RATIO * POLE_PAIRS;
        encData->motor_elec_angle += angle_step;

        // 0~360 범위 유지
        encData->motor_elec_angle -= 360.0f * floorf(encData->motor_elec_angle / 360.0f);

        // 데이터 갱신
        encData->last_raw_angle = new_raw;
    }
}
#endif
