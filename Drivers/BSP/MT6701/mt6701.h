/*
 * mt6701.h
 *
 *  Created on: Jan 24, 2026
 *  Author: kth59
 */

#ifndef INC_MT6701_H_
#define INC_MT6701_H_

#ifdef MAGNETIC_ENCODER
#include "main.h"
#include "spi.h"
#include "main.h"
#include <math.h> // floorf 함수 사용
// 기어비 (51 / 9)
#define GEAR_RATIO (51.0f / 9.0f)
// 극쌍수 1
#define POLE_PAIRS 1.0f

// 상수 미리 정의 (연산 속도 최적화)
#define ENC_RES 16384.0f
#define ENC_HALF 8192
#define DEG_PER_TICK (360.0f / ENC_RES) // 0.02197...

typedef struct {
    uint16_t raw_angle;
    float wheel_angle_deg;
    uint16_t last_raw_angle;
    float motor_elec_angle;
    uint8_t status;
    uint8_t crc;
    GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} MT6701_Data_t;

extern MT6701_Data_t encDataL;
extern MT6701_Data_t encDataR;

HAL_StatusTypeDef MT6701_Init(MT6701_Data_t *encData, uint8_t *rxBuffer);
void MT6701_ReadSSI(MT6701_Data_t *encData);

#endif

#endif /* INC_MT6701_H_ */
