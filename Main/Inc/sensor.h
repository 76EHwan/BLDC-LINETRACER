/*
 * sensor.h
 *
 *  Created on: 2026. 5. 2.
 *      Author: kth59
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "sdcard.h"

#define NUM_SENSORS 18
#define LEFT_MARK_SENSOR_INDEX 16
#define RIGHT_MARK_SENSOR_INDEX 17   // idx16/17 전용 마커 포토인터럽터는 예비용 (현재 미사용)

// === 위치 추정에 실제로 쓰는 창(window) ======================================
// 중앙 16개(idx 0~15) 중, 현재 라인 위치(center idx) 기준 좌우로 4개씩(총 8개)만
// centroid 계산에 사용한다. 창 바깥쪽은 교차로(cross) 마커 후보 검출용으로 쓴다.
#define POS_WINDOW_HALF     4
#define POS_WINDOW_SIZE     (POS_WINDOW_HALF * 2)   // 8


typedef struct {
	volatile uint8_t idx;
	uint16_t raw[NUM_SENSORS];
	uint16_t whitemax[NUM_SENSORS];
	uint16_t blackmax[NUM_SENSORS];
	uint16_t normalized_coef_bias[NUM_SENSORS];
	uint16_t normalized[NUM_SENSORS];
	uint32_t state;
	float_t line_position;
	uint16_t threshold;
	uint8_t pos_center_idx;   // 현재 위치 창의 중심 인덱스 (0~15), 다음 프레임 창 재중심에 사용
	uint8_t cross_left;       // 창 바깥 좌측(idx < window_start)에서 마커 후보 검출됨
	uint8_t cross_right;      // 창 바깥 우측(idx > window_end)에서 마커 후보 검출됨
	uint8_t line_w_bandwidth;
	uint8_t line_lost_sum_min;
} SensorDataTypeDef;

typedef struct {
	uint32_t is_calibration;
	uint32_t is_lost_position;
	volatile SensorDataTypeDef *data;
} Sensor_TypeDef;

extern volatile Sensor_TypeDef IR_Sensor;
extern uint16_t adc3_buffer[3];

void Sensor_Start();
void Sensor_Stop();

FRESULT Sensor_Save_Calibration(void);
FRESULT Sensor_Load_Calibration(void);

void Sensor_Calibration();
void Sensor_Raw_Printf();
void Sensor_Normalize_Printf();
void Sensor_State_Printf();
void Sensor_Position_Printf();

void TIM7_IRQ_Handler(void);
void ADC3_IRQ_Handler(void);

void  Sensor_Line_LUT_Init(void);
float Sensor_Line_Estimate(void);

void IMU_Test(void);


#endif /* INC_SENSOR_H_ */
