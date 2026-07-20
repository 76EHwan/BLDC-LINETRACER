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
#define POS_WINDOW_HALF     3
#define POS_WINDOW_SIZE     (POS_WINDOW_HALF * 2)   // 6

// === 2단계(interleaved) 스캔 슬롯 구조 =======================================
// ADC3 single-conversion 시퀀스는 한 사이클 최대 16슬롯까지만 쓸 수 있어서,
// group2는 8개 후보 중 1차 패스 결과(line_position1)를 보고 2개를 제외한
// 6개만 그때그때 골라서 스캔한다 (아래 SENSOR_POS 관련 상수는 sensor.c 참조).
// 슬롯 0~7  : group1 (거친 해상도, 윈도우 없이 전체 폭 커버) -> 1차 position
// 슬롯 8    : 좌측 마커
// 슬롯 9    : 우측 마커
// 슬롯 10~15: group2 (8개 후보 중 동적으로 고른 6개) -> 2차 position
#define LINE_N_SENSORS      16
#define SCAN_GROUP_LEN 			10
#define SCAN_SLOT_MARK_L        8
#define SCAN_SLOT_MARK_R        9
#define SCAN_CYCLE_LEN          20   // SCAN_GROUP1_LEN + 2(mark) + SCAN_GROUP2_LEN
#define SCAN_CYCLE_LEN_HALF		(SCAN_CYCLE_LEN / 2)


typedef struct {
	volatile uint8_t idx;        // 0 ~ SCAN_CYCLE_LEN-1, 현재 스캔 슬롯 번호
	uint16_t raw[NUM_SENSORS];   // 물리 센서 인덱스로 저장 (스캔 순서와 무관)
	uint16_t whitemax[NUM_SENSORS];
	uint16_t blackmax[NUM_SENSORS];
	uint16_t normalized_coef_bias[NUM_SENSORS];
	uint16_t normalized[NUM_SENSORS];
	uint32_t state;
	uint16_t threshold;
	uint8_t line_lost_sum_min;

	uint8_t mark_left;
	uint8_t mark_right;
} SensorDataTypeDef;

typedef struct {
	uint8_t scan_group;
	uint8_t is_calibration;
	uint8_t is_lost_position;
	uint8_t is_position;
	volatile SensorDataTypeDef *data;
} Sensor_TypeDef;

extern volatile Sensor_TypeDef IR_Sensor;
extern uint16_t adc3_buffer[1];   // NbrOfConversion=1 (매 슬롯 채널 1개씩 single conversion)
extern volatile uint32_t count_sensor_irq;
extern const float line_sensor_pos[LINE_N_SENSORS];

void Sensor_Start();
void Sensor_Stop();

void Sensor_Printf(uint8_t idx, volatile uint16_t *sensor_data);

FRESULT Sensor_Save_Calibration(void);
FRESULT Sensor_Load_Calibration(void);
float Sensor_Get_Position(void);

void Sensor_Calibration();
void Sensor_Raw_Printf();
void Sensor_Normalize_Printf();
void Sensor_State_Printf();
void Sensor_Position_Printf();

void TIM7_IRQ_Handler(void);
void ADC3_IRQ_Half_Handler(void);
void ADC3_IRQ_Cplt_Handler(void);

void  Sensor_Line_LUT_Init(void);
float Sensor_Line_Estimate_Pass1(void);  // group1(8개) 완료 시점 호출: 무윈도우 대략 위치 + 윈도우 확정
float Sensor_Line_Estimate_Pass2(void);  // group2(8개) 완료 시점 호출: 윈도우 내부만 반영한 정밀 위치

void IMU_Test(void);


#endif /* INC_SENSOR_H_ */
