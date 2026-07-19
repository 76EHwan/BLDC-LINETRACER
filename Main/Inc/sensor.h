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

// === 2단계(interleaved) 스캔 슬롯 구조 =======================================
// ADC3 single-conversion 시퀀스는 한 사이클 최대 16슬롯까지만 쓸 수 있어서,
// group2는 8개 후보 중 1차 패스 결과(line_position1)를 보고 2개를 제외한
// 6개만 그때그때 골라서 스캔한다 (아래 SENSOR_POS 관련 상수는 sensor.c 참조).
// 슬롯 0~7  : group1 (거친 해상도, 윈도우 없이 전체 폭 커버) -> 1차 position
// 슬롯 8    : 좌측 마커
// 슬롯 9    : 우측 마커
// 슬롯 10~15: group2 (8개 후보 중 동적으로 고른 6개) -> 2차 position
#define SCAN_GROUP1_LEN         8
#define SCAN_GROUP2_CANDIDATE_LEN 8   // group2 전체 후보 개수
#define SCAN_GROUP2_LEN         6     // 실제로 스캔하는 개수 (후보에서 2개 제외)
#define SCAN_SLOT_MARK_L        8
#define SCAN_SLOT_MARK_R        9
#define SCAN_SLOT_GROUP2_START  10
#define SCAN_CYCLE_LEN          16   // SCAN_GROUP1_LEN + 2(mark) + SCAN_GROUP2_LEN


typedef struct {
	volatile uint8_t idx;        // 0 ~ SCAN_CYCLE_LEN-1, 현재 스캔 슬롯 번호
	uint16_t raw[NUM_SENSORS];   // 물리 센서 인덱스로 저장 (스캔 순서와 무관)
	uint16_t whitemax[NUM_SENSORS];
	uint16_t blackmax[NUM_SENSORS];
	uint16_t normalized_coef_bias[NUM_SENSORS];
	uint16_t normalized[NUM_SENSORS];
	uint32_t state;
	float_t line_position;    // 최신 확정값 (1차 또는 2차 갱신 결과가 그때그때 반영됨)
	float_t line_position1;   // 이번 사이클의 1차(무윈도우, group1 8개) 추정값
	uint16_t threshold;
	uint8_t pos_center_idx;   // 현재 위치 창의 중심 인덱스 (0~15), 1차 패스에서 매 사이클 갱신
	uint8_t win_start;        // 이번 사이클 윈도우 시작 인덱스 (1차 패스에서 확정)
	uint8_t win_end;          // 이번 사이클 윈도우 끝 인덱스
	uint8_t cross_left;       // 창 바깥 좌측에서 마커 후보 검출됨
	uint8_t cross_right;      // 창 바깥 우측에서 마커 후보 검출됨
	uint8_t line_w_bandwidth;
	uint8_t line_lost_sum_min;
} SensorDataTypeDef;

typedef struct {
	uint32_t is_calibration;
	uint32_t is_lost_position;
	volatile SensorDataTypeDef *data;
} Sensor_TypeDef;

extern volatile Sensor_TypeDef IR_Sensor;
extern uint16_t adc3_buffer[16];   // NbrOfConversion=1 (매 슬롯 채널 1개씩 single conversion)
extern volatile uint32_t count_sensor_irq;

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
void ADC3_IRQ_Half_Handler(void);
void ADC3_IRQ_Cplt_Handler(void);

void  Sensor_Line_LUT_Init(void);
float Sensor_Line_Estimate_Pass1(void);  // group1(8개) 완료 시점 호출: 무윈도우 대략 위치 + 윈도우 확정
float Sensor_Line_Estimate_Pass2(void);  // group2(8개) 완료 시점 호출: 윈도우 내부만 반영한 정밀 위치

void IMU_Test(void);


#endif /* INC_SENSOR_H_ */
