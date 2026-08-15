/*
 * sd_ui.h
 *
 *  Created on: 2026. 7. 24.
 *      Author: kth59
 */

#ifndef INC_SD_UI_H_
#define INC_SD_UI_H_

#include "main.h"
#include "ff.h"
#include "sensor.h" // CrossEvent_t 및 CROSS_LOG_MAX 사용을 위해 필요

#define CALIBRATION_PATH	"/Sensor_Data/calibration_result.txt"

#define FOC_PARAM_PATH		"/Foc_Data/foc_param.txt"

// ============================================================================
// ★ 2차 주행 로그 기록용 데이터 구조체 및 전역 변수 선언 추가
// ============================================================================
typedef struct {
    uint16_t ref_idx;     // 매칭된 참조 인덱스 (state->idx)
    CrossEvent_t type;    // 인식된 마커 종류
    uint8_t accel_active; // 가속 여부 (1: 가속 중, 0: 기본 속도)
    uint8_t mismatch;     // 마커 놓침(불일치) 여부 (1: Mismatch 상태, 0: 정상)
    float dist;           // 누적 거리
} SecondDriveLog_t;

extern SecondDriveLog_t g_second_log[CROSS_LOG_MAX];
extern uint16_t g_second_log_count;
// ============================================================================

// SD카드 저장/불러오기 통합 함수
FRESULT Sensor_Save_Calibration(void);
FRESULT Sensor_Load_Calibration(void);
FRESULT Save_FOC_Parameters(void);
FRESULT Load_FOC_Parameters(void);
void Save_MarkerLog_To_SD(uint8_t slot_number);
void Save_SecondDriveLog_To_SD(uint8_t slot_number); // 2차 주행 로그 저장 함수 선언

// UI 함수
uint8_t Select_Save_Slot(void);

#endif /* INC_SD_UI_H_ */
