/*
 * drive.h
 *
 *  Created on: 2026. 7. 2.
 *      Author: kth59
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "tim.h"
#include "sensor.h"

#define RAMP_TIM	(&htim14)
#define Ramp_TIM_IRQ_Handler TIM14_IRQ_Handler

typedef struct {
	float_t mpsL;
	float_t mpsR;
	float_t base_mps;
	float_t accel;
	float_t decel;
	float_t max_mps;
	float_t steer_gain;
	float_t pos_atten_gain;
} DriveParam_t;

// === 교차로(cross) 마커 이벤트 ==============================================
// 위치 창(POS_WINDOW) 바깥쪽 라인센서(idx0~15 중 창에서 제외된 부분)에서
// 검출되는 cross_left/cross_right 후보를 기반으로 판정한다(sensor.c).
// idx16/17 전용 마커 포토인터럽터는 예비용이라 여기서는 사용하지 않는다.
// 좌우 동시 검출 시 정지 지점(STOP)으로 처리한다.
typedef enum {
	CROSS_NONE = 0,
	CROSS_LEFT,
	CROSS_RIGHT,
	CROSS_STOP,   // 좌/우 마커 동시 검출 -> 주행 정지
} CrossEvent_t;

#define CROSS_LOG_MAX 32

typedef struct {
	CrossEvent_t type;
	float dist_from_prev_m;   // 이전 마커로부터의 주행 거리 (엔코더/FOC 속도 적분 기반, m)
} CrossMarkerLog_t;

extern DriveParam_t driveData;
extern CrossMarkerLog_t g_cross_log[CROSS_LOG_MAX];
extern uint8_t g_cross_log_count;   // 누적 기록 횟수 (버퍼는 CROSS_LOG_MAX에서 순환)

void Line_Follow_Drive(void);

#endif /* INC_DRIVE_H_ */
