#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "tim.h"
#include "sensor.h"
#include "arm_math.h"

// motor.h에서 이동된 주행 타이머 설정
#define RAMP_DT		0.0005f
#define Ramp_TIM_IRQ_Handler TIM14_IRQ_Handler

#define SENSOR_DIST_L       0.13f  // 바퀴 회전 축 중심부터 센서바까지의 앞뒤 거리
#define WHEEL_TRACK_W       0.186f  // 좌우 바퀴 중심 사이의 간격
#define SENSOR_HALF_WIDTH   0.08f // 센서바 정중앙부터 맨 끝 15번 센서까지의 거리

typedef struct {
	float_t mpsL;
	float_t mpsR;
	float_t base_mps;
	float_t accel;
	float_t decel;
	float_t max_mps;
	float_t steer_gain_p;
	float_t steer_gain_d;
	float_t pos_atten_gain;
	float_t pit_in_distance_m;
	uint8_t fan_en;

	// ★ 3, 4회차 곡선 분류 및 가속용 파라미터 추가
	float_t turn45_len_s_m;
	float_t turn45_len_c_m;
	float_t turn90_len_s_m;
	float_t turn90_len_c_m;
	float_t add45_mps;
	float_t add90_mps;

	// ★ 영점 이동(Zero Point Shift) 파라미터 추가
	float_t zero_offset;         // 영점 최대 이동 폭 (단위: 정규화된 포지션 또는 미터)
	float_t zero_out_turn_m;     // 코너에서 탈출할 때 미리 영점을 옮기기 시작할 여유 거리
	float_t zero_out_straight_m; // 직선에서 코너로 진입할 때 미리 영점을 옮기기 시작할 여유 거리
} DriveParam_t;

extern DriveParam_t driveData;

// drive.c에서 정의된 가감속 및 주행 상태 변수들
extern float_t accel;
extern float_t decel;
extern volatile uint8_t g_is_braking;
extern volatile float g_target_base_mps;
extern volatile float g_current_base_mps;

// 주행 시퀀스 함수
void Drive_Stop_At_Distance(float_t target_distance_m);
void Drive_First(void);
void Drive_Second(void);
void Drive_Vibration_Test(void);
void Drive_Third(void);
void Drive_Fourth(void);

// 가감속 제어 함수
void Ramp_Start(void);
void Ramp_Stop(void);

#endif /* INC_DRIVE_H_ */
