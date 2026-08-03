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

	// ===== 3회차 주행: 곡선 각도 판별 및 곡선 가속 =====
	// 곡선 구간의 길이(m)로 45도 / 90도 / 긴 곡선을 판별한다.
	// 다음 구간이 곡선이면 마커 간 거리가 더 길게 측정되므로 임계값을 따로 둔다.
	float_t turn45_len_s_m;		// 다음이 직선일 때 45도로 볼 최대 길이
	float_t turn45_len_c_m;		// 다음이 곡선일 때 45도로 볼 최대 길이
	float_t turn90_len_s_m;		// 다음이 직선일 때 90도로 볼 최대 길이
	float_t turn90_len_c_m;		// 다음이 곡선일 때 90도로 볼 최대 길이
	float_t add45_mps;			// 45도 곡선에서 base_mps에 더할 속도
	float_t add90_mps;			// 90도 곡선에서 base_mps에 더할 속도

	// ===== 4회차 주행: 영점이동 =====
	float_t zero_offset;		// 최대 영점 오프셋 (0 ~ 1, 센서 정규화 단위)
	float_t zero_shift_rate;	// 1m 전진당 옮길 오프셋 양 (1/m)
	float_t zero_out_turn_m;	// 곡선 구간 탈출 시 추가 여유 거리
	float_t zero_out_straight_m;	// 직선 구간 탈출 시 추가 여유 거리
} DriveParam_t;

extern DriveParam_t driveData;

// drive.c에서 정의된 가감속 및 주행 상태 변수들
extern float_t accel;
extern float_t decel;
extern volatile uint8_t g_is_braking;
extern volatile float g_target_base_mps;
extern volatile float g_current_base_mps;

// ===== 영점이동 (4회차 주행) =====
// Steer_Motor()에서 센서 위치에 더해지는 오프셋. 0이면 영점이동을 하지 않는다.
// 음수 = 로봇이 왼쪽으로, 양수 = 로봇이 오른쪽으로 이동한다.
extern volatile float_t g_line_offset;
extern volatile float_t g_line_offset_target;
extern volatile float_t g_line_offset_rate;	// 1m 전진당 이동량. 0이면 슬루 정지


// 주행 시퀀스 함수
void Drive_Stop_At_Distance(float_t target_distance_m);
void Drive_First(void);
void Drive_Second(void); // 2회차 주행 함수 추가
void Drive_Third(void);  // 3회차 주행: 곡선 각도 판별 + 곡선 가속
void Drive_Fourth(void); // 4회차 주행: 3회차 + 영점이동

// 가감속 제어 함수
void Ramp_Start(void);
void Ramp_Stop(void);

#endif /* INC_DRIVE_H_ */
