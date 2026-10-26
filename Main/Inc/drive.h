#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "tim.h"
#include "sensor.h"
#include "arm_math.h"
#include "ff.h"

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
	float_t target_shift_val;
	// Cascade steering (Steer_Motor): the line PD gives a yaw-rate reference (speed x steer),
	// an inner gyro loop tracks it. yaw_gain_p = yaw_gain_i = 0: the plain line PD as before.
	float_t yaw_gain_p;     // [-]     inner loop P on the yaw-rate error
	float_t yaw_gain_i;     // [1/s]   inner loop I on the yaw-rate error
	float_t lat_acc_max;    // [m/s^2] grip limit: slow down while speed x gyro yaw rate exceeds it (0 = off)
	float_t target_curve_gain; // [-] drive 3/4: target_pos x this inside curve segments (1 = unchanged)
	float_t slalom_ratio;      // [-] drive 4: 45 deg segments at base + (max - base) x this
	float_t brake_margin_m;    // [m] 감속 여유 거리: 직선 가속 후 다음 곡선 진입 전 미리 감속 완료할 마진
} DriveParam_t;
#define DRIVE_HAS_SLALOM_RATIO 1
#define DRIVE_HAS_CASCADE_STEER 1
#define DRIVE_HAS_TARGET_CURVE_GAIN 1
#define DRIVE_HAS_BRAKE_MARGIN 1

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
void Drive_Second(void); // 2회차 주행 함수 추가
void Drive_Third(void);
void Drive_Fourth(void);
void Drive_Fifth(void);

void Drive_Vibration_Test(void);
void Drive_SysID_Test(void);
FRESULT Drive_Save_SysID_To_SD(void);

// 가감속 제어 함수
void Ramp_Start(void);
void Ramp_Stop(void);

// 실기(Sim2Real) 주행 텔레메트리 진단 및 온보드 AI Auto-Heal
typedef struct {
	uint8_t drive_mode;        // 최근 실행된 주행 모드 (1~5)
	uint8_t line_lost;         // 라인 이탈 여부 (1 = Line Lost, 0 = 완주)
	uint8_t was_oversteer;     // 오버스티어/스핀 감지 여부 (1 = Oversteer, 0 = Understeer)
	float_t max_beta_deg;      // 추정 최대 슬립각 [deg]
	float_t max_lat_mm;        // 센서바 기준 최대 횡방향 오차 [mm]
	float_t weave_dps;         // 주행 중 자이로 고주파 진동 에너지 (RMS) [deg/s]
	float_t fail_v;            // 이탈 직전 주행 속도 [m/s]
	uint32_t heal_count;       // 누적 Auto-Heal 적용 횟수
	char last_role[16];        // 마지막 적용된 힐링 처방 이름
} DriveDiag_t;

extern DriveDiag_t g_drive_diag;

// SysID 500Hz 다채널 주행/모터 텔레메트리 샘플 버퍼 (최대 15초 = 7500 샘플)
#define SYSID_DRIVE_SAMPLE_MAX 7500

typedef struct {
	uint16_t t_ms;          // 경과 시간 [ms]
	int16_t  line_pos_x10k; // 센서바 라인 위치 * 10000 (-10000 .. +10000)
	int16_t  tgt_mps_mmps;  // 목표 속도 [mm/s]
	int16_t  mps_L_mmps;    // 좌측 바퀴 실측 선속도 [mm/s] (전진 +)
	int16_t  mps_R_mmps;    // 우측 바퀴 실측 선속도 [mm/s] (전진 +)
	int16_t  tgt_iq_L_mA;   // 좌측 모터 목표 전류 [mA] (전진 +)
	int16_t  tgt_iq_R_mA;   // 우측 모터 목표 전류 [mA] (전진 +)
	int16_t  iq_L_mA;       // 좌측 모터 실측 전류 [mA] (전진 +)
	int16_t  iq_R_mA;       // 우측 모터 실측 전류 [mA] (전진 +)
	int16_t  gyro_z_ddps;   // 자이로 Z 각속도 (오프셋 차감) * 10 [0.1 deg/s] (좌회전 CCW +)
	uint16_t odom_mm;       // 구간 누적 오도메트리 거리 [mm]
	uint8_t  marker_cnt;    // 현재까지 누적된 마커 개수 (g_cross_log_count)
	uint8_t  vbus_dV;       // 배터리 전압 * 10 [0.1 V]
} SysID_DriveSample_t;

extern SysID_DriveSample_t g_sysid_drive_buf[SYSID_DRIVE_SAMPLE_MAX];
extern volatile uint32_t g_sysid_drive_count;

void Drive_Diag_Reset(uint8_t drive_mode);
void Drive_Diag_Update_ISR(float_t line_pos_norm, float_t active_mps,
		float_t yaw_meas_rads, float_t yaw_ref_rads, float_t gyro_z_dps);
void Drive_Diag_Finalize(uint8_t line_lost);
void Drive_AutoHeal_Apply(void);
void Drive_AutoHeal_Reset_Anchor(void);
void Drive_AutoHeal_Menu(void);

#endif /* INC_DRIVE_H_ */
