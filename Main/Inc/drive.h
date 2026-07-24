#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "tim.h"
#include "sensor.h"
#include "arm_math.h"

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
} DriveParam_t;

extern DriveParam_t driveData;
extern CrossMarkerLog_t g_cross_log[CROSS_LOG_MAX];
extern uint16_t g_cross_log_count; // 256개 대응을 위해 uint16_t로 변경
extern arm_pid_instance_f32 steer_pid; // motor.c에서 연산 수행

void Cross_Log_Push(CrossEvent_t type);
void Drive_Stop_At_Distance(float_t target_distance_m);
void Drive_First(void);

#endif /* INC_DRIVE_H_ */
