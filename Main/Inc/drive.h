#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "tim.h"
#include "sensor.h"
#include "arm_math.h"

// motor.hìì ì´ëë ì£¼í íì´ë¨¸ ì¤ì 
#define RAMP_DT		0.0005f
#define Ramp_TIM_IRQ_Handler TIM14_IRQ_Handler

#define SENSOR_DIST_L       0.13f  // ë°í´ íì  ì¶ ì¤ì¬ë¶í° ì¼ìë°ê¹ì§ì ìë¤ ê±°ë¦¬
#define WHEEL_TRACK_W       0.186f  // ì¢ì° ë°í´ ì¤ì¬ ì¬ì´ì ê°ê²©
#define SENSOR_HALF_WIDTH   0.08f // ì¼ìë° ì ì¤ìë¶í° ë§¨ ë 15ë² ì¼ìê¹ì§ì ê±°ë¦¬

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
} DriveParam_t;
#define DRIVE_HAS_SLALOM_RATIO 1
#define DRIVE_HAS_CASCADE_STEER 1
#define DRIVE_HAS_TARGET_CURVE_GAIN 1

extern DriveParam_t driveData;

// drive.cìì ì ìë ê°ê°ì ë° ì£¼í ìí ë³ìë¤
extern float_t accel;
extern float_t decel;
extern volatile uint8_t g_is_braking;
extern volatile float g_target_base_mps;
extern volatile float g_current_base_mps;


// ì£¼í ìíì¤ í¨ì
void Drive_Stop_At_Distance(float_t target_distance_m);
void Drive_First(void);
void Drive_Second(void); // 2íì°¨ ì£¼í í¨ì ì¶ê°
void Drive_Third(void);
void Drive_Fourth(void);

void Drive_Vibration_Test(void);

// ê°ê°ì ì ì´ í¨ì
void Ramp_Start(void);
void Ramp_Stop(void);

#endif /* INC_DRIVE_H_ */
