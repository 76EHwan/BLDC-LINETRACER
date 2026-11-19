#ifndef INC_PP_STEER_H_
#define INC_PP_STEER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Geometric Pure Pursuit + Fixed-Radius Corner Curvature Controller.
 * Pure C module with zero HAL, sensor, motor, or global state dependencies.
 */
typedef struct {
	float lookahead_m;      /* Forward arc distance L_d [m] (e.g. 0.13213f) */
	float rad_per_unit;     /* Normalized position -> heading angle scale [rad/unit] */
	float corner_kappa;     /* Fixed corner curvature 1/R_min [m^-1] (e.g. 4.0f for R=0.25m) */
	float corner_e0;        /* Corner blend transition threshold in normalized units */
	float rejoin_gain_k;    /* Quadratic outer-arc rejoin boost coefficient */
	float rejoin_gain_max;  /* Upper clamp for rejoin boost */
	float kp_ref;           /* Reference Kp for gain scaling (e.g. 20.4f) */
	float max_alpha_rad;    /* Clamp on lookahead angle alpha [rad] */
} PPSteer_Config_t;

typedef struct {
	float error;            /* Normalized lateral tracking error (line_pos - target_pos) */
	float filtered_deriv;   /* Low-pass filtered derivative of line_pos */
	float kp;               /* Proportional gain (scaled relative to cfg->kp_ref) */
	float kd;               /* Derivative gain */
	float active_mps;       /* Current active forward speed [m/s] */
	float base_mps;         /* Reference base corner speed [m/s] */
	uint8_t is_straight;    /* 1: disable corner curvature & use full 1/v straight scaling */
	uint8_t is_braking;     /* 1: straight braking active (slight Kd boost) */
} PPSteer_Input_t;

/* Returns default configuration matching ZETIN 16-ch Option-B sensor bar & R=0.25m rule. */
PPSteer_Config_t PPSteer_DefaultConfig(void);

/* Pure function: computes desired path curvature [m^-1] = p_term + d_term. */
float PPSteer_ComputeCurvature(const PPSteer_Config_t *cfg, const PPSteer_Input_t *in);

/* Pure helper: optional inner gyro yaw-rate cascade loop returning yaw_cmd [rad/s]. */
float PPSteer_CascadeYaw(float steer_kappa, float active_mps, float yaw_meas_rads,
                         float yaw_kp, float yaw_ki, float dt_s,
                         uint8_t freeze_integ, float p_boost_scale,
                         float *io_yaw_integ, float integ_max);

#ifdef __cplusplus
}
#endif

#endif /* INC_PP_STEER_H_ */
