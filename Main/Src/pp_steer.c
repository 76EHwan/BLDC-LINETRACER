#include "pp_steer.h"
#include <math.h>

PPSteer_Config_t PPSteer_DefaultConfig(void) {
	PPSteer_Config_t cfg;
	cfg.lookahead_m = 0.13213f;
	cfg.rad_per_unit = 0.41450f;
	cfg.corner_kappa = 4.00000f;
	cfg.corner_e0 = 0.12f;
	cfg.rejoin_gain_k = 1.85f;
	cfg.rejoin_gain_max = 1.30f;
	cfg.kp_ref = 20.4f;
	cfg.max_alpha_rad = 1.2f;
	return cfg;
}

float PPSteer_ComputeCurvature(const PPSteer_Config_t *cfg, const PPSteer_Input_t *in) {
	float alpha = cfg->rad_per_unit * in->error;
	if (alpha > cfg->max_alpha_rad) alpha = cfg->max_alpha_rad;
	else if (alpha < -cfg->max_alpha_rad) alpha = -cfg->max_alpha_rad;

	/* Pure Pursuit curvature: kappa_pp = 2 * sin(alpha) / L_d */
	float kappa_pp = (2.0f * sinf(alpha)) / cfg->lookahead_m;

	/* Smooth nonlinear blend for fixed-radius corner curvature */
	float abs_err = fabsf(in->error);
	float err_sq = abs_err * abs_err;
	float e0_sq = cfg->corner_e0 * cfg->corner_e0;
	float corner_weight = err_sq / (err_sq + e0_sq);

	float rejoin_gain = 1.0f + cfg->rejoin_gain_k * err_sq;
	if (rejoin_gain > cfg->rejoin_gain_max) rejoin_gain = cfg->rejoin_gain_max;

	float sign_err = (in->error >= 0.0f) ? 1.0f : -1.0f;
	float kappa_corner = sign_err * cfg->corner_kappa * corner_weight * rejoin_gain;

	float v_base = (in->base_mps > 0.5f) ? in->base_mps : 1.0f;
	float eff_speed = in->active_mps;
	float speed_ratio = 1.0f;
	float pp_speed_ratio = 1.0f;
	if (eff_speed > v_base) {
		speed_ratio = v_base / eff_speed;
		pp_speed_ratio = sqrtf(speed_ratio);
	}

	if (in->is_straight) {
		kappa_corner = 0.0f;
		if (eff_speed > v_base) {
			pp_speed_ratio = speed_ratio;
		}
	}

	float kp_scale = in->kp / cfg->kp_ref;
	float p_term = ((kappa_pp * pp_speed_ratio) + (kappa_corner * speed_ratio)) * kp_scale;

	float high_speed_weight = 1.0f - speed_ratio;
	float kd_scale = 0.65f + 0.35f * corner_weight + 0.60f * high_speed_weight;
	if (in->is_straight && in->is_braking) {
		kd_scale *= 1.05f;
	}
	float d_term = (in->kd * kd_scale) * in->filtered_deriv * speed_ratio;

	return p_term + d_term;
}

float PPSteer_CascadeYaw(float steer_kappa, float active_mps, float yaw_meas_rads,
                         float yaw_kp, float yaw_ki, float dt_s,
                         uint8_t freeze_integ, float p_boost_scale,
                         float *io_yaw_integ, float integ_max) {
	float yaw_ref = active_mps * steer_kappa;
	float yaw_cmd = yaw_ref;

	if (yaw_kp > 0.0f || yaw_ki > 0.0f) {
		float yaw_err = yaw_ref - yaw_meas_rads;
		float integ = io_yaw_integ ? *io_yaw_integ : 0.0f;
		if (active_mps > 0.05f && !freeze_integ) {
			integ += yaw_ki * yaw_err * dt_s;
			if (integ > integ_max) integ = integ_max;
			else if (integ < -integ_max) integ = -integ_max;
		} else {
			integ = 0.0f;
		}
		if (io_yaw_integ) *io_yaw_integ = integ;

		float p_yaw = yaw_kp * p_boost_scale;
		yaw_cmd = yaw_ref + p_yaw * yaw_err / (1.0f + p_yaw) + integ;
	} else if (io_yaw_integ) {
		*io_yaw_integ = 0.0f;
	}

	return yaw_cmd;
}
