#include "main.h"
#include "dac.h"
#include "drive.h"
#include "sensor.h"
#include "foc.h"
#include "motor.h"
#include "user_init.h"
#include "button.h"
#include "sd_ui.h"
#include "buzzer.h"
#include "lptim.h"
#include "lsm6ds.h"
#include "fifth_rl_weights.h"
#ifndef STM32H743xx
#include <stdio.h>
#include <stdlib.h>
#endif

#define RAMP_TIM        (&htim14)
#define VIB_SAMPLE_MAX  10000

// @formatter:off
DriveParam_t driveData = {
    .base_mps = 1.80f,
    .max_mps = 7.00f,
    .accel = 5.5f,
    .decel = 5.5f,
    .steer_gain_p = 19.200f,
    .steer_gain_d = 184.0f,
    .pos_atten_gain = 0.00f,
    .pit_in_distance_m = 0.15f,
    .fan_en = 0,
	.target_shift_val = 0.58f,
	.yaw_gain_p = 0.25f,
	.yaw_gain_i = 0.0f,
	.target_curve_gain = 1.25f,
	.slalom_ratio = 0.03f,
	.brake_margin_m = 0.06f,
	.bulge_lat_acc = 5.50f,
	.corner45_max_mps = 3.00f,
	.omega_add_mps = 0.04f,
	.wave_add_mps = 0.20f,
	.ramp_jerk_s = 0.00f,
	.ramp_settle_ms = 0.0f,
	.fourth_in_mm = 80.0f,
	.fourth_side_mm = 65.0f,
	.fourth_alat_gain = 0.90f,
	.fourth_hgain = 2.00f,
};
// @formatter:on

uint8_t g_total_L = 0;
uint8_t g_total_R = 0;
uint8_t g_total_C = 0;
uint8_t g_total_STOP = 0;

// ============================================================================
// 타이머 및 가감속(Ramp) 변수
// ============================================================================

float_t accel;
float_t decel;
volatile float_t g_target_base_mps = 0.0f;
volatile float_t g_current_base_mps = 0.0f;
volatile uint8_t g_is_braking = 0;
volatile uint8_t g_is_straight = 0;
volatile uint8_t g_straight_cut_active = 0;
volatile uint8_t g_suppress_marker_detect = 0;
volatile float g_min_same_side_odom_m = 0.070f;
volatile float_t g_straight_cut_yaw_ref = 0.0f;
volatile float_t g_straight_cut_kappa_ref = 0.0f;
volatile float_t g_straight_cut_line_blend = 0.0f;
static uint8_t g_drive_fifth_active = 0;
static float g_fifth_rl_weights[FIFTH_RL_PARAM_COUNT];
static float g_fifth_rl_speed_scale = 1.0f;
uint32_t count_irq = 0;

static void Fifth_RL_Load_Weights(void) {
	for (int i = 0; i < FIFTH_RL_PARAM_COUNT; i++) {
		g_fifth_rl_weights[i] = g_fifth_rl_default_weights[i];
	}
#ifndef STM32H743xx
	const char *path = getenv("LINESIM_FIFTH_WEIGHTS");
	if (!path || !path[0]) {
		path = "runs/drive5_rl/best_weights.txt";
	}
	if (path && path[0]) {
		FILE *f = fopen(path, "r");
		if (f) {
			for (int i = 0; i < FIFTH_RL_PARAM_COUNT; i++) {
				if (fscanf(f, "%f", &g_fifth_rl_weights[i]) != 1) {
					break;
				}
			}
			fclose(f);
		}
	}
#endif
}

static void Fifth_RL_Infer(const float obs[FIFTH_RL_OBS_DIM], float act[FIFTH_RL_ACT_DIM]) {
	float hid[FIFTH_RL_HID_DIM];
	int off = 0;
	for (int h = 0; h < FIFTH_RL_HID_DIM; h++) {
		float sum = 0.0f;
		for (int i = 0; i < FIFTH_RL_OBS_DIM; i++) {
			sum += g_fifth_rl_weights[off++] * obs[i];
		}
		hid[h] = sum;
	}
	for (int h = 0; h < FIFTH_RL_HID_DIM; h++) {
		hid[h] = tanhf(hid[h] + g_fifth_rl_weights[off++]);
	}
	for (int a = 0; a < FIFTH_RL_ACT_DIM; a++) {
		float sum = 0.0f;
		for (int h = 0; h < FIFTH_RL_HID_DIM; h++) {
			sum += g_fifth_rl_weights[off++] * hid[h];
		}
		act[a] = sum;
	}
	for (int a = 0; a < FIFTH_RL_ACT_DIM; a++) {
		act[a] = tanhf(act[a] + g_fifth_rl_weights[off++]);
	}
}

void Ramp_TIM_IRQ_Handler(void) {
	Odom_Accumulate(RAMP_DT);

	// 가속 시작 완화 (driveData.ramp_jerk_s / ramp_settle_ms, 둘 다 0 이면 기존 램프와 동일):
	// 곡선 탈출 직후 남은 방향 흔들림이 곧바로 최대 가속을 만나 속도에 비례해 커지는 것을 줄인다.
	static float s_acc_now = 0.0f;      // 저크 제한 중인 현재 가속도 [m/s^2]
	static uint16_t s_settle_wait = 0;  // 안정 대기 경과 [RAMP_DT tick]
	float d_mps = g_target_base_mps - g_current_base_mps;
	if (d_mps > 0.0f) {
		float acc_eff = accel;
		// 1) 기준 속도에서 가속을 시작하려 할 때 라인 오차·요레이트가 안정될 때까지 최대 ramp_settle_ms 대기
		if (driveData.ramp_settle_ms > 0.0f && !g_steer_settled
				&& g_current_base_mps > 0.3f && g_current_base_mps <= driveData.base_mps + 0.05f
				&& s_settle_wait < (uint16_t) (driveData.ramp_settle_ms * 0.001f / RAMP_DT)) {
			acc_eff = 0.0f;
			s_settle_wait++;
		}
		// 2) 저크 제한: 가속도를 ramp_jerk_s 동안 0 → accel 로 증가
		if (driveData.ramp_jerk_s > 0.0f) {
			if (acc_eff > 0.0f) {
				s_acc_now += accel * RAMP_DT / driveData.ramp_jerk_s;
				if (s_acc_now > acc_eff)
					s_acc_now = acc_eff;
			} else {
				s_acc_now = 0.0f;
			}
			acc_eff = s_acc_now;
		}
		if (d_mps > acc_eff * RAMP_DT)
			g_current_base_mps += acc_eff * RAMP_DT;
		else
			g_current_base_mps = g_target_base_mps;
	} else {
		s_acc_now = 0.0f;
		s_settle_wait = 0;
		if (d_mps < -decel * RAMP_DT)
			g_current_base_mps -= decel * RAMP_DT;
		else
			g_current_base_mps = g_target_base_mps;
	}
	Steer_Motor();
}

void Ramp_Start(void) {
	g_target_base_mps = 0.f;
	g_current_base_mps = 0.f;
	HAL_TIM_Base_Start_IT(RAMP_TIM); // RAMP_TIM
}

void Ramp_Stop(void) {
	HAL_TIM_Base_Stop_IT(RAMP_TIM);
}

// ============================================================================
// 주행 제어 공통 함수
// ============================================================================
void Drive_Stop_At_Distance(float target_distance_m) {
	if (g_current_base_mps <= 0.0f || target_distance_m <= 0.0f) {
		g_target_base_mps = 0.0f;
		HAL_Delay(500);
		return;
	}

	decel = (g_current_base_mps * g_current_base_mps)
			/ (2.0f * target_distance_m);
	g_target_base_mps = 0.0f;
	g_is_braking = 1;

	while (g_current_base_mps > 0.001f) {
	}
	g_is_braking = 0;
}

// ============================================================================
// 실기(Sim2Real) 주행 텔레메트리 실시간 진단 및 온보드 AI Auto-Heal
// ============================================================================
DriveDiag_t g_drive_diag = { 0 };
static DriveParam_t g_heal_anchor;
static uint8_t g_heal_anchor_init = 0;

// ISR 실시간 누적용 내부 상태 변수
static volatile float_t s_beta_rad = 0.0f;
static volatile float_t s_gyro_lpf_dps = 0.0f;
static volatile float_t s_weave_sq_sum = 0.0f;
static volatile uint32_t s_weave_cnt = 0;
static volatile float_t s_recent_v = 0.0f;

// SysID 500Hz 주행 텔레메트리 버퍼 (2ms 간격, 최대 15초)
SysID_DriveSample_t g_sysid_drive_buf[SYSID_DRIVE_SAMPLE_MAX];
volatile uint32_t g_sysid_drive_count = 0;
static volatile uint8_t s_sysid_isr_div = 0;

__STATIC_INLINE float_t Clamp_F32(float_t x, float_t lo, float_t hi) {
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

void Drive_AutoHeal_Reset_Anchor(void) {
	if (g_heal_anchor_init) {
		driveData.steer_gain_p = g_heal_anchor.steer_gain_p;
		driveData.steer_gain_d = g_heal_anchor.steer_gain_d;
		driveData.brake_margin_m = g_heal_anchor.brake_margin_m;
		driveData.target_shift_val = g_heal_anchor.target_shift_val;
		driveData.target_curve_gain = g_heal_anchor.target_curve_gain;
		driveData.slalom_ratio = g_heal_anchor.slalom_ratio;
		driveData.bulge_lat_acc = g_heal_anchor.bulge_lat_acc;
		driveData.corner45_max_mps = g_heal_anchor.corner45_max_mps;
		driveData.omega_add_mps = g_heal_anchor.omega_add_mps;
		driveData.wave_add_mps = g_heal_anchor.wave_add_mps;
		driveData.yaw_gain_p = g_heal_anchor.yaw_gain_p;
		steer_pid.Kp = driveData.steer_gain_p;
		steer_pid.Kd = driveData.steer_gain_d;
	}
}

void Drive_Diag_Reset(uint8_t drive_mode) {
	if (!g_heal_anchor_init) {
		g_heal_anchor = driveData;
		// 포화 상태 방지용 안전 범위 클램핑 후 초기 기준점(Anchor) 저장
		g_heal_anchor.steer_gain_p = (g_heal_anchor.steer_gain_p < 0.0f) ? 0.0f : g_heal_anchor.steer_gain_p;
		g_heal_anchor.steer_gain_d = (g_heal_anchor.steer_gain_d < 0.0f) ? 0.0f : g_heal_anchor.steer_gain_d;
		g_heal_anchor.brake_margin_m = Clamp_F32(g_heal_anchor.brake_margin_m, 0.06f, 0.24f);
		g_heal_anchor_init = 1;
	}

	g_drive_diag.drive_mode = drive_mode;
	g_drive_diag.line_lost = 0;
	g_drive_diag.was_oversteer = 0;
	g_drive_diag.max_beta_deg = 0.0f;
	g_drive_diag.max_lat_mm = 0.0f;
	g_drive_diag.weave_dps = 0.0f;
	g_drive_diag.fail_v = driveData.base_mps;
	g_drive_diag.max_a_lat = 0.0f;

	s_beta_rad = 0.0f;
	s_gyro_lpf_dps = 0.0f;
	s_weave_sq_sum = 0.0f;
	s_weave_cnt = 0;
	s_recent_v = driveData.base_mps;

	g_sysid_drive_count = 0;
	s_sysid_isr_div = 0;
}

void Drive_Diag_Update_ISR(float_t line_pos_norm, float_t active_mps,
		float_t yaw_meas_rads, float_t yaw_ref_rads, float_t gyro_z_dps) {
	// 0. 500Hz (2ms 주기 = 2kHz TIM14 4틱마다) SysID 다채널 주행 텔레메트리 샘플링
	if (!IR_Sensor.is_lost_position && (g_target_base_mps > 0.01f || g_current_base_mps > 0.01f)) {
		uint8_t div = s_sysid_isr_div + 1;
		if (div >= 4) {
			div = 0;
			uint32_t idx = g_sysid_drive_count;
			if (idx < SYSID_DRIVE_SAMPLE_MAX) {
				SysID_DriveSample_t *sp = &g_sysid_drive_buf[idx];
				float_t mps_l = foc_L.omega_e_meas / MPS_TO_OMEGA;
				float_t mps_r = -foc_R.omega_e_meas / MPS_TO_OMEGA;
				sp->t_ms = (uint16_t) (idx * 2U);
				sp->line_pos_x10k = (int16_t) Clamp_F32(line_pos_norm * 10000.0f, -32000.0f, 32000.0f);
				sp->tgt_mps_mmps = (int16_t) Clamp_F32(g_current_base_mps * 1000.0f, -32000.0f, 32000.0f);
				sp->mps_L_mmps = (int16_t) Clamp_F32(mps_l * 1000.0f, -32000.0f, 32000.0f);
				sp->mps_R_mmps = (int16_t) Clamp_F32(mps_r * 1000.0f, -32000.0f, 32000.0f);
				sp->tgt_iq_L_mA = (int16_t) Clamp_F32(foc_L.target_Iq * 1000.0f, -32000.0f, 32000.0f);
				sp->tgt_iq_R_mA = (int16_t) Clamp_F32(-foc_R.target_Iq * 1000.0f, -32000.0f, 32000.0f);
				sp->iq_L_mA = (int16_t) Clamp_F32(foc_L.I_q * 1000.0f, -32000.0f, 32000.0f);
				sp->iq_R_mA = (int16_t) Clamp_F32(-foc_R.I_q * 1000.0f, -32000.0f, 32000.0f);
				sp->gyro_z_ddps = (int16_t) Clamp_F32(gyro_z_dps * 10.0f, -32000.0f, 32000.0f);
				sp->acc_x_raw = imu_accel_capture_en ? imu_data.Accel_X_Raw : 0;
				sp->acc_y_raw = imu_accel_capture_en ? imu_data.Accel_Y_Raw : 0;
				sp->odom_mm = (uint16_t) Clamp_F32(g_odom_distance_m * 1000.0f, 0.0f, 65000.0f);
				sp->marker_cnt = (uint8_t) ((g_cross_log_count < 255U) ? g_cross_log_count : 255U);
				sp->vbus_dV = (uint8_t) Clamp_F32(g_vbus_filt * 10.0f, 0.0f, 255.0f);
				g_sysid_drive_count = idx + 1;
			}
		}
		s_sysid_isr_div = div;
	}

	if (IR_Sensor.is_lost_position || g_target_base_mps <= 0.05f) {
		return;
	}

	// 1. 센서바 기준 횡방향 오차 [mm] (정규화 1.0 = SENSOR_HALF_WIDTH = 80mm)
	float_t lat_mm = fabsf(line_pos_norm) * (SENSOR_HALF_WIDTH * 1000.0f);
	if (lat_mm > g_drive_diag.max_lat_mm) {
		g_drive_diag.max_lat_mm = lat_mm;
	}

	// 2. 이탈 직전(최근 ~0.25s) 주행 속도 엔벨로프 추적
	if (g_current_base_mps > s_recent_v) {
		s_recent_v = g_current_base_mps;
	} else {
		s_recent_v = 0.9985f * s_recent_v + 0.0015f * g_current_base_mps;
	}

	if (active_mps < 0.35f) {
		s_beta_rad *= 0.98f;
		return;
	}

	// 3. 추정 슬립각(Beta): 실제 자이로 요레이트와 조향 목표 요레이트의 편차를 누수 적분 (tau ~= 0.18s)
	float_t yaw_diff = yaw_meas_rads - yaw_ref_rads;
	s_beta_rad = 0.9972f * s_beta_rad + yaw_diff * RAMP_DT;
	float_t beta_deg = fabsf(s_beta_rad) * (180.0f / PI);
	if (beta_deg > g_drive_diag.max_beta_deg) {
		g_drive_diag.max_beta_deg = beta_deg;
	}

	// 4. 자이로 고주파 진동 에너지(Weave): 60ms LPF 대비 고역통과 성분 제곱 누적
	s_gyro_lpf_dps = 0.983f * s_gyro_lpf_dps + 0.017f * gyro_z_dps;
	float_t hp_dps = gyro_z_dps - s_gyro_lpf_dps;
	s_weave_sq_sum += hp_dps * hp_dps;
	s_weave_cnt++;

	// 5. 실측 횡가속도 피크 추적: v * |omega_z|
	float_t a_lat_obs = fabsf(active_mps * yaw_meas_rads);
	if (a_lat_obs > g_drive_diag.max_a_lat) {
		g_drive_diag.max_a_lat = a_lat_obs;
	}
}

void Drive_Diag_Finalize(uint8_t line_lost) {
	g_drive_diag.line_lost = line_lost;
	g_drive_diag.fail_v = s_recent_v;
	if (s_weave_cnt > 100) {
		g_drive_diag.weave_dps = sqrtf(s_weave_sq_sum / (float_t) s_weave_cnt);
	} else {
		g_drive_diag.weave_dps = 0.0f;
	}
	g_drive_diag.was_oversteer = (g_drive_diag.max_beta_deg >= 25.0f) ? 1 : 0;
}

// 텔레메트리 기반 횡가속도(그립 한계) 온보드 자동 학습 및 적응 함수
uint8_t Drive_AutoAdapt_Grip(char *diag_str, uint8_t diag_str_size) {
	(void) diag_str;
	(void) diag_str_size;
	return 0;
}

// 실패/주행 텔레메트리 비례 게인 보정 + 누적 포화(Ratchet) 방지 소프트 블렌딩
void Drive_AutoHeal_Apply(void) {
	if (!g_heal_anchor_init) {
		Drive_Diag_Reset(g_drive_diag.drive_mode);
	}

	// 1. 한쪽 쏠림(누적 포화) 방지: 현재 게인을 초기 기준점(Anchor) 쪽으로 15% 복원한 base 계산
	DriveParam_t base = driveData;
	base.steer_gain_p = 0.85f * driveData.steer_gain_p + 0.15f * g_heal_anchor.steer_gain_p;
	base.steer_gain_d = 0.85f * driveData.steer_gain_d + 0.15f * g_heal_anchor.steer_gain_d;
	base.brake_margin_m = 0.85f * driveData.brake_margin_m + 0.15f * g_heal_anchor.brake_margin_m;
	base.target_shift_val = 0.85f * driveData.target_shift_val + 0.15f * g_heal_anchor.target_shift_val;
	base.target_curve_gain = 0.85f * driveData.target_curve_gain + 0.15f * g_heal_anchor.target_curve_gain;

	// D게인 과포화(anchor 대비 1.6배 초과) 및 P게인 과소 감지 및 해소
	const float_t anchor_kd_ref = (g_heal_anchor.steer_gain_d > 50.0f) ? g_heal_anchor.steer_gain_d : driveData.steer_gain_d;
	uint8_t kd_oversaturated = (driveData.steer_gain_d > anchor_kd_ref * 1.60f) ? 1 : 0;
	if (kd_oversaturated) {
		base.steer_gain_d = fmaxf(0.0f, 0.65f * driveData.steer_gain_d + 0.35f * anchor_kd_ref);
	}
	if (driveData.steer_gain_p < 8.13f) {
		float_t anchor_kp = (g_heal_anchor.steer_gain_p > 9.47f) ? g_heal_anchor.steer_gain_p : 9.47f;
		base.steer_gain_p = Clamp_F32(0.65f * driveData.steer_gain_p + 0.35f * anchor_kp, 8.53f, 25.0f);
	}

	// 2. 주행 텔레메트리 기반 정규화 심각도(0.0 ~ 1.5) 비례 계산
	float_t slip_sev = Clamp_F32((g_drive_diag.max_beta_deg - 22.0f) / 38.0f, 0.0f, 1.5f);
	float_t under_sev = g_drive_diag.was_oversteer ? 0.0f
			: Clamp_F32((26.0f - g_drive_diag.max_beta_deg) / 16.0f
					+ (g_drive_diag.max_lat_mm - 55.0f) / 40.0f, 0.25f, 1.5f);
	float_t osc_sev = Clamp_F32((g_drive_diag.weave_dps - 165.0f) / 135.0f, 0.0f, 1.5f);
	float_t speed_sev = (g_drive_diag.drive_mode <= 1) ? 0.0f
			: Clamp_F32((g_drive_diag.fail_v - driveData.base_mps) / 0.75f, 0.0f, 1.5f);

	DriveParam_t healed = base;
	const char *role = "PropPD";

	// 3. 실패 원인별 비례 처방 계산
	if (kd_oversaturated && osc_sev > 0.30f) {
		// [DeSat] D게인 과포화로 인한 고주파 노이즈 발진 -> Anchor 기준 복원
		role = "DeSat";
		healed.steer_gain_p = fmaxf(0.0f, roundf((0.55f * driveData.steer_gain_p + 0.45f * g_heal_anchor.steer_gain_p + 0.23f * under_sev) * 1000.0f) / 1000.0f);
		healed.steer_gain_d = fmaxf(0.0f, 0.50f * driveData.steer_gain_d + 0.50f * anchor_kd_ref);
	} else if (speed_sev > 0.45f && g_drive_diag.drive_mode >= 2) {
		// [Entry] 직선 고속 가속 후 코너 진입 감속 부족 -> 진입 초과 속도에 비례해 브레이크 마진 증가
		role = "Entry";
		float_t entry_sev = (speed_sev > slip_sev) ? speed_sev : slip_sev;
		healed.steer_gain_p = fmaxf(0.0f, roundf((base.steer_gain_p + (g_drive_diag.was_oversteer ? -0.17f * slip_sev : +0.27f * under_sev)) * 1000.0f) / 1000.0f);
		float_t target_kd = (anchor_kd_ref > healed.steer_gain_p * 28.68f) ? anchor_kd_ref : (healed.steer_gain_p * 28.68f);
		healed.steer_gain_d = fmaxf(0.0f, 0.70f * base.steer_gain_d + 0.30f * target_kd);
		healed.brake_margin_m = Clamp_F32(base.brake_margin_m + (0.015f + 0.030f * entry_sev), 0.07f, 0.24f);
	} else if (g_drive_diag.was_oversteer) {
		// [PropOVS] 오버스티어/드리프트 과다 -> 슬립각 및 진동량에 비례해 KP 감쇠 & KD 보강
		role = "PropOVS";
		float_t dkp = -(0.23f * slip_sev + 0.30f * osc_sev);
		healed.steer_gain_p = fmaxf(0.0f, roundf((base.steer_gain_p + dkp) * 1000.0f) / 1000.0f);
		if (kd_oversaturated) {
			healed.steer_gain_d = fmaxf(0.0f, driveData.steer_gain_d - (16.0f + 24.0f * osc_sev));
		} else {
			float_t dkd = 9.3f * slip_sev + 13.3f * osc_sev;
			healed.steer_gain_d = fmaxf(0.0f, base.steer_gain_d + dkd);
		}
		healed.yaw_gain_p = Clamp_F32(base.yaw_gain_p + 0.03f * slip_sev, 0.15f, 0.60f);
		if (g_drive_diag.drive_mode >= 2) {
			healed.brake_margin_m = Clamp_F32(base.brake_margin_m + (0.012f * slip_sev + 0.022f * speed_sev), 0.06f, 0.24f);
		}
	} else {
		// [PropUND] 언더스티어/조향 부족 이탈 -> 이탈 심각도에 비례해 KP 상향 & KD 정렬
		role = "PropUND";
		float_t dkp = 0.43f * under_sev - 0.17f * osc_sev;
		healed.steer_gain_p = fmaxf(0.0f, roundf((base.steer_gain_p + dkp) * 1000.0f) / 1000.0f);
		float_t ideal_kd = (anchor_kd_ref > healed.steer_gain_p * 28.68f) ? anchor_kd_ref : (healed.steer_gain_p * 28.68f);
		healed.steer_gain_d = fmaxf(0.0f, 0.70f * base.steer_gain_d + 0.30f * ideal_kd + 6.7f * osc_sev);
		if (g_drive_diag.drive_mode >= 3) {
			healed.target_shift_val = Clamp_F32(base.target_shift_val + 0.030f * under_sev, 0.20f, 1.45f);
			healed.target_curve_gain = Clamp_F32(base.target_curve_gain + 0.050f * under_sev, 1.00f, 5.00f);
		}
	}

	// 4. 소프트 블렌딩 (65% Healed + 35% Current) 및 비포화 상하한 클램핑
	const float_t a = 0.15f;
	driveData.steer_gain_p = fmaxf(0.0f, roundf((a * healed.steer_gain_p + (1.0f - a) * driveData.steer_gain_p) * 1000.0f) / 1000.0f);
	driveData.steer_gain_d = fmaxf(0.0f, a * healed.steer_gain_d + (1.0f - a) * driveData.steer_gain_d);
	
	driveData.brake_margin_m = Clamp_F32(a * healed.brake_margin_m + (1.0f - a) * driveData.brake_margin_m, 0.06f, 0.24f);
	driveData.target_shift_val = Clamp_F32(a * healed.target_shift_val + (1.0f - a) * driveData.target_shift_val, 0.20f, 1.45f);
	driveData.target_curve_gain = Clamp_F32(a * healed.target_curve_gain + (1.0f - a) * driveData.target_curve_gain, 1.00f, 5.00f);
	driveData.yaw_gain_p = Clamp_F32(a * healed.yaw_gain_p + (1.0f - a) * driveData.yaw_gain_p, 0.15f, 0.65f);
	driveData.pos_atten_gain = 0.0f;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Kd = driveData.steer_gain_d;

	g_drive_diag.heal_count++;
	snprintf(g_drive_diag.last_role, sizeof(g_drive_diag.last_role), "%s", role);
}

// 주행 종료 화면에서 K_HOLD(종료) 외에 R_HOLD(온보드 AI Auto-Heal 즉시 적용), L_HOLD(초기 게인 복원), U_HOLD(SysID 저장) 지원
static void Drive_Wait_Exit_Or_Heal(uint8_t show_diag_lines) {
	if (show_diag_lines) {
		LCD_Printf(0, 6, "B:%.1fd W:%.0f", g_drive_diag.max_beta_deg, g_drive_diag.weave_dps);
		LCD_Printf(0, 7, "Vf:%.2f [R:Heal]", g_drive_diag.fail_v);
	}
	while (1) {
		UserInput_t btn = Button_Get_Input();
		if (btn == INPUT_CMD_K_HOLD) {
			break;
		} else if (btn == INPUT_CMD_R_HOLD) {
			Drive_AutoHeal_Apply();
			LCD_Printf(0, 8, "H:%s", g_drive_diag.last_role);
			LCD_Printf(0, 9, "P:%.3f D:%.0f", driveData.steer_gain_p, driveData.steer_gain_d);
			LCD_Printf(0, 10, "Brake: %.2f", driveData.brake_margin_m);
		} else if (btn == INPUT_CMD_L_HOLD) {
			Drive_AutoHeal_Reset_Anchor();
			LCD_Printf(0, 8, "Reset Anchor!");
			LCD_Printf(0, 9, "P:%.3f D:%.0f", driveData.steer_gain_p, driveData.steer_gain_d);
			LCD_Printf(0, 10, "Brake: %.2f", driveData.brake_margin_m);
		} else if (btn == INPUT_CMD_U_HOLD) {
			LCD_Printf(0, 8, "Saving SysID..");
			FRESULT r = Drive_Save_SysID_To_SD();
			LCD_Printf(0, 8, (r == FR_OK) ? "SysID Saved!  " : "SysID Err:%d  ", r);
		}
	}
}

// Drive Menu에서 진입 가능한 온보드 AI Auto-Heal 진단/적용 화면
void Drive_AutoHeal_Menu(void) {
	LCD_Clear();
	while (1) {
		LCD_Printf(0, 0, "AI Auto-Heal #%lu", (unsigned long) g_drive_diag.heal_count);
		LCD_Printf(0, 1, "Last: D%d %s", g_drive_diag.drive_mode,
				g_drive_diag.line_lost ? (g_drive_diag.was_oversteer ? "OVS" : "UND") : "OK");
		LCD_Printf(0, 2, "Beta:%.1fd Lat:%.0f", g_drive_diag.max_beta_deg, g_drive_diag.max_lat_mm);
		LCD_Printf(0, 3, "Weav:%.0f Vf:%.2f", g_drive_diag.weave_dps, g_drive_diag.fail_v);
		LCD_Printf(0, 4, "KP:%.3f KD:%.0f", driveData.steer_gain_p, driveData.steer_gain_d);
		LCD_Printf(0, 5, "BRK:%.2f", driveData.brake_margin_m);
		LCD_Printf(0, 6, "SHF:%.2f CRV:%.2f", driveData.target_shift_val, driveData.target_curve_gain);
		LCD_Printf(0, 8, "[R Hold] Heal Now");
		LCD_Printf(0, 9, "[L Hold] Anchor");
		LCD_Printf(0, 10, "[K Hold] Exit");

		UserInput_t btn = Button_Get_Input();
		if (btn == INPUT_CMD_K_HOLD) {
			break;
		} else if (btn == INPUT_CMD_R_HOLD) {
			Drive_AutoHeal_Apply();
			LCD_Clear();
			LCD_Printf(0, 7, "-> %s Applied!", g_drive_diag.last_role);
		} else if (btn == INPUT_CMD_L_HOLD) {
			Drive_AutoHeal_Reset_Anchor();
			LCD_Clear();
			LCD_Printf(0, 7, "-> Anchor Restored");
		}
	}
	LCD_Clear();
}

// ============================================================================
// 1회차 주행 함수 모음
// ============================================================================
static uint8_t is_ref_log_copied = 0;

void Drive_Reset_Ref_Log_Flag(void) {
    is_ref_log_copied = 0;
}

__STATIC_INLINE void Drive_IMU_Auto_Calibrate(void) {
	if (imu_data.Model != LSM6DS_MODEL_UNKNOWN) {
		LCD_Printf(0, 0, "IMU Calibrating ");
		LCD_Printf(0, 1, "Keep Still...   ");
		LSM6DS3_Gyro_Calibrate_Z_Only();
		LCD_Printf(0, 1, "Ofst:%+6.2f dps", (double)imu_data.Gyro_Z_Offset);
		HAL_Delay(200);
	}
}

__STATIC_INLINE uint8_t Drive_Init_Sequence(void) {
	LCD7789_Invert(0);

	is_ref_log_copied = 0;

	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0;
		}
	}

	Drive_IMU_Auto_Calibrate();

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();
	g_total_odom_m = 0.0f;
	imu_straight_bias_track = 0;
	g_straight_cut_active = 0;
	g_suppress_marker_detect = 0;
	g_min_same_side_odom_m = 0.070f;
	g_straight_cut_yaw_ref = 0.0f;
	g_straight_cut_kappa_ref = 0.0f;
	Steer_Reset_State();
	Drive_Diag_Reset(1);
	g_cross_log_count = 0;
	Cross_Detect_Reset();

	IR_Sensor.is_lost_position = 0;
	IR_Sensor.data->mark_left = 0;
	IR_Sensor.data->mark_right = 0;
    IR_Sensor.data->target_pos = 0.0f;

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;

	arm_pid_init_f32(&steer_pid, 1);
	arm_pid_init_f32(&steer_pid, 0);

	Buzzer_Discount_Start();

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();
	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event(CrossEvent_t cross) {
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2)
			return 1;
		return 0;
	}

	if (cross == CROSS_LEFT)
		g_total_L++;
	else if (cross == CROSS_RIGHT)
		g_total_R++;
	else if (cross == CROSS_CROSS)
		g_total_C++;
	return 0;
}

extern uint16_t g_last_stop_state;
extern uint8_t g_last_stop_count;

void Drive_First(void) {
	g_is_straight = 0;
	if (!Drive_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();

	uint8_t exit_reason_lost = 0;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event(cross))
				break;
		}
	}

	if (IR_Sensor.is_lost_position) {
		exit_reason_lost = 1;
	}
	Drive_Diag_Finalize(exit_reason_lost);
	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;

	if (exit_reason_lost) {
		LCD_Printf(0, 0, "Line Lost");
		Drive_Save_MapLog_Show(1, lap_time, 1);
#ifndef SIM_FW_REAL_MAIN_H
		Drive_Save_SysID_To_SD();
#endif
		Drive_Wait_Exit_Or_Heal(1);
	} else {
		LCD_Printf(0, 0, "End");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
		LCD_Printf(0, 5, "U_Bit:0x%04X", g_last_stop_state);
		LCD_Printf(0, 6, "U_Cnt:%d (<12)", g_last_stop_count);
		Drive_Save_MapLog_Show(1, lap_time, 0);
		uint8_t slot = Select_Save_Slot();
		Save_MarkerLog_To_SD(slot);
#ifndef SIM_FW_REAL_MAIN_H
		Drive_Save_SysID_To_SD();
#endif
		Drive_Wait_Exit_Or_Heal(0);
	}

	LCD_Clear();
}

// ============================================================================
// 2회차 주행 함수 모음
// ============================================================================
typedef struct {
	uint8_t accel_ok;
} SegmentPlan_t;

static CrossMarkerLog_t ref_log[CROSS_LOG_MAX];
static uint16_t ref_log_count = 0;
static SegmentPlan_t seg_plan[CROSS_LOG_MAX];
static float ref_log_cum_dist[CROSS_LOG_MAX];
static float ref_pose_x[CROSS_LOG_MAX];
static float ref_pose_y[CROSS_LOG_MAX];
static float ref_pose_th[CROSS_LOG_MAX];

#define BRAKE_MARGIN_M 0.06f
#define ACCEL_START_MARGIN_M 0.25f // 코너 탈출 직후 차체 직진 정렬 완료 후 가속 개시 마진 (기존 0.08m -> 0.25m)

// 각도 차이를 [-180, 180] 범위로 정규화 [deg] (시뮬레이터 while-yield 매크로 회피를 위해 for 사용)
__STATIC_INLINE float Wrap_Angle_Deg(float deg) {
	for (; deg > 180.0f; deg -= 360.0f) {}
	for (; deg < -180.0f; deg += 360.0f) {}
	return deg;
}

// ZETIN 45도 규칙 기반 세그먼트 회전각 스냅 (직선 노이즈 제거, 커브 45°*k 스냅)
static float Snap_Zetin_Segment_Yaw(float dy_raw, float seg_d) {
	if (fabsf(dy_raw) < 22.0f || seg_d <= 0.005f) {
		return 0.0f;
	}
	const float arc45 = 0.25f * (45.0f * PI / 180.0f);
	// 직선 구간도 직전 곡선의 요 지연/오버슈트로 최대 ~24도가 읽힌다. 45도 곡선은 길이가 ~0.23 m 이하이므로
	// 34도 미만이면서 45도 곡선 길이의 1.6배보다 긴 구간은 직선으로 본다 (45도 오인 시 이후 맵 헤딩 전체가 45도 틀어짐)
	if (fabsf(dy_raw) < 34.0f && seg_d > 1.6f * arc45) {
		return 0.0f;
	}
	// 곡선에서는 요가 ~5%, 오도메트리 거리가 ~8% 크게 읽힌다 (보정 없으면 225도 곡선을 270도로 오인)
	float k_est = 0.5f * (fabsf(dy_raw) / (45.0f * 1.05f)) + 0.5f * (seg_d / (arc45 * 1.08f));
	int k = (int)roundf(k_est);
	if (k < 1) k = 1;
	if (k > 6) k = 6;
	return ((dy_raw > 0.0f) ? 1.0f : -1.0f) * (float)k * 45.0f;
}

// 1차 주행 ref_log로부터 누적 거리 및 2D 참조 포즈(x, y, th) 구축
static void Build_Ref_Log_Geometry(void) {
	float cum = 0.0f;
	for (uint16_t i = 0; i < ref_log_count; i++) {
		cum += ref_log[i].dist_from_prev_m;
		ref_log_cum_dist[i] = cum;
	}
	if (ref_log_count == 0) return;
	float px = 0.0f, py = 0.0f, pth = 0.0f;
	ref_pose_x[0] = 0.0f;
	ref_pose_y[0] = 0.0f;
	ref_pose_th[0] = 0.0f;
	for (uint16_t i = 1; i < ref_log_count; i++) {
		float d = ref_log[i].dist_from_prev_m;
		float dyaw = Snap_Zetin_Segment_Yaw(ref_log[i].yaw_angle, d);
		float mid_rad = (pth + 0.5f * dyaw) * (PI / 180.0f);
		px += d * cosf(mid_rad);
		py += d * sinf(mid_rad);
		pth = Wrap_Angle_Deg(pth + dyaw);
		ref_pose_x[i] = px;
		ref_pose_y[i] = py;
		ref_pose_th[i] = pth;
	}
}

// STOP 마커 처리 및 코스 중간 인접 트랙 스침으로 인한 허위 STOP 방어
// 반환: 0 = 계속 진행, 1 = 완주 정지, 2 = 무시(return 0)
static uint8_t Handle_Stop_Marker_Common(uint16_t idx, float eff_odom, CrossEvent_t *cross) {
	float total_course_dist = (ref_log_count > 0) ? ref_log_cum_dist[ref_log_count - 1] : 50.0f;
	if (g_total_STOP == 0 || (eff_odom >= total_course_dist - 2.0f && idx + 3 >= ref_log_count)) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2) return 1;
	} else {
		if (idx < ref_log_count && ref_log[idx].type != CROSS_STOP) {
			*cross = ref_log[idx].type;
			if (g_cross_log_count > 0) {
				g_cross_log[(g_cross_log_count - 1) % CROSS_LOG_MAX].type = *cross;
			}
		} else {
			return 2;
		}
	}
	return 0;
}

// 최근 10개 마커 시퀀스 상관 + 2D 벡터/방향 유사도 기반 재동기화 (교차로 없는 구간 포함)
static void Resync_Marker_Sequence(CrossEvent_t cross, uint16_t *idx, uint8_t *mismatch,
                                   float *odom_offset_m, float imu_grid_bias, float eff_odom) {
	uint16_t n_cur = g_cross_log_count;
	if (n_cur > CROSS_LOG_MAX) n_cur = CROSS_LOG_MAX;
	if (n_cur < 3 || ref_log_count < 3) {
		int best_k = -1;
		float min_dist_err = 0.45f;
		int search_start = (int)(*idx) - 2;
		if (search_start < 0) search_start = 0;
		int search_end = (int)(*idx) + 8;
		if (search_end > (int)ref_log_count) search_end = (int)ref_log_count;
		for (int k = search_start; k < search_end; k++) {
			if (ref_log[k].type == cross) {
				float d_err = fabsf(eff_odom - ref_log_cum_dist[k]);
				if (d_err < min_dist_err) {
					min_dist_err = d_err;
					best_k = k;
				}
			}
		}
		if (best_k >= 0) {
			*idx = (uint16_t)best_k;
			*mismatch = 0;
		}
		return;
	}

	uint16_t W = (n_cur < 10) ? n_cur : 10;
	float cur_yaw = Wrap_Angle_Deg(imu_data.Yaw_Angle - imu_grid_bias);

	// 최근 W개 마커의 현재 헤딩 기준 역방향 상대 2D 벡터 (cx, cy) 적분
	float cx = 0.0f, cy = 0.0f, cth = 0.0f;
	for (uint16_t w = 0; w + 1 < W; w++) {
		uint16_t c = n_cur - 1 - w;
		float d = g_cross_log[c].dist_from_prev_m;
		float y = g_cross_log[c].yaw_angle;
		float mid = (cth - 0.5f * y) * (PI / 180.0f);
		cx -= d * cosf(mid);
		cy -= d * sinf(mid);
		cth = Wrap_Angle_Deg(cth - y);
	}

	int search_start = (int)(*idx) - 6;
	if (search_start < 1) search_start = 1;
	int search_end = (int)(*idx) + 28;
	if (search_end > (int)ref_log_count) search_end = (int)ref_log_count;

	int best_k = -1;
	float best_score = -1e9f;
	float second_score = -1e9f;

	for (int k = search_start; k < search_end; k++) {
		if (ref_log[k].type != cross) continue;
		if (fabsf(eff_odom - ref_log_cum_dist[k]) > 2.50f) continue;
		if (fabsf(Wrap_Angle_Deg(cur_yaw - ref_pose_th[k])) > 45.0f) continue;

		float score = 0.0f;
		int r = k;
		for (uint16_t w = 0; w < W; w++) {
			if (r < 0) {
				score -= 1.0f;
				continue;
			}
			uint16_t c = n_cur - 1 - w;
			CrossEvent_t ctype = g_cross_log[c].type;
			float cd = g_cross_log[c].dist_from_prev_m;
			float cyaw = g_cross_log[c].yaw_angle;

			if (ref_log[r].type == ctype) {
				float dd = fabsf(cd - ref_log[r].dist_from_prev_m);
				float dy = fabsf(Wrap_Angle_Deg(cyaw - ref_log[r].yaw_angle));
				float d_sim = 1.0f - Clamp_F32(dd / 0.35f, 0.0f, 1.0f);
				float y_sim = 1.0f - Clamp_F32(dy / 40.0f, 0.0f, 1.0f);
				score += 1.0f + d_sim + y_sim;
				r--;
			} else if (r >= 2 && ref_log[r - 1].type == ctype) {
				float md = ref_log[r].dist_from_prev_m + ref_log[r - 1].dist_from_prev_m;
				float my = Wrap_Angle_Deg(ref_log[r].yaw_angle + ref_log[r - 1].yaw_angle);
				float dd = fabsf(cd - md);
				float dy = fabsf(Wrap_Angle_Deg(cyaw - my));
				float d_sim = 1.0f - Clamp_F32(dd / 0.45f, 0.0f, 1.0f);
				float y_sim = 1.0f - Clamp_F32(dy / 45.0f, 0.0f, 1.0f);
				score += 0.6f + d_sim + y_sim;
				r -= 2;
			} else if (r >= 3 && ref_log[r - 2].type == ctype) {
				score += 0.3f;
				r -= 3;
			} else {
				score -= 1.0f;
				r--;
			}
		}

		int r_start = r + 1;
		if (r_start < 0) r_start = 0;
		float dx_w = ref_pose_x[r_start] - ref_pose_x[k];
		float dy_w = ref_pose_y[r_start] - ref_pose_y[k];
		float th_rad = -ref_pose_th[k] * (PI / 180.0f);
		float rx_loc = dx_w * cosf(th_rad) - dy_w * sinf(th_rad);
		float ry_loc = dx_w * sinf(th_rad) + dy_w * cosf(th_rad);
		float vec_err = hypotf(cx - rx_loc, cy - ry_loc);
		score += 2.0f * (1.0f - Clamp_F32(vec_err / 0.60f, 0.0f, 1.0f));
		score -= 0.20f * fabsf(eff_odom - ref_log_cum_dist[k]);

		if (score > best_score) {
			second_score = best_score;
			best_score = score;
			best_k = k;
		} else if (score > second_score) {
			second_score = score;
		}
	}

	if (best_k >= 0 && best_score >= (float)W * 1.45f &&
	    (second_score < -1e8f || (best_score - second_score) >= 0.70f)) {
		*idx = (uint16_t)best_k;
		*mismatch = 0;
		*odom_offset_m = ref_log_cum_dist[best_k] - g_total_odom_m;
	}
}

// 교차로(CROSS_CROSS) 우선 재동기화 + 시퀀스/벡터 기반 복구
static void Resync_Marker_Common(CrossEvent_t cross, uint16_t *idx, uint8_t *mismatch,
                                 float *odom_offset_m, float imu_grid_bias, float eff_odom) {
	if (cross == CROSS_CROSS && (*mismatch || *idx >= ref_log_count || ref_log[*idx].type != CROSS_CROSS)) {
		float cur_yaw = Wrap_Angle_Deg(imu_data.Yaw_Angle - imu_grid_bias);
		int best_c = -1;
		float min_c_err = 9999.0f;
		int c_start = (int)(*idx) - 4;
		if (c_start < 0) c_start = 0;
		for (uint16_t i = (uint16_t)c_start; i < ref_log_count; i++) {
			if (ref_log[i].type == CROSS_CROSS) {
				if (fabsf(Wrap_Angle_Deg(cur_yaw - ref_pose_th[i])) > 45.0f) continue;
				float d_err = fabsf(eff_odom - ref_log_cum_dist[i]);
				if (d_err < min_c_err) {
					min_c_err = d_err;
					best_c = (int)i;
				}
			}
		}
		if (best_c >= 0) {
			*idx = (uint16_t)best_c;
			*mismatch = 0;
		}
	}

	if (*mismatch) {
		Resync_Marker_Sequence(cross, idx, mismatch, odom_offset_m, imu_grid_bias, eff_odom);
	}
}

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
	float imu_grid_bias;
	float odom_offset_m;
} DriveSecondState_t;

__STATIC_INLINE void Build_Segment_Plan(void) {
	uint8_t curve_state = 0;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t is_straight = 0;

		if (cur == CROSS_CROSS) {
			curve_state = 0;
			is_straight = 1;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				curve_state = 0;
				is_straight = 1;
			} else {
				float curve_yaw = (i + 1 < ref_log_count) ? fabsf(ref_log[i + 1].yaw_angle) : 90.0f;
				if (curve_yaw < 25.0f) {
					curve_state = 0;
					is_straight = 0; // 직선 감속 구간
				} else {
					curve_state = 1;
					is_straight = 0;
				}
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				curve_state = 0;
				is_straight = 1;
			} else {
				float curve_yaw = (i + 1 < ref_log_count) ? fabsf(ref_log[i + 1].yaw_angle) : 90.0f;
				if (curve_yaw < 25.0f) {
					curve_state = 0;
					is_straight = 0; // 직선 감속 구간
				} else {
					curve_state = 2;
					is_straight = 0;
				}
			}
		}

		if (cur == CROSS_STOP) {
			is_straight = (i == 0) ? 1 : 0;
		}

		seg_plan[i].accel_ok = is_straight;
	}
}

__STATIC_INLINE uint8_t Drive_Second_Init_Sequence(void) {
	LCD7789_Invert(0);

	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0;
		}
	}

	if (!is_ref_log_copied) {
		ref_log_count = g_cross_log_count;

		if (ref_log_count == 0) {
			LCD_Printf(0, 0, "No Log Data");
			HAL_Delay(1000);
			return 0;
		}

		for (uint16_t i = 0; i < ref_log_count; i++) {
			ref_log[i] = g_cross_log[i];
		}

		Build_Segment_Plan();
		is_ref_log_copied = 1;
	} else {
		if (ref_log_count == 0) {
			LCD_Printf(0, 0, "No Log Data");
			HAL_Delay(1000);
			return 0;
		}
		Build_Segment_Plan();
	}

	Build_Ref_Log_Geometry();

	Drive_IMU_Auto_Calibrate();

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();
	g_total_odom_m = 0.0f;
	imu_straight_bias_track = 0;
	g_straight_cut_active = 0;
	g_suppress_marker_detect = 0;
	g_min_same_side_odom_m = 0.070f;
	g_straight_cut_yaw_ref = 0.0f;
	g_straight_cut_kappa_ref = 0.0f;
	Steer_Reset_State();
	Drive_Diag_Reset(2);

	g_cross_log_count = 0;
	Cross_Detect_Reset();

	IR_Sensor.is_lost_position = 0;
	IR_Sensor.data->mark_left = 0;
	IR_Sensor.data->mark_right = 0;
    IR_Sensor.data->target_pos = 0.0f;

	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0;

	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;

	arm_pid_init_f32(&steer_pid, 1);
	arm_pid_init_f32(&steer_pid, 0);

	Buzzer_Discount_Start();

	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();
	LSM6DS3_Reset_Yaw();
	g_target_base_mps = driveData.base_mps;
	return 1;
}

__STATIC_INLINE uint8_t Process_Marker_Event_Second(CrossEvent_t cross,
		DriveSecondState_t *state) {
	float eff_odom = g_total_odom_m + state->odom_offset_m;

	if (cross == CROSS_STOP) {
		uint8_t stop_res = Handle_Stop_Marker_Common(state->idx, eff_odom, &cross);
		if (stop_res == 1) return 1;
		if (stop_res == 2) return 0;
	}
	if (cross == CROSS_LEFT)
		g_total_L++;
	else if (cross == CROSS_RIGHT)
		g_total_R++;
	else if (cross == CROSS_CROSS)
		g_total_C++;

	Resync_Marker_Common(cross, &state->idx, &state->mismatch,
	                     &state->odom_offset_m, state->imu_grid_bias, eff_odom);

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	if (!state->mismatch && state->idx < ref_log_count
			&& seg_plan[state->idx].accel_ok
			&& state->idx + 1 < ref_log_count) {
		state->accel_active = 1;

		float total_straight_dist = 0.0f;
		for (uint16_t i = state->idx + 1; i < ref_log_count; i++) {
			total_straight_dist += ref_log[i].dist_from_prev_m;
			if (!seg_plan[i].accel_ok) {
				break;
			}
		}
		state->seg_len_predicted = total_straight_dist;

		uint8_t prev_was_curve = (state->idx >= 1 && !seg_plan[state->idx - 1].accel_ok);
		if (state->idx == 0 || !prev_was_curve) {
			g_target_base_mps = driveData.max_mps;
		} else {
			g_target_base_mps = driveData.base_mps;
		}

	} else {
		state->accel_active = 0;
		g_target_base_mps = driveData.base_mps;
		if (state->idx < ref_log_count && seg_plan[state->idx].accel_ok) {
			g_is_straight = 1;
		} else {
			g_is_straight = 0;
		}
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;

	if (!state->mismatch) {
		if (state->idx < ref_log_count) {
			state->odom_offset_m = ref_log_cum_dist[state->idx] - g_total_odom_m;
		}
		state->idx++;
	}

	return 0;
}

__STATIC_INLINE void Check_Distance_And_Brake(DriveSecondState_t *state) {
	if (state->accel_active) {
		if (state->braking_started) {
			g_target_base_mps = driveData.base_mps;
			return;
		}
		float_t traveled = g_odom_distance_m - state->marker_start_dist;
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = driveData.base_mps;

		float_t brake_dist = (driveData.brake_margin_m > 0.0f) ? driveData.brake_margin_m : BRAKE_MARGIN_M;
		if (v1 > v2) {
			const float_t tau_sys = (g_target_base_mps > v1 + 0.30f && v1 < 4.0f) ? 0.120f : 0.060f;
			float_t v_eff = v1;
			if (g_target_base_mps > v1) {
				v_eff = v1 + driveData.accel * 0.055f;
				if (v_eff > g_target_base_mps) v_eff = g_target_base_mps;
			}
			brake_dist += (v_eff * v_eff - v2 * v2) / (2.0f * driveData.decel);
			brake_dist += v_eff * tau_sys;
			/* linear tau_sys=0.085f replaces excess mid-speed penalty */
			/* high-speed aero drag already aids decel; no extra >4.5f penalty */
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = driveData.base_mps;
			state->braking_started = 1;
		} else {
			uint8_t prev_was_curve = (state->idx >= 2 && !seg_plan[state->idx - 2].accel_ok);
			float_t yaw_rate = fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset);
			float_t line_err = fabsf(Sensor_Get_Last_Position());
			uint8_t is_settled = (traveled >= ACCEL_START_MARGIN_M) &&
			                     (!prev_was_curve || (traveled >= 0.50f) || (yaw_rate < 40.0f && line_err < 0.20f));

			if (is_settled) {
				g_target_base_mps = driveData.max_mps;
				g_is_straight = 1;
			}
		}
	}
}

void Drive_Second(void) {
	g_is_straight = 0;
	if (!Drive_Second_Init_Sequence())
		return;
	uint32_t start_tick = HAL_GetTick();
	DriveSecondState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;

	if (ref_log_count > 0) {
		state.accel_active = 1;
		state.seg_len_predicted = (ref_log[0].type == CROSS_STOP && ref_log_count > 1)
				? (ref_log[0].dist_from_prev_m + ref_log[1].dist_from_prev_m)
				: ref_log[0].dist_from_prev_m;

		if (state.seg_len_predicted > 0.3f) {
			g_target_base_mps = driveData.max_mps;
		} else {
			g_target_base_mps = driveData.base_mps;
		}
	}

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Second(cross, &state))
				break;
		}
		Check_Distance_And_Brake(&state);
	}

	Drive_Diag_Finalize(IR_Sensor.is_lost_position);
	LCD7789_Invert(0);

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(2nd)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}

	Drive_Save_MapLog_Show(2, lap_time, IR_Sensor.is_lost_position);
	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}

// 2차 이후 주행 공통: target_pos 물리/제어 한계
#define TARGET_POS_LIMIT         1.45f

// ============================================================================
// 3회차 주행: 경로 스무딩 플래너 (tools/path_smooth.py 프로토타입의 온보드 버전)
//  1) 1차 마커 맵(ref_log) 구간 곡률 = 45도 스냅 요 변화 / 구간 거리
//     마커 이벤트는 곡률 변화점보다 약 40 mm 늦게 잡히므로 THIRD_MARKER_LAG_M 만큼 앞당긴다.
//  2) 횡오프셋 d(s) = L^2 * H*kappa(s). 경사하강 스무딩 y += a(x-y) + b(y- + y+ - 2y)
//     (a 0.005, b 0.4, 10 mm 간격)의 수렴해와 같은 닫힌 해. H = 전진 + 후진 1차 저역통과, L = 89 mm.
//     |d| <= THIRD_DMAX_M, L/R 마커 근처는 THIRD_MARKER_DMAX_M (마커 판독 범위 유지).
//  3) 오프셋 경로 곡률 kappa + kappa^2 d + d'' 로 횡가속 한계 속도, 가감속 전진/후진 패스.
//     횡가속 한계 = 90도 이상 곡선(맵 곡률)을 base_mps 로 도는 횡가속 * THIRD_ALAT_GAIN.
//  4) 주행 중: 마커로 거리 동기화, target_pos = d / 52 mm, 속도 지령 = 프로파일 (선행 거리 포함).
// 모든 계산 루프는 for 문 (시뮬레이터 while-yield 매크로 회피).
// ============================================================================
#define THIRD_PROF_MAX          8192     // 프로파일 최대 점 수 (10 mm 간격 81.9 m, 더 긴 코스는 간격 자동 확대)
#define THIRD_DS_M              0.010f   // 프로파일 간격 [m]
#define THIRD_SMOOTH_LEN_M      0.0894f  // 스무딩 길이 [m] = 0.01 * sqrt(0.4 / 0.005)
#define THIRD_DMAX_M            0.040f   // 횡오프셋 한계 [m]
#define THIRD_MARKER_DMAX_M     0.020f   // L/R 마커 근처 횡오프셋 한계 [m]
#define THIRD_MARKER_WIN_M      0.050f   // 마커 근처 판정 반폭 [m]
#define THIRD_MARKER_LAG_M      0.040f   // 마커 이벤트 - 곡률 변화점 거리 [m]
#define THIRD_MM_PER_UNIT       52.0f    // target_pos 1 단위당 횡이동 [mm] (직선 기준)
#define THIRD_ALAT_GAIN         1.00f    // 횡가속 한계 배율
#define THIRD_VMIN_RATIO        0.85f    // 최저 계획 속도 = base_mps * ratio
#define THIRD_DECEL_RATIO       0.85f    // 프로파일 감속도 = decel * ratio (제동 지연 여유)
#define THIRD_KAPPA_SMOOTH_M    0.050f   // 속도 계산용 경로 곡률 평활 길이 [m]
#define THIRD_LOOK_TAU_S        0.08f    // 속도 지령 선행 시간 [s]
#define THIRD_LOOK_MARGIN_M     0.06f    // 속도 지령 선행 거리 여유 [m]
#define THIRD_LOOK_V2_K         0.012f   // 고속 제동 여유 [m per (m/s)^2]: 긴 제동 중 휠 슬립으로 오도메트리가 덜 읽히는 만큼
#define THIRD_EDGE_ZERO_M       0.30f    // 출발 직후 / 맵 끝 직전 이 거리 안은 오프셋 0

// 3·4차 공용 계획 버퍼 (두 주행은 동시에 돌지 않는다): 3차는 앞 2 * THIRD_PROF_MAX, 4차는 전체를 쓴다
#define PLAN_POOL_FLOATS        (3U * THIRD_PROF_MAX)
static float plan_pool[PLAN_POOL_FLOATS];
static float *const third_off = plan_pool;                 // 횡오프셋 [m], 왼쪽 + (구축 중에는 곡률 버퍼)
static float *const third_v = plan_pool + THIRD_PROF_MAX;  // 계획 속도 [m/s] (구축 중에는 곡률 버퍼)
static uint16_t third_n = 0;
static float third_ds = THIRD_DS_M;

// 전진 + 후진 1차 저역통과 (영위상, DC 이득 1). 이산 연산자 1 + c(2 - z - 1/z) 의 역, c = (len/ds)^2
static void Third_TwoPass_LowPass(float *x, uint16_t n, float ds, float len) {
	if (n < 2 || len <= 0.0f || ds <= 0.0f) return;
	float c = len / ds;
	c = c * c;
	float q = 2.0f + 1.0f / c;
	float r = 0.5f * (q - sqrtf(q * q - 4.0f));
	float acc = x[0];
	for (uint16_t i = 0; i < n; i++) {
		acc = (1.0f - r) * x[i] + r * acc;
		x[i] = acc;
	}
	acc = x[n - 1];
	for (int i = (int)n - 1; i >= 0; i--) {
		acc = (1.0f - r) * x[i] + r * acc;
		x[i] = acc;
	}
}

static float Third_Profile_At(const float *p, float s) {
	if (third_n == 0) return 0.0f;
	float fi = s / third_ds;
	if (fi <= 0.0f) return p[0];
	if (fi >= (float)(third_n - 1)) return p[third_n - 1];
	uint16_t i = (uint16_t)fi;
	float u = fi - (float)i;
	return p[i] + u * (p[i + 1] - p[i]);
}

static void Third_Build_Profile(void) {
	const float total = ref_log_cum_dist[ref_log_count - 1];
	float ds = THIRD_DS_M;
	if (total / ds + 2.0f > (float)THIRD_PROF_MAX) {
		ds = total / (float)(THIRD_PROF_MAX - 2);
	}
	uint32_t n32 = (uint32_t)(total / ds) + 2U;
	if (n32 > THIRD_PROF_MAX) n32 = THIRD_PROF_MAX;
	const uint16_t n = (uint16_t)n32;
	third_ds = ds;
	third_n = n;

	// 1) 구간 곡률 (third_off 를 곡률 버퍼로 사용). 90도 이상 곡선의 평균 맵 곡률 = 기준 곡률
	float k_ref_sum = 0.0f;
	uint16_t k_ref_cnt = 0;
	for (uint16_t i = 0; i < n; i++) third_off[i] = 0.0f;
	for (uint16_t seg = 1; seg < ref_log_count; seg++) {
		float d = ref_log[seg].dist_from_prev_m;
		float dyaw = Snap_Zetin_Segment_Yaw(ref_log[seg].yaw_angle, d);
		if (dyaw == 0.0f) continue;
		float k = dyaw * (PI / 180.0f) / fmaxf(d, 0.005f);
		if (fabsf(dyaw) >= 89.0f) {
			k_ref_sum += fabsf(k);
			k_ref_cnt++;
		}
		int i0 = (int)ceilf((ref_log_cum_dist[seg - 1] - THIRD_MARKER_LAG_M) / ds);
		int i1 = (int)floorf((ref_log_cum_dist[seg] - THIRD_MARKER_LAG_M) / ds);
		if (i0 < 0) i0 = 0;
		if (i1 > (int)n - 1) i1 = (int)n - 1;
		for (int i = i0; i <= i1; i++) third_off[i] = k;
	}
	const float k_ref = (k_ref_cnt > 0) ? (k_ref_sum / (float)k_ref_cnt) : 4.0f;
	for (uint16_t i = 0; i < n; i++) third_v[i] = third_off[i];

	// 2) 횡오프셋 d = L^2 * H*kappa, 한계 클램프 (출발/끝 구간 0, 마커 근처 축소)
	Third_TwoPass_LowPass(third_off, n, ds, THIRD_SMOOTH_LEN_M);
	const float l2 = THIRD_SMOOTH_LEN_M * THIRD_SMOOTH_LEN_M;
	for (uint16_t i = 0; i < n; i++) {
		float s = (float)i * ds;
		float lim = (s < THIRD_EDGE_ZERO_M || s > total - THIRD_EDGE_ZERO_M) ? 0.0f : THIRD_DMAX_M;
		third_off[i] = Clamp_F32(l2 * third_off[i], -lim, lim);
	}
	for (uint16_t m = 0; m < ref_log_count; m++) {
		if (ref_log[m].type != CROSS_LEFT && ref_log[m].type != CROSS_RIGHT) continue;
		int i0 = (int)ceilf((ref_log_cum_dist[m] - THIRD_MARKER_LAG_M - THIRD_MARKER_WIN_M) / ds);
		int i1 = (int)floorf((ref_log_cum_dist[m] + THIRD_MARKER_WIN_M) / ds);
		if (i0 < 0) i0 = 0;
		if (i1 > (int)n - 1) i1 = (int)n - 1;
		for (int i = i0; i <= i1; i++) {
			third_off[i] = Clamp_F32(third_off[i], -THIRD_MARKER_DMAX_M, THIRD_MARKER_DMAX_M);
		}
	}

	// 3) 오프셋 경로 곡률 kappa_p = kappa + kappa^2 d + d'' (third_v 에 맵 곡률이 들어 있음)
	const float inv_ds2 = 1.0f / (ds * ds);
	for (uint16_t i = 1; i + 1 < n; i++) {
		float k = third_v[i];
		float dpp = (third_off[i - 1] - 2.0f * third_off[i] + third_off[i + 1]) * inv_ds2;
		third_v[i] = k + k * k * third_off[i] + dpp;
	}
	Third_TwoPass_LowPass(third_v, n, ds, THIRD_KAPPA_SMOOTH_M);

	// 4) 횡가속 한계 속도
	const float base = driveData.base_mps;
	const float vmax = driveData.max_mps;
	const float a_lat = base * base * k_ref * THIRD_ALAT_GAIN;
	const float vmin = base * THIRD_VMIN_RATIO;
	for (uint16_t i = 0; i < n; i++) {
		float ak = fabsf(third_v[i]);
		float v = (ak > 1e-3f) ? sqrtf(a_lat / ak) : vmax;
		third_v[i] = Clamp_F32(v, vmin, vmax);
	}

	// 5) 출발 / 맵 끝은 base_mps, 가속 전진 패스 + 감속 후진 패스
	third_v[0] = fminf(third_v[0], base);
	third_v[n - 1] = fminf(third_v[n - 1], base);
	const float acc = driveData.accel;
	const float dec = driveData.decel * THIRD_DECEL_RATIO;
	for (uint16_t i = 0; i + 1 < n; i++) {
		float lim = sqrtf(third_v[i] * third_v[i] + 2.0f * acc * ds);
		if (third_v[i + 1] > lim) third_v[i + 1] = lim;
	}
	for (int i = (int)n - 2; i >= 0; i--) {
		float lim = sqrtf(third_v[i + 1] * third_v[i + 1] + 2.0f * dec * ds);
		if (third_v[i] > lim) third_v[i] = lim;
	}
}

void Drive_Third(void) {
	g_is_straight = 0;
	if (!is_ref_log_copied && g_cross_log_count > 0) {
		ref_log_count = g_cross_log_count;
		for (uint16_t i = 0; i < ref_log_count; i++) {
			ref_log[i] = g_cross_log[i];
		}
	}
	if (ref_log_count < 2) {
		return;
	}
	if (!Drive_Init_Sequence()) {
		return;
	}
	is_ref_log_copied = 1;
	Build_Ref_Log_Geometry();
	Third_Build_Profile();

	uint32_t start_tick = HAL_GetTick();
	uint16_t idx = 0;
	float odom_offset = 0.0f;
	uint32_t last_ctrl_tick = HAL_GetTick();

	g_target_base_mps = driveData.base_mps;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		float eff_s = g_total_odom_m + odom_offset;

		if (cross == CROSS_STOP) {
			uint8_t sr = Handle_Stop_Marker_Common(idx, eff_s, &cross);
			if (sr == 1) {
				break;
			}
			if (sr == 2) {
				cross = CROSS_NONE;
			}
			if (g_total_STOP == 1 && idx == 0 && ref_log_count > 1) {
				idx = 1;
				odom_offset = ref_log_cum_dist[0] - g_total_odom_m;
				eff_s = g_total_odom_m + odom_offset;
			}
		}

		if (cross == CROSS_LEFT || cross == CROSS_RIGHT || cross == CROSS_CROSS) {
			if (cross == CROSS_LEFT) g_total_L++;
			else if (cross == CROSS_RIGHT) g_total_R++;
			else g_total_C++;

			for (uint16_t m = idx; m < ref_log_count && m <= idx + 3U; m++) {
				if (ref_log[m].type == cross && fabsf(ref_log_cum_dist[m] - eff_s) < 0.50f) {
					for (uint16_t sk = idx; sk < m; sk++) {
						CrossEvent_t st = ref_log[sk].type;
						if (st == CROSS_LEFT) g_total_L++;
						else if (st == CROSS_RIGHT) g_total_R++;
						else if (st == CROSS_CROSS) g_total_C++;
						Sensor_Log_Virtual_Marker(st, ref_log[sk].yaw_angle);
					}
					odom_offset = ref_log_cum_dist[m] - g_total_odom_m;
					eff_s = g_total_odom_m + odom_offset;
					idx = m + 1U;
					break;
				}
			}
		}

		uint32_t now_tick = HAL_GetTick();
		if (now_tick == last_ctrl_tick) {
			continue;
		}
		last_ctrl_tick = now_tick;

		float off = Third_Profile_At(third_off, eff_s);
		IR_Sensor.data->target_pos = Clamp_F32(off * 1000.0f / THIRD_MM_PER_UNIT, -TARGET_POS_LIMIT, TARGET_POS_LIMIT);

		float v_here = Third_Profile_At(third_v, eff_s);
		float vc = g_current_base_mps;
		float look = eff_s + vc * THIRD_LOOK_TAU_S + THIRD_LOOK_MARGIN_M + THIRD_LOOK_V2_K * vc * vc;
		float v_cmd = fminf(v_here, Third_Profile_At(third_v, look));
		g_target_base_mps = Clamp_F32(v_cmd, 0.5f, driveData.max_mps);
		g_is_straight = (v_here > driveData.base_mps + 0.8f && fabsf(off) < 0.002f) ? 1 : 0;
	}

	g_is_straight = 0;

	uint8_t lost_exit = IR_Sensor.is_lost_position;
	Drive_Diag_Finalize(lost_exit);
	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();
	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();
	IR_Sensor.data->target_pos = 0.0f;

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (lost_exit) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(3rd)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}
	Drive_Save_MapLog_Show(3, lap_time, lost_exit);
	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}

// ============================================================================
// 4회차 주행: 코리더 레이싱 라인 (라인 중심 ±150 mm 범위 안 최단시간 경로, 프로토타입 tools/race_line.py)
//  1) 1차 마커 맵 → 중심선 (구간 회전각 = 45도 스냅 요, 마커 이벤트 지연만큼 경계를 앞당김), 50 mm 격자
//  2) 센서바 횡오프셋 n(s): 경로 법선 방향 곡률^2 의 합 최소 (5대각 QP, 활성집합 + LDL^T).
//     범위: 곡선 안쪽 W_IN (80 mm, 라인 센서 76 mm 밖 = 블라인드), 바깥쪽과 직선 W (65 mm, 라인이 늘 보임).
//     곡선 시작점 앞뒤는 안쪽 50 mm 이하 (라인이 보이는 채로 곡률 시작을 관측해 진행 방향 위치를 맞춘다).
//     바깥 반복마다 경로 법선과 점 간격 가중치를 갱신한다. 출발 / 끝 1 m 는 n = 0 (STOP 마커 판독).
//     기하학적 숏컷(S자 직선 관통, 코너 안쪽 파고들기)은 이 최적화가 코리더 안에서 스스로 만든다.
//  3) 몸체 경로 = 센서바 경로의 트레일러(무슬립), 몸체 곡률 → 마찰원 속도 프로파일.
//  4) 주행: 처음부터 끝까지 IMU 방향 추종 모드(g_straight_cut_active). 센서바 위치 (s, n) 를
//     오도메트리 + IMU + 슬립각 모델로 추정하고, 라인이 센서 범위 안에 있으면 라인 센서로 n 을 보정한다.
//     방향 지령 = 라인 방향 + 계획 몸체 상대각 + 계획 슬립각 + 위치 오차 보정,
//     요레이트 피드포워드 = v * (계획 몸체 곡률 + 슬립각 변화율).
//     계획 오프셋이 작은 곳에서만 마커를 읽어 거리를 맞추고, 나머지 마커는 가상 마커로 기록한다.
// 모든 계산 루프는 for 문 (시뮬레이터 while-yield 매크로 회피).
// ============================================================================
#define FOURTH_DS_M             0.050f   // 계획 격자 간격 [m]
#define FOURTH_NMAX             1700U    // 최대 격자 점 수 (85 m)
#define FOURTH_W_IN_M           0.080f   // 곡선 안쪽 횡오프셋 한계 [m] (레이싱 라인 이득은 거의 안쪽에서 나온다).
                                         // 시뮬 16/16 완주, 최대 lat 139 mm (범위 150). 85 mm 는 1/8 초과, 100 mm 는 4/8 초과
#define FOURTH_W_OUT_M          0.065f   // 곡선 바깥쪽 한계 [m]
#define FOURTH_W_M              0.065f   // 직선 횡오프셋 한계 (센서바 중심) [m]: 라인이 항상 센서(±76 mm) 안
#define FOURTH_EDGE_M           1.00f    // 출발선 이후 / 맵 끝 직전 이 거리는 n = 0 (STOP 마커 판독, 라인 정렬)
#define FOURTH_LINE_END_M       0.80f    // 골 앞 이 거리와 출발선 이전은 라인 PD 추종 (STOP 좌우 마커를 함께 읽도록 중앙 정렬)
#define FOURTH_GOAL_ZONE_M      1.20f    // 골 STOP 앞 이 거리부터는 마커 판독을 항상 켠다
#define FOURTH_MARK_WIN_M       0.15f    // 다음 예상 마커 위치 앞 이 거리부터 판독 (그 밖은 허위 마커 차단)
#define FOURTH_LAG_M            0.040f   // 마커 이벤트 - 곡률 변화점 거리 [m]
#define FOURTH_SSCALE           1.000f   // 직선 길이 배율 (1차 오도메트리 직선 과다 측정 보정)
#define FOURTH_ONSET_PRE_M      0.10f    // 곡선 시작점 앞 이 거리부터
#define FOURTH_ONSET_POST_M     0.08f    // 시작점 뒤 이 거리까지는 안쪽 오프셋을 FOURTH_ONSET_N_M 이하로
#define FOURTH_ONSET_N_M        0.050f   //  (라인이 보이는 채로 곡률 시작을 관측해 진행 방향 위치를 맞춘 뒤 안쪽 블라인드로 들어간다)
#define FOURTH_BAR_X_M          0.13213f // 몸체 중심 - 센서바 거리 [m]
#define FOURTH_MM_PER_UNIT      52.0f    // 라인 위치 1 단위 [mm] (센서바 방향)
#define FOURTH_OUTER_IT         6        // QP 바깥 반복 (경로 법선 / 가중치 갱신)
#define FOURTH_AS_IT            50       // 활성집합 반복 한계
#define FOURTH_WT               0.02f    // 접선 정규화 가중치 (|D2|^2)
#define FOURTH_ALAT_GAIN        0.90f    // 횡가속 한계 배율 (3차 기준 base_mps 곡선 횡가속 대비)
#define FOURTH_VMIN_RATIO       0.85f    // 최저 계획 속도 = base_mps * ratio
#define FOURTH_DECEL_RATIO      0.85f    // 프로파일 감속도 = decel * ratio
#define FOURTH_KB_SMOOTH_M      0.050f   // 몸체 곡률 평활 길이 [m]
#define FOURTH_BETA_K           0.062f   // 슬립각 [rad] per 요레이트 [rad/s] (4차 주행 시뮬 측정: 속도와 무관하게 0.062, rms 2.2도)
#define FOURTH_BETA_MAX         0.60f    // 슬립각 상한 [rad] (약 34도)
#define FOURTH_HGAIN            2.0f     // IMU 방향 추종 이득 배율 (motor.c 직선 컷 모드 기본 16 rad/s 대비)
#define FOURTH_VIS_M            0.090f   // 라인이 예상과 달리 안 보이면 |n| 을 이만큼으로 밀어 제어가 라인 쪽으로 돌아오게 한다
#define FOURTH_HBIAS_GAIN       0.004f   // 직선에서 라인 각도 관측으로 헤딩 편차를 추정하는 이득 (1 ms 당)
#define FOURTH_HBIAS_MAX        0.06f    // 헤딩 편차 추정 한계 [rad] (약 3.4도)
#define FOURTH_MARK_CHI         0.15f    // 마커 판독은 라인과의 상대각이 이 안일 때만 [rad] (비스듬히 가로지를 때 허위 마커)
#define FOURTH_ANG_WIN_MS       15U      // 라인 각도 관측 창 [ms]
#define FOURTH_ANG_GAIN         0.05f    // 라인 각도 관측으로 s 를 맞추는 이득 (1 ms 당)
#define FOURTH_ANG_KMIN         1.5f     // 맵 곡률이 이 이상인 곡선에서만 [1/m]
#define FOURTH_LP0_M            0.10f    // 위치 보정 선행 거리 [m] = LP0 + LPT * v (짧을수록 추종 오차 작음, 시뮬 0.10 / 0.03 이 최적)
#define FOURTH_LPT_S            0.03f
#define FOURTH_MEAS_K           0.30f    // 라인 측정 보정 이득 (1 ms 당)
#define FOURTH_GATE_M           0.035f   // 라인 측정 게이트 [m] (넘으면 30 ms 일관될 때만 재획득: 마커 테이프 오인 차단)
#define FOURTH_GATE_MARK_M      0.020f   // 마커 테이프 근처 게이트 [m] (재획득 금지)
#define FOURTH_MARK_ZONE_M      0.12f    // 마커 이벤트 위치 앞 이 거리 ~ 뒤 0.05 m 는 마커 구간
#define FOURTH_MARK_N_M         0.020f   // |계획 n| 과 |추정 n| 이 이 안일 때만 마커 판독
#define FOURTH_SKIP_M           0.15f    // 마커 예상 위치를 이만큼 지나면 가상 마커로 기록
#define FOURTH_BLIND_ABORT_M    8.0f     // 라인 측정 없이 이 거리를 달리면 중단
#define FOURTH_LAMBDA           0.0f     // 중앙 당김 가중치 (n^2): 곡률 이득이 없는 긴 직선은 라인이 보이는 쪽으로
#define FOURTH_LOOK_TAU_S       0.08f    // 속도 지령 선행 시간 [s]
#define FOURTH_LOOK_MARGIN_M    0.06f    // 속도 지령 선행 거리 여유 [m]
#define FOURTH_LOOK_V2_K        0.012f   // 고속 제동 여유 [m per (m/s)^2]
#define FOURTH_BETA_TAU_S       0.04f    // 슬립각 1차 지연 [s] (시뮬 측정: 지연 없으면 rms 7도, 40 ms 지연이면 2.2도)
#define FOURTH_SEP_M            0.095f   // 이웃 라인(평행/역평행)과 센서바 최소 거리 [m]: 라인 센서 범위 76 mm + 라인 반폭
#define FOURTH_NEAR_M           0.25f    // 이 안에 이웃 라인이 있으면 게이트 밖 재획득 금지 [m]
#define FOURTH_XING_M           0.20f    // 교차 라인 근처 이 거리 안은 라인 측정 사용 안 함 [m]
#define FOURTH_R0_M             0.25f    // 규칙 곡선 반경 [m]: 맵 곡선은 이 반경으로 그린다 (1차 오도메트리는 곡선에서 ~8% 길게 읽힘)
#define FOURTH_KODO             1.00f    // 오도메트리 거리 배율
#define FOURTH_KB_CAP           4.8f     // 계획 몸체 곡률 상한 [1/m] (반경 0.21 m = 라인 위 곡선 주행과 같은 수준)
#define FOURTH_CAP_ROUNDS       12       // 몸체 곡률 상한 반복 수
#define FOURTH_SLIP_C           0.065f   // 휠 종방향 슬립 속도 [m/s] per 램프 가감속 [m/s^2] (시뮬: 가속 5.5 에서 +0.35, 감속에서 -0.27)
#define FOURTH_SLIP_TAU_S       0.10f    // 슬립 속도 1차 지연 [s]

// 계획 버퍼 배치 (plan_pool, 배열당 FOURTH_NMAX). 주행 중에는 D0/D1/D2 를 몸체 상대각 / 몸체 곡률 / 속도로 재사용
enum { F4_CX = 0, F4_CY, F4_TH, F4_K, F4_N, F4_D0, F4_D1, F4_D2, F4_B, F4_E0, F4_E1, F4_E2, F4_R, F4_IX, F4_NARR };
#define F4_CHI  F4_D0
#define F4_KB   F4_D1
#define F4_V    F4_D2
#define F4_ARR(k) (plan_pool + (uint32_t)(k) * FOURTH_NMAX)
typedef char fourth_pool_size_check[((uint32_t)F4_NARR * FOURTH_NMAX <= PLAN_POOL_FLOATS) ? 1 : -1];

static uint16_t fourth_m = 0;
static float f4_dyaw[CROSS_LOG_MAX];   // 구간 회전각 [rad] (스냅)
static float f4_cum[CROSS_LOG_MAX];    // 마커 이벤트 위치 [m] (실제 라인 길이: 곡선 = R0 * 회전각, 직선 = 1차 오도메트리)
static int16_t f4_lo_mm[FOURTH_NMAX];  // 격자점별 횡오프셋 하한 / 상한 [mm] (안쪽 / 바깥쪽 / 직선 폭, 이웃 라인 반영)
static int16_t f4_hi_mm[FOURTH_NMAX];
static uint8_t f4_flag[FOURTH_NMAX];   // bit0: 이웃 라인 가까움, bit1: 교차 구간
#define F4_FLAG_NEAR  0x01U
#define F4_FLAG_XING  0x02U
#define F4_FLAG_MARK  0x04U

// 튜닝 값: 실기 빌드는 상수, 시뮬 빌드는 환경변수 F4_* 로 덮어쓸 수 있다 (재빌드 없이 탐색)
#ifndef STM32H743xx
static float Fourth_Tune(const char *name, float def) {
	const char *e = getenv(name);
	return (e != NULL && e[0] != '\0') ? (float)atof(e) : def;
}
#else
#define Fourth_Tune(name, def) (def)
#endif

static float f4_w, f4_w_in, f4_w_out, f4_alat_gain, f4_beta_k, f4_beta_max, f4_lp0, f4_lpt, f4_meas_k, f4_vmin_ratio, f4_decel_ratio, f4_kodo;
static float f4_kb_cap, f4_slip_c, f4_hgain, f4_ang_gain, f4_lambda;
static int f4_pk_min = 1, f4_cnt_max = 4;
static float f4_lag = FOURTH_LAG_M, f4_sscale = FOURTH_SSCALE, f4_dbeta_ff = 1.0f, f4_onset_n = FOURTH_ONSET_N_M, f4_onset_post = FOURTH_ONSET_POST_M;

static void Fourth_Load_Tune(void) {
	// 폭 / 횡가속 배율 / 헤딩 이득은 driveData (메뉴, 시뮬 뷰어에서 조정). 0 이하면 기본 상수
	const float side = (driveData.fourth_side_mm > 0.0f) ? 0.001f * driveData.fourth_side_mm : FOURTH_W_M;
	const float in = (driveData.fourth_in_mm > 0.0f) ? 0.001f * driveData.fourth_in_mm : FOURTH_W_IN_M;
	f4_w = Fourth_Tune("F4_W", side);
	f4_w_in = Fourth_Tune("F4_WIN", in);
	f4_w_out = Fourth_Tune("F4_WOUT", side);
	f4_alat_gain = Fourth_Tune("F4_ALAT", (driveData.fourth_alat_gain > 0.0f) ? driveData.fourth_alat_gain : FOURTH_ALAT_GAIN);
	f4_beta_k = Fourth_Tune("F4_BETA_K", FOURTH_BETA_K);
	f4_beta_max = Fourth_Tune("F4_BETA_MAX", FOURTH_BETA_MAX);
	f4_lp0 = Fourth_Tune("F4_LP0", FOURTH_LP0_M);
	f4_lpt = Fourth_Tune("F4_LPT", FOURTH_LPT_S);
	f4_meas_k = Fourth_Tune("F4_MEAS_K", FOURTH_MEAS_K);
	f4_vmin_ratio = Fourth_Tune("F4_VMIN", FOURTH_VMIN_RATIO);
	f4_decel_ratio = Fourth_Tune("F4_DECEL", FOURTH_DECEL_RATIO);
	f4_kodo = Fourth_Tune("F4_KODO", FOURTH_KODO);
	f4_kb_cap = Fourth_Tune("F4_KBCAP", FOURTH_KB_CAP);
	f4_slip_c = Fourth_Tune("F4_SLIP_C", FOURTH_SLIP_C);
	f4_hgain = Fourth_Tune("F4_HGAIN", (driveData.fourth_hgain > 0.0f) ? driveData.fourth_hgain : FOURTH_HGAIN);
	f4_lambda = Fourth_Tune("F4_LAMBDA", FOURTH_LAMBDA);
	f4_pk_min = (int)Fourth_Tune("F4_PKMIN", 1.0f);
	f4_cnt_max = (int)Fourth_Tune("F4_CNTMAX", 4.0f);
	f4_lag = Fourth_Tune("F4_LAG", FOURTH_LAG_M);
	f4_sscale = Fourth_Tune("F4_SSCALE", FOURTH_SSCALE);
	f4_dbeta_ff = Fourth_Tune("F4_DBETA", 1.0f);
	f4_onset_n = Fourth_Tune("F4_ONSET", FOURTH_ONSET_N_M);
	f4_onset_post = Fourth_Tune("F4_ONSET_POST", FOURTH_ONSET_POST_M);
	f4_ang_gain = Fourth_Tune("F4_ANG", FOURTH_ANG_GAIN);
}

static float Fourth_At(const float *p, float s) {
	if (fourth_m == 0) return 0.0f;
	float fi = s / FOURTH_DS_M;
	if (fi <= 0.0f) return p[0];
	if (fi >= (float)(fourth_m - 1)) return p[fourth_m - 1];
	uint16_t i = (uint16_t)fi;
	float u = fi - (float)i;
	return p[i] + u * (p[i + 1] - p[i]);
}

// 각도 배열 보간 (2pi 래핑 없이 연속값으로 저장되어 있음)
#define Fourth_At_Angle Fourth_At

// 맵 헤딩 θ(s) [rad] 와 곡률: 구간 j (1..count-1) 는 [cum[j-1]-LAG, cum[j]-LAG) 에서 회전각을 선형 분배.
// s 는 단조 증가로만 질의한다 (*cur, *acc 커서).
static float Fourth_Map_Theta(float s, uint16_t *cur, float *acc, float *kappa) {
	for (; *cur < ref_log_count && s >= f4_cum[*cur] - f4_lag; (*cur)++) {
		*acc += f4_dyaw[*cur];
	}
	*kappa = 0.0f;
	if (*cur >= ref_log_count || *cur == 0) return *acc;
	float a = f4_cum[*cur - 1] - f4_lag;
	float b = f4_cum[*cur] - f4_lag;
	if (s <= a || b - a < 1e-4f) return *acc;
	*kappa = f4_dyaw[*cur] / (b - a);
	return *acc + f4_dyaw[*cur] * (s - a) / (b - a);
}

// 5대각 대칭 행렬 (대각 e0, 1차 e1, 2차 e2) LDL^T 분해 후 풀이, 제자리 연산 (r 에 해)
static void Fourth_Penta_Solve(float *e0, float *e1, float *e2, float *r, uint16_t nf) {
	for (uint16_t k = 0; k < nf; k++) {
		float l2k = 0.0f, l1k = 0.0f, dk = e0[k];
		if (k >= 2) l2k = e2[k - 2] / e0[k - 2];
		if (k >= 1) {
			float l1prev = (k >= 2) ? e1[k - 2] : 0.0f;
			l1k = (e1[k - 1] - ((k >= 2) ? l2k * e0[k - 2] * l1prev : 0.0f)) / e0[k - 1];
			dk -= l1k * l1k * e0[k - 1];
		}
		if (k >= 2) dk -= l2k * l2k * e0[k - 2];
		if (k >= 2) e2[k - 2] = l2k;
		if (k >= 1) e1[k - 1] = l1k;
		e0[k] = (dk > 1e-12f) ? dk : 1e-12f;
	}
	for (uint16_t k = 1; k < nf; k++) {
		r[k] -= e1[k - 1] * r[k - 1] + ((k >= 2) ? e2[k - 2] * r[k - 2] : 0.0f);
	}
	for (int k = (int)nf - 1; k >= 0; k--) {
		float v = r[k] / e0[k];
		if (k + 1 < (int)nf) v -= e1[k] * r[k + 1];
		if (k + 2 < (int)nf) v -= e2[k] * r[k + 2];
		r[k] = v;
	}
}

// y = H x (H: 대각 d0, 1차 d1, 2차 d2)
static float Fourth_Hx(const float *d0, const float *d1, const float *d2, const float *x, uint16_t m, uint16_t i) {
	float y = d0[i] * x[i];
	if (i + 1 < m) y += d1[i] * x[i + 1];
	if (i >= 1) y += d1[i - 1] * x[i - 1];
	if (i + 2 < m) y += d2[i] * x[i + 2];
	if (i >= 2) y += d2[i - 2] * x[i - 2];
	return y;
}

// min n'Hn + 2b'n,  lo <= n <= hi  (lo = hi = 0 은 고정). 원쌍대 활성집합.
static void Fourth_Box_QP(float *n, uint16_t m) {
	float *d0 = F4_ARR(F4_D0), *d1 = F4_ARR(F4_D1), *d2 = F4_ARR(F4_D2), *b = F4_ARR(F4_B);
	float *e0 = F4_ARR(F4_E0), *e1 = F4_ARR(F4_E1), *e2 = F4_ARR(F4_E2), *r = F4_ARR(F4_R);
	float *ix = F4_ARR(F4_IX);
	float *xa = F4_ARR(F4_K);   // 임시: 고정값 벡터 (곡률은 QP 뒤에 다시 채운다)
	for (int it = 0; it < FOURTH_AS_IT; it++) {
		// 활성집합 결정과 고정값
		uint16_t nf = 0;
		for (uint16_t i = 0; i < m; i++) {
			float g = Fourth_Hx(d0, d1, d2, n, m, i) + b[i];
			const float lo = 0.001f * (float)f4_lo_mm[i], hi = 0.001f * (float)f4_hi_mm[i];
			uint8_t pinned = (hi - lo < 1e-4f);
			if (pinned) {
				xa[i] = 0.5f * (lo + hi);
			} else if (n[i] <= lo + 1e-7f && g > 0.0f) {
				xa[i] = lo;
			} else if (n[i] >= hi - 1e-7f && g < 0.0f) {
				xa[i] = hi;
			} else {
				xa[i] = 0.0f;
				ix[nf++] = (float)i;
			}
		}
		// 자유 변수 부분계 구성: rhs = -(b + H xa)
		for (uint16_t q = 0; q < nf; q++) {
			uint16_t i = (uint16_t)ix[q];
			r[q] = -(b[i] + Fourth_Hx(d0, d1, d2, xa, m, i));
			e0[q] = d0[i];
			uint16_t g1 = (q + 1 < nf) ? (uint16_t)((uint16_t)ix[q + 1] - i) : 99;
			uint16_t g2 = (q + 2 < nf) ? (uint16_t)((uint16_t)ix[q + 2] - i) : 99;
			e1[q] = (g1 == 1) ? d1[i] : ((g1 == 2 && i + 2 < m) ? d2[i] : 0.0f);
			e2[q] = (g2 == 2 && i + 2 < m) ? d2[i] : 0.0f;
		}
		if (nf > 0) Fourth_Penta_Solve(e0, e1, e2, r, nf);
		// 새 해: 고정 + 자유, 범위 위반 확인 후 클램프
		uint8_t viol = 0;
		float dmax = 0.0f;
		uint16_t q = 0;
		for (uint16_t i = 0; i < m; i++) {
			float xn = xa[i];
			if (q < nf && (uint16_t)ix[q] == i) {
				xn = r[q++];
				const float lo = 0.001f * (float)f4_lo_mm[i], hi = 0.001f * (float)f4_hi_mm[i];
				if (xn < lo - 1e-6f || xn > hi + 1e-6f) viol = 1;
				xn = Clamp_F32(xn, lo, hi);
			}
			float dd = fabsf(xn - n[i]);
			if (dd > dmax) dmax = dd;
			n[i] = xn;
		}
		if (!viol && dmax < 1e-6f) break;
	}
}

// 경로 법선 방향 곡률^2 (+ 접선 정규화) 의 5대각 계수
static void Fourth_Build_Bands(const float *cx, const float *cy, const float *th, const float *n, uint16_t m) {
	float *d0 = F4_ARR(F4_D0), *d1 = F4_ARR(F4_D1), *d2 = F4_ARR(F4_D2), *b = F4_ARR(F4_B);
	for (uint16_t i = 0; i < m; i++) {
		d0[i] = 1e-9f + f4_lambda;
		d1[i] = 0.0f;
		d2[i] = 0.0f;
		b[i] = 0.0f;
	}
	const float ds = FOURTH_DS_M;
	for (uint16_t i = 1; i + 1 < m; i++) {
		float nx[3], ny[3], px[3], py[3];
		for (int a = 0; a < 3; a++) {
			uint16_t j = (uint16_t)(i - 1 + a);
			nx[a] = -sinf(th[j]);
			ny[a] = cosf(th[j]);
			px[a] = cx[j] + n[j] * nx[a];
			py[a] = cy[j] + n[j] * ny[a];
		}
		float tx = px[2] - px[0], ty = py[2] - py[0];
		float tl = sqrtf(tx * tx + ty * ty);
		if (tl < 1e-6f) continue;
		tx /= tl;
		ty /= tl;
		const float npx = -ty, npy = tx;   // 경로 법선
		float dl1 = hypotf(px[1] - px[0], py[1] - py[0]);
		float dl2 = hypotf(px[2] - px[1], py[2] - py[1]);
		float dloc = fmaxf(0.5f * (dl1 + dl2), 0.2f * ds);
		float wr = ds / dloc;
		float w = wr * wr * wr;   // 곡률^2 * 길이
		const float ax = cx[i - 1] - 2.0f * cx[i] + cx[i + 1];
		const float ay = cy[i - 1] - 2.0f * cy[i] + cy[i + 1];
		const float cf[3] = {1.0f, -2.0f, 1.0f};
		float e[3];
		for (int a = 0; a < 3; a++) e[a] = cf[a] * (nx[a] * npx + ny[a] * npy);
		const float a0 = ax * npx + ay * npy;
		const float wt = FOURTH_WT;
		for (int a = 0; a < 3; a++) {
			uint16_t j = (uint16_t)(i - 1 + a);
			d0[j] += w * e[a] * e[a] + wt * cf[a] * cf[a];
			b[j] += w * e[a] * a0 + wt * cf[a] * (ax * nx[a] + ay * ny[a]);
		}
		d1[i - 1] += w * e[0] * e[1] + wt * cf[0] * cf[1] * (nx[0] * nx[1] + ny[0] * ny[1]);
		d1[i] += w * e[1] * e[2] + wt * cf[1] * cf[2] * (nx[1] * nx[2] + ny[1] * ny[2]);
		d2[i - 1] += w * e[0] * e[2] + wt * cf[0] * cf[2] * (nx[0] * nx[2] + ny[0] * ny[2]);
	}
}

// 계획 수립. 성공 1
static uint8_t Fourth_Build_Plan(void) {
	if (ref_log_count < 3) return 0;

	// 1) 구간 회전각과 실제 길이: 곡선 = R0 * |회전각| (규칙), 직선 = 1차 오도메트리. 마커 위치 f4_cum
	f4_dyaw[0] = 0.0f;
	f4_cum[0] = ref_log[0].dist_from_prev_m;
	for (uint16_t j = 1; j < ref_log_count; j++) {
		float d = ref_log[j].dist_from_prev_m;
		float dy = Snap_Zetin_Segment_Yaw(ref_log[j].yaw_angle, d);
		f4_dyaw[j] = dy * (PI / 180.0f);
		float len = (dy != 0.0f) ? FOURTH_R0_M * fabsf(f4_dyaw[j]) : d * f4_sscale;
		f4_cum[j] = f4_cum[j - 1] + fmaxf(len, 0.005f);
	}
	const float k_ref = 1.0f / FOURTH_R0_M;
	const float total = f4_cum[ref_log_count - 1];
	uint32_t m32 = (uint32_t)(total / FOURTH_DS_M) + 2U;
	if (m32 > FOURTH_NMAX) return 0;
	const uint16_t m = (uint16_t)m32;
	fourth_m = m;
	float *cx = F4_ARR(F4_CX), *cy = F4_ARR(F4_CY), *th = F4_ARR(F4_TH), *kk = F4_ARR(F4_K), *n = F4_ARR(F4_N);

	// 2) 중심선 (5 mm 소구간 중점 적분)
	{
		uint16_t cur = 1;
		float acc = 0.0f, kap = 0.0f, x = 0.0f, y = 0.0f;
		const int sub = 10;
		const float h = FOURTH_DS_M / (float)sub;
		for (uint16_t i = 0; i < m; i++) {
			float s = (float)i * FOURTH_DS_M;
			if (i > 0) {
				for (int q = 0; q < sub; q++) {
					float sm = s - FOURTH_DS_M + ((float)q + 0.5f) * h;
					float t = Fourth_Map_Theta(sm, &cur, &acc, &kap);
					x += h * cosf(t);
					y += h * sinf(t);
				}
			}
			th[i] = Fourth_Map_Theta(s, &cur, &acc, &kap);
			cx[i] = x;
			cy[i] = y;
			n[i] = 0.0f;
		}
	}

	// 2b) 격자점별 오프셋 범위: 기본 ±W, 출발 / 끝 0, 이웃 라인(진행 방향 45도 이내) 쪽은 FOURTH_SEP_M 만큼 떨어지게.
	//     교차 라인(45도 초과)은 범위를 줄이지 않고 라인 측정 금지 구간으로 표시한다.
	{
		const float s_lo = f4_cum[0] + FOURTH_EDGE_M;
		const float s_hi = total - FOURTH_EDGE_M;
		const float W = Clamp_F32(f4_w, 0.0f, 0.145f);
		const float W_in = Clamp_F32(f4_w_in, 0.0f, 0.145f), W_out = Clamp_F32(f4_w_out, 0.0f, 0.145f);
		// 곡선 방향: 맵 헤딩 변화율을 앞뒤 0.3 m 로 평활 (진입 / 탈출 구간도 같은 쪽으로 본다)
		const int kwin = (int)(0.30f / FOURTH_DS_M);
		const int excl = (int)(0.6f / FOURTH_DS_M);
		for (uint16_t i = 0; i < m; i++) {
			float s = (float)i * FOURTH_DS_M;
			float lo = -W, hi = W;
			{
				int ja = (int)i - kwin, jb = (int)i + kwin;
				if (ja < 0) ja = 0;
				if (jb > (int)m - 1) jb = (int)m - 1;
				const float kav = (jb > ja) ? (th[jb] - th[ja]) / ((float)(jb - ja) * FOURTH_DS_M) : 0.0f;
				if (kav > 0.3f) {          // 좌회전: 안쪽 = +n
					hi = W_in;
					lo = -W_out;
				} else if (kav < -0.3f) {  // 우회전: 안쪽 = -n
					lo = -W_in;
					hi = W_out;
				}
			}
			uint8_t fl = 0;
			const float tx = cosf(th[i]), ty = sinf(th[i]);
			for (uint16_t j = 0; j < m; j++) {
				if ((int)j > (int)i - excl && (int)j < (int)i + excl) continue;
				const float dx = cx[j] - cx[i], dy = cy[j] - cy[i];
				const float along = dx * tx + dy * ty;
				if (fabsf(along) > 0.6f * FOURTH_DS_M) continue;
				const float lat = -dx * ty + dy * tx;
				if (fabsf(lat) > 0.35f) continue;
				const float par = fabsf(cosf(th[j] - th[i]));
				if (par > 0.70f) {
					if (lat > 0.0f) hi = fminf(hi, lat - FOURTH_SEP_M);
					else lo = fmaxf(lo, lat + FOURTH_SEP_M);
					if (fabsf(lat) < FOURTH_NEAR_M) fl |= F4_FLAG_NEAR;
				} else if (fabsf(lat) < FOURTH_XING_M) {
					fl |= F4_FLAG_XING;
				}
			}
			// 곡선 시작점 근처: 안쪽 오프셋 제한 (맵 곡선 시작이 실제와 몇 cm 어긋나면 안쪽 블라인드에서 그대로 횡오차가 된다)
			if (f4_onset_n < 0.14f) {
				for (uint16_t j = 1; j < ref_log_count; j++) {
					if (f4_dyaw[j] == 0.0f) continue;
					if (f4_dyaw[j - 1] != 0.0f && (f4_dyaw[j - 1] > 0.0f) == (f4_dyaw[j] > 0.0f)) continue;
					const float sa = f4_cum[j - 1] - f4_lag;
					if (s < sa - FOURTH_ONSET_PRE_M || s > sa + f4_onset_post) continue;
					if (f4_dyaw[j] > 0.0f) hi = fminf(hi, f4_onset_n);
					else lo = fmaxf(lo, -f4_onset_n);
				}
			}
			if (s < s_lo || s > s_hi) lo = hi = 0.0f;
			if (lo > hi) lo = hi = 0.5f * (lo + hi);
			f4_lo_mm[i] = (int16_t)lroundf(1000.0f * lo);
			f4_hi_mm[i] = (int16_t)lroundf(1000.0f * hi);
			f4_flag[i] = fl;
		}
		// 교차 표시는 앞뒤 FOURTH_XING_M 로 넓힌다 (교차 라인은 격자 표본 사이로 지나갈 수 있음)
		const int wide = (int)(FOURTH_XING_M / FOURTH_DS_M);
		for (uint16_t i = 0; i < m; i++) {
			if (!(f4_flag[i] & F4_FLAG_XING) || (f4_flag[i] & 0x80U)) continue;
			for (int j = (int)i - wide; j <= (int)i + wide; j++) {
				if (j >= 0 && j < (int)m && !(f4_flag[j] & F4_FLAG_XING)) f4_flag[j] |= (F4_FLAG_XING | 0x80U);
			}
		}
		for (uint16_t i = 0; i < m; i++) f4_flag[i] &= 0x7FU;
		// 마커 테이프 구간 (L / R / C): 라인 옆 50-100 mm 테이프가 라인 센서에 라인처럼 보일 수 있다
		for (uint16_t j = 1; j < ref_log_count; j++) {
			if (ref_log[j].type == CROSS_STOP) continue;
			int i0 = (int)ceilf((f4_cum[j] - FOURTH_MARK_ZONE_M) / FOURTH_DS_M);
			int i1 = (int)floorf((f4_cum[j] + 0.05f) / FOURTH_DS_M);
			for (int i = i0; i <= i1; i++) {
				if (i >= 0 && i < (int)m) f4_flag[i] |= F4_FLAG_MARK;
			}
			if (ref_log[j].type == CROSS_CROSS) {   // 교차선 자체도 측정 금지
				for (int i = i0; i <= i1; i++) {
					if (i >= 0 && i < (int)m) f4_flag[i] |= F4_FLAG_XING;
				}
			}
		}
	}

	// 3) 레이싱 라인 QP (바깥 반복: 경로 법선 / 점 간격 가중치 갱신)
	//    + 몸체 곡률 상한: 트레일러 몸체 곡률이 상한을 넘는 곳은 안쪽 범위를 좁혀 다시 푼다
	//      (긴 곡선 안쪽에 붙는 해는 몸체 반경이 0.1 m 대까지 줄어 요레이트 한계를 넘는다)
	float *chi = F4_ARR(F4_CHI), *kb = F4_ARR(F4_KB), *v = F4_ARR(F4_V), *dq = F4_ARR(F4_E0);
	float *qx = F4_ARR(F4_E1), *qy = F4_ARR(F4_E2), *thb = F4_ARR(F4_R);
	for (int round = 0; round <= FOURTH_CAP_ROUNDS; round++) {
	for (int it = 0; it < ((round == 0) ? FOURTH_OUTER_IT : 3); it++) {
		Fourth_Build_Bands(cx, cy, th, n, m);
		Fourth_Box_QP(n, m);
	}

	// 4) 맵 곡률 다시 채움 (QP 가 임시 버퍼로 썼음)
	{
		uint16_t cur = 1;
		float acc = 0.0f, kap = 0.0f;
		for (uint16_t i = 0; i < m; i++) {
			(void)Fourth_Map_Theta((float)i * FOURTH_DS_M, &cur, &acc, &kap);
			kk[i] = kap;
		}
	}

	// 5) 센서바 경로 → 트레일러 몸체 헤딩 / 위치 / 곡률
	{
		const float L = FOURTH_BAR_X_M;
		float b_th = th[0];
		for (uint16_t i = 0; i < m; i++) {
			float px = cx[i] - n[i] * sinf(th[i]);
			float py = cy[i] + n[i] * cosf(th[i]);
			if (i > 0) {
				float ppx = cx[i - 1] - n[i - 1] * sinf(th[i - 1]);
				float ppy = cy[i - 1] + n[i - 1] * cosf(th[i - 1]);
				float dx = px - ppx, dy = py - ppy;
				float dsp = sqrtf(dx * dx + dy * dy);
				float phi = atan2f(dy, dx);
				for (int q = 0; q < 4; q++) {
					b_th += 0.25f * dsp * sinf(Wrap_Angle_Deg((phi - b_th) * (180.0f / PI)) * (PI / 180.0f)) / L;
				}
			}
			thb[i] = b_th;
			qx[i] = px - L * cosf(b_th);
			qy[i] = py - L * sinf(b_th);
			chi[i] = b_th - th[i];
		}
		for (uint16_t i = 0; i < m; i++) {
			dq[i] = (i + 1 < m) ? hypotf(qx[i + 1] - qx[i], qy[i + 1] - qy[i]) : FOURTH_DS_M;
		}
		for (uint16_t i = 0; i < m; i++) {
			uint16_t i0 = (i > 0) ? (uint16_t)(i - 1) : i;
			uint16_t i1 = (i + 1 < m) ? (uint16_t)(i + 1) : i;
			float len = 0.0f;
			for (uint16_t j = i0; j < i1; j++) len += dq[j];
			kb[i] = (len > 1e-5f) ? (thb[i1] - thb[i0]) / len : 0.0f;
		}
		Third_TwoPass_LowPass(kb, m, FOURTH_DS_M, FOURTH_KB_SMOOTH_M);
	}

	// 5b) 몸체 곡률 상한 검사: 넘는 곳 앞뒤 ±0.15 m 의 안쪽 범위를 현재 해보다 5 mm 바깥으로
	{
		uint8_t any = 0;
		const int win = 3;
		for (uint16_t i = 0; i < m; i++) {
			if (fabsf(kb[i]) <= f4_kb_cap) continue;
			any = 1;
			for (int j = (int)i - win; j <= (int)i + win; j++) {
				if (j < 0 || j >= (int)m) continue;
				int lo = f4_lo_mm[j], hi = f4_hi_mm[j];
				int nm = (int)lroundf(1000.0f * n[j]);
				if (kb[i] > 0.0f) {   // 좌회전: 안쪽 = +n
					int nh = nm - 5;
					if (nh < hi) hi = nh;
					if (hi < lo) hi = lo;
				} else {
					int nl = nm + 5;
					if (nl > lo) lo = nl;
					if (lo > hi) lo = hi;
				}
				f4_lo_mm[j] = (int16_t)lo;
				f4_hi_mm[j] = (int16_t)hi;
			}
		}
		if (!any || round == FOURTH_CAP_ROUNDS) break;
	}
	}

	// 6) 속도: 3차와 같은 기준 (base_mps 로 맵 곡선을 도는 몸체 횡가속) * 배율, 마찰원 가감속 패스
	{
		const float base = driveData.base_mps;
		const float vmax = driveData.max_mps;
		const float r_bar = 1.0f / k_ref;
		const float r_body = sqrtf(fmaxf(r_bar * r_bar - FOURTH_BAR_X_M * FOURTH_BAR_X_M, 0.01f));
		const float a_lat = base * base / r_body * f4_alat_gain;
		const float vmin = base * f4_vmin_ratio;
		for (uint16_t i = 0; i < m; i++) {
			float ak = fabsf(kb[i]);
			float vl = (ak > 1e-3f) ? sqrtf(a_lat / ak) : vmax;
			v[i] = Clamp_F32(vl, vmin, vmax);
		}
		v[0] = fminf(v[0], base);
		v[m - 1] = fminf(v[m - 1], base);
		const float acc = driveData.accel;
		const float dec = driveData.decel * f4_decel_ratio;
		for (uint16_t i = 0; i + 1 < m; i++) {
			float al = v[i] * v[i] * fabsf(kb[i]);
			float av = (al < a_lat) ? sqrtf(a_lat * a_lat - al * al) : 0.0f;
			float a = fmaxf(fminf(acc, av), 0.3f * acc);
			float lim = sqrtf(v[i] * v[i] + 2.0f * a * dq[i]);
			if (v[i + 1] > lim) v[i + 1] = lim;
		}
		for (int i = (int)m - 2; i >= 0; i--) {
			float al = v[i + 1] * v[i + 1] * fabsf(kb[i + 1]);
			float av = (al < a_lat) ? sqrtf(a_lat * a_lat - al * al) : 0.0f;
			float a = fmaxf(fminf(dec, av), 0.3f * dec);
			float lim = sqrtf(v[i + 1] * v[i + 1] + 2.0f * a * dq[i]);
			if (v[i] > lim) v[i] = lim;
		}
	}

	// 7) 계획 슬립각 beta(s) = K * 요레이트(= v * 몸체 곡률), 시간 지연 TAU 를 거리 지연 v * TAU 로 (F4_B 버퍼)
	//    주행 중 헤딩 지령 = 라인 방향 + 몸체 상대각 + beta(s), 요레이트 피드포워드 = v * (몸체 곡률 + d beta / ds)
	{
		float *bp = F4_ARR(F4_B);
		float acc = 0.0f;
		for (uint16_t i = 0; i < m; i++) {
			float bs = Clamp_F32(f4_beta_k * v[i] * kb[i], -f4_beta_max, f4_beta_max);
			float lag = fmaxf(v[i], 0.3f) * FOURTH_BETA_TAU_S;
			float a = 1.0f - expf(-dq[i] / lag);
			acc += a * (bs - acc);
			bp[i] = acc;
		}
	}

#ifndef STM32H743xx
	{
		const char *path = getenv("F4_PLAN_CSV");
		if (path != NULL && path[0] != '\0') {
			FILE *fp = fopen(path, "w");
			if (fp != NULL) {
				fprintf(fp, "s,cx,cy,th,k,n,chi,kb,v,lo,hi,flag\n");
				for (uint16_t i = 0; i < m; i++) {
					fprintf(fp, "%.3f,%.4f,%.4f,%.5f,%.4f,%.5f,%.5f,%.4f,%.3f,%d,%d,%u\n", (double)((float)i * FOURTH_DS_M),
					        (double)cx[i], (double)cy[i], (double)th[i], (double)kk[i], (double)n[i],
					        (double)chi[i], (double)kb[i], (double)v[i], (int)f4_lo_mm[i], (int)f4_hi_mm[i],
					        (unsigned)f4_flag[i]);
				}
				fclose(fp);
			}
		}
	}
#endif
	return 1;
}

// 라인 센서 직접 측정: 성공 1, *pos [unit] (+ = 센서바가 라인 왼쪽). 교차선 / 두 줄 / 가장자리 센서는 거부
static uint8_t Fourth_Line_Measure(float *pos) {
	const uint16_t st = (uint16_t)(IR_Sensor.data->state & 0xFFFF);
	if (st == 0) return 0;
	int pk = 0;
	uint16_t pv = 0;
	for (int i = 0; i < LINE_N_SENSORS; i++) {
		uint16_t v = IR_Sensor.data->normalized[i];
		if (v > pv) {
			pv = v;
			pk = i;
		}
	}
	// 가장자리 센서에 피크가 있으면 무게중심이 안쪽으로 치우친다: f4_pk_min 보다 바깥 피크는 버린다
	if (pv < 120U || pk < f4_pk_min || pk > LINE_N_SENSORS - 1 - f4_pk_min) return 0;
	int cnt = 0;
	for (int i = 0; i < LINE_N_SENSORS; i++) {
		if (st & (1U << i)) {
			if (i < pk - 2 || i > pk + 2) return 0;   // 떨어진 두 번째 라인 (교차선, 이웃 트랙)
			cnt++;
		}
	}
	if (cnt > f4_cnt_max) return 0;   // 라인(20 mm)은 센서 2-3 개, 마커 테이프(50 mm)는 4-5 개
	int i0 = (pk >= 2) ? pk - 2 : 0;
	int i1 = (pk + 2 < LINE_N_SENSORS) ? pk + 2 : LINE_N_SENSORS - 1;
	uint16_t fl = 255;
	for (int i = i0; i <= i1; i++) {
		if (IR_Sensor.data->normalized[i] < fl) fl = IR_Sensor.data->normalized[i];
	}
	float ws = 0.0f, tw = 0.0f;
	for (int i = i0; i <= i1; i++) {
		float w = (float)(IR_Sensor.data->normalized[i] - fl);
		w *= w;
		ws += w * line_sensor_pos[i];
		tw += w;
	}
	if (tw <= 0.0f) return 0;
	*pos = ws / tw;
	return 1;
}

void Drive_Fourth(void) {
	g_is_straight = 0;
	if (!is_ref_log_copied && g_cross_log_count > 0) {
		ref_log_count = g_cross_log_count;
		for (uint16_t i = 0; i < ref_log_count; i++) {
			ref_log[i] = g_cross_log[i];
		}
	}
	if (ref_log_count < 3) {
		return;
	}
	Fourth_Load_Tune();
	Build_Ref_Log_Geometry();
	if (!Fourth_Build_Plan()) {
		LCD_Clear();
		LCD_Printf(0, 0, "4th: plan fail");
		HAL_Delay(1000);
		LCD_Clear();
		return;
	}
	if (!Drive_Init_Sequence()) {
		return;
	}
	is_ref_log_copied = 1;

	const float *pn = F4_ARR(F4_N), *pth = F4_ARR(F4_TH), *pk = F4_ARR(F4_K);
	const float *pcx = F4_ARR(F4_CX), *pcy = F4_ARR(F4_CY);
	const float *pchi = F4_ARR(F4_CHI), *pkb = F4_ARR(F4_KB), *pv = F4_ARR(F4_V), *pbeta = F4_ARR(F4_B);
	const float L = FOURTH_BAR_X_M;
	const float total = f4_cum[ref_log_count - 1];
	const float odo_scale = ref_log_cum_dist[ref_log_count - 1] / fmaxf(total, 1.0f);   // STOP 판정용 (1차 오도메트리 기준)

	uint32_t start_tick = HAL_GetTick();
	uint32_t last_tick = start_tick;
	uint16_t idx = 0;
	// 상태: 몸체 위치 (맵 평면), 연속 헤딩 (맵 프레임, 출발 시 라인 방향 = 0), 센서바 투영 s 와 횡오프셋 n
	float bx = -L, by = 0.0f, psi = 0.0f;
	float s = 0.0f, n = 0.0f, ths = 0.0f, ks = 0.0f;
	float odo_prev = g_total_odom_m;
	float yaw_prev = imu_data.Yaw_Angle;
	float blind_m = 0.0f, blind_max = 0.0f;
	float n_gate_prev = 0.0f;
	float alat_f = 0.0f;
	float vramp_prev = g_current_base_mps, slip_v = 0.0f;
	float n_last_seen = 0.0f;
	// 라인 각도 관측: 측정 n 의 변화와 추정 센서바 이동(보정 전)으로 라인 방향을 구해 맵 헤딩과 비교
	float rbx = 0.0f, rby = 0.0f;   // 센서바 누적 이동 (전파만, 측정 보정 제외)
	float hbias = 0.0f;              // 헤딩 편차 추정 [rad] (맵 헤딩 = IMU 헤딩 - hbias)
	uint32_t ang_t[32];
	float ang_n[32], ang_x[32], ang_y[32];
	uint8_t ang_head = 0, ang_cnt = 0;
	uint16_t gate_cnt = 0;
	uint8_t abort_blind = 0;

	g_straight_cut_line_blend = 0.0f;
	g_straight_cut_kappa_ref = 0.0f;
	g_straight_cut_yaw_ref = imu_data.Yaw_Angle;
	g_straight_cut_active = 1;
	g_target_base_mps = driveData.base_mps;
#ifndef STM32H743xx
	FILE *f4_trace = NULL;
	{
		const char *tp = getenv("F4_TRACE_CSV");
		if (tp != NULL && tp[0] != '\0') {
			f4_trace = fopen(tp, "w");
			if (f4_trace != NULL) fprintf(f4_trace, "ms,s,n,nref,chi,chi_ref,psi,thl,meas,nm,beta,v,kb,psi_cmd,idx,bx,by\n");
		}
	}
#endif

	while (1) {   // 폴링 루프는 while (시뮬레이터에서 펌웨어 스레드 양보 지점)
		CrossEvent_t cross = Cross_Detect_Update();
		float s_mark = -1.0f;   // 이번 주기에 확정된 마커 위치 (진행 방향 보정)

		if (cross == CROSS_STOP) {
			uint8_t sr = Handle_Stop_Marker_Common(idx, s * odo_scale, &cross);
			yaw_prev = imu_data.Yaw_Angle;   // STOP 처리의 요 리셋을 연속 헤딩에 넣지 않는다
			if (sr == 1) break;
			if (sr == 2) cross = CROSS_NONE;
			if (g_total_STOP == 1 && idx == 0 && ref_log_count > 1) {
				idx = 1;
				s_mark = f4_cum[0];
			}
		}

		if (cross == CROSS_LEFT || cross == CROSS_RIGHT || cross == CROSS_CROSS) {
			if (cross == CROSS_LEFT) g_total_L++;
			else if (cross == CROSS_RIGHT) g_total_R++;
			else g_total_C++;
			for (uint16_t mk = idx; mk < ref_log_count && mk <= idx + 3U; mk++) {
				if (ref_log[mk].type == cross && fabsf(f4_cum[mk] - s) < 0.30f) {
					for (uint16_t sk = idx; sk < mk; sk++) {
						CrossEvent_t st = ref_log[sk].type;
						if (st == CROSS_LEFT) g_total_L++;
						else if (st == CROSS_RIGHT) g_total_R++;
						else if (st == CROSS_CROSS) g_total_C++;
						Sensor_Log_Virtual_Marker(st, ref_log[sk].yaw_angle);
					}
					s_mark = f4_cum[mk];
					idx = mk + 1U;
					break;
				}
			}
		}
		if (s_mark >= 0.0f) {
			// 마커 이벤트 = 센서바가 마커 위치: 몸체를 라인 접선 방향으로 옮겨 s 를 맞춘다
			const float dsm = s_mark - s;
			bx += dsm * cosf(ths);
			by += dsm * sinf(ths);
			s = s_mark;
		}

		uint32_t now = HAL_GetTick();
		if (now == last_tick) continue;
		const float dt = (float)(now - last_tick) * 0.001f;
		last_tick = now;

		// 1) 추정 전파: 몸체 위치 += 오도메트리 * (헤딩 - 슬립각). 슬립은 횡가속을 약 50 ms 늦게 따라간다
		//    가감속 중에는 휠이 미끄러져 오도메트리가 더(가속) / 덜(감속) 읽힌다: 슬립 속도 = C * 램프 가감속 (지연)
		const float a_ramp = (g_current_base_mps - vramp_prev) / dt;
		vramp_prev = g_current_base_mps;
		slip_v += (f4_slip_c * a_ramp - slip_v) * fminf(1.0f, dt / FOURTH_SLIP_TAU_S);
		float d = (g_total_odom_m - odo_prev) * f4_kodo - slip_v * dt;
		if (d < 0.0f) d = 0.0f;
		odo_prev = g_total_odom_m;
		const float yaw = imu_data.Yaw_Angle;
		const float dpsi = Wrap_Angle_Deg(yaw - yaw_prev) * (PI / 180.0f);
		yaw_prev = yaw;
		const float vv = d / dt;
		const float wz = dpsi / dt;
		alat_f += (wz - alat_f) * fminf(1.0f, dt / FOURTH_BETA_TAU_S);   // (요레이트 저역통과)
		float beta = fminf(f4_beta_max, f4_beta_k * fabsf(alat_f));
		if (alat_f < 0.0f) beta = -beta;
		const float hm = psi - hbias + 0.5f * dpsi - beta;
		bx += d * cosf(hm);
		by += d * sinf(hm);
		rbx += d * cosf(hm) - L * dpsi * sinf(psi - hbias + 0.5f * dpsi);
		rby += d * sinf(hm) + L * dpsi * cosf(psi - hbias + 0.5f * dpsi);
		psi += dpsi;

		// 2) 센서바를 맵 라인에 투영 (뉴턴 3회): s, n, 라인 헤딩 ths, 곡률 ks
		const float psim = psi - hbias;   // 맵 프레임 헤딩
		const float barx = bx + L * cosf(psim), bary = by + L * sinf(psim);
		for (int it = 0; it < 3; it++) {
			// 맵 양 끝 밖은 끝점 접선으로 직선 연장 (끝점에 고정하면 뉴턴 반복이 발산한다)
			float fi = Clamp_F32(s / FOURTH_DS_M, 0.0f, (float)(fourth_m - 2));
			uint16_t i0 = (uint16_t)fi;
			float u = s - (float)i0 * FOURTH_DS_M;
			float lx, ly;
			// 연장 분기는 맵 양 끝 밖에서만. 격자 경계에서는 float 반올림으로 u 가 -1e-6 처럼 될 수 있어
			// u 부호만 보면 맵 시작점 기준으로 투영해 s 가 수 m 튄다
			if (i0 + 2 >= fourth_m && u > FOURTH_DS_M) {
				ths = pth[i0 + 1];
				ks = 0.0f;
				lx = pcx[i0 + 1] + (u - FOURTH_DS_M) * cosf(ths);
				ly = pcy[i0 + 1] + (u - FOURTH_DS_M) * sinf(ths);
			} else if (i0 == 0 && u < 0.0f) {
				ths = pth[0];
				ks = 0.0f;
				lx = pcx[0] + u * cosf(ths);
				ly = pcy[0] + u * sinf(ths);
			} else {
				u = Clamp_F32(u, 0.0f, FOURTH_DS_M);
				float t0 = pth[i0], t1 = pth[i0 + 1];
				ths = t0 + (t1 - t0) * (u / FOURTH_DS_M);
				ks = pk[i0];
				float tm = 0.5f * (t0 + ths);
				lx = pcx[i0] + u * cosf(tm);
				ly = pcy[i0] + u * sinf(tm);
			}
			float ex = barx - lx, ey = bary - ly;
			float along = ex * cosf(ths) + ey * sinf(ths);
			n = -ex * sinf(ths) + ey * cosf(ths);
			float den = 1.0f - ks * n;
			if (den < 0.3f) den = 0.3f;
			s += along / den;
		}
		const float chi = psim - ths;

		// 3) 라인 측정 보정: 라인 법선 방향으로 몸체 위치를 옮긴다 (곡선에서는 진행 방향 오차도 함께 줄어듦)
		float lp;
		uint8_t meas_ok = 0;
		float nm_log = 0.0f;
		const uint16_t gi = (uint16_t)Clamp_F32(s / FOURTH_DS_M + 0.5f, 0.0f, (float)(fourth_m - 1));
		const uint8_t zone = f4_flag[gi];
		if (!(zone & F4_FLAG_XING) && Fourth_Line_Measure(&lp)) {
			const float nm = lp * (FOURTH_MM_PER_UNIT * 0.001f) * cosf(chi);
			meas_ok = 1;
			nm_log = nm;
			const float gate = (zone & F4_FLAG_MARK) ? FOURTH_GATE_MARK_M : FOURTH_GATE_M;
			uint8_t accept = (fabsf(nm - n) < gate);
			if (!accept) {
				if (fabsf(nm - n_gate_prev) < 0.005f && !(zone & (F4_FLAG_NEAR | F4_FLAG_MARK))) {
					if (++gate_cnt > 30U) accept = 1;
				} else {
					gate_cnt = 0;
				}
				n_gate_prev = nm;
			}
			if (accept) {
				const float corr = f4_meas_k * (nm - n);
				n_last_seen = nm;
				bx -= corr * sinf(ths);
				by += corr * cosf(ths);
				n += corr;
				gate_cnt = 0;
				blind_m = 0.0f;
				// 라인 각도 관측 (곡선에서 진행 방향 위치 보정): 창이 끊기면 다시 모은다
				{
					const uint32_t tms = now - start_tick;
					if (ang_cnt > 0) {
						uint8_t last = (uint8_t)((ang_head + 31U) & 31U);
						if (tms - ang_t[last] > 3U) ang_cnt = 0;
					}
					ang_t[ang_head] = tms;
					ang_n[ang_head] = nm;
					ang_x[ang_head] = rbx;
					ang_y[ang_head] = rby;
					ang_head = (uint8_t)((ang_head + 1U) & 31U);
					if (ang_cnt < 32U) ang_cnt++;
					if (ang_cnt > FOURTH_ANG_WIN_MS && f4_ang_gain > 0.0f) {
						uint8_t old = (uint8_t)((ang_head + 31U - FOURTH_ANG_WIN_MS) & 31U);
						const float dxw = rbx - ang_x[old], dyw = rby - ang_y[old];
						const float dl = sqrtf(dxw * dxw + dyw * dyw);
						const float dn = nm - ang_n[old];
						if (dl > 0.02f && fabsf(dn) < 0.85f * dl) {
							const float chib = asinf(dn / dl);
							const float th_obs = atan2f(dyw, dxw) - chib;
							const float th_map = ths - ks * 0.5f * dl;
							const float dth = Wrap_Angle_Deg((th_obs - th_map) * (180.0f / PI)) * (PI / 180.0f);
							if (fabsf(ks) >= FOURTH_ANG_KMIN) {   // 곡선: 맵 헤딩 차이 = 곡률 * 진행 방향 위치 오차
								const float dsx = Clamp_F32(dth / ks, -0.08f, 0.08f) * f4_ang_gain;
								bx += dsx * cosf(ths);
								by += dsx * sinf(ths);
								s += dsx;
							} else if (fabsf(ks) < 0.3f && fabsf(alat_f) < 0.5f) {   // 직선: 헤딩 편차
								hbias = Clamp_F32(hbias + FOURTH_HBIAS_GAIN * Clamp_F32(dth, -0.1f, 0.1f), -FOURTH_HBIAS_MAX, FOURTH_HBIAS_MAX);
							}
						}
					}
				}
			} else {
				blind_m += d;
			}
		} else {
			blind_m += d;
			// 라인이 안 보임 = |n| > 센서 범위: 추정이 범위 안이면 바깥으로 민다 (교차 구간 제외)
			if (!(zone & F4_FLAG_XING) && (IR_Sensor.data->state & 0xFFFFU) == 0U && fabsf(n) < FOURTH_VIS_M && blind_m > 0.01f) {
				// 미는 방향: 추정이 0 에서 떨어져 있으면 현재 추정 쪽 (교차 구간 등 블라인드 중 라인을 넘어간 뒤 오래된 부호로 반대쪽에 밀지 않게)
				const float side = (fabsf(n) > 0.02f) ? ((n > 0.0f) ? 1.0f : -1.0f) : ((n_last_seen >= 0.0f) ? 1.0f : -1.0f);
				const float corr = 0.05f * (side * FOURTH_VIS_M - n);
				bx -= corr * sinf(ths);
				by += corr * cosf(ths);
				n += corr;
			}
		}
		if (blind_m > blind_max) blind_max = blind_m;
		if (blind_m > FOURTH_BLIND_ABORT_M) {
			abort_blind = 1;
			break;
		}

		// 4) 마커: 계획 / 추정 오프셋이 작을 때만 판독, 지나친 마커는 가상 기록
		const float nref = Fourth_At(pn, s);
		{
			const uint8_t near_line = (fabsf(nref) <= FOURTH_MARK_N_M && fabsf(n) <= FOURTH_MARK_N_M && fabsf(chi) <= FOURTH_MARK_CHI);
			// 교차로(C) 는 실제로 읽지 않는다: 비스듬히 지나면 한쪽 마커 센서가 교차선을 먼저 밟아 L/R 로 오인 (지나가면 가상 기록)
			const uint8_t expected = (idx < ref_log_count && s > f4_cum[idx] - FOURTH_MARK_WIN_M && ref_log[idx].type != CROSS_CROSS);
			const uint8_t goal_zone = (s > total - FOURTH_GOAL_ZONE_M) || (g_total_STOP == 0);
			g_suppress_marker_detect = (goal_zone || (near_line && expected)) ? 0 : 1;
		}
		for (; idx + 1U < ref_log_count && ref_log[idx].type != CROSS_STOP &&
		       s > f4_cum[idx] + FOURTH_SKIP_M; idx++) {
			CrossEvent_t st = ref_log[idx].type;
			if (st == CROSS_LEFT) g_total_L++;
			else if (st == CROSS_RIGHT) g_total_R++;
			else if (st == CROSS_CROSS) g_total_C++;
			Sensor_Log_Virtual_Marker(st, ref_log[idx].yaw_angle);
		}
		if (s > total + 1.0f) break;   // 골 STOP 을 놓침

		// 5) 방향 지령 (IMU 방향 추종 모드): 라인 방향 + 계획 몸체 상대각 + 슬립각 - 위치 오차 보정
		const float chi_ref = Fourth_At(pchi, s);
		const float kb = Fourth_At(pkb, s);
		const float beta_ff = Fourth_At(pbeta, s);
		const float dbeta_ds = (Fourth_At(pbeta, s + 0.025f) - Fourth_At(pbeta, s - 0.025f)) / 0.05f;
		const float e = n - nref;
		const float lpv = f4_lp0 + f4_lpt * fmaxf(vv, 0.0f);
		const float psi_cmd = ths + chi_ref + beta_ff - atanf(e / lpv);
		g_straight_cut_yaw_ref = Wrap_Angle_Deg(yaw + f4_hgain * (psi_cmd - psim) * (180.0f / PI));
		g_straight_cut_kappa_ref = kb + f4_dbeta_ff * dbeta_ds;
		{
			// 출발선 이전 / 골 직전: 라인 PD 추종 (target_pos = 0), 그 밖: IMU 방향 추종
			const uint8_t line_mode = (g_total_STOP == 0) || (s > total - FOURTH_LINE_END_M);
			if (line_mode && g_straight_cut_active) {
				Steer_Clear_Target_Shift();
				g_straight_cut_active = 0;
			} else if (!line_mode && !g_straight_cut_active) {
				g_straight_cut_active = 1;
			}
		}
		IR_Sensor.data->target_pos = g_straight_cut_active ? n * (1000.0f / FOURTH_MM_PER_UNIT) : 0.0f;   // 텔레메트리: 추정 n (라인 모드에서는 0)
#ifndef STM32H743xx
		if (f4_trace != NULL) {
			fprintf(f4_trace, "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.3f,%.3f,%.4f,%u,%.4f,%.4f\n",
			        (unsigned long)(now - start_tick), (double)s, (double)n, (double)nref, (double)chi, (double)chi_ref,
			        (double)psi, (double)ths, (int)meas_ok, (double)nm_log, (double)beta, (double)vv, (double)kb,
			        (double)psi_cmd, (unsigned)idx, (double)bx, (double)by);
		}
#endif

		// 6) 속도 지령 (선행 거리)
		const float vc = g_current_base_mps;
		const float look = s + vc * FOURTH_LOOK_TAU_S + FOURTH_LOOK_MARGIN_M + FOURTH_LOOK_V2_K * vc * vc;
		const float v_cmd = fminf(Fourth_At(pv, s), Fourth_At(pv, look));
		g_target_base_mps = Clamp_F32(v_cmd, 0.5f, driveData.max_mps);
	}

#ifndef STM32H743xx
	if (f4_trace != NULL) fclose(f4_trace);
#endif
	g_straight_cut_active = 0;
	g_suppress_marker_detect = 0;
	g_straight_cut_kappa_ref = 0.0f;
	IR_Sensor.data->target_pos = 0.0f;
	Steer_Clear_Target_Shift();
	g_is_straight = 0;

	uint8_t lost_exit = (uint8_t)(IR_Sensor.is_lost_position || abort_blind);
	Drive_Diag_Finalize(lost_exit);
	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();
	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (lost_exit) {
		LCD_Printf(0, 0, abort_blind ? "Line Lost(blind)" : "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(4th)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
		LCD_Printf(0, 5, "Blind:%.2fm", blind_max);
	}
	Drive_Save_MapLog_Show(4, lap_time, lost_exit);
	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}

// ============================================================================
// 5회차 주행 함수 모음 (2~4차 비의존 독립 자유 강화학습 주행)
// ============================================================================
static void Fifth_Sample_Map_2D(float s, float *out_x, float *out_y, float *out_th_deg, float *out_kappa) {
	if (ref_log_count < 2) {
		*out_x = 0.0f;
		*out_y = 0.0f;
		*out_th_deg = 0.0f;
		*out_kappa = 0.0f;
		return;
	}
	float total_s = ref_log_cum_dist[ref_log_count - 1];
	float sc = Clamp_F32(s, 0.0f, total_s);
	uint16_t seg = 1;
	for (; seg + 1 < ref_log_count && ref_log_cum_dist[seg] < sc; seg++) {
	}
	float s0 = ref_log_cum_dist[seg - 1];
	float d = fmaxf(ref_log[seg].dist_from_prev_m, 0.005f);
	float u = Clamp_F32((sc - s0) / d, 0.0f, 1.0f);
	float dyaw = Snap_Zetin_Segment_Yaw(ref_log[seg].yaw_angle, d);
	float th0 = ref_pose_th[seg - 1];
	float mid_rad = (th0 + 0.5f * u * dyaw) * (PI / 180.0f);
	float ds = u * d;
	*out_x = ref_pose_x[seg - 1] + ds * cosf(mid_rad);
	*out_y = ref_pose_y[seg - 1] + ds * sinf(mid_rad);
	*out_th_deg = Wrap_Angle_Deg(th0 + u * dyaw);
	*out_kappa = (dyaw * (PI / 180.0f)) / d;
}

void Drive_Fifth(void) {
	g_is_straight = 0;
	if (!is_ref_log_copied && g_cross_log_count > 0) {
		ref_log_count = g_cross_log_count;
		for (uint16_t i = 0; i < ref_log_count; i++) {
			ref_log[i] = g_cross_log[i];
		}
	}
	if (ref_log_count < 2) {
		return;
	}
	if (!Drive_Init_Sequence()) {
		return;
	}
	is_ref_log_copied = 1;
	Build_Ref_Log_Geometry();
	Fifth_RL_Load_Weights();
	g_drive_fifth_active = 1;

	uint32_t start_tick = HAL_GetTick();
	uint16_t idx = 0;
	float odom_offset = 0.0f;
	float imu_grid_bias = 0.0f;
	float prev_act0 = 0.0f;
	float total_s = fmaxf(ref_log_cum_dist[ref_log_count - 1], 1.0f);
	uint32_t last_ctrl_tick = HAL_GetTick();

	g_target_base_mps = driveData.base_mps;
	Ramp_Start();

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		float eff_s = g_total_odom_m + odom_offset;

		if (cross == CROSS_STOP) {
			uint8_t sr = Handle_Stop_Marker_Common(idx, eff_s, &cross);
			if (sr == 1) {
				break;
			}
			if (sr == 2) {
				cross = CROSS_NONE;
			}
			if (g_total_STOP == 1 && idx == 0 && ref_log_count > 1) {
				idx = 1;
				odom_offset = ref_log_cum_dist[0] - g_total_odom_m;
				eff_s = g_total_odom_m + odom_offset;
			}
		}

		if (cross == CROSS_LEFT || cross == CROSS_RIGHT || cross == CROSS_CROSS) {
			if (cross == CROSS_LEFT) g_total_L++;
			else if (cross == CROSS_RIGHT) g_total_R++;
			else g_total_C++;

			for (uint16_t m = idx; m < ref_log_count && m <= idx + 3U; m++) {
				if (ref_log[m].type == cross && fabsf(ref_log_cum_dist[m] - eff_s) < 0.50f) {
					for (uint16_t sk = idx; sk < m; sk++) {
						CrossEvent_t st = ref_log[sk].type;
						if (st == CROSS_LEFT) g_total_L++;
						else if (st == CROSS_RIGHT) g_total_R++;
						else if (st == CROSS_CROSS) g_total_C++;
						Sensor_Log_Virtual_Marker(st, ref_log[sk].yaw_angle);
					}
					odom_offset = ref_log_cum_dist[m] - g_total_odom_m;
					eff_s = g_total_odom_m + odom_offset;
					idx = m + 1U;
					break;
				}
			}
		}

		/* 오프라인 숏컷 중 지나친 마커 가상 합성 (STOP 제외, 물리 마커와 중복 방지 위해 +0.03m 여유) */
		while (g_straight_cut_active && idx + 1 < ref_log_count &&
		       ref_log[idx].type != CROSS_STOP && eff_s >= ref_log_cum_dist[idx] + 0.03f) {
			CrossEvent_t vt = ref_log[idx].type;
			if (vt == CROSS_LEFT) g_total_L++;
			else if (vt == CROSS_RIGHT) g_total_R++;
			else if (vt == CROSS_CROSS) g_total_C++;
			Sensor_Log_Virtual_Marker(vt, ref_log[idx].yaw_angle);
			idx++;
		}

		uint32_t now_tick = HAL_GetTick();
		if (now_tick == last_ctrl_tick) {
			continue;
		}
		last_ctrl_tick = now_tick;

		float rx = 0.0f, ry = 0.0f, th0 = 0.0f, k0 = 0.0f;
		float x04 = 0.0f, y04 = 0.0f, th04 = 0.0f, k04 = 0.0f;
		float x08 = 0.0f, y08 = 0.0f, th08 = 0.0f, k08 = 0.0f;
		float x14 = 0.0f, y14 = 0.0f, th14 = 0.0f, k14 = 0.0f;
		Fifth_Sample_Map_2D(eff_s, &rx, &ry, &th0, &k0);
		Fifth_Sample_Map_2D(eff_s + 0.40f, &x04, &y04, &th04, &k04);
		Fifth_Sample_Map_2D(eff_s + 0.85f, &x08, &y08, &th08, &k08);
		Fifth_Sample_Map_2D(eff_s + 1.45f, &x14, &y14, &th14, &k14);

		float max_k = fmaxf(fabsf(k0), fmaxf(fabsf(k04), fabsf(k08)));
		float v_curv = driveData.max_mps;
		for (int li = 0; li <= 18; li++) {
			float s_la = (float)li * 0.24f;
			float x_la = 0.0f, y_la = 0.0f, th_la = 0.0f, k_la = 0.0f;
			Fifth_Sample_Map_2D(eff_s + s_la, &x_la, &y_la, &th_la, &k_la);
			float ak = fabsf(k_la);
			if (ak > 0.45f) {
				float vc_la = fmaxf(driveData.base_mps, sqrtf(5.5f / ak));
				float v_brk = sqrtf(vc_la * vc_la + 2.0f * (0.72f * driveData.decel) * s_la);
				if (v_brk < v_curv) {
					v_curv = v_brk;
				}
			}
		}
		v_curv = Clamp_F32(v_curv, driveData.base_mps, driveData.max_mps);

		float obs[FIFTH_RL_OBS_DIM];
		obs[0] = Clamp_F32(g_current_base_mps / 7.0f, 0.0f, 1.0f);
		obs[1] = Clamp_F32(g_straight_cut_active ? 0.0f : Sensor_Get_Last_Position(), -1.0f, 1.0f);
		obs[2] = Clamp_F32(Wrap_Angle_Deg(th04 - th0) / 90.0f, -2.0f, 2.0f);
		obs[3] = Clamp_F32(Wrap_Angle_Deg(th08 - th0) / 90.0f, -2.0f, 2.0f);
		obs[4] = Clamp_F32(Wrap_Angle_Deg(th14 - th0) / 90.0f, -2.0f, 2.0f);
		obs[5] = Clamp_F32(max_k / 6.0f, 0.0f, 1.5f);
		obs[6] = Clamp_F32((total_s - eff_s) / 10.0f, 0.0f, 1.0f);
		obs[7] = Clamp_F32(eff_s / total_s, 0.0f, 1.0f);
		obs[8] = g_straight_cut_active ? 1.0f : 0.0f;
		obs[9] = prev_act0;

		float act[FIFTH_RL_ACT_DIM] = {0.0f, 0.0f};
		Fifth_RL_Infer(obs, act);
		prev_act0 = act[0];

		g_target_base_mps = Clamp_F32(v_curv * (1.0f + 0.45f * act[1]), 1.40f, driveData.max_mps);

		/* STOP 직전 0.6m는 라인 중앙 복귀; 2D 현(Chord)이 코리도(255mm) 및 회전각(115도) 이내인 최대 주시거리까지만 숏컷 허용 (연속 오메가 직진 관통 방지) */
		float L_want = (total_s - eff_s < 0.60f) ? 0.25f
		             : ((act[0] >= 0.0f) ? (0.40f + 1.30f * act[0]) : (0.40f + 0.15f * act[0]));
		float L_safe = 0.25f;
		float best_tx = rx, best_ty = ry, best_signed_dev = 0.0f, best_max_dev = 0.0f;
		for (int step = 1; step <= 10; step++) {
			float L_try = 0.25f + (L_want - 0.25f) * ((float)step / 10.0f);
			if (L_try <= 0.26f && step > 1) break;
			float tx = 0.0f, ty = 0.0f, tth = 0.0f, tk = 0.0f;
			Fifth_Sample_Map_2D(eff_s + L_try, &tx, &ty, &tth, &tk);
			float dx = tx - rx, dy = ty - ry;
			float clen = sqrtf(dx * dx + dy * dy);
			if (clen < 0.05f) break;
			float nx_c = -dy / clen, ny_c = dx / clen;
			float max_d = 0.0f, mid_d = 0.0f;
			uint8_t ok = 1;
			for (int k = 1; k <= 5; k++) {
				float sk = eff_s + L_try * ((float)k / 6.0f);
				float mx = 0.0f, my = 0.0f, mth = 0.0f, mk = 0.0f;
				Fifth_Sample_Map_2D(sk, &mx, &my, &mth, &mk);
				if (fabsf(Wrap_Angle_Deg(mth - th0)) > 115.0f) {
					ok = 0;
					break;
				}
				float sd = (mx - rx) * nx_c + (my - ry) * ny_c;
				if (k == 3) mid_d = sd;
				if (fabsf(sd) > max_d) max_d = fabsf(sd);
				if (max_d > 0.255f) {
					ok = 0;
					break;
				}
			}
			if (!ok) break;
			L_safe = L_try;
			best_tx = tx;
			best_ty = ty;
			best_signed_dev = -mid_d;
			best_max_dev = max_d;
		}

		if (best_max_dev > 0.055f && L_safe >= 0.42f && (total_s - eff_s >= 0.60f)) {
			float chord_th = atan2f(best_ty - ry, best_tx - rx) * (180.0f / PI);
			g_straight_cut_active = 1;
			g_straight_cut_yaw_ref = Wrap_Angle_Deg(chord_th + imu_grid_bias);
			g_straight_cut_kappa_ref = 0.0f;
			g_straight_cut_line_blend = (!IR_Sensor.is_lost_position && ((IR_Sensor.data->state & 0x3FFC) != 0)) ? 0.25f : 0.0f;
			IR_Sensor.data->target_pos = 0.0f;
		} else {
			g_straight_cut_active = 0;
			g_straight_cut_kappa_ref = 0.0f;
			g_straight_cut_line_blend = 0.0f;
			IR_Sensor.data->target_pos = Clamp_F32(best_signed_dev / 0.075f, -0.88f, 0.88f);
			if (fabsf(k0) < 0.3f && fabsf(Sensor_Get_Last_Position()) < 0.12f &&
			    fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset) < 8.0f) {
				imu_grid_bias = Wrap_Angle_Deg(imu_grid_bias + 0.02f * Wrap_Angle_Deg(imu_data.Yaw_Angle - (th0 + imu_grid_bias)));
			}
		}
	}

	g_straight_cut_active = 0;
	g_straight_cut_kappa_ref = 0.0f;
	g_straight_cut_line_blend = 0.0f;
	g_drive_fifth_active = 0;

	uint8_t lost_exit = IR_Sensor.is_lost_position;
	Drive_Diag_Finalize(lost_exit);
	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();
	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();
	IR_Sensor.data->target_pos = 0.0f;

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (lost_exit) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(5th)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}
	Drive_Save_MapLog_Show(5, lap_time, lost_exit);
	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}


// ============================================================================
// 진동 분석을 위한 테스트 주행 모드
// ============================================================================

static float vib_data_buf[VIB_SAMPLE_MAX];
static uint32_t vib_sample_count = 0;

void Drive_Vibration_Test(void) {
	if (!Drive_Init_Sequence())
		return;
	vib_sample_count = 0;
	uint32_t start_tick = HAL_GetTick();
	uint32_t last_sample_tick = start_tick;
	g_target_base_mps = driveData.base_mps;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross == CROSS_STOP) {
			break;
		}

		uint32_t current_tick = HAL_GetTick();
		if ((current_tick - last_sample_tick) >= 1) {
			last_sample_tick = current_tick;

			if (vib_sample_count < VIB_SAMPLE_MAX) {
				vib_data_buf[vib_sample_count++] = imu_data.Gyro_Z;
			} else {
				break;
			}
		}
	}

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	LCD_Clear();
	LCD_Printf(0, 0, "Saving Vib Data...");

	if (SDCard_Mount() == FR_OK) {
		FIL file;
		UINT bw;
		char filepath[64];

		sprintf(filepath, "/Drive_Data/vib_log_%lu.csv", HAL_GetTick() % 1000);

		if (f_open(&file, filepath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
			char line_buf[64];

			f_write(&file, "Index,Value\n", 12, &bw);

			for (uint32_t i = 0; i < vib_sample_count; i++) {
				int len = snprintf(line_buf, sizeof(line_buf), "%lu,%.4f\n", i,
						vib_data_buf[i]);
				f_write(&file, line_buf, len, &bw);
			}
			f_close(&file);
			LCD_Printf(0, 1, "Saved!");
		} else {
			LCD_Printf(0, 1, "SD Open Fail");
		}

		SDCard_Unmount();
	} else {
		LCD_Printf(0, 1, "SD Mount Fail");
	}

	LCD_Printf(0, 3, "Samples: %lu", vib_sample_count);
	LCD_Printf(0, 4, "Press Hold");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	LCD_Clear();
}


// ============================================================================
// Sim2Real 물성치 추출용 500Hz 다채널 주행 텔레메트리 저장 및 전용 주행 모드
// ============================================================================

FRESULT Drive_Save_SysID_To_SD(void) {
	FRESULT res = SD_SysID_Open(SYSID_DRIVE_PATH);
	if (res != FR_OK)
		return res;

	char line[512];
	int len = snprintf(line, sizeof(line),
			"# base_mps=%.2f\n# max_mps=%.2f\n# accel=%.2f\n# decel=%.2f\n"
			"# steer_kp=%.3f\n# steer_kd=%.1f\n# pos_atten_gain=%.2f\n"
			"# yaw_kp=%.3f\n# yaw_ki=%.3f\n"
			"# target_shift=%.2f\n# target_curve_gain=%.2f\n# brake_margin=%.2f\n"
			"# bulge_lat_acc=%.2f\n# corner45_max_mps=%.2f\n# omega_add_mps=%.2f\n# wave_add_mps=%.2f\n"
			"# line_lost=%u\n# max_beta_deg=%.2f\n# max_lat_mm=%.1f\n# weave_dps=%.1f\n# fail_v=%.2f\n",
			driveData.base_mps, driveData.max_mps, driveData.accel, driveData.decel,
			driveData.steer_gain_p, driveData.steer_gain_d, driveData.pos_atten_gain,
			driveData.yaw_gain_p, driveData.yaw_gain_i, 			driveData.target_shift_val, driveData.target_curve_gain, driveData.brake_margin_m,
			driveData.bulge_lat_acc, driveData.corner45_max_mps, driveData.omega_add_mps, driveData.wave_add_mps,
			(unsigned) g_drive_diag.line_lost, g_drive_diag.max_beta_deg,
			g_drive_diag.max_lat_mm, g_drive_diag.weave_dps, g_drive_diag.fail_v);
	SD_SysID_WriteChunk(line, (UINT) len);

	// 가속도계 캡처 정보: acc_x/acc_y 열은 센서 좌표계 원시값 [m/s^2] (바이어스 미차감).
	// acc_bias_* 는 출발 전 정지 캘리브레이션 평균 (중력 기울기 성분 포함). acc_capture=0 이면 열이 전부 0.
	const float_t acc_lsb_mps2 = ACCEL_SENSITIVITY_4G * 9.80665f;
	len = snprintf(line, sizeof(line),
			"# fan_en=%u\n# acc_capture=%u\n# acc_bias_x=%.4f\n# acc_bias_y=%.4f\n",
			(unsigned) driveData.fan_en, (unsigned) imu_accel_capture_en,
			imu_data.Accel_X_Offset * 9.80665f, imu_data.Accel_Y_Offset * 9.80665f);
	SD_SysID_WriteChunk(line, (UINT) len);

	uint16_t m_cnt = (g_cross_log_count < CROSS_LOG_MAX) ? g_cross_log_count : CROSS_LOG_MAX;
	for (uint16_t i = 0; i < m_cnt; i++) {
		CrossEvent_t type = g_cross_log[i].type;
		const char *t_str = (type == CROSS_LEFT) ? "L" :
							(type == CROSS_RIGHT) ? "R" :
							(type == CROSS_CROSS) ? "C" : "U";
		len = snprintf(line, sizeof(line), "# marker_%u=%s,%.3f,%.1f\n",
				(unsigned) i, t_str, g_cross_log[i].dist_from_prev_m, g_cross_log[i].yaw_angle);
		SD_SysID_WriteChunk(line, (UINT) len);
	}

	len = snprintf(line, sizeof(line),
			"t_ms,line_pos,target_mps,mps_L,mps_R,target_iq_L,target_iq_R,"
			"iq_L,iq_R,gyro_z_dps,odom_m,marker_cnt,vbus,acc_x,acc_y\n");
	SD_SysID_WriteChunk(line, (UINT) len);

	uint32_t total = g_sysid_drive_count;
	if (total > SYSID_DRIVE_SAMPLE_MAX)
		total = SYSID_DRIVE_SAMPLE_MAX;

	for (uint32_t i = 0; i < total; i++) {
		const SysID_DriveSample_t *sp = &g_sysid_drive_buf[i];
		len = snprintf(line, sizeof(line),
				"%u,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%u,%.1f,%.3f,%.3f\n",
				(unsigned) sp->t_ms,
				sp->line_pos_x10k * 0.0001f,
				sp->tgt_mps_mmps * 0.001f,
				sp->mps_L_mmps * 0.001f,
				sp->mps_R_mmps * 0.001f,
				sp->tgt_iq_L_mA * 0.001f,
				sp->tgt_iq_R_mA * 0.001f,
				sp->iq_L_mA * 0.001f,
				sp->iq_R_mA * 0.001f,
				sp->gyro_z_ddps * 0.1f,
				sp->odom_mm * 0.001f,
				(unsigned) sp->marker_cnt,
				sp->vbus_dV * 0.1f,
				sp->acc_x_raw * acc_lsb_mps2,
				sp->acc_y_raw * acc_lsb_mps2);
		SD_SysID_WriteChunk(line, (UINT) len);
	}

	return SD_SysID_Close();
}

void Drive_SysID_Test(void) {
	// 가속도계 X/Y 동시 캡처 (IMU I2C 전송 2→6바이트, 이 모드 전용). 정지 캘리브레이션에서 바이어스도 측정
	imu_accel_capture_en = 1;
	if (!Drive_Init_Sequence()) {
		imu_accel_capture_en = 0;
		return;
	}

	uint32_t start_tick = HAL_GetTick();
	uint8_t exit_reason_lost = 0;

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event(cross))
				break;
		}
	}

	if (IR_Sensor.is_lost_position) {
		exit_reason_lost = 1;
	}
	Drive_Diag_Finalize(exit_reason_lost);
	LCD7789_Invert(0);
	Drive_Stop_At_Distance(driveData.pit_in_distance_m);

	uint32_t end_tick = HAL_GetTick();

	HAL_Delay(500);
	Ramp_Stop();
	Buzzer_Discount_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	float lap_time = (end_tick - start_tick) / 1000.0f;

	LCD_Clear();
	LCD_Printf(0, 0, "Saving SysID...");
	FRESULT res = Drive_Save_SysID_To_SD();
	imu_accel_capture_en = 0;

	char grip_msg[20] = { 0 };
	Drive_AutoAdapt_Grip(grip_msg, sizeof(grip_msg));

	LCD_Clear();
	LCD_Printf(0, 0, exit_reason_lost ? "SysID(Lost)" : "SysID(End)");
	LCD_Printf(0, 1, "LatMax:%.2f m/s2", g_drive_diag.max_a_lat);
	LCD_Printf(0, 2, "B:%.1fd W:%.0f", g_drive_diag.max_beta_deg, g_drive_diag.weave_dps);
	LCD_Printf(0, 3, "%-16s", grip_msg);
	LCD_Printf(0, 4, "T:%.2fs", lap_time);
	LCD_Printf(0, 5, (res == FR_OK) ? "Saved SD OK!" : "Save Err:%d", res);

	Drive_Wait_Exit_Or_Heal(0);
	LCD_Clear();
}
