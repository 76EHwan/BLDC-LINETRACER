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

#define RAMP_TIM        (&htim14)
#define VIB_SAMPLE_MAX  10000

// @formatter:off
DriveParam_t driveData = {
    .base_mps = 1.80f,
    .max_mps = 7.00f,
    .accel = 8.0f,
    .decel = 8.0f,
    .steer_gain_p = 16.400f,
    .steer_gain_d = 800.0f,
    .pos_atten_gain = 0.00f,
    .pit_in_distance_m = 0.15f,
    .fan_en = 0,
	.target_shift_val = 0.90f,
	.yaw_gain_p = 0.25f,
	.yaw_gain_i = 0.0f,
	.target_curve_gain = 1.25f,
	.slalom_ratio = 0.03f,
	.brake_margin_m = 0.12f,
	.bulge_lat_acc = 6.50f,
	.corner45_max_mps = 3.30f,
	.omega_add_mps = 0.04f,
	.wave_add_mps = 0.25f,
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
uint32_t count_irq = 0;

void Ramp_TIM_IRQ_Handler(void) {
	Odom_Accumulate(RAMP_DT);

	float d_mps = g_target_base_mps - g_current_base_mps;
	if (d_mps > accel * RAMP_DT)
		g_current_base_mps += accel * RAMP_DT;
	else if (d_mps < -decel * RAMP_DT)
		g_current_base_mps -= decel * RAMP_DT;
	else
		g_current_base_mps = g_target_base_mps;
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

	// D게인 과포화(> 390) 및 P게인 과소(< 12.2) 감지 및 해소
	uint8_t kd_oversaturated = (driveData.steer_gain_d > 260.0f) ? 1 : 0;
	if (kd_oversaturated) {
		float_t anchor_kd = (g_heal_anchor.steer_gain_d < 220.0f) ? g_heal_anchor.steer_gain_d : 220.0f;
		base.steer_gain_d = Clamp_F32(0.65f * driveData.steer_gain_d + 0.35f * anchor_kd, 160.0f, 280.0f);
	}
	if (driveData.steer_gain_p < 8.13f) {
		float_t anchor_kp = (g_heal_anchor.steer_gain_p > 9.47f) ? g_heal_anchor.steer_gain_p : 9.47f;
		base.steer_gain_p = Clamp_F32(0.65f * driveData.steer_gain_p + 0.35f * anchor_kp, 8.53f, 16.0f);
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
		// [DeSat] D게인 과포화로 인한 고주파 노이즈 발진 -> KD 하향 및 기준점 복원
		role = "DeSat";
		float_t nom_kd = Clamp_F32(g_heal_anchor.steer_gain_d, 180.0f, 235.0f);
		healed.steer_gain_p = Clamp_F32(roundf((0.55f * driveData.steer_gain_p + 0.45f * g_heal_anchor.steer_gain_p + 0.23f * under_sev) * 1000.0f) / 1000.0f, 8.13f, 15.67f);
		healed.steer_gain_d = Clamp_F32(0.50f * driveData.steer_gain_d + 0.50f * nom_kd - 11.0f, 133.0f, 267.0f);
	} else if (speed_sev > 0.45f && g_drive_diag.drive_mode >= 2) {
		// [Entry] 직선 고속 가속 후 코너 진입 감속 부족 -> 진입 초과 속도에 비례해 브레이크 마진 증가 & 횡가속도 하향
		role = "Entry";
		float_t entry_sev = (speed_sev > slip_sev) ? speed_sev : slip_sev;
		healed.steer_gain_p = Clamp_F32(roundf((base.steer_gain_p + (g_drive_diag.was_oversteer ? -0.17f * slip_sev : +0.27f * under_sev)) * 1000.0f) / 1000.0f, 7.87f, 15.67f);
		float_t ideal_kd = Clamp_F32(healed.steer_gain_p * 28.68f, 147.0f, 293.0f);
		healed.steer_gain_d = Clamp_F32(0.60f * base.steer_gain_d + 0.40f * ideal_kd, 133.0f, 293.0f);
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
		// [PropUND] 언더스티어/조향 부족 이탈 -> 이탈 심각도에 비례해 KP 상향 & KD는 최적 댐핑비(22.4*KP)로 정렬
		role = "PropUND";
		float_t dkp = 0.43f * under_sev - 0.17f * osc_sev;
		healed.steer_gain_p = fmaxf(0.0f, roundf((base.steer_gain_p + dkp) * 1000.0f) / 1000.0f);
		float_t ideal_kd = fmaxf(0.0f, healed.steer_gain_p * 28.68f);
		healed.steer_gain_d = fmaxf(0.0f, 0.65f * base.steer_gain_d + 0.35f * ideal_kd + 6.7f * osc_sev);
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
	float k_est = 0.5f * (fabsf(dy_raw) / 45.0f) + 0.5f * (seg_d / arc45);
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

	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}

// ============================================================================
// 3회차 주행 함수 모음 (사용자 원본 4차 알고리즘: 마커 기반 선형 보간 영점 이동 & 직선/45도 가속)
// ============================================================================

// 45도(슬라롬) 구간 속도: base + (max - base) x driveData.slalom_ratio (기본 0.05)
__STATIC_INLINE float Slalom_Mps(void) {
	return driveData.base_mps + (driveData.max_mps - driveData.base_mps) * driveData.slalom_ratio;
}

// 구간 종류별 목표 속도: 0 깊은 커브 = base, 1 직선 = max, 2 45도 = 슬라롬 속도
__STATIC_INLINE float Segment_Mps(uint8_t accel_type) {
	if (accel_type == 1) return driveData.max_mps;
	if (accel_type == 2) return Slalom_Mps();
	return driveData.base_mps;
}

typedef struct {
	uint8_t accel_type; // 0: 감속(깊은 커브), 1: 직선 가속, 2: 45도(슬라롬) 가속
	uint8_t curve_dir;  // 0: 직선, 1: 좌(L), 2: 우(R)
} SegmentPlanThird_t;

static SegmentPlanThird_t seg_plan_3[CROSS_LOG_MAX];

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
	float target_mps; // 현재 가속 구간의 목표 최대 속도
	float end_mps;    // 구간 끝(다음 구간 진입)에서 맞춰야 할 속도
	float imu_grid_bias;
	float odom_offset_m;
} DriveThirdState_t;

// 3차 주행용 구간 계획 빌드 (마커 페어링 기반 직선/곡선 판별 및 45도 곡선 가속)
__STATIC_INLINE void Build_Segment_Plan_Third(void) {
	uint8_t curve_state = 0; // 0: 직선, 1: 좌(L) 곡선, 2: 우(R) 곡선

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t a_type = 0;

		if (cur == CROSS_STOP) {
			a_type = (i == 0) ? 1 : 0; // 첫 출발 마커(i==0) 직후는 시작 직선이므로 풀 가속(1), 도착 마커는 감속(0)
			curve_state = 0;
		} else if (cur == CROSS_CROSS) {
			// 크로스는 직선 상태 유지 및 풀 가속
			a_type = 1;
			curve_state = 0;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				// 좌회전 탈출 마커 -> 곡선 종료, 이후 구간은 무조건 직선 풀 가속
				curve_state = 0;
				a_type = 1;
			} else {
				// 좌회전 진입 마커 판정: 다음 마커까지의 회전각(yaw_angle) 검증
				float curve_yaw = (i + 1 < ref_log_count) ? fabsf(ref_log[i + 1].yaw_angle) : 90.0f;
				if (curve_yaw < 18.0f) {
					// 18도 미만이면 직선 상의 단독 감속 예고 마커! 곡선이 아니므로 인코스 시프트 끔
					curve_state = 0;
					a_type = 0; // 안전 감속
				} else if (curve_yaw < 65.0f) {
					curve_state = 1;
					a_type = 2; // 45도(슬라롬) 가속
				} else {
					curve_state = 1;
					a_type = 0; // 90도/180도 등 깊은 커브는 안전하게 감속
				}
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				// 우회전 탈출 마커 -> 곡선 종료, 이후 구간은 무조건 직선 풀 가속
				curve_state = 0;
				a_type = 1;
			} else {
				// 우회전 진입 마커 판정: 다음 마커까지의 회전각(yaw_angle) 검증
				float curve_yaw = (i + 1 < ref_log_count) ? fabsf(ref_log[i + 1].yaw_angle) : 90.0f;
				if (curve_yaw < 18.0f) {
					// 18도 미만이면 직선 상의 단독 감속 예고 마커! 곡선이 아니므로 인코스 시프트 끔
					curve_state = 0;
					a_type = 0; // 안전 감속
				} else if (curve_yaw < 65.0f) {
					curve_state = 2;
					a_type = 2; // 45도(슬라롬) 가속
				} else {
					curve_state = 2;
					a_type = 0; // 90도/180도 등 깊은 커브는 안전하게 감속
				}
			}
		}

		seg_plan_3[i].curve_dir = curve_state;
		seg_plan_3[i].accel_type = a_type;
	}

	// [사용자 요청: 45도 S자(슬라롬)와 독립 45도 코너(직-45-직) 분리]
	// 앞뒤가 모두 직선(0.30m 이상)인 독립 45도 코너는 Slalom 가속(type 2)을 끄고 안전 감속(type 0, base_mps) 적용
	for (uint16_t i = 0; i < ref_log_count; i++) {
		if (seg_plan_3[i].curve_dir != 0 && seg_plan_3[i].accel_type == 2) {
			float dist_from_prev_curve = 0.0f;
			uint8_t prev_has_curve = 0;
			for (int p = (int)i - 1; p >= 0; p--) {
				dist_from_prev_curve += ref_log[p + 1].dist_from_prev_m;
				if (ref_log[p].type == CROSS_CROSS) continue;
				if (seg_plan_3[p].curve_dir != 0) {
					prev_has_curve = 1;
					break;
				}
				if (dist_from_prev_curve > 0.30f) break;
			}

			uint16_t exit_idx = i + 1;
			while (exit_idx < ref_log_count && seg_plan_3[exit_idx].curve_dir != 0) {
				exit_idx++;
			}

			float dist_to_next_curve = 0.0f;
			uint8_t next_has_curve = 0;
			for (uint16_t n = exit_idx; n < ref_log_count; n++) {
				if (n + 1 < ref_log_count) dist_to_next_curve += ref_log[n + 1].dist_from_prev_m;
				if (ref_log[n].type == CROSS_CROSS) continue;
				if (seg_plan_3[n].curve_dir != 0) {
					next_has_curve = 1;
					break;
				}
				if (dist_to_next_curve > 0.30f) break;
			}

			if ((!prev_has_curve || dist_from_prev_curve >= 0.30f) &&
			    (!next_has_curve || dist_to_next_curve >= 0.30f)) {
				seg_plan_3[i].accel_type = 0;
			}
		}
	}
}

// 3차 주행 초기화
__STATIC_INLINE uint8_t Drive_Third_Init_Sequence(void) {
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
		if (ref_log_count == 0) return 0;
		for (uint16_t i = 0; i < ref_log_count; i++) ref_log[i] = g_cross_log[i];
		Build_Segment_Plan_Third();
		is_ref_log_copied = 1;
	} else {
		if (ref_log_count == 0) return 0;
		Build_Segment_Plan_Third();
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
	Drive_Diag_Reset(3);
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

// 3차 주행 마커 이벤트 및 가속 구간 계산 (사용자 원본 4차 로직)
__STATIC_INLINE uint8_t Process_Marker_Event_Third(CrossEvent_t cross, DriveThirdState_t *state) {
	float eff_odom = g_total_odom_m + state->odom_offset_m;

	if (cross == CROSS_STOP) {
		uint8_t stop_res = Handle_Stop_Marker_Common(state->idx, eff_odom, &cross);
		if (stop_res == 1) return 1;
		if (stop_res == 2) return 0;
	}
	if (cross == CROSS_LEFT) g_total_L++;
	else if (cross == CROSS_RIGHT) g_total_R++;
	else if (cross == CROSS_CROSS) g_total_C++;

	Resync_Marker_Common(cross, &state->idx, &state->mismatch,
	                     &state->odom_offset_m, state->imu_grid_bias, eff_odom);

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	// 가속 구간 계산: 같은 종류(직선끼리 / 45도끼리)만 묶는다.
	if (!state->mismatch && state->idx < ref_log_count && seg_plan_3[state->idx].accel_type > 0) {
		state->accel_active = 1;
		const uint8_t type = seg_plan_3[state->idx].accel_type;
		float total_dist = 0.0f;
		uint8_t next_type = 0;
		uint16_t next_seg_idx = state->idx;

		for (uint16_t i = state->idx; i < ref_log_count; i++) {
			if (seg_plan_3[i].accel_type != type) {
				next_type = seg_plan_3[i].accel_type;
				next_seg_idx = i;
				break;
			}
			if (i + 1 < ref_log_count) {
				total_dist += ref_log[i + 1].dist_from_prev_m;
			} else {
				break;
			}
		}
		state->seg_len_predicted = total_dist;

		state->target_mps = Segment_Mps(type);
		state->end_mps = Segment_Mps(next_type);

		// 곡률 피드포워드 속도 제한
		if (next_seg_idx + 1 < ref_log_count && (next_type == 0 || next_type == 2)) {
			float next_yaw_deg = fabsf(ref_log[next_seg_idx + 1].yaw_angle);
			float next_dist = ref_log[next_seg_idx + 1].dist_from_prev_m;
			if (next_yaw_deg > 10.0f && next_dist > 0.05f) {
				float dtheta_rad = next_yaw_deg * (PI / 180.0f);
				float r_eff = next_dist / dtheta_rad;
				
			}
		}

		if (state->end_mps < driveData.base_mps)
			state->end_mps = driveData.base_mps;

		if (state->end_mps > state->target_mps)
			state->end_mps = state->target_mps;

		uint8_t prev_was_curve = (state->idx >= 1 && seg_plan_3[state->idx - 1].curve_dir != 0);
		if (state->idx == 0 || !prev_was_curve) {
			g_target_base_mps = state->target_mps;
		} else {
			g_target_base_mps = driveData.base_mps; // 코너 탈출 직후 차체 직진 정렬 완료까지 base_mps 유지
		}

	} else {
		state->accel_active = 0;
		state->target_mps = driveData.base_mps;
		state->end_mps = driveData.base_mps;
		g_target_base_mps = driveData.base_mps;
		if (state->idx < ref_log_count && seg_plan_3[state->idx].curve_dir == 0) {
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

// 3차 주행 감속 및 가속 실행 제어 (사용자 원본 4차 로직)
__STATIC_INLINE void Check_Distance_And_Brake_Third(DriveThirdState_t *state) {
	if (state->accel_active) {
		if (state->braking_started) {
			g_target_base_mps = state->end_mps;
			return;
		}
		float_t traveled = g_odom_distance_m - state->marker_start_dist;
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = state->end_mps;

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
			g_target_base_mps = state->end_mps;
			state->braking_started = 1;
		} else {
			uint8_t prev_was_curve = (state->idx >= 2 && seg_plan_3[state->idx - 2].curve_dir != 0);
			float_t yaw_rate = fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset);
			float_t line_err = fabsf(Sensor_Get_Last_Position());
			uint8_t is_settled = (traveled >= ACCEL_START_MARGIN_M) &&
			                     (!prev_was_curve || (traveled >= 0.50f) || (yaw_rate < 40.0f && line_err < 0.20f));

			if (is_settled) {
				g_target_base_mps = state->target_mps;
				if (state->target_mps == driveData.max_mps) g_is_straight = 1;
			}
		}
	}
}

// 곡선 정점(apex)에서 target_curve_gain 적용 (코너 길이 비례 보정 + 직후 역방향/컷오프 합류 시 과도한 리바운드 차단)
#define TARGET_POS_LIMIT         1.45f
#define TARGET_APEX_LIMIT        1.25f
__STATIC_INLINE float Compute_Corner_Apex_Shift(float edge_shift, float total_dist, uint8_t restrict_exit) {
	if (edge_shift <= 0.0f || total_dist < 0.45f || driveData.target_curve_gain <= 1.0f || restrict_exit) {
		return edge_shift;
	}
	float len_frac = (total_dist - 0.45f) / 0.25f;
	if (len_frac < 0.0f) len_frac = 0.0f;
	else if (len_frac > 1.0f) len_frac = 1.0f;
	float eff_gain = 1.0f + (driveData.target_curve_gain - 1.0f) * len_frac;
	float apex = edge_shift * eff_gain;
	if (apex > TARGET_APEX_LIMIT) apex = TARGET_APEX_LIMIT;
	return apex;
}

// 3차 주행 전용 Target Position 예지 이동 로직
// - 일반 곡선: 코너 내부에서 인코스를 100% 꽉 물고 회전 (원심력 밀림 방지)
// - 직선 구간: 직전 코너 탈출점 -> 다음 코너 진입점까지 사행 주행(반대 코너) 또는 영점 유지(동일 코너)
// - 직선 없는 연속 반대 곡선(S자/오메가): Apex(50%)까지 In 유지 후 탈출 시 Out(=다음 코너 In)으로 사행 전환
__STATIC_INLINE float Get_Segment_Shift_Val(int seg_idx) {
	if (seg_idx < 0 || seg_idx >= (int)ref_log_count) return driveData.target_shift_val;
	float yaw = (seg_idx + 1 < (int)ref_log_count) ? fabsf(ref_log[seg_idx + 1].yaw_angle) : 90.0f;
	if (yaw < 25.0f) return 0.0f;
	if (yaw < 65.0f) {
		// 45도 슬라롬: 마커 간격이 짧고 고속이므로 60% 수준으로 완화하여 마커 센서(±80mm) 이탈 방지
		return driveData.target_shift_val * 0.60f;
	}
	return driveData.target_shift_val;
}

__STATIC_INLINE void Update_Target_Pos_Third(DriveThirdState_t *state) {
	if (state->mismatch || state->idx >= ref_log_count) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	int prev_idx = -1;
	for (int i = (int)state->idx - 1; i >= 0; i--) {
		if (ref_log[i].type != CROSS_CROSS) {
			prev_idx = i;
			break;
		}
	}

	int next_idx = (int)state->idx;
	for (int i = (int)state->idx; i < (int)ref_log_count; i++) {
		if (ref_log[i].type != CROSS_CROSS) {
			next_idx = i;
			break;
		}
	}

	float total_dist = 0.0f;
	for (int i = prev_idx + 1; i <= next_idx; i++) {
		total_dist += ref_log[i].dist_from_prev_m;
	}

	float traveled = g_odom_distance_m - state->marker_start_dist;
	for (int i = (int)state->idx - 1; i > prev_idx; i--) {
		traveled += ref_log[i].dist_from_prev_m;
	}

	float ratio = 0.0f;
	if (total_dist > 0.001f) {
		ratio = traveled / total_dist;
		if (ratio > 1.0f) ratio = 1.0f;
		if (ratio < 0.0f) ratio = 0.0f;
	}

	// 1. 현재 구간(prev_idx -> next_idx) 및 다음 구간(next_idx -> ...)의 곡선 방향
	uint8_t cur_curve_dir = (prev_idx >= 0) ? seg_plan_3[prev_idx].curve_dir : 0;
	uint8_t next_curve_dir = (next_idx < (int)ref_log_count) ? seg_plan_3[next_idx].curve_dir : 0;

	if (cur_curve_dir != 0) {
		// ====================================================================
		// [곡선 구간]
		// ====================================================================
		float shift_cur = Get_Segment_Shift_Val(prev_idx);
		float cur_in_target = (cur_curve_dir == 1) ? shift_cur : -shift_cur;

		// 중간에 직선 없이 바로 반대 방향 곡선으로 이어지는 연속 복합 코너(S자/오메가 루프)
		if (next_curve_dir != 0 && (cur_curve_dir != next_curve_dir)) {
			float shift_next = Get_Segment_Shift_Val(next_idx);
			float next_in_target = (next_curve_dir == 1) ? shift_next : -shift_next;

			// Apex(50%)까지는 현재 코너 In을 깊게 유지하여 원심력 이탈 방지
			// Apex 이후(50%~100%)에는 Out(=다음 코너 In)으로 부드럽게 사행 전환
			const float apex_ratio = 0.50f;
			if (ratio < apex_ratio) {
				IR_Sensor.data->target_pos = cur_in_target;
			} else {
				float u = (ratio - apex_ratio) / (1.0f - apex_ratio);
				IR_Sensor.data->target_pos = cur_in_target + (next_in_target - cur_in_target) * u;
			}
		} else {
			// 일반 단독 곡선(다음이 직선이거나 동일 방향):
			// 마커 부근은 edge_shift 유지, 코너 내부(15%~85%)에서 target_curve_gain으로 apex 심화
			float apex_cur = Compute_Corner_Apex_Shift(shift_cur, total_dist, 0);
			float sign_cur = (cur_curve_dir == 1) ? 1.0f : -1.0f;
			float marker_safe = (shift_cur > 0.50f) ? 0.50f : shift_cur;
			if (ratio < 0.15f) {
				IR_Sensor.data->target_pos = sign_cur * (marker_safe + (shift_cur - marker_safe) * (ratio / 0.15f));
			} else if (ratio > 0.85f) {
				IR_Sensor.data->target_pos = sign_cur * (marker_safe + (shift_cur - marker_safe) * ((1.0f - ratio) / 0.15f));
			} else if (apex_cur > shift_cur) {
				float um = (ratio - 0.15f) / 0.70f;
				IR_Sensor.data->target_pos = sign_cur * (shift_cur + (apex_cur - shift_cur) * sinf(um * PI));
			} else {
				IR_Sensor.data->target_pos = cur_in_target;
			}
		}
	} else {
		// ====================================================================
		// [직선 구간]
		// ====================================================================
		// 직전 코너 탈출점(start_target) -> 다음 코너 진입점(end_target)
		float start_target = 0.0f;
		if (prev_idx > 0) {
			int p = prev_idx - 1;
			while (p >= 0 && ref_log[p].type == CROSS_CROSS) p--;
			if (p >= 0) {
				float shift_p = Get_Segment_Shift_Val(p);
				if (seg_plan_3[p].curve_dir == 1) start_target = shift_p;
				else if (seg_plan_3[p].curve_dir == 2) start_target = -shift_p;
			}
		}

		float end_target = 0.0f;
		if (next_idx < (int)ref_log_count) {
			float shift_n = Get_Segment_Shift_Val(next_idx);
			if (next_curve_dir == 1) end_target = shift_n;
			else if (next_curve_dir == 2) end_target = -shift_n;
		}

		if (start_target > 0.50f) start_target = 0.50f;
		else if (start_target < -0.50f) start_target = -0.50f;
		if (end_target > 0.50f) end_target = 0.50f;
		else if (end_target < -0.50f) end_target = -0.50f;

		if (total_dist > 0.001f) {
			// 마커가 반대면 대각선 사행 주행, 마커가 같으면 영점이동 상태 균일 유지
			IR_Sensor.data->target_pos = start_target + (end_target - start_target) * ratio;
		} else {
			IR_Sensor.data->target_pos = start_target;
		}
	}
}

void Drive_Third(void) {
	g_is_straight = 0;
	if (!Drive_Third_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveThirdState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;
	state.end_mps = driveData.base_mps;

	if (ref_log_count > 0) {
		state.accel_active = 1;
		state.seg_len_predicted = (ref_log[0].type == CROSS_STOP && ref_log_count > 1)
				? (ref_log[0].dist_from_prev_m + ref_log[1].dist_from_prev_m)
				: ref_log[0].dist_from_prev_m;
		if (state.seg_len_predicted > 0.3f) {
			state.target_mps = driveData.max_mps;
			g_target_base_mps = state.target_mps;
		} else {
			state.target_mps = driveData.base_mps;
			g_target_base_mps = state.target_mps;
		}
	}

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Third(cross, &state))
				break;
		}

		Check_Distance_And_Brake_Third(&state);
		Update_Target_Pos_Third(&state);
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

	IR_Sensor.data->target_pos = 0.0f;

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(3rd)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}

	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}

// ============================================================================
// 4회차 주행 함수 모음 (한국 ZETIN FULL 16.8V 전용: IN-편향 가상 코리도 + 직선화 4th)
// ============================================================================

#define DRIVE4_CORRIDOR_CUT_HALF_M   0.105f // 가상 코리도 반폭 105mm (순수 직선 컷 시 150mm 경계 내 45mm 여유 확보)
#define DRIVE4_IN_MARKER_SHIFT_MAX   1.450f // target_shift_val 락 해제 (물리/제어 한계 TARGET_POS_LIMIT=1.45f만 적용)
#define DRIVE4_IN_SAME45_EDGE_MAX    0.240f // 동일방향 45-직-45 마커 구간 시프트 상한 (M_L/M_R [50,100] 안전 보장)
#define DRIVE4_IN_SAME45_BRIDGE_MID  0.560f // 동일방향 45-직-45 중간부 인코스 브릿징 (IR +-80mm 폐루프 유지)

typedef struct {
	uint8_t accel_type;          // 0: 감속(깊은 커브), 1: 직선 가속, 2: 45도(슬라롬) 가속, 3: 45도 연속 관통, 4: 코리도 직선/완만 컷, 5: 동일방향 45-직-45 IN 브릿지
	uint8_t curve_dir;           // 0: 직선, 1: 좌(L), 2: 우(R)
	uint8_t bridge_dir;          // accel_type==5 구간의 회전 방향 (1: 좌, 2: 우)
	uint8_t is_straight_cut;     // 1이면 가상 코리도 컷 적용 구간
	uint16_t cut_start_idx;      // 컷 시작 마커 (진입 직선 시작 마커)
	uint16_t cut_first_curve_idx;// 첫 45도 커브 진입 마커
	uint16_t cut_last_exit_idx;  // 마지막 45도 커브 탈출 마커 (탈출 직선 시작 마커)
	uint16_t cut_end_idx;        // 컷 종료 지점 (다음 코너/교차로/정지선 마커)
	float cut_straight_dist;     // 시작점부터 종료 마커까지의 기하학적 경로 거리 [m]
	float cut_angle_deg;         // 사선 진행 각도 [deg]
	float cut_seg_x;             // 해당 세그먼트의 진행 축 투영 거리 ds [m]
	float cut_bulge_offset;      // 코리도 제약 초과 시 완만 스침 호 최대 횡오프셋 [m] (0이면 순수 직선 컷)
	float cut_bulge_mps;         // 완만 스침 호 구간 안전 속도 [m/s]
	float cut_bulge_warp;        // 비대칭 스침 호 정점 위상 보정 계수 (0이면 대칭 코사인)
	float cut_land_blend;        // 단차 시프트(직-45-45-직) 탈출 직선 안착 블렌드 거리 [m]
	float cut_lock_dist;         // 컷 시작 마커 기준 Phase 2 조향 잠금 개시 거리 [m]
	float cut_exit_yaw_off;      // 컷 탈출 시 진입 격자 기준 순 각도 변화량 [deg] (우회=0, 편측레그=+-45, 오메가=180)
	uint8_t is_omega_cut;        // Ω 루프(+-45 / -+270 / +-45) 단일 U턴 컷 여부
	uint8_t is_corner90_chained; // 연쇄 45도 원호 컷 조기 전환 여부
	int8_t omega_dir;            // 진입 45도 방향 (+1: +45/-270/+45 -> CW -180도, -1: -45/+270/-45 -> CCW +180도)
	float omega_r_arc;           // 동심 U턴 원호 반경 [m]
	float omega_tan1_len;        // 진입 접선 직선 길이 [m]
	float omega_arc_len;         // 동심 U턴 원호 길이 [m]
	float omega_tan2_len;        // 탈출 접선 직선 길이 [m]
	float omega_tan1_deg;        // 진입 접선 각도 [deg] (진입 직선 기준)
	float omega_tan2_deg;        // 탈출 접선 각도 [deg] (진입 직선 기준)
	float omega_sweep_deg;       // 원호 회전 각도 크기 [deg]
	float omega_arc_mps;         // U턴 원호 안전 주행 속도 [m/s]
	uint8_t is_wave180_cut;      // 연속 180도 웨이브 코리도 평탄화 컷 여부
	uint8_t wave_count;          // 연속 180도 반파 개수 N (>= 2)
	int8_t wave_first_dir;       // 첫 번째 180도 반파 회전 방향 (+1: CCW +180, -1: CW -180)
	float wave_half_len;         // 컷 반파 원호 길이 [m]
	float wave_r_arc;            // 컷 반파 원호 반경 [m]
	float wave_alpha_deg;        // 컷 반파 반각 [deg]
	float wave_mps;              // 웨이브 컷 목표 속도 [m/s]
} SegmentPlanFourth_t;

static SegmentPlanFourth_t seg_plan_4[CROSS_LOG_MAX];

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	uint8_t omega_braking_started;
	uint8_t zone_start_from_deep;
	float marker_start_dist;
	float corridor_start_dist;
	float seg_len_predicted;
	float target_mps;
	float end_mps;
	uint8_t in_straight_cut;
	uint16_t cut_start_idx;
	uint16_t cut_first_curve_idx;
	uint16_t cut_last_exit_idx;
	uint16_t cut_end_idx;
	float cut_start_odom;
	float cut_straight_dist;
	float cut_angle_deg;
	float cut_base_yaw_ref;
	float cut_bulge_offset;
	float cut_bulge_mps;
	float cut_bulge_warp;
	float cut_land_blend;
	float cut_lock_dist;
	float cut_exit_yaw_off;
	float imu_grid_bias;
	uint32_t last_grid_bias_tick;
	float odom_offset_m;
	uint8_t cut_p1_latched;
	uint8_t cut_p3_preshift_dir;
	uint16_t cut_p3_preshift_corner;
	uint8_t is_omega_cut;
	int8_t omega_dir;
	float omega_r_arc;
	float omega_tan1_len;
	float omega_arc_len;
	float omega_tan2_len;
	float omega_tan1_deg;
	float omega_tan2_deg;
	float omega_sweep_deg;
	float omega_arc_mps;
	uint8_t is_wave180_cut;
	uint8_t wave_count;
	int8_t wave_first_dir;
	float wave_half_len;
	float wave_r_arc;
	float wave_alpha_deg;
	float wave_mps;
} DriveFourthState_t;

__STATIC_INLINE float Chicane_Mps(void) {
	// 45도 연속 비컷 S자 구간 고속 주행 속도 (R=0.25m + 0.85x IN 시프트 기준 beta<25deg 안정권)
	float c_mps = driveData.base_mps + 0.35f;
	float s_mps = Slalom_Mps();
	if (c_mps < s_mps) c_mps = s_mps;
	if (c_mps > driveData.max_mps) c_mps = driveData.max_mps;
	return c_mps;
}

__STATIC_INLINE float Same45_Mps(void) {
	// 동일방향 45도-짧은직선-45도 IN 브릿지 (FULL 16.8V 기준 후속 코너 진입 전 감속 여유 확보, beta<25deg)
	float mps = driveData.base_mps + 0.22f;
	float s_mps = Slalom_Mps();
	if (mps < s_mps) mps = s_mps;
	if (mps > driveData.max_mps) mps = driveData.max_mps;
	return mps;
}

__STATIC_INLINE float Fourth_Corner_Mps(void) {
	// R=0.25m 급커브에서 대폭 IN 시프트(>=0.45) 시 유효 반경 감소(~0.17m)에 맞춘 코너 진입/선회 안정 속도
	return (driveData.target_shift_val >= 0.45f) ? (driveData.base_mps - 0.08f) : driveData.base_mps;
}

__STATIC_INLINE float Segment_Mps_Fourth(uint8_t accel_type) {
	if (accel_type == 1 || accel_type == 4)
		return driveData.max_mps;
	if (accel_type == 3)
		return Chicane_Mps();
	if (accel_type == 5)
		return Same45_Mps();
	if (accel_type == 2)
		return Slalom_Mps();
	return Fourth_Corner_Mps();
}

// 시케인 구간 세그먼트의 기하학적 요 변화량 [rad] 산출 (3-호 우회로 / 다중 45도 시케인 / 쉐브론 레그 공용)
__STATIC_INLINE float Cut_Seg_Dyr_Rad(uint16_t seg, uint16_t first_curve_idx, uint16_t last_curve_idx, uint8_t is_pattern_a, float y1) {
	if (seg < first_curve_idx || seg > last_curve_idx) {
		return 0.0f;
	}
	if (is_pattern_a) {
		if (seg == first_curve_idx || seg == first_curve_idx + 2) {
			return (y1 > 0.0f ? 45.0f : -45.0f) * (PI / 180.0f);
		}
		if (seg == first_curve_idx + 1) {
			return (y1 > 0.0f ? -90.0f : 90.0f) * (PI / 180.0f);
		}
		return 0.0f;
	}
	float raw_yaw = ref_log[seg + 1].yaw_angle;
	float cd = ref_log[seg + 1].dist_from_prev_m;
	if (raw_yaw >= 65.0f && raw_yaw <= 115.0f) {
		return 90.0f * (PI / 180.0f);
	}
	if (raw_yaw <= -65.0f && raw_yaw >= -115.0f) {
		return -90.0f * (PI / 180.0f);
	}
	if (raw_yaw > 18.0f || (raw_yaw >= 14.0f && cd <= 0.25f) || (raw_yaw >= 10.0f && seg == last_curve_idx)) {
		return 45.0f * (PI / 180.0f);
	}
	if (raw_yaw < -18.0f || (raw_yaw <= -14.0f && cd <= 0.25f) || (raw_yaw <= -10.0f && seg == last_curve_idx)) {
		return -45.0f * (PI / 180.0f);
	}
	return 0.0f;
}

// Straight-Cut 시작점(from_idx)부터 목표 마커(to_idx)까지의 투영 누적 거리 산출 [m]
__STATIC_INLINE float Cut_Milestone_Dist(uint16_t from_idx, uint16_t to_idx) {
	float dist = 0.0f;
	for (uint16_t k = from_idx; k < to_idx; k++) {
		dist += seg_plan_4[k].cut_seg_x;
	}
	return dist;
}

typedef struct {
	uint8_t ok;
	uint16_t land_idx;
	uint16_t corner_idx;
	float exit_dist;
	float lock_dist;
	float land_blend;
	float L_cut;
	float cut_angle_deg;
	float cut_angle_rad;
	float bulge_offset;
	float bulge_mps;
	float bulge_warp;
	float arc_scale;
	float total_straight;
	float exit_yaw_off;
} CutEval_t;

// 패턴 B 잔여 곡선 구간의 자체 현 대비 최대 이탈량 산출 (불필요한 이중 S-시케인 병합 방지용)
static float Cut_Tail_Max_Dev(uint16_t tail_start, uint16_t tail_last) {
	if (tail_start > tail_last || tail_last + 1 >= ref_log_count) {
		return 999.0f;
	}
	uint16_t fc = tail_start;
	while (fc <= tail_last && fabsf(ref_log[fc + 1].yaw_angle) < 18.0f) {
		fc++;
	}
	if (fc > tail_last) {
		return 999.0f;
	}
	float x = 0.0f, y = 0.0f, th = 0.0f;
	for (uint16_t seg = fc; seg <= tail_last; seg++) {
		float d = ref_log[seg + 1].dist_from_prev_m;
		float dyr = Cut_Seg_Dyr_Rad(seg, fc, tail_last, 0, ref_log[fc + 1].yaw_angle);
		float d_sub = d * 0.25f;
		float dyr_sub = dyr * 0.25f;
		for (int sub = 0; sub < 4; sub++) {
			x += d_sub * cosf(th + 0.5f * dyr_sub);
			y += d_sub * sinf(th + 0.5f * dyr_sub);
			th += dyr_sub;
		}
	}
	float lock = 0.02f;
	float ca = atan2f(y, x - lock);
	float nx = -sinf(ca);
	float ny = cosf(ca);
	float px = 0.0f, py = 0.0f, pth = 0.0f;
	float max_dev = 0.0f;
	for (uint16_t seg = fc; seg <= tail_last; seg++) {
		float d = ref_log[seg + 1].dist_from_prev_m;
		float dyr = Cut_Seg_Dyr_Rad(seg, fc, tail_last, 0, ref_log[fc + 1].yaw_angle);
		float d_sub = d * 0.25f;
		float dyr_sub = dyr * 0.25f;
		for (int sub = 0; sub < 4; sub++) {
			px += d_sub * cosf(pth + 0.5f * dyr_sub);
			py += d_sub * sinf(pth + 0.5f * dyr_sub);
			pth += dyr_sub;
			float dev = fabsf((px - lock) * nx + py * ny);
			if (dev > max_dev) max_dev = dev;
		}
	}
	return max_dev;
}

// Straight-Cut 후보 기하 검증 (side-effect 없음)
static uint8_t Cut_Eval_Candidate(
    uint16_t start_idx, uint16_t first_curve_idx, float entry_dist, float y1,
    uint16_t last_curve_idx, uint8_t num_curves, uint8_t last_is_split, uint8_t ext_has_mid90,
    uint8_t is_pattern_a, uint8_t is_apex_leg1, uint8_t is_apex_leg2, uint8_t is_chained_cut, CutEval_t *out) {
	if (last_curve_idx == 0 || num_curves < 2) {
		return 0;
	}
	uint16_t land_idx = last_curve_idx + 1;
	uint16_t corner_idx = land_idx;
	float exit_dist = 0.0f;
	if (!is_apex_leg1) {
		if (last_is_split && corner_idx + 1 < ref_log_count &&
		    ref_log[corner_idx].type != CROSS_STOP && ref_log[corner_idx].type != CROSS_CROSS) {
			exit_dist += ref_log[corner_idx + 1].dist_from_prev_m;
			corner_idx++;
		}
		while (corner_idx + 1 < ref_log_count) {
			if (ref_log[corner_idx].type == CROSS_STOP ||
			    (ref_log[corner_idx].type == CROSS_CROSS && exit_dist < 0.25f)) {
				break;
			}
			float next_cy = fabsf(ref_log[corner_idx + 1].yaw_angle);
			if (next_cy >= 18.0f) {
				break;
			}
			exit_dist += ref_log[corner_idx + 1].dist_from_prev_m;
			corner_idx++;
			if (ref_log[corner_idx].type == CROSS_STOP ||
			    (ref_log[corner_idx].type == CROSS_CROSS && exit_dist < 0.25f)) {
				break;
			}
		}
	}
	if (!(is_apex_leg1 || exit_dist >= 0.16f) || corner_idx >= ref_log_count) {
		return 0;
	}

	float x_land = 0.0f, y_land = 0.0f, th = 0.0f;
	for (uint16_t seg = start_idx; seg < land_idx; seg++) {
		float d = ref_log[seg + 1].dist_from_prev_m;
		float dyr = Cut_Seg_Dyr_Rad(seg, first_curve_idx, last_curve_idx, is_pattern_a, y1);
		float d_sub = d * 0.25f;
		float dyr_sub = dyr * 0.25f;
		for (int sub = 0; sub < 4; sub++) {
			x_land += d_sub * cosf(th + 0.5f * dyr_sub);
			y_land += d_sub * sinf(th + 0.5f * dyr_sub);
			th += dyr_sub;
		}
	}

	float th_deg_signed = th * (180.0f / PI);
	float th_deg = fabsf(th_deg_signed);
	uint8_t is_step_shift = (!is_pattern_a && ((is_apex_leg1 || is_apex_leg2) || (th_deg <= 12.0f && fabsf(y_land) > 0.040f))) ? 1 : 0;
	float lock_dist;
	if (is_apex_leg2 || is_chained_cut) {
		lock_dist = 0.02f;
	} else if (is_step_shift && num_curves >= 3) {
		lock_dist = (entry_dist > 0.28f) ? (entry_dist - 0.22f) : ((entry_dist > 0.18f) ? (entry_dist - 0.16f) : 0.02f);
	} else if (is_step_shift) {
		lock_dist = (entry_dist > 0.42f) ? (entry_dist - 0.36f)
		          : ((entry_dist > 0.24f) ? (entry_dist - 0.20f)
		          : ((entry_dist > 0.16f) ? (entry_dist - 0.13f) : 0.02f));
	} else {
		lock_dist = (entry_dist > 0.12f) ? (entry_dist - 0.05f) : 0.02f;
	}
	float land_blend = (is_step_shift && last_is_split) ? fminf(0.06f, exit_dist * 0.25f) : 0.0f;
	float x_target = is_step_shift ? x_land : (x_land + land_blend * cosf(th));
	float y_target = is_step_shift ? y_land : (y_land + land_blend * sinf(th));

	float dx_cut = x_target - lock_dist;
	float dy_cut = y_target;
	float L_cut = sqrtf(dx_cut * dx_cut + dy_cut * dy_cut);
	float dx_aim = (is_step_shift && !is_pattern_a && !ext_has_mid90 && num_curves >= 4 && dx_cut > 0.40f) ? (dx_cut - 0.08f) : dx_cut;
	float cut_angle_deg = atan2f(dy_cut, dx_aim) * (180.0f / PI);
	float cut_angle_rad = cut_angle_deg * (PI / 180.0f);
	float cos_ca = cosf(cut_angle_rad);
	float sin_ca = sinf(cut_angle_rad);
	float chord_norm_x = -sin_ca;
	float chord_norm_y = cos_ca;
	float inv_L = (L_cut > 0.05f) ? (1.0f / L_cut) : 0.0f;

	float px = 0.0f, py = 0.0f, pth = 0.0f;
	float max_pos_dev = 0.0f;
	float min_neg_dev = 0.0f;
	float u_pos_peak = 0.5f;
	float u_neg_peak = 0.5f;
	float cum_entry = 0.0f;
	for (uint16_t seg = start_idx; seg < land_idx; seg++) {
		float d = ref_log[seg + 1].dist_from_prev_m;
		float dyr = Cut_Seg_Dyr_Rad(seg, first_curve_idx, last_curve_idx, is_pattern_a, y1);
		float d_sub = d * 0.25f;
		float dyr_sub = dyr * 0.25f;
		for (int sub = 0; sub < 4; sub++) {
			px += d_sub * cosf(pth + 0.5f * dyr_sub);
			py += d_sub * sinf(pth + 0.5f * dyr_sub);
			pth += dyr_sub;
			if (seg >= first_curve_idx) {
				float signed_dev = (px - lock_dist) * chord_norm_x + py * chord_norm_y;
				float u_cur = ((px - lock_dist) * cos_ca + py * sin_ca) * inv_L;
				if (signed_dev > max_pos_dev) { max_pos_dev = signed_dev; u_pos_peak = u_cur; }
				if (signed_dev < min_neg_dev) { min_neg_dev = signed_dev; u_neg_peak = u_cur; }
			}
		}
		if (seg < first_curve_idx) {
			cum_entry += d;
			if (cum_entry > lock_dist) {
				float signed_dev = (px - lock_dist) * chord_norm_x + py * chord_norm_y;
				float u_cur = ((px - lock_dist) * cos_ca + py * sin_ca) * inv_L;
				if (signed_dev > max_pos_dev) { max_pos_dev = signed_dev; u_pos_peak = u_cur; }
				if (signed_dev < min_neg_dev) { min_neg_dev = signed_dev; u_neg_peak = u_cur; }
			}
		}
	}

	float max_dev = (max_pos_dev > -min_neg_dev) ? max_pos_dev : -min_neg_dev;
	float bulge_offset = 0.0f;
	float bulge_mps = driveData.max_mps;
	float bulge_warp = 0.0f;
	float arc_scale = 1.0f;
	uint8_t dev_ok = 0;
	float cut_dist_total = lock_dist + L_cut;

	float sign = (max_pos_dev >= -min_neg_dev) ? 1.0f : -1.0f;
	float opp_dev = (sign > 0.0f) ? -min_neg_dev : max_pos_dev;
	float u_pk = (sign > 0.0f) ? u_pos_peak : u_neg_peak;
	uint8_t allow_wide_bulge = (!is_apex_leg1 && !is_apex_leg2 &&
	                            (ext_has_mid90 || (num_curves >= 4 && L_cut >= 1.20f)) &&
	                            L_cut >= 1.20f && L_cut <= 3.60f &&
	                            u_pk >= 0.28f &&
	                            opp_dev <= 0.105f && (max_dev + opp_dev) <= 0.560f &&
	                            max_dev <= 0.530f &&
	                            (max_dev <= 0.360f || max_dev <= 0.25f * L_cut)) ? 1 : 0;

	if (max_dev <= DRIVE4_CORRIDOR_CUT_HALF_M) {
		dev_ok = 1;
	} else if ((!is_step_shift && max_dev <= DRIVE4_CORRIDOR_CUT_HALF_M + 0.220f &&
	            opp_dev <= 0.080f && L_cut >= 0.65f) ||
	           allow_wide_bulge) {
		float mag = allow_wide_bulge ? (max_dev - 0.075f) : (max_dev - 0.068f);
		float min_mag = allow_wide_bulge ? 0.050f : 0.020f;
		float max_mag = allow_wide_bulge ? 0.420f : 0.252f;
		if (mag < min_mag) mag = min_mag;
		else if (mag > max_mag) mag = max_mag;
		bulge_offset = sign * mag;

		if (allow_wide_bulge && u_pk >= 0.32f && u_pk <= 0.72f && fabsf(u_pk - 0.50f) > 0.04f) {
			float warp_lim = (L_cut >= 2.20f) ? 0.18f : 0.12f;
			bulge_warp = (0.50f - u_pk) / sinf(PI * u_pk);
			if (bulge_warp < -warp_lim) bulge_warp = -warp_lim;
			else if (bulge_warp > warp_lim) bulge_warp = warp_lim;
		}

		float slope_k = PI * mag / L_cut;
		arc_scale = 1.0f + 0.25f * slope_k * slope_k * (1.0f + 0.5f * bulge_warp * bulge_warp * PI * PI);
		float L_cos = L_cut * arc_scale;
		cut_dist_total = lock_dist + L_cos;

		float dtau_pk = 1.0f + bulge_warp * PI * cosf(PI * u_pk);
		float r_min = (L_cos * L_cos) / (2.0f * PI * PI * mag * dtau_pk * dtau_pk);
		if (allow_wide_bulge && max_dev > 0.360f && r_min < 0.58f) {
			dev_ok = 0;
		} else {
			float b_lat = (driveData.bulge_lat_acc > 0.5f) ? driveData.bulge_lat_acc : 6.50f;
			bulge_mps = sqrtf(b_lat * r_min);
			if (bulge_mps < driveData.base_mps) bulge_mps = driveData.base_mps;
			if (bulge_mps > driveData.max_mps) bulge_mps = driveData.max_mps;
			float bx = 0.0f, by = 0.0f, bth = 0.0f, max_b_err = 0.0f, b_cum = 0.0f;
			for (uint16_t seg = start_idx; seg < land_idx; seg++) {
				float d = ref_log[seg + 1].dist_from_prev_m;
				float dyr = Cut_Seg_Dyr_Rad(seg, first_curve_idx, last_curve_idx, is_pattern_a, y1);
				float d_sub = d * 0.25f, dyr_sub = dyr * 0.25f;
				for (int sub = 0; sub < 4; sub++) {
					bx += d_sub * cosf(bth + 0.5f * dyr_sub);
					by += d_sub * sinf(bth + 0.5f * dyr_sub);
					bth += dyr_sub;
					if (seg >= first_curve_idx || b_cum + (sub + 1) * d_sub > lock_dist) {
						float u_c = ((bx - lock_dist) * cos_ca + by * sin_ca) * inv_L;
						if (u_c < 0.0f) u_c = 0.0f;
						else if (u_c > 1.0f) u_c = 1.0f;
						float tau = u_c + bulge_warp * sinf(PI * u_c);
						float y_plan = bulge_offset * sinf(PI * tau);
						float sd = (bx - lock_dist) * chord_norm_x + by * chord_norm_y;
						float err = fabsf(sd - y_plan);
						if (err > max_b_err) max_b_err = err;
					}
				}
				if (seg < first_curve_idx) b_cum += d;
			}
			if (max_b_err > 0.088f) return 0;
			else dev_ok = 1;
			dev_ok = 1;
		}
	}

	float total_straight = is_apex_leg1 ? cut_dist_total : (cut_dist_total + (exit_dist - land_blend));
	uint8_t geom_ok = 0;
	if (is_apex_leg1) {
		geom_ok = (dev_ok && bulge_offset == 0.0f && total_straight >= 0.55f &&
		           fabsf(cut_angle_deg) <= ((bulge_offset == 0.0f && num_curves >= 3) ? 28.0f : 24.0f) && fabsf(y_land) <= ((bulge_offset == 0.0f) ? 2.500f : 1.200f) && th_deg <= 12.0f);
	} else if (is_apex_leg2) {
		geom_ok = (dev_ok && bulge_offset == 0.0f && exit_dist >= 0.20f && total_straight >= 0.85f &&
		           fabsf(cut_angle_deg) <= 26.0f && fabsf(y_land) <= 0.900f && fabsf(th_deg - 45.0f) <= 12.0f);
	} else if (is_step_shift) {
		geom_ok = (dev_ok && (bulge_offset == 0.0f || allow_wide_bulge) &&
		           exit_dist >= 0.16f && total_straight >= (is_chained_cut ? 0.80f : 0.90f) &&
		           fabsf(cut_angle_deg) <= ((bulge_offset == 0.0f && num_curves >= 3) ? 28.0f : 24.0f) && fabsf(y_land) <= ((bulge_offset == 0.0f) ? 2.500f : 1.200f) && th_deg <= 12.0f);
	} else {
		geom_ok = (dev_ok && cut_dist_total >= (is_chained_cut ? 0.65f : 0.70f) && total_straight >= (is_chained_cut ? 0.85f : 1.05f) &&
		           fabsf(cut_angle_deg) <= (allow_wide_bulge ? 8.0f : 6.0f) &&
		           fabsf(y_land) <= 0.080f && th_deg <= 10.0f);
	}
	if (!geom_ok) {
		return 0;
	}

	out->ok = 1;
	out->land_idx = land_idx;
	out->corner_idx = corner_idx;
	out->exit_dist = exit_dist;
	out->lock_dist = lock_dist;
	out->land_blend = land_blend;
	out->L_cut = L_cut;
	out->cut_angle_deg = cut_angle_deg;
	out->cut_angle_rad = cut_angle_rad;
	out->bulge_offset = bulge_offset;
	out->bulge_mps = bulge_mps;
	out->bulge_warp = bulge_warp;
	out->arc_scale = arc_scale;
	out->total_straight = total_straight;
	out->exit_yaw_off = is_apex_leg2 ? th_deg_signed : 0.0f;
	return 1;
}

// 확정된 Straight-Cut 후보의 투영 거리(cut_seg_x) 및 계획 테이블 반영
static void Cut_Apply_Plan(
    uint16_t start_idx, uint16_t first_curve_idx, uint16_t last_curve_idx,
    uint8_t is_pattern_a, float y1, const CutEval_t *res) {
	float pth = 0.0f;
	float cum_entry = 0.0f;
	float cos_ca = cosf(res->cut_angle_rad);
	float sin_ca = sinf(res->cut_angle_rad);
	for (uint16_t seg = start_idx; seg < res->land_idx; seg++) {
		float d = ref_log[seg + 1].dist_from_prev_m;
		float dyr = Cut_Seg_Dyr_Rad(seg, first_curve_idx, last_curve_idx, is_pattern_a, y1);
		float d_sub = d * 0.25f;
		float dyr_sub = dyr * 0.25f;
		float seg_dx = 0.0f, seg_dy = 0.0f;
		for (int sub = 0; sub < 4; sub++) {
			seg_dx += d_sub * cosf(pth + 0.5f * dyr_sub);
			seg_dy += d_sub * sinf(pth + 0.5f * dyr_sub);
			pth += dyr_sub;
		}
		if (seg < first_curve_idx) {
			float prev_entry = cum_entry;
			cum_entry += d;
			if (cum_entry > res->lock_dist) {
				float over = cum_entry - res->lock_dist;
				float stay = (res->lock_dist > prev_entry) ? (res->lock_dist - prev_entry) : 0.0f;
				seg_plan_4[seg].cut_seg_x = stay + over * cos_ca;
			} else {
				seg_plan_4[seg].cut_seg_x = d;
			}
		} else {
			float proj = seg_dx * cos_ca + seg_dy * sin_ca;
			if (res->bulge_offset != 0.0f && res->L_cut > 0.20f) {
				proj *= res->arc_scale;
			}
			seg_plan_4[seg].cut_seg_x = proj;
		}
	}
	for (uint16_t seg = res->land_idx; seg < res->corner_idx; seg++) {
		seg_plan_4[seg].cut_seg_x = ref_log[seg + 1].dist_from_prev_m;
	}
	for (uint16_t seg = start_idx; seg < res->corner_idx; seg++) {
		seg_plan_4[seg].accel_type = 4;
		seg_plan_4[seg].curve_dir = 0;
		seg_plan_4[seg].is_straight_cut = 1;
		seg_plan_4[seg].cut_start_idx = start_idx;
		seg_plan_4[seg].cut_first_curve_idx = first_curve_idx;
		seg_plan_4[seg].cut_last_exit_idx = res->land_idx;
		seg_plan_4[seg].cut_end_idx = res->corner_idx;
		seg_plan_4[seg].cut_straight_dist = res->total_straight;
		seg_plan_4[seg].cut_angle_deg = res->cut_angle_deg;
		seg_plan_4[seg].cut_bulge_offset = res->bulge_offset;
		seg_plan_4[seg].cut_bulge_mps = res->bulge_mps;
		seg_plan_4[seg].cut_bulge_warp = res->bulge_warp;
		seg_plan_4[seg].cut_land_blend = res->land_blend;
		seg_plan_4[seg].cut_lock_dist = res->lock_dist;
		seg_plan_4[seg].cut_exit_yaw_off = res->exit_yaw_off;
	}
}

typedef struct {
	uint8_t ok;
	uint16_t land_idx;
	uint16_t corner_idx;
	int8_t dir;
	float r_arc;
	float lock_dist;
	float exit_travel;
	float tan1_len;
	float arc_len;
	float tan2_len;
	float tan1_deg;
	float tan2_deg;
	float sweep_deg;
	float arc_mps;
	float s_m1;
	float exit_dist;
	float total_straight;
} OmegaCutEval_t;

// Ω 루프(+-45 / -+270 / +-45) -> 단일 동심 원호 U턴 + 진입/탈출 접선 컷 기하 검증
static uint8_t Omega_Eval_Candidate(
    uint16_t start_idx, uint16_t first_curve_idx, float entry_dist, float y1, OmegaCutEval_t *out) {
	if (first_curve_idx + 3 >= ref_log_count) {
		return 0;
	}
	for (uint16_t m = first_curve_idx + 1; m <= first_curve_idx + 3; m++) {
		if (ref_log[m].type == CROSS_STOP || ref_log[m].type == CROSS_CROSS) {
			return 0;
		}
	}
	float y2 = ref_log[first_curve_idx + 2].yaw_angle;
	float y3 = ref_log[first_curve_idx + 3].yaw_angle;
	if ((y1 * y2 >= 0.0f) || (y1 * y3 <= 0.0f)) {
		return 0;
	}
	if (fabsf(y1) < 20.0f || fabsf(y1) > 62.0f ||
	    fabsf(y2) < 210.0f || fabsf(y2) > 325.0f ||
	    fabsf(y3) < 20.0f || fabsf(y3) > 62.0f) {
		return 0;
	}
	float net_abs = fabsf(y1 + y2 + y3);
	if (fabsf(net_abs - 180.0f) > 25.0f) {
		return 0;
	}

	uint16_t land_idx = first_curve_idx + 3; // M4 (탈출 45도 종료 마커)
	uint16_t corner_idx = land_idx;
	float exit_dist = 0.0f;
	while (corner_idx + 1 < ref_log_count) {
		if (ref_log[corner_idx].type == CROSS_STOP || ref_log[corner_idx].type == CROSS_CROSS) {
			break;
		}
		float next_cy = fabsf(ref_log[corner_idx + 1].yaw_angle);
		if (next_cy >= 18.0f) {
			break;
		}
		exit_dist += ref_log[corner_idx + 1].dist_from_prev_m;
		corner_idx++;
		if (ref_log[corner_idx].type == CROSS_STOP || ref_log[corner_idx].type == CROSS_CROSS) {
			break;
		}
	}
	float min_exit = (corner_idx < ref_log_count && ref_log[corner_idx].type == CROSS_CROSS) ? 0.14f : 0.18f;
	if (exit_dist < min_exit || corner_idx >= ref_log_count) {
		return 0;
	}

	int8_t s_dir = (y1 > 0.0f) ? 1 : -1;
	float dir_f = (float)s_dir;
	const float SQRT2_HALF = 0.70710678f;

	// 마커 위치 편차 상쇄를 위해 플레어(45)+벌브(270)+플레어(45)=360도 전체 호 길이로 단일 반경 추정
	float arc_sum = ref_log[first_curve_idx + 1].dist_from_prev_m +
	                ref_log[first_curve_idx + 2].dist_from_prev_m +
	                ref_log[first_curve_idx + 3].dist_from_prev_m;
	float R_omega = arc_sum / (2.0f * PI);
	if (R_omega < 0.21f) R_omega = 0.21f;
	else if (R_omega > 0.32f) R_omega = 0.32f;

	// M1=(entry_dist, 0) -> 대칭 벌브 중심 (cx, cy) -> M4=(x4, y4)
	float cx = entry_dist + R_omega * (2.0f * SQRT2_HALF);
	float cy = -dir_f * R_omega * (2.0f * SQRT2_HALF - 1.0f);
	float x4 = entry_dist;
	float y4_pos = 2.0f * cy;

	// 동심 U턴 반경: 외곽 벌브 라인 안쪽 ~55mm 유지 (오프트랙 및 IR 센서 여유 확보)
	float r_arc = R_omega - 0.045f;
	if (r_arc < 0.195f) r_arc = 0.195f;
	if (r_arc > 0.235f) r_arc = 0.235f;

	float lock_dist;
	if (start_idx > 0 && ref_log[start_idx].type == CROSS_CROSS) {
		lock_dist = fminf(0.14f, entry_dist * 0.45f);
		if (lock_dist < 0.05f) lock_dist = 0.05f;
	} else {
		lock_dist = fminf(0.22f, entry_dist - 0.08f);
		if (lock_dist < 0.10f) lock_dist = 0.10f;
	}
	if (lock_dist > entry_dist - 0.05f) {
		return 0;
	}

	float exit_hold_min = (ref_log[corner_idx].type == CROSS_CROSS)
	                          ? fminf(0.16f, exit_dist * 0.55f)
	                          : fminf(0.24f, exit_dist * 0.65f);
	float exit_travel = exit_dist - exit_hold_min;
	float max_exit_travel = (ref_log[corner_idx].type == CROSS_CROSS) ? 0.42f : 0.36f;
	if (exit_travel > max_exit_travel) exit_travel = max_exit_travel;
	if (exit_travel < 0.0f) exit_travel = 0.0f;
	float exit_hold = exit_dist - exit_travel;

	float dx1 = cx - lock_dist;
	float dy1 = cy;
	float D1 = sqrtf(dx1 * dx1 + dy1 * dy1);

	float x_land = x4 - exit_travel;
	float dx2 = x_land - cx;
	float dy2 = y4_pos - cy;
	float D2 = sqrtf(dx2 * dx2 + dy2 * dy2);

	if (D1 <= r_arc + 0.06f || D2 <= r_arc + 0.06f) {
		return 0;
	}

	float th_in_rad = atan2f(dy1, dx1) + dir_f * asinf(r_arc / D1);
	float th_out_rad = atan2f(dy2, dx2) - dir_f * asinf(r_arc / D2);
	float th_in_deg = Wrap_Angle_Deg(th_in_rad * (180.0f / PI));
	float th_out_deg = Wrap_Angle_Deg(th_out_rad * (180.0f / PI));

	// 선회 방향(-s_dir)에 맞춘 원호 스윕 각도 [deg]
	float sweep_deg = dir_f * (th_in_deg - th_out_deg);
	while (sweep_deg < 0.0f) sweep_deg += 360.0f;
	while (sweep_deg >= 360.0f) sweep_deg -= 360.0f;

	if (sweep_deg < 150.0f || sweep_deg > 230.0f || fabsf(th_in_deg) > 22.0f) {
		return 0;
	}

	float tan1_len = sqrtf(D1 * D1 - r_arc * r_arc);
	float tan2_len = sqrtf(D2 * D2 - r_arc * r_arc);
	float arc_len = r_arc * (sweep_deg * (PI / 180.0f));
	if (tan1_len < 0.08f || tan2_len <= exit_travel + 0.05f || arc_len < 0.35f) {
		return 0;
	}

	float s_m1 = lock_dist + (entry_dist - lock_dist) * cosf(th_in_rad);
	float s_m1_min = lock_dist + 0.03f;
	float s_m1_max = lock_dist + tan1_len - 0.05f;
	if (s_m1 < s_m1_min) s_m1 = s_m1_min;
	if (s_m1 > s_m1_max) s_m1 = s_m1_max;

	float arc_mps = sqrtf(13.0f * r_arc);
	float max_corner = Fourth_Corner_Mps() + driveData.omega_add_mps;
	if (arc_mps > max_corner) arc_mps = max_corner;
	if (arc_mps < 1.30f) arc_mps = 1.30f;

	out->ok = 1;
	out->land_idx = land_idx;
	out->corner_idx = corner_idx;
	out->dir = s_dir;
	out->r_arc = r_arc;
	out->lock_dist = lock_dist;
	out->exit_travel = exit_travel;
	out->tan1_len = tan1_len;
	out->arc_len = arc_len;
	out->tan2_len = tan2_len;
	out->tan1_deg = th_in_deg;
	out->tan2_deg = th_out_deg;
	out->sweep_deg = sweep_deg;
	out->arc_mps = arc_mps;
	out->s_m1 = s_m1;
	out->exit_dist = exit_dist;
	out->total_straight = lock_dist + tan1_len + arc_len + tan2_len + exit_hold;
	return 1;
}

static void Omega_Apply_Plan(
    uint16_t start_idx, uint16_t first_curve_idx, float entry_dist, const OmegaCutEval_t *res) {
	if (entry_dist > 0.01f) {
		float scale = res->s_m1 / entry_dist;
		for (uint16_t seg = start_idx; seg < first_curve_idx; seg++) {
			seg_plan_4[seg].cut_seg_x = ref_log[seg + 1].dist_from_prev_m * scale;
		}
	}
	seg_plan_4[first_curve_idx].cut_seg_x = (res->lock_dist + res->tan1_len) - res->s_m1;
	seg_plan_4[first_curve_idx + 1].cut_seg_x = res->arc_len;
	seg_plan_4[first_curve_idx + 2].cut_seg_x = res->tan2_len - res->exit_travel;
	for (uint16_t seg = res->land_idx; seg < res->corner_idx; seg++) {
		seg_plan_4[seg].cut_seg_x = ref_log[seg + 1].dist_from_prev_m;
	}
	for (uint16_t seg = start_idx; seg < res->corner_idx; seg++) {
		seg_plan_4[seg].accel_type = 4;
		seg_plan_4[seg].curve_dir = 0;
		seg_plan_4[seg].is_straight_cut = 1;
		seg_plan_4[seg].is_omega_cut = 1;
		seg_plan_4[seg].is_corner90_chained = 0;
		seg_plan_4[seg].omega_dir = res->dir;
		seg_plan_4[seg].omega_r_arc = res->r_arc;
		seg_plan_4[seg].omega_tan1_len = res->tan1_len;
		seg_plan_4[seg].omega_arc_len = res->arc_len;
		seg_plan_4[seg].omega_tan2_len = res->tan2_len;
		seg_plan_4[seg].omega_tan1_deg = res->tan1_deg;
		seg_plan_4[seg].omega_tan2_deg = res->tan2_deg;
		seg_plan_4[seg].omega_sweep_deg = res->sweep_deg;
		seg_plan_4[seg].omega_arc_mps = res->arc_mps;
		seg_plan_4[seg].cut_start_idx = start_idx;
		seg_plan_4[seg].cut_first_curve_idx = first_curve_idx;
		seg_plan_4[seg].cut_last_exit_idx = res->land_idx;
		seg_plan_4[seg].cut_end_idx = res->corner_idx;
		seg_plan_4[seg].cut_straight_dist = res->total_straight;
		seg_plan_4[seg].cut_angle_deg = res->tan1_deg;
		seg_plan_4[seg].cut_bulge_offset = 0.0f;
		seg_plan_4[seg].cut_bulge_mps = res->arc_mps;
		seg_plan_4[seg].cut_bulge_warp = 0.0f;
		seg_plan_4[seg].cut_land_blend = res->exit_travel;
		seg_plan_4[seg].cut_lock_dist = res->lock_dist;
		seg_plan_4[seg].cut_exit_yaw_off = 180.0f;
	}
}

static void Wave180_Apply_Plan(uint16_t start_idx, uint8_t count, int8_t first_dir) {
	const float r_arc = 0.3125f;
	const float alpha_deg = 53.1301f;
	const float half_len = 0.595f;
	float wave_mps = Fourth_Corner_Mps() + driveData.wave_add_mps;
	if (wave_mps < 1.00f) wave_mps = 1.00f;
	if (wave_mps > driveData.max_mps) wave_mps = driveData.max_mps;

	uint16_t end_idx = start_idx + count;
	for (uint16_t seg = start_idx; seg < end_idx; seg++) {
		seg_plan_4[seg].accel_type = 4;
		seg_plan_4[seg].curve_dir = 0;
		seg_plan_4[seg].is_straight_cut = 1;
		seg_plan_4[seg].is_wave180_cut = 1;
		seg_plan_4[seg].wave_count = count;
		seg_plan_4[seg].wave_first_dir = first_dir;
		seg_plan_4[seg].wave_half_len = half_len;
		seg_plan_4[seg].wave_r_arc = r_arc;
		seg_plan_4[seg].wave_alpha_deg = alpha_deg;
		seg_plan_4[seg].wave_mps = wave_mps;
		seg_plan_4[seg].cut_start_idx = start_idx;
		seg_plan_4[seg].cut_first_curve_idx = start_idx;
		seg_plan_4[seg].cut_last_exit_idx = end_idx - 1;
		seg_plan_4[seg].cut_end_idx = end_idx;
		seg_plan_4[seg].cut_seg_x = half_len;
		seg_plan_4[seg].cut_straight_dist = (float)count * half_len;
		seg_plan_4[seg].cut_angle_deg = 0.0f;
		seg_plan_4[seg].cut_bulge_offset = 0.0f;
		seg_plan_4[seg].cut_bulge_mps = wave_mps;
		seg_plan_4[seg].cut_bulge_warp = 0.0f;
		seg_plan_4[seg].cut_land_blend = half_len;
		seg_plan_4[seg].cut_lock_dist = 0.0f;
		seg_plan_4[seg].cut_exit_yaw_off = (count & 1U) ? 180.0f : 0.0f;
	}
}

#if 0
static void Corner90_Apply_Plan(uint16_t start_idx, uint16_t corner_idx, uint16_t end_idx,
                                float entry_dist, float exit_dist, float yaw_deg, uint8_t chained) {
	float min_leg = (entry_dist < exit_dist) ? entry_dist : exit_dist;
	float lead = min_leg * 0.50f;
	if (lead > 0.22f) lead = 0.22f;
	if (lead < 0.12f) lead = 0.12f;
	float r_arc = 0.25f + lead;
	float arc_mps = Fourth_Corner_Mps() + 0.35f + lead * 0.90f;
	const float arc_len = r_arc * (PI * 0.5f);
	if (arc_mps > driveData.max_mps) arc_mps = driveData.max_mps;

	int8_t odir = (yaw_deg > 0.0f) ? -1 : 1; // +90(L, CCW) -> omega_dir = -1; -90(R, CW) -> +1
	float lock_dist = entry_dist - lead;
	float exit_rem = exit_dist - lead;
	if (exit_rem < 0.05f) exit_rem = 0.05f;
	float total_straight = lock_dist + arc_len + exit_rem;

	if (!chained) {
		for (uint16_t seg = start_idx; seg < corner_idx; seg++) {
			seg_plan_4[seg].cut_seg_x = ref_log[seg + 1].dist_from_prev_m;
		}
	}
	seg_plan_4[corner_idx].cut_seg_x = arc_len - 2.0f * lead;

	uint16_t plan_start = chained ? corner_idx : start_idx;
	uint16_t plan_end = corner_idx + 1;
	for (uint16_t seg = plan_start; seg < plan_end; seg++) {
		seg_plan_4[seg].accel_type = 4;
		seg_plan_4[seg].curve_dir = 0;
		seg_plan_4[seg].is_straight_cut = chained ? 0 : 1;
		seg_plan_4[seg].is_omega_cut = 1;
		seg_plan_4[seg].is_corner90_chained = 0;
		seg_plan_4[seg].is_corner90_cut = 1;
		seg_plan_4[seg].is_corner90_chained = chained;
		seg_plan_4[seg].omega_dir = odir;
		seg_plan_4[seg].omega_r_arc = r_arc;
		seg_plan_4[seg].omega_tan1_len = 0.0f;
		seg_plan_4[seg].omega_arc_len = arc_len;
		seg_plan_4[seg].is_corner90_chained = is_chained;
		seg_plan_4[seg].omega_tan2_len = 0.0f;
		seg_plan_4[seg].omega_tan1_deg = 0.0f;
		seg_plan_4[seg].omega_tan2_deg = -(float)odir * 90.0f;
		seg_plan_4[seg].omega_sweep_deg = 90.0f;
		seg_plan_4[seg].omega_arc_mps = arc_mps;
		seg_plan_4[seg].cut_start_idx = plan_start;
		seg_plan_4[seg].cut_first_curve_idx = corner_idx;
		seg_plan_4[seg].cut_last_exit_idx = corner_idx + 1;
		seg_plan_4[seg].cut_end_idx = end_idx;
		seg_plan_4[seg].cut_straight_dist = total_straight;
		seg_plan_4[seg].cut_angle_deg = 0.0f;
		seg_plan_4[seg].cut_bulge_offset = 0.0f;
		seg_plan_4[seg].cut_bulge_mps = arc_mps;
		seg_plan_4[seg].cut_bulge_warp = 0.0f;
		seg_plan_4[seg].cut_land_blend = lead;
		seg_plan_4[seg].cut_lock_dist = lock_dist;
		seg_plan_4[seg].cut_exit_yaw_off = -(float)odir * 90.0f;
	}
}

#endif
// 고립된 단독 직-45-직 코너: 진입/탈출 직선을 활용해 선회 반경 R_arc를 확대(인코스 대원호 주행)
static uint16_t Corner45_Try_Apply(uint16_t start_idx, uint16_t curve_idx, float entry_dist, float y1, uint8_t is_chained) {
	if (entry_dist < (is_chained ? 0.22f : 0.16f) || entry_dist > 1.05f ||
	    fabsf(y1) < 28.0f || fabsf(y1) > 62.0f || curve_idx + 2 >= ref_log_count) {
		return 0;
	}
	uint16_t land_idx = curve_idx + 1;
	uint16_t corner_idx = land_idx;
	float exit_dist = 0.0f;
	float first_cross_dist = 999.0f;
	while (corner_idx + 1 < ref_log_count) {
		if (ref_log[corner_idx].type == CROSS_STOP) {
			break;
		}
		if (ref_log[corner_idx].type == CROSS_CROSS && exit_dist < first_cross_dist) {
			first_cross_dist = exit_dist;
		}
		if (fabsf(ref_log[corner_idx + 1].yaw_angle) >= 18.0f) {
			break;
		}
		exit_dist += ref_log[corner_idx + 1].dist_from_prev_m;
		corner_idx++;
		if (ref_log[corner_idx].type == CROSS_STOP) {
			break;
		}
		if (ref_log[corner_idx].type == CROSS_CROSS && exit_dist < first_cross_dist) {
			first_cross_dist = exit_dist;
		}
	}
	if (exit_dist < 0.16f || corner_idx >= ref_log_count) {
		return 0;
	}
	float min_leg = fminf(entry_dist - (is_chained ? 0.06f : 0.16f), exit_dist - 0.10f);
	if (min_leg < 0.08f) {
		return 0;
	}
	float lead = min_leg;
	if (lead > (is_chained ? 0.25f : 0.30f)) lead = is_chained ? 0.25f : 0.30f;
	if (first_cross_dist < 900.0f && lead > first_cross_dist - 0.40f) {
		lead = first_cross_dist - 0.40f;
		if (lead < 0.08f) return 0;
	}
	if (!is_chained && corner_idx + 1 < ref_log_count && exit_dist >= 0.36f) {
		float ny = fabsf(ref_log[corner_idx + 1].yaw_angle);
		if (ny >= 28.0f && ny <= 62.0f && lead > exit_dist * 0.42f) {
			lead = exit_dist * 0.42f;
		}
	}

	float r0 = ref_log[curve_idx + 1].dist_from_prev_m / (45.0f * (PI / 180.0f));
	if (r0 < 0.20f) r0 = 0.20f;
	else if (r0 > 0.30f) r0 = 0.30f;

	// tan(22.5 deg) = 0.41421356, sec(22.5 deg) - 1 = 0.08239220 -> dev_max <= 70mm
	float dR = lead / 0.41421356f;
	float r_arc = r0 + dR;
	const float arc_len = r_arc * (45.0f * (PI / 180.0f));
	float arc_mps = sqrtf(8.5f * r_arc);
	float min_c = Fourth_Corner_Mps() + 0.15f;
	if (arc_mps < min_c) arc_mps = min_c;
	float c45_cap = (driveData.corner45_max_mps > 0.1f) ? driveData.corner45_max_mps : 3.30f;
	if (arc_mps > c45_cap) arc_mps = c45_cap;
	if (arc_mps < 1.00f) arc_mps = 1.00f;
	if (arc_mps > driveData.max_mps) arc_mps = driveData.max_mps;

	int8_t odir = (y1 > 0.0f) ? -1 : 1; // +45(L) -> -1, -45(R) -> +1
	float lock_dist = is_chained ? 0.0f : (entry_dist - lead);
	float exit_rem = exit_dist - lead;
	float total_straight = lock_dist + arc_len + exit_rem;

	for (uint16_t seg = start_idx; seg < curve_idx; seg++) {
		seg_plan_4[seg].cut_seg_x = ref_log[seg + 1].dist_from_prev_m;
	}
	seg_plan_4[curve_idx].cut_seg_x = is_chained ? (arc_len - lead) : (arc_len - 2.0f * lead);
	for (uint16_t seg = land_idx; seg < corner_idx; seg++) {
		seg_plan_4[seg].cut_seg_x = ref_log[seg + 1].dist_from_prev_m;
	}

	for (uint16_t seg = start_idx; seg < corner_idx; seg++) {
		seg_plan_4[seg].accel_type = 4;
		seg_plan_4[seg].curve_dir = 0;
		seg_plan_4[seg].is_straight_cut = 1;
		seg_plan_4[seg].is_omega_cut = 1;
		seg_plan_4[seg].is_corner90_chained = 0;
		seg_plan_4[seg].omega_dir = odir;
		seg_plan_4[seg].omega_r_arc = r_arc;
		seg_plan_4[seg].omega_tan1_len = 0.0f;
		seg_plan_4[seg].omega_arc_len = arc_len;
		seg_plan_4[seg].is_corner90_chained = is_chained;
		seg_plan_4[seg].omega_tan2_len = 0.0f;
		seg_plan_4[seg].omega_tan1_deg = 0.0f;
		seg_plan_4[seg].omega_tan2_deg = -(float)odir * 45.0f;
		seg_plan_4[seg].omega_sweep_deg = 45.0f;
		seg_plan_4[seg].omega_arc_mps = arc_mps;
		seg_plan_4[seg].cut_start_idx = start_idx;
		seg_plan_4[seg].cut_first_curve_idx = curve_idx;
		seg_plan_4[seg].cut_last_exit_idx = land_idx;
		seg_plan_4[seg].cut_end_idx = corner_idx;
		seg_plan_4[seg].cut_straight_dist = total_straight;
		seg_plan_4[seg].cut_angle_deg = 0.0f;
		seg_plan_4[seg].cut_bulge_offset = 0.0f;
		seg_plan_4[seg].cut_bulge_mps = arc_mps;
		seg_plan_4[seg].cut_bulge_warp = 0.0f;
		seg_plan_4[seg].cut_land_blend = lead;
		seg_plan_4[seg].cut_lock_dist = lock_dist;
		seg_plan_4[seg].cut_exit_yaw_off = -(float)odir * 45.0f;
	}
	return corner_idx;
}

// 4차 주행용 구간 계획 빌드 (마커 페어링 기반 직선/곡선 판별 및 직선-45도 연속-직선 통합 관통 브릿징)
__STATIC_INLINE void Build_Segment_Plan_Fourth(void) {
	uint8_t curve_state = 0;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t a_type = 0;

		if (cur == CROSS_STOP) {
			a_type = (i == 0) ? 1 : 0;
			curve_state = 0;
		} else if (cur == CROSS_CROSS) {
			a_type = 1;
			curve_state = 0;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				curve_state = 0;
				a_type = 1;
			} else {
				float curve_yaw = (i + 1 < ref_log_count) ? fabsf(ref_log[i + 1].yaw_angle) : 90.0f;
				if (curve_yaw < 18.0f) {
					curve_state = 0;
					a_type = 0;
				} else {
					curve_state = 1;
					a_type = (curve_yaw < 65.0f) ? 2 : 0;
				}
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				curve_state = 0;
				a_type = 1;
			} else {
				float curve_yaw = (i + 1 < ref_log_count) ? fabsf(ref_log[i + 1].yaw_angle) : 90.0f;
				if (curve_yaw < 18.0f) {
					curve_state = 0;
					a_type = 0;
				} else {
					curve_state = 2;
					a_type = (curve_yaw < 65.0f) ? 2 : 0;
				}
			}
		}

		seg_plan_4[i].curve_dir = curve_state;
		seg_plan_4[i].accel_type = a_type;
	}

	// 2a단계: 90도 이상 급커브(type 0)와 직선 여유 없이 직결된 45도 진입/탈출부는 안전 감속(type 0)으로 먼저 분리
	for (uint16_t i = 0; i < ref_log_count; i++) {
		if (seg_plan_4[i].curve_dir != 0 && seg_plan_4[i].accel_type == 2) {
			uint8_t adj_deep_prev = (i >= 1 && seg_plan_4[i - 1].curve_dir != 0 && seg_plan_4[i - 1].accel_type == 0);
			uint8_t adj_deep_next = (i + 1 < ref_log_count && seg_plan_4[i + 1].curve_dir != 0 && seg_plan_4[i + 1].accel_type == 0);
			if (adj_deep_prev || adj_deep_next) {
				seg_plan_4[i].accel_type = 0;
			}
		}
	}

	// 2b단계: 좌/우가 교차하는 45도 연속 시케인/S자 슬라롬(c45_detour 등)을 고속 관통 구간(type 3)으로 승격
	for (uint16_t i = 0; i < ref_log_count; i++) {
		if (seg_plan_4[i].curve_dir != 0 && seg_plan_4[i].accel_type == 2) {
			uint8_t dir_i = seg_plan_4[i].curve_dir;

			float dist_prev = 0.0f;
			uint8_t prev_opp_45 = 0;
			for (int p = (int)i - 1; p >= 0; p--) {
				if (seg_plan_4[p].curve_dir != 0) {
					if (seg_plan_4[p].accel_type >= 2 && seg_plan_4[p].curve_dir != dir_i) {
						prev_opp_45 = 1;
					}
					break;
				}
				dist_prev += ref_log[p + 1].dist_from_prev_m;
				if (dist_prev > 0.40f) break;
			}

			float dist_next = 0.0f;
			uint8_t next_opp_45 = 0;
			for (uint16_t n = i + 1; n < ref_log_count; n++) {
				if (seg_plan_4[n].curve_dir != 0) {
					if (seg_plan_4[n].accel_type >= 2 && seg_plan_4[n].curve_dir != dir_i) {
						next_opp_45 = 1;
					}
					break;
				}
				if (n + 1 < ref_log_count) dist_next += ref_log[n + 1].dist_from_prev_m;
				if (dist_next > 0.40f) break;
			}

			if (prev_opp_45 || next_opp_45) {
				seg_plan_4[i].accel_type = 3;
			}
		}
	}

	// 2c단계: 45도 연속 시케인(type 3) 내부의 짧은 중간 직선 및 시케인 직전/직후의 짧은 연결 직선(< 0.55m)을
	//         시케인 관통 구간(type 3, curve_dir=0)으로 통합하여 불필요한 가감속 리셋 방지 및 제동 거리 확보
	for (uint16_t i = 0; i < ref_log_count; i++) {
		if (seg_plan_4[i].curve_dir == 0 && seg_plan_4[i].accel_type == 1) {
			float seg_len = (i + 1 < ref_log_count) ? ref_log[i + 1].dist_from_prev_m : 0.0f;
			uint8_t prev_is_3 = (i >= 1 && seg_plan_4[i - 1].accel_type == 3);
			uint8_t next_is_3 = (i + 1 < ref_log_count && seg_plan_4[i + 1].accel_type == 3);
			if (prev_is_3 && next_is_3 && seg_len <= 0.55f) {
				seg_plan_4[i].accel_type = 3;
			} else if (prev_is_3 && seg_len < 0.55f && (i + 1 < ref_log_count && seg_plan_4[i + 1].accel_type != 1)) {
				seg_plan_4[i].accel_type = 3;
			} else if (next_is_3 && seg_len < 0.55f && (i >= 1 && seg_plan_4[i - 1].accel_type != 1)) {
				seg_plan_4[i].accel_type = 3;
			}
		}
	}

	// 2d단계: 한국 ZETIN 가상 코리도(±110mm) 기반 시케인/단차 시프트(직-45-45-직) 통합 직선화 및 완만 스침 컷
	for (uint16_t i = 0; i < ref_log_count; i++) {
		seg_plan_4[i].bridge_dir = 0;
		seg_plan_4[i].is_straight_cut = 0;
		seg_plan_4[i].cut_start_idx = 0;
		seg_plan_4[i].cut_first_curve_idx = 0;
		seg_plan_4[i].cut_last_exit_idx = 0;
		seg_plan_4[i].cut_end_idx = 0;
		seg_plan_4[i].cut_straight_dist = 0.0f;
		seg_plan_4[i].cut_angle_deg = 0.0f;
		seg_plan_4[i].cut_seg_x = (i + 1 < ref_log_count) ? ref_log[i + 1].dist_from_prev_m : 0.0f;
		seg_plan_4[i].cut_bulge_offset = 0.0f;
		seg_plan_4[i].cut_bulge_mps = driveData.max_mps;
		seg_plan_4[i].cut_land_blend = 0.0f;
		seg_plan_4[i].cut_lock_dist = 0.0f;
		seg_plan_4[i].cut_exit_yaw_off = 0.0f;
		seg_plan_4[i].is_omega_cut = 0;
		seg_plan_4[i].is_corner90_chained = 0;
		seg_plan_4[i].omega_dir = 0;
		seg_plan_4[i].omega_r_arc = 0.0f;
		seg_plan_4[i].omega_tan1_len = 0.0f;
		seg_plan_4[i].omega_arc_len = 0.0f;
		seg_plan_4[i].omega_tan2_len = 0.0f;
		seg_plan_4[i].omega_tan1_deg = 0.0f;
		seg_plan_4[i].omega_tan2_deg = 0.0f;
		seg_plan_4[i].omega_sweep_deg = 0.0f;
		seg_plan_4[i].omega_arc_mps = driveData.base_mps;
		seg_plan_4[i].is_wave180_cut = 0;
		seg_plan_4[i].wave_count = 0;
		seg_plan_4[i].wave_first_dir = 0;
		seg_plan_4[i].wave_half_len = 0.0f;
		seg_plan_4[i].wave_r_arc = 0.0f;
		seg_plan_4[i].wave_alpha_deg = 0.0f;
		seg_plan_4[i].wave_mps = driveData.base_mps;
	}

	uint16_t k = 0;
	while (k + 2 < ref_log_count) {
		float y1 = ref_log[k + 1].yaw_angle;
		float ay1 = fabsf(y1);

		// 연속 180도 웨이브 (교대 ±180도, 직선 없이 또는 아주 짧게 직결된 N >= 2 반파)
		if (ay1 >= 150.0f && ay1 <= 210.0f &&
		    ref_log[k].type != CROSS_CROSS && ref_log[k].type != CROSS_STOP &&
		    ref_log[k + 1].dist_from_prev_m >= 0.65f && ref_log[k + 1].dist_from_prev_m <= 1.00f) {
			uint8_t w_cnt = 1;
			float prev_wy = y1;
			while (k + w_cnt + 1 < ref_log_count && w_cnt < 30) {
				uint16_t wi = k + w_cnt;
				float wy = ref_log[wi + 1].yaw_angle;
				float awy = fabsf(wy);
				float wd = ref_log[wi + 1].dist_from_prev_m;
				if (ref_log[wi].type == CROSS_CROSS || ref_log[wi].type == CROSS_STOP ||
				    awy < 150.0f || awy > 210.0f || (prev_wy * wy >= 0.0f) ||
				    wd < 0.65f || wd > 1.00f) {
					break;
				}
				prev_wy = wy;
				w_cnt++;
			}
			if (w_cnt >= 2) {
				Wave180_Apply_Plan(k, w_cnt, (y1 > 0.0f) ? 1 : -1);
				k += w_cnt;
				continue;
			}
		}



		// 첫 커브 조건: 18도 ~ 62도 (45도 커브의 주행 중 요 오차 여유 포함)
		if (ay1 >= 18.0f && ay1 <= 62.0f) {
			uint16_t first_curve_idx = k;

			// 0. +-90도 정점 직후 반대 방향 45도 쉐브론 하강 레그(Leg 2) 체크 (entry_dist == 0)
			uint8_t is_apex_leg2 = 0;
			float apex_prev_yaw = (k >= 1) ? ref_log[k].yaw_angle : 0.0f;
			if (k >= 1 && fabsf(apex_prev_yaw) >= 65.0f && fabsf(apex_prev_yaw) <= 115.0f && (apex_prev_yaw * y1 < 0.0f)) {
				is_apex_leg2 = 1;
			}

			// 1. 진입 직선 스캔 (first_curve_idx 직전부터 역방향 탐색)
			uint16_t start_idx = first_curve_idx;
			float entry_dist = 0.0f;
			if (!is_apex_leg2) {
				int p = (int)first_curve_idx - 1;
				while (p >= 0) {
					if (seg_plan_4[p].is_straight_cut) {
						break;
					}
					float pyaw = fabsf(ref_log[p + 1].yaw_angle);
					if (pyaw >= 18.0f || ref_log[p + 1].type == CROSS_STOP || ref_log[p + 1].type == CROSS_CROSS) {
						break;
					}
					entry_dist += ref_log[p + 1].dist_from_prev_m;
					start_idx = (uint16_t)p;
					if (p == 0 || fabsf(ref_log[p].yaw_angle) >= 70.0f) {
						break;
					}
					p--;
				}
			}

			// Ω 루프(+-45 / -+270 / +-45) 우선 판별: 단일 동심 U턴 컷 적용
			float min_omega_entry = (start_idx > 0 && ref_log[start_idx].type == CROSS_CROSS) ? 0.12f : 0.18f;
			if (!is_apex_leg2 && entry_dist >= min_omega_entry && first_curve_idx + 3 < ref_log_count) {
				OmegaCutEval_t o_res = {0};
				if (Omega_Eval_Candidate(start_idx, first_curve_idx, entry_dist, y1, &o_res)) {
					Omega_Apply_Plan(start_idx, first_curve_idx, entry_dist, &o_res);
					k = o_res.corner_idx;
					continue;
				}
			}

			uint8_t is_chained_cut = 0;
			float chained_exit_dist = 0.0f;
			if (!is_apex_leg2 && entry_dist < 0.18f && k >= 1 &&
			    seg_plan_4[k - 1].is_straight_cut && seg_plan_4[k - 1].cut_end_idx == k &&
			    !seg_plan_4[k - 1].is_wave180_cut) {
				float prev_exit_dist = 0.0f;
				uint16_t ple = seg_plan_4[k - 1].cut_last_exit_idx;
				for (uint16_t s = ple; s < k; s++) {
					prev_exit_dist += ref_log[s + 1].dist_from_prev_m;
				}
				prev_exit_dist -= seg_plan_4[k - 1].cut_land_blend;
				if (prev_exit_dist >= 0.14f) {
					is_chained_cut = 1;
					chained_exit_dist = prev_exit_dist;
					start_idx = first_curve_idx;
					entry_dist = 0.0f;
				}
			}
			if (is_apex_leg2 || is_chained_cut || entry_dist >= 0.12f) {
				uint16_t last_curve_idx = 0;
				float tot_yaw = 0.0f;
				uint8_t num_curves = 0;
				uint8_t is_pattern_a = 0;
				uint8_t last_is_split = 0;
				uint8_t is_apex_leg1 = 0;

				// 패턴 A: 3-호 우회 시케인 (+45, -90, +45 또는 -45, +90, -45)
				if (!is_apex_leg2 && first_curve_idx + 3 < ref_log_count) {
					float y2 = ref_log[first_curve_idx + 2].yaw_angle;
					float y3 = ref_log[first_curve_idx + 3].yaw_angle;
					float tot_yaw_3 = y1 + y2 + y3;

					if ((y1 * y2 < 0.0f) && (y1 * y3 > 0.0f) &&
					    (fabsf(y2) >= 65.0f && fabsf(y2) <= 105.0f)) {
						if (fabsf(y3) >= 20.0f && fabsf(y3) <= 60.0f && fabsf(tot_yaw_3) <= 18.0f) {
							last_curve_idx = first_curve_idx + 2;
							tot_yaw = tot_yaw_3;
							num_curves = 3;
							is_pattern_a = 1;
						} else if (first_curve_idx + 4 < ref_log_count &&
						           fabsf(y3) >= 10.0f && fabsf(y3) < 20.0f &&
						           ref_log[first_curve_idx + 3].dist_from_prev_m <= 0.22f) {
							float y4 = ref_log[first_curve_idx + 4].yaw_angle;
							if ((y3 * y4 > 0.0f) && fabsf(y3 + y4) >= 24.0f && fabsf(y3 + y4) <= 60.0f) {
								last_curve_idx = first_curve_idx + 2;
								tot_yaw = 0.0f;
								num_curves = 3;
								is_pattern_a = 1;
								last_is_split = 1;
							}
						}
					}
				}

				// 패턴 B: 2-호 단차 시프트 / 다중-45도 계단(짝수 스텝) / 중간 +-90도 정점 포함 긴 S-시케인 / 완만 범프 아치 / 쉐브론 레그
				#define CUT_MAX_ZC 8
				uint16_t zc_last[CUT_MAX_ZC] = {0};
				float zc_yaw[CUT_MAX_ZC] = {0.0f};
				uint8_t zc_count_arr[CUT_MAX_ZC] = {0};
				uint8_t zc_split[CUT_MAX_ZC] = {0};
				uint8_t zc_mid90[CUT_MAX_ZC] = {0};
				uint8_t zc_len = 0;
				uint8_t ext_has_mid90 = 0;
				uint8_t has_apex1_fb = 0;
				uint16_t even_last = 0;
				float even_yaw = 0.0f;
				uint8_t even_count = 0;
				uint8_t even_split = 0;
				uint16_t odd_last = 0;
				float odd_yaw = 0.0f;
				uint8_t odd_count = 0;
				uint8_t odd_split = 0;
				float hit_apex_cy = 0.0f;
				uint8_t cand_count = (last_curve_idx > 0 && num_curves >= 2) ? 1 : 0;

				if (last_curve_idx == 0) {
					uint16_t cur = first_curve_idx;
					float cur_yaw_sum = 0.0f;
					uint8_t c_count = 0;
					uint16_t last_cand = 0;
					float expect_sign = (y1 > 0.0f) ? 1.0f : -1.0f;
					uint8_t mid90_used = 0;
					float max_internal_straight = is_apex_leg2 ? 0.35f : 0.72f;

					while (cur + 1 < ref_log_count) {
						float cy = ref_log[cur + 1].yaw_angle;
						CrossEvent_t ct = ref_log[cur + 1].type;
						float cd = ref_log[cur + 1].dist_from_prev_m;
						uint8_t split_exit = 0;
						if (fabsf(cy) >= 65.0f || ct == CROSS_STOP || (ct == CROSS_CROSS && c_count == 0)) {
							// 연속 S-시케인 내부의 동방향 45+45(+-90도) 정점은 1회 통과 허용 후 순 각도 0도 복귀 여부 확인
							if (!is_apex_leg2 && !mid90_used && c_count >= 1 &&
							    fabsf(cy) >= 65.0f && fabsf(cy) <= 115.0f &&
							    ct != CROSS_STOP && ct != CROSS_CROSS &&
							    fabsf(cur_yaw_sum) >= 35.0f && fabsf(cur_yaw_sum) <= 55.0f &&
							    (cur_yaw_sum * cy < 0.0f) && cur + 2 < ref_log_count) {
								if (hit_apex_cy == 0.0f) {
									hit_apex_cy = cy;
								}
								float step_s = (cy > 0.0f) ? 90.0f : -90.0f;
								cur_yaw_sum += step_s;
								c_count++;
								last_cand = cur;
								expect_sign = (cy > 0.0f) ? -45.0f : 45.0f;
								mid90_used = 1;
							} else {
								if (hit_apex_cy == 0.0f && fabsf(cy) >= 65.0f && fabsf(cy) <= 115.0f &&
								    ct != CROSS_STOP && ct != CROSS_CROSS) {
									hit_apex_cy = cy;
								}
								break;
							}
						} else if (fabsf(cy) >= 18.0f || (c_count >= 1 && fabsf(cy) >= 14.0f && cd <= 0.25f && (cy * expect_sign > 0.0f))) {
							float step_s = (cy > 0.0f) ? 45.0f : -45.0f;
							cur_yaw_sum += step_s;
							c_count++;
							last_cand = cur;
							expect_sign = -step_s;
						} else if (c_count >= 1 && fabsf(cy) >= 10.0f && cd <= 0.22f && cur + 2 < ref_log_count) {
							// 마지막 45도 호의 탈출 마커 직전에 요 변화가 절반만 기록되고 나머지 절반이 탈출 직선에 걸린 경우
							float cy2 = ref_log[cur + 2].yaw_angle;
							if (cy * cy2 > 0.0f && fabsf(cy + cy2) >= 24.0f && fabsf(cy + cy2) <= 60.0f) {
								float step_s = (cy > 0.0f) ? 45.0f : -45.0f;
								cur_yaw_sum += step_s;
								c_count++;
								last_cand = cur;
								expect_sign = -step_s;
								split_exit = 1;
							} else if (cd > max_internal_straight) {
								break;
							}
						} else if (cd > max_internal_straight) {
							break;
						}
						cur++;
						if (!is_apex_leg2 && c_count >= 2 && fabsf(cur_yaw_sum) <= 5.0f) {
							if (!mid90_used) {
								even_last = last_cand;
								even_yaw = cur_yaw_sum;
								even_count = c_count;
								even_split = split_exit;
							}
							uint16_t ex_next = last_cand + 1 + (split_exit ? 1 : 0);
							if (ex_next + 1 < ref_log_count &&
							    ref_log[ex_next].type != CROSS_STOP && ref_log[ex_next].type != CROSS_CROSS &&
							    fabsf(ref_log[ex_next + 1].yaw_angle) < 18.0f &&
							    ref_log[ex_next + 1].dist_from_prev_m >= 0.16f) {
								if (zc_len < CUT_MAX_ZC && (zc_len == 0 || last_cand != zc_last[zc_len - 1])) {
									zc_last[zc_len] = last_cand;
									zc_yaw[zc_len] = cur_yaw_sum;
									zc_count_arr[zc_len] = c_count;
									zc_split[zc_len] = split_exit;
									zc_mid90[zc_len] = mid90_used;
									zc_len++;
								}
							}
						} else if (!mid90_used && c_count >= 3 && (c_count % 2) == 1 && fabsf(cur_yaw_sum) >= 35.0f && fabsf(cur_yaw_sum) <= 55.0f) {
							odd_last = last_cand;
							odd_yaw = cur_yaw_sum;
							odd_count = c_count;
							odd_split = split_exit;
						}
						if (split_exit) {
							break;
						}
					}

					if (is_apex_leg2) {
						if (odd_last > 0 && (odd_yaw * apex_prev_yaw < 0.0f)) {
							last_curve_idx = odd_last;
							tot_yaw = odd_yaw;
							num_curves = odd_count;
							last_is_split = odd_split;
							cand_count = 1;
						}
					} else {
						has_apex1_fb = (even_last > 0 && odd_last > 0 &&
						                fabsf(hit_apex_cy) >= 65.0f && (odd_yaw * hit_apex_cy < 0.0f) &&
						                (zc_len == 0 || even_last != zc_last[zc_len - 1])) ? 1 : 0;
						cand_count = zc_len + (has_apex1_fb ? 1 : 0);
					}
				}

				uint8_t cut_applied = 0;
				for (uint8_t cand = 0; cand < cand_count; cand++) {
					uint8_t zi = 0;
					if (!is_pattern_a && !is_apex_leg2) {
						is_apex_leg1 = 0;
						ext_has_mid90 = 0;
						if (cand < zc_len) {
							zi = zc_len - 1 - cand;
							last_curve_idx = zc_last[zi];
							tot_yaw = zc_yaw[zi];
							num_curves = zc_count_arr[zi];
							last_is_split = zc_split[zi];
							ext_has_mid90 = zc_mid90[zi];
						} else if (has_apex1_fb) {
							last_curve_idx = even_last;
							tot_yaw = even_yaw;
							num_curves = even_count;
							last_is_split = even_split;
							is_apex_leg1 = 1;
						} else {
							continue;
						}
					}

					CutEval_t res = {0};
					if (!Cut_Eval_Candidate(start_idx, first_curve_idx, entry_dist, y1,
					                        last_curve_idx, num_curves, last_is_split, ext_has_mid90,
					                        is_pattern_a, is_apex_leg1, is_apex_leg2, is_chained_cut, &res)) {
						continue;
					}

					// 두 순수 직선 컷(bulge == 0)이 0.35m 이상 직선을 사이에 두고 불필요하게 큰 bulge로 병합되는 것을 방지
					if (!is_pattern_a && !is_apex_leg1 && !is_apex_leg2 &&
					    cand < zc_len && zi > 0 && res.bulge_offset != 0.0f) {
						uint8_t split_clean = 0;
						for (int sj = (int)zi - 1; sj >= 0; sj--) {
							CutEval_t sub1 = {0};
							if (Cut_Eval_Candidate(start_idx, first_curve_idx, entry_dist, y1,
							                       zc_last[sj], zc_count_arr[sj], zc_split[sj], zc_mid90[sj],
							                       0, 0, 0, is_chained_cut, &sub1) &&
							    sub1.bulge_offset == 0.0f && sub1.exit_dist >= 0.35f) {
								if (Cut_Tail_Max_Dev(sub1.corner_idx, last_curve_idx) <= 0.060f) {
									split_clean = 1;
									break;
								}
							}
						}
						if (split_clean) {
							continue;
						}
					}

					Cut_Apply_Plan(start_idx, first_curve_idx, last_curve_idx, is_pattern_a, y1, &res);
					k = res.corner_idx;
					cut_applied = 1;
					break;
				}
					if (!cut_applied && !is_apex_leg2) {
						uint16_t c45_end = 0;
						if (!is_chained_cut && entry_dist >= 0.16f) {
							c45_end = Corner45_Try_Apply(start_idx, first_curve_idx, entry_dist, y1, 0);
						} else if (is_chained_cut && k >= 1 &&
						           seg_plan_4[k - 1].is_omega_cut &&
						           fabsf(seg_plan_4[k - 1].cut_exit_yaw_off) <= 60.0f &&
						           chained_exit_dist >= 0.22f) {
							c45_end = Corner45_Try_Apply(first_curve_idx, first_curve_idx, chained_exit_dist, y1, 1);
						}
						if (c45_end > 0) {
							k = c45_end;
							continue;
						}
					}
					if (cut_applied) {
						continue;
					}
				}
			}
		k++;
	}

	// 2e단계: 동일 방향 연속 45도(45도 - 짧은 직선 <= 0.48m - 45도) IN-코리도 브릿징 (accel_type = 5)
	//         OUT 스윙 없이 인코스만 연결하여 유효 반경 R_eff 확대 및 중간 감속 리셋 방지
	for (uint16_t i = 0; i + 2 < ref_log_count; i++) {
		if (seg_plan_4[i].curve_dir != 0 && seg_plan_4[i].accel_type == 2) {
			uint8_t dir_i = seg_plan_4[i].curve_dir;
			float s_mid = 0.0f;
			uint16_t j = i + 1;
			uint8_t found_same = 0;
			while (j < ref_log_count) {
				if (seg_plan_4[j].accel_type == 4 || ref_log[j].type == CROSS_STOP) break;
				if (seg_plan_4[j].curve_dir != 0) {
					if (seg_plan_4[j].curve_dir == dir_i && seg_plan_4[j].accel_type == 2 && s_mid > 0.01f && s_mid <= 0.48f) {
						found_same = 1;
					}
					break;
				}
				if (j + 1 < ref_log_count) s_mid += ref_log[j + 1].dist_from_prev_m;
				if (s_mid > 0.48f) break;
				j++;
			}
			if (found_same) {
				for (uint16_t m = i; m <= j; m++) {
					seg_plan_4[m].accel_type = 5;
					seg_plan_4[m].bridge_dir = dir_i;
				}
			}
		}
	}
}

// 4차 주행 초기화
__STATIC_INLINE uint8_t Drive_Fourth_Init_Sequence(void) {
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
		if (ref_log_count == 0) return 0;
		for (uint16_t i = 0; i < ref_log_count; i++) ref_log[i] = g_cross_log[i];
		Build_Segment_Plan_Fourth();
		is_ref_log_copied = 1;
	} else {
		if (ref_log_count == 0) return 0;
		Build_Segment_Plan_Fourth();
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
	Drive_Diag_Reset(4);
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

// 4차 주행 가속 구간 길이 및 진입/탈출 목표 속도 산출 (컷/크로스 경계 분할)
static void Fourth_Plan_Accel_Segment(uint16_t start_idx, float *out_dist, float *out_target_mps, float *out_end_mps) {
	const uint8_t type = seg_plan_4[start_idx].accel_type;
	float total_dist = 0.0f;
	uint8_t next_type = 0;
	uint16_t next_idx = ref_log_count;

	for (uint16_t i = start_idx; i < ref_log_count; i++) {
		if (seg_plan_4[i].accel_type != type ||
		    (i > start_idx && seg_plan_4[i].is_straight_cut && seg_plan_4[i].cut_start_idx == i)) {
			next_type = seg_plan_4[i].accel_type;
			next_idx = i;
			break;
		}
		if (i + 1 < ref_log_count) {
			total_dist += ref_log[i + 1].dist_from_prev_m;
		} else {
			break;
		}
	}

	float target_mps = Segment_Mps_Fourth(type);
	float end_mps = Segment_Mps_Fourth(next_type);
	if (type == 1 && next_type == 0) {
		float min_c = Fourth_Corner_Mps();
		if (total_dist < 0.60f) {
			target_mps = min_c;
		} else if (total_dist < 1.35f) {
			float short_cap = min_c + (total_dist - 0.60f) * 2.0f;
			if (short_cap < target_mps) {
				target_mps = short_cap;
			}
		}
	}
	if (next_idx < ref_log_count && seg_plan_4[next_idx].is_omega_cut) {
		float v_arc = seg_plan_4[next_idx].omega_arc_mps;
		float d_in = seg_plan_4[next_idx].cut_lock_dist + seg_plan_4[next_idx].omega_tan1_len;
		float margin_b = ((driveData.brake_margin_m > 0.0f) ? driveData.brake_margin_m : BRAKE_MARGIN_M) + 0.10f;
		float d_eff = d_in - margin_b - v_arc * 0.06f;
		if (d_eff < 0.0f) d_eff = 0.0f;
		float v_cap = sqrtf(v_arc * v_arc + 2.0f * (driveData.decel * 0.78f) * d_eff);
		if (v_cap < end_mps) end_mps = v_cap;
	}

	float min_end = Fourth_Corner_Mps();
	if (end_mps < min_end) end_mps = min_end;
	if (end_mps > target_mps) end_mps = target_mps;

	*out_dist = total_dist;
	*out_target_mps = target_mps;
	*out_end_mps = end_mps;
}

// 4차 주행 마커 이벤트 및 가속 구간 계산
__STATIC_INLINE uint8_t Process_Marker_Event_Fourth(CrossEvent_t cross, DriveFourthState_t *state) {
	float eff_odom = g_total_odom_m + state->odom_offset_m;

	if (cross == CROSS_STOP) {
		uint8_t stop_res = Handle_Stop_Marker_Common(state->idx, eff_odom, &cross);
		if (stop_res == 1) return 1;
		if (stop_res == 2) return 0;
	}

	if (cross == CROSS_LEFT) g_total_L++;
	else if (cross == CROSS_RIGHT) g_total_R++;
	else if (cross == CROSS_CROSS) g_total_C++;

	if (state->in_straight_cut) {
		float traveled = g_total_odom_m - state->cut_start_odom;

		float last_exit_dist = Cut_Milestone_Dist(state->cut_start_idx, state->cut_last_exit_idx);
		// 탈출 직선 이후 교차로(CROSS_CROSS) 물리 감지 시 코너 마커 미검출 상태여도 즉시 Straight-Cut 종료 후 재동기화
		if (state->idx < state->cut_end_idx && !(cross == CROSS_CROSS && traveled >= last_exit_dist)) {
			if (cross == ref_log[state->idx].type) {
				state->idx++;
			}
			return 0;
		} else {
			// 탈출 직선 끝의 90도 이상 급커브 진입 마커(cut_end_idx) 또는 교차로를 물리적으로 정상 인식!
			// Straight-Cut 모드를 즉시 해제하고 아래의 일반 마커 처리 로직으로 폴스루하여 코너 감속/조향 진입
			state->in_straight_cut = 0;
			state->is_omega_cut = 0;
			state->is_wave180_cut = 0;
			g_straight_cut_active = 0;
			g_suppress_marker_detect = 0;
			g_straight_cut_kappa_ref = 0.0f;
		}
	}

	// 1. 기대 마커 타입 불일치 시 즉시 mismatch 판정 후 현 마커에서 재동기화 시도
	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	// 2. 교차로(CROSS_CROSS) 우선 재동기화 + 시퀀스/벡터 기반 복구
	Resync_Marker_Common(cross, &state->idx, &state->mismatch,
	                     &state->odom_offset_m, state->imu_grid_bias, eff_odom);

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	// 숏컷 진입 판단:
	// 진입 시작 마커(cut_start_idx)에서만 숏컷을 개시하며, 마커 타입 일치 + 헤딩 정합(ref_pose_th 대비 <= 25도)을
	// 필수 검증해 직전 마커 누락 시 다음 마커에서 45도 오정렬된 방향으로 반대 꺾임하는 현상을 원천 차단
	float cut_corr_yaw = Wrap_Angle_Deg(imu_data.Yaw_Angle - state->imu_grid_bias);
	float cut_yaw_err = (state->idx < ref_log_count)
	                        ? fabsf(Wrap_Angle_Deg(cut_corr_yaw - ref_pose_th[state->idx]))
	                        : 180.0f;

	if (state->idx < ref_log_count &&
	    ref_log[state->idx].type == cross &&
	    seg_plan_4[state->idx].is_straight_cut &&
	    state->idx == seg_plan_4[state->idx].cut_start_idx) {
		if (state->mismatch) {
			float d_err = fabsf(eff_odom - ref_log_cum_dist[state->idx]);
			float line_err = fabsf(Sensor_Get_Last_Position());
			if (d_err <= 0.35f && line_err < 0.20f && cut_yaw_err <= 22.0f) {
				state->mismatch = 0;
			}
		}
	}

	if (!state->mismatch && state->idx < ref_log_count &&
	    ref_log[state->idx].type == cross &&
	    cut_yaw_err <= 22.0f &&
	    seg_plan_4[state->idx].is_straight_cut &&
	    state->idx == seg_plan_4[state->idx].cut_start_idx) {
		state->odom_offset_m = ref_log_cum_dist[state->idx] - g_total_odom_m;
		uint8_t prev_type = (state->idx >= 1) ? seg_plan_4[state->idx - 1].accel_type : 1;
		uint8_t prev_curve = (state->idx >= 1) ? seg_plan_4[state->idx - 1].curve_dir : 0;

		state->in_straight_cut = 1;
		state->cut_p3_preshift_dir = 0;
		state->cut_p3_preshift_corner = 0;
		g_straight_cut_active = 0;      // 진입 직선 정렬 완료 후 첫 45도 커브 직전에 활성화
		g_suppress_marker_detect = 0;   // 진입 직선 정렬 중에는 물리 감지 허용 (첫 45도 커브 진입 시 1로 변경)

		// 진입 직선에서 정확한 격자 방위각(grid_yaw) 및 사선 목표각 사전 산출
		float corr_yaw = imu_data.Yaw_Angle - state->imu_grid_bias;
		float grid_yaw = roundf(corr_yaw / 45.0f) * 45.0f;
		if (grid_yaw > 180.0f) grid_yaw -= 360.0f;
		else if (grid_yaw < -180.0f) grid_yaw += 360.0f;

		float target_yaw = grid_yaw + seg_plan_4[state->idx].cut_angle_deg + state->imu_grid_bias;
		if (target_yaw > 180.0f) target_yaw -= 360.0f;
		else if (target_yaw < -180.0f) target_yaw += 360.0f;

		g_straight_cut_yaw_ref = target_yaw;
		g_straight_cut_kappa_ref = 0.0f;

		IR_Sensor.data->target_pos = 0.0f;
		Steer_Clear_Target_Shift();
		state->accel_active = 1;
		state->cut_start_idx = seg_plan_4[state->idx].cut_start_idx;
		state->cut_first_curve_idx = seg_plan_4[state->idx].cut_first_curve_idx;
		state->cut_last_exit_idx = seg_plan_4[state->idx].cut_last_exit_idx;
		state->cut_end_idx = seg_plan_4[state->idx].cut_end_idx;
		state->cut_start_odom = g_total_odom_m;
		state->cut_straight_dist = seg_plan_4[state->idx].cut_straight_dist;
		state->cut_angle_deg = seg_plan_4[state->idx].cut_angle_deg;
		state->cut_base_yaw_ref = target_yaw;
		state->cut_bulge_offset = seg_plan_4[state->idx].cut_bulge_offset;
		state->cut_bulge_mps = seg_plan_4[state->idx].cut_bulge_mps;
		state->cut_bulge_warp = seg_plan_4[state->idx].cut_bulge_warp;
		state->cut_land_blend = seg_plan_4[state->idx].cut_land_blend;
		state->cut_lock_dist = seg_plan_4[state->idx].cut_lock_dist;
		state->cut_exit_yaw_off = seg_plan_4[state->idx].cut_exit_yaw_off;
		state->is_omega_cut = seg_plan_4[state->idx].is_omega_cut;
		state->omega_dir = seg_plan_4[state->idx].omega_dir;
		state->omega_r_arc = seg_plan_4[state->idx].omega_r_arc;
		state->omega_tan1_len = seg_plan_4[state->idx].omega_tan1_len;
		state->omega_arc_len = seg_plan_4[state->idx].omega_arc_len;
		state->omega_tan2_len = seg_plan_4[state->idx].omega_tan2_len;
		state->omega_tan1_deg = seg_plan_4[state->idx].omega_tan1_deg;
		state->omega_tan2_deg = seg_plan_4[state->idx].omega_tan2_deg;
		state->omega_sweep_deg = seg_plan_4[state->idx].omega_sweep_deg;
		state->omega_arc_mps = seg_plan_4[state->idx].omega_arc_mps;
		state->is_wave180_cut = seg_plan_4[state->idx].is_wave180_cut;
		state->wave_count = seg_plan_4[state->idx].wave_count;
		state->wave_first_dir = seg_plan_4[state->idx].wave_first_dir;
		state->wave_half_len = seg_plan_4[state->idx].wave_half_len;
		state->wave_r_arc = seg_plan_4[state->idx].wave_r_arc;
		state->wave_alpha_deg = seg_plan_4[state->idx].wave_alpha_deg;
		state->wave_mps = seg_plan_4[state->idx].wave_mps;
		state->cut_p1_latched = (state->cut_lock_dist <= 0.03f) ? 1 : 0; // 정점 직후 즉시 진입 레그는 진입 마커에서 격자각 즉시 확정
		state->seg_len_predicted = state->cut_straight_dist;
		state->target_mps = driveData.max_mps;
		if (state->is_wave180_cut) {
			state->target_mps = state->wave_mps;
			state->end_mps = Fourth_Corner_Mps();
		} else if (state->is_omega_cut && fabsf(state->cut_exit_yaw_off) <= 60.0f &&
		           state->cut_end_idx < ref_log_count &&
		           seg_plan_4[state->cut_end_idx].is_corner90_chained) {
			state->end_mps = seg_plan_4[state->cut_end_idx].omega_arc_mps;
		} else if (!state->is_omega_cut && !state->is_wave180_cut &&
		           state->cut_end_idx < ref_log_count &&
		           seg_plan_4[state->cut_end_idx].is_straight_cut &&
		           state->cut_end_idx == seg_plan_4[state->cut_end_idx].cut_start_idx &&
		           ref_log[state->cut_end_idx].type != CROSS_STOP) {
			float next_ca = fabsf(seg_plan_4[state->cut_end_idx].cut_angle_deg);
			float trans_mps = (next_ca <= 10.0f) ? (Fourth_Corner_Mps() + 1.50f)
			                : ((next_ca <= 22.0f) ? (Fourth_Corner_Mps() + 0.70f) : Fourth_Corner_Mps());
			if (trans_mps > driveData.max_mps) trans_mps = driveData.max_mps;
			state->end_mps = trans_mps;
		} else if (state->cut_end_idx < ref_log_count &&
		           ref_log[state->cut_end_idx].type == CROSS_CROSS &&
		           seg_plan_4[state->cut_end_idx].curve_dir == 0 &&
		           seg_plan_4[state->cut_end_idx].accel_type > 0) {
			float v_allow = Fourth_Corner_Mps();
			if (seg_plan_4[state->cut_end_idx].is_omega_cut &&
			    state->cut_end_idx == seg_plan_4[state->cut_end_idx].cut_start_idx) {
				float v_arc = seg_plan_4[state->cut_end_idx].omega_arc_mps;
				float d_in = seg_plan_4[state->cut_end_idx].cut_lock_dist + seg_plan_4[state->cut_end_idx].omega_tan1_len;
				float margin_b = ((driveData.brake_margin_m > 0.0f) ? driveData.brake_margin_m : BRAKE_MARGIN_M) + 0.10f;
				float d_eff = d_in - margin_b - v_arc * 0.06f;
				if (d_eff < 0.0f) d_eff = 0.0f;
				v_allow = sqrtf(v_arc * v_arc + 2.0f * (driveData.decel * 0.78f) * d_eff);
				if (v_allow > driveData.max_mps) v_allow = driveData.max_mps;
			} else {
				float post_dist = 0.0f, post_target = 0.0f, post_end = 0.0f;
				Fourth_Plan_Accel_Segment(state->cut_end_idx, &post_dist, &post_target, &post_end);
				if (seg_plan_4[state->cut_end_idx].is_straight_cut &&
				    seg_plan_4[state->cut_end_idx].cut_bulge_offset != 0.0f &&
				    seg_plan_4[state->cut_end_idx].cut_bulge_mps < post_target) {
					post_target = seg_plan_4[state->cut_end_idx].cut_bulge_mps;
				}
				float margin_b = ((driveData.brake_margin_m > 0.0f) ? driveData.brake_margin_m : BRAKE_MARGIN_M) + 0.10f;
				float d_eff = post_dist - margin_b - post_end * 0.06f;
				if (d_eff < 0.0f) d_eff = 0.0f;
				v_allow = sqrtf(post_end * post_end + 2.0f * (driveData.decel * 0.78f) * d_eff);
				if (v_allow > post_target) v_allow = post_target;
			}
			float min_c = Fourth_Corner_Mps();
			if (v_allow < min_c) v_allow = min_c;
			state->end_mps = v_allow;
		} else if (state->is_omega_cut && fabsf(state->cut_exit_yaw_off) > 90.0f && state->cut_end_idx < ref_log_count &&
		    seg_plan_4[state->cut_end_idx].accel_type > 0 &&
		    ref_log[state->cut_end_idx].type != CROSS_STOP) {
			float next_mps = Segment_Mps_Fourth(seg_plan_4[state->cut_end_idx].accel_type);
			if (seg_plan_4[state->cut_end_idx].accel_type == 1 || seg_plan_4[state->cut_end_idx].accel_type == 4) {
				float post_dist = 0.0f, post_target = 0.0f, post_end = 0.0f;
				Fourth_Plan_Accel_Segment(state->cut_end_idx, &post_dist, &post_target, &post_end);
				float margin_b = ((driveData.brake_margin_m > 0.0f) ? driveData.brake_margin_m : BRAKE_MARGIN_M) + 0.10f;
				float d_eff = post_dist - margin_b - post_end * 0.06f;
				if (d_eff < 0.0f) d_eff = 0.0f;
				float v_allow = sqrtf(post_end * post_end + 2.0f * (driveData.decel * 0.78f) * d_eff);
				if (v_allow > post_target) v_allow = post_target;
				if (v_allow < next_mps) next_mps = v_allow;
			}
			float min_c = Fourth_Corner_Mps();
			if (next_mps < min_c) next_mps = min_c;
			state->end_mps = next_mps;
		} else {
			state->end_mps = driveData.base_mps;
		}

		float init_cut_mps = (!state->is_omega_cut && state->cut_bulge_offset != 0.0f)
		                         ? state->cut_bulge_mps
		                         : state->target_mps;

		if (!state->is_wave180_cut && prev_curve != 0 && prev_type == 0) {
			state->corridor_start_dist = g_total_odom_m;
			state->zone_start_from_deep = 1;
			g_target_base_mps = driveData.base_mps;
		} else {
			state->corridor_start_dist = g_total_odom_m;
			state->zone_start_from_deep = 0;
			g_target_base_mps = init_cut_mps;
		}
		g_is_straight = 1;
		state->marker_start_dist = g_odom_distance_m;
		state->braking_started = 0;
		state->omega_braking_started = 0;
		state->idx++;
		return 0;
	}

	if (!state->mismatch && state->idx < ref_log_count && seg_plan_4[state->idx].accel_type > 0) {
		state->accel_active = 1;
		const uint8_t type = seg_plan_4[state->idx].accel_type;
		Fourth_Plan_Accel_Segment(state->idx, &state->seg_len_predicted, &state->target_mps, &state->end_mps);

		uint8_t prev_type = (state->idx >= 1) ? seg_plan_4[state->idx - 1].accel_type : 1;
		uint8_t prev_curve = (state->idx >= 1) ? seg_plan_4[state->idx - 1].curve_dir : 0;
		float_t cur_yaw_rate = fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset);
		float_t cur_line_err = fabsf(Sensor_Get_Last_Position());

		if (state->idx == 0) {
			state->corridor_start_dist = g_odom_distance_m;
			state->zone_start_from_deep = 0;
			g_target_base_mps = state->target_mps;
		} else if ((prev_curve != 0 && prev_type == 0) ||
		           (type == 1 && (prev_curve != 0 || (cross != CROSS_CROSS && (cur_yaw_rate >= 35.0f || cur_line_err >= 0.18f))))) {
			// 커브 탈출 직후 또는 잔여 요레이트/횡오차가 큰 상태: 차체 정렬 완료까지 코너 안정 속도 유지 후 가속
			state->corridor_start_dist = g_odom_distance_m;
			state->zone_start_from_deep = 1;
			g_target_base_mps = Fourth_Corner_Mps();
		} else if (prev_curve != 0 && prev_type == 2 && type == 1) {
			// 단방향/독립 45도 코너 탈출 직후: 슬라롬 속도 유지 후 마진 경과 시 직선 풀가속
			state->corridor_start_dist = g_odom_distance_m;
			state->zone_start_from_deep = 0;
			g_target_base_mps = Segment_Mps_Fourth(2);
		} else {
			// 직선(type 1) 또는 45도 연속 시케인(type 3) 내부/전후 연속 주행:
			// 마커 통과 시 base_mps 리셋이나 0.25m 대기 없이 즉시 목표 가속 유지
			if (!state->zone_start_from_deep || (g_odom_distance_m - state->corridor_start_dist) >= ACCEL_START_MARGIN_M) {
				state->zone_start_from_deep = 0;
				g_target_base_mps = state->target_mps;
			}
		}
		g_is_straight = (seg_plan_4[state->idx].curve_dir == 0 && seg_plan_4[state->idx].accel_type != 5) ? 1 : 0;

	} else {
		float c_mps = Fourth_Corner_Mps();
		state->accel_active = 0;
		state->zone_start_from_deep = 0;
		state->target_mps = c_mps;
		state->end_mps = c_mps;
		g_target_base_mps = c_mps;
		state->corridor_start_dist = g_odom_distance_m;
		if (state->idx < ref_log_count && seg_plan_4[state->idx].curve_dir == 0) {
			g_is_straight = 1;
		} else {
			g_is_straight = 0;
		}
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;
	state->omega_braking_started = 0;

	if (!state->mismatch) {
		if (state->idx < ref_log_count) {
			state->odom_offset_m = ref_log_cum_dist[state->idx] - g_total_odom_m;
		}
		state->idx++;
	}

	return 0;
}

// 4차 주행 전용 가상 마커 추정기 (시케인 직선 가로지르기 중 기하학적 투영 거리 기반 가상 마커 합성 및 상태 전환)
__STATIC_INLINE void Check_Virtual_Markers_Fourth(DriveFourthState_t *state) {
	if (!state->in_straight_cut) return;

	float traveled = g_total_odom_m - state->cut_start_odom;

	// 1. 시케인 내부 마커(cut_first_curve_idx .. cut_last_exit_idx)는 기하학적 투영 거리 도달 시 가상 합성
	while (state->idx <= state->cut_last_exit_idx && state->idx < ref_log_count) {
		float milestone = Cut_Milestone_Dist(state->cut_start_idx, state->idx);
		if (traveled >= milestone) {
			CrossEvent_t vtype = ref_log[state->idx].type;
			if (vtype == CROSS_LEFT) g_total_L++;
			else if (vtype == CROSS_RIGHT) g_total_R++;
			else if (vtype == CROSS_CROSS) g_total_C++;
			Sensor_Log_Virtual_Marker(vtype, ref_log[state->idx].yaw_angle);
			state->idx++;
		} else {
			break;
		}
	}

	// 2. 코너 마커(cut_end_idx) 안전 워치독:
	//    물리 마커 미검출 상태라도 직선 종료 거리를 넘기거나 탈출 직선 끝단에서 명백한 코너 선회(|dyaw| >= 24도 또는 예상 방향 선회)가 시작되면 해제
	float phase2_end_dist = Cut_Milestone_Dist(state->cut_start_idx, state->cut_last_exit_idx) + state->cut_land_blend;
	// 탈출 직선(Phase 3)의 기준 방위각은 사선 각도(cut_angle_deg)를 제외하고 순 각도 오프셋(cut_exit_yaw_off)을 반영한 격자 직선 각도
	float exit_yaw_ref = Wrap_Angle_Deg(state->cut_base_yaw_ref - state->cut_angle_deg + state->cut_exit_yaw_off);
	float yaw_diff = Wrap_Angle_Deg(imu_data.Yaw_Angle - exit_yaw_ref);
	if (state->is_omega_cut && fabsf(state->cut_exit_yaw_off) <= 60.0f &&
	    state->idx == state->cut_end_idx && state->cut_end_idx < ref_log_count &&
	    seg_plan_4[state->cut_end_idx].is_corner90_chained) {
		uint16_t ni = state->cut_end_idx;
		float next_lead = seg_plan_4[ni].cut_land_blend;
		if (traveled >= state->cut_straight_dist - next_lead) {
			CrossEvent_t vtype = ref_log[ni].type;
			if (vtype == CROSS_LEFT) g_total_L++;
			else if (vtype == CROSS_RIGHT) g_total_R++;
			else if (vtype == CROSS_CROSS) g_total_C++;
			Sensor_Log_Virtual_Marker(vtype, ref_log[ni].yaw_angle);
			state->odom_offset_m = ref_log_cum_dist[ni] - g_total_odom_m;
			state->idx++;
			state->cut_start_idx = seg_plan_4[ni].cut_start_idx;
			state->cut_first_curve_idx = seg_plan_4[ni].cut_first_curve_idx;
			state->cut_last_exit_idx = seg_plan_4[ni].cut_last_exit_idx;
			state->cut_end_idx = seg_plan_4[ni].cut_end_idx;
			state->cut_start_odom = g_total_odom_m;
			state->cut_straight_dist = seg_plan_4[ni].cut_straight_dist;
			state->cut_angle_deg = 0.0f;
			state->cut_base_yaw_ref = exit_yaw_ref;
			state->cut_p1_latched = 1;
			state->cut_lock_dist = 0.0f;
			state->cut_land_blend = next_lead;
			state->cut_exit_yaw_off = seg_plan_4[ni].cut_exit_yaw_off;
			state->omega_dir = seg_plan_4[ni].omega_dir;
			state->omega_r_arc = seg_plan_4[ni].omega_r_arc;
			state->omega_tan1_len = 0.0f;
			state->omega_arc_len = seg_plan_4[ni].omega_arc_len;
			state->omega_tan2_len = 0.0f;
			state->omega_tan1_deg = 0.0f;
			state->omega_tan2_deg = seg_plan_4[ni].omega_tan2_deg;
			state->omega_sweep_deg = 45.0f;
			state->omega_arc_mps = seg_plan_4[ni].omega_arc_mps;
			state->seg_len_predicted = state->cut_straight_dist;
			state->target_mps = driveData.max_mps;
			state->end_mps = (state->cut_end_idx < ref_log_count && seg_plan_4[state->cut_end_idx].is_corner90_chained)
			                     ? seg_plan_4[state->cut_end_idx].omega_arc_mps
			                     : driveData.base_mps;
			state->braking_started = 0;
			state->omega_braking_started = 0;
			g_straight_cut_active = 1;
			g_suppress_marker_detect = 1;
			return;
		}
	}
	uint8_t reading_now = Cross_Is_Reading();
	uint8_t end_is_cross = (state->cut_end_idx < ref_log_count && ref_log[state->cut_end_idx].type == CROSS_CROSS) ? 1 : 0;
	float fallback_margin = end_is_cross ? 0.35f : (reading_now ? 0.09f : 0.05f);
	uint8_t next_dir = (state->cut_end_idx < ref_log_count) ? seg_plan_4[state->cut_end_idx].curve_dir : 0;
	float dir_sign = (next_dir == 1) ? 1.0f : ((next_dir == 2) ? -1.0f : 0.0f);
	float gyro_z = imu_data.Gyro_Z - imu_data.Gyro_Z_Offset;
	uint8_t dir_turn_start = (dir_sign != 0.0f && traveled >= phase2_end_dist &&
	                          traveled >= state->cut_straight_dist - 0.01f &&
	                          (yaw_diff * dir_sign) >= 6.0f && (gyro_z * dir_sign) >= 60.0f) ? 1 : 0;
	uint8_t zero_exit_done = (state->cut_end_idx <= state->cut_last_exit_idx && traveled >= phase2_end_dist) ? 1 : 0;
	float corner_yaw_err = (dir_sign != 0.0f) ? (yaw_diff * dir_sign) : fabsf(yaw_diff);
	if (zero_exit_done || traveled >= state->cut_straight_dist + fallback_margin || dir_turn_start ||
	    (!end_is_cross && traveled >= phase2_end_dist && traveled >= state->cut_straight_dist - 0.04f && corner_yaw_err >= 24.0f &&
	     (!reading_now || corner_yaw_err >= 32.0f))) {
		if (zero_exit_done && state->cut_end_idx < ref_log_count) {
			state->odom_offset_m = ref_log_cum_dist[state->cut_end_idx] - g_total_odom_m;
		}
		if (state->idx == state->cut_end_idx && state->idx < ref_log_count) {
			CrossEvent_t vtype = ref_log[state->idx].type;
			if (vtype == CROSS_LEFT) g_total_L++;
			else if (vtype == CROSS_RIGHT) g_total_R++;
			else if (vtype == CROSS_CROSS) g_total_C++;
			Sensor_Log_Virtual_Marker(vtype, ref_log[state->idx].yaw_angle);
			state->odom_offset_m = ref_log_cum_dist[state->idx] - g_total_odom_m;
			state->idx++;
		}
		state->in_straight_cut = 0;
		state->is_omega_cut = 0;
		g_straight_cut_active = 0;
		g_suppress_marker_detect = 0;
		g_straight_cut_kappa_ref = 0.0f;
		g_is_straight = 0;
		float c_mps = Fourth_Corner_Mps();
		state->accel_active = 0;
		state->braking_started = 0;
		state->omega_braking_started = 0;
		state->zone_start_from_deep = 0;
		state->target_mps = c_mps;
		state->end_mps = c_mps;
		g_target_base_mps = c_mps;
		state->marker_start_dist = g_odom_distance_m;
	}
}

// 4차 주행 감속 및 가속 실행 제어
__STATIC_INLINE void Check_Distance_And_Brake_Fourth(DriveFourthState_t *state) {
	if (state->accel_active) {
		if (state->braking_started) {
			g_target_base_mps = state->end_mps;
			return;
		}
		float_t traveled = state->in_straight_cut
				? (g_total_odom_m - state->cut_start_odom)
				: (g_odom_distance_m - state->marker_start_dist);
		float_t remaining = state->seg_len_predicted - traveled;

		float_t v1 = g_current_base_mps;
		float_t v2 = state->end_mps;

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
			g_target_base_mps = state->end_mps;
			state->braking_started = 1;
		} else {
			float_t desired_mps = state->target_mps;
			if (state->in_straight_cut && state->is_omega_cut) {
				float_t tan1_end = state->cut_lock_dist + state->omega_tan1_len;
				float_t arc_end = tan1_end + state->omega_arc_len;
				if (traveled < tan1_end) {
					if (state->omega_braking_started) {
						desired_mps = state->omega_arc_mps;
					} else {
						float_t rem_to_arc = tan1_end - traveled;
						float_t bdist = ((driveData.brake_margin_m > 0.0f) ? driveData.brake_margin_m : BRAKE_MARGIN_M) + 0.06f;
						if (v1 > state->omega_arc_mps) {
							bdist += (v1 * v1 - state->omega_arc_mps * state->omega_arc_mps) / (2.0f * driveData.decel * 0.80f) + v1 * 0.055f;
							if (v1 > 3.5f) bdist += (v1 - 3.5f) * 0.15f + 0.10f;
						}
						if (rem_to_arc <= bdist) {
							state->omega_braking_started = 1;
							desired_mps = state->omega_arc_mps;
						}
					}
				} else if (traveled < arc_end) {
					desired_mps = state->omega_arc_mps;
				}
			} else if (state->in_straight_cut && state->cut_bulge_offset != 0.0f) {
				float last_exit_dist = Cut_Milestone_Dist(state->cut_start_idx, state->cut_last_exit_idx);
				if (traveled < last_exit_dist && state->cut_bulge_mps < desired_mps) {
					desired_mps = state->cut_bulge_mps;
				}
			}

			float_t corridor_traveled = state->in_straight_cut
					? (g_total_odom_m - state->corridor_start_dist)
					: (g_odom_distance_m - state->corridor_start_dist);
			float_t yaw_rate = fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset);
			float_t line_err = fabsf(Sensor_Get_Last_Position());
			uint8_t is_settled;
			if (state->in_straight_cut) {
				is_settled = !state->zone_start_from_deep ||
				             ((corridor_traveled >= 0.10f) && ((corridor_traveled >= 0.18f) || (yaw_rate < 55.0f && line_err < 0.25f)));
			} else {
				is_settled = !state->zone_start_from_deep
				             ? ((corridor_traveled >= ACCEL_START_MARGIN_M) || (g_target_base_mps == desired_mps))
				             : ((corridor_traveled >= 0.12f) &&
				                ((corridor_traveled >= 0.35f) || (yaw_rate < 35.0f && line_err < 0.15f)));
			}

			if (is_settled) {
				state->zone_start_from_deep = 0;
				g_target_base_mps = desired_mps;
				if (state->in_straight_cut || (state->idx >= 1 && state->idx - 1 < ref_log_count && seg_plan_4[state->idx - 1].curve_dir == 0)) {
					g_is_straight = 1;
				}
			}
		}
	}
}

// 4차 주행 유효 target_shift_val 반환 (하한 데드밴드 및 제어 상한 적용)
__STATIC_INLINE float Drive4_Effective_Shift(void) {
	float s = driveData.target_shift_val;
	if (s < 0.02f) return 0.0f;
	if (s > DRIVE4_IN_MARKER_SHIFT_MAX) return DRIVE4_IN_MARKER_SHIFT_MAX;
	return s;
}

// 직선 중심 추종 중 45도 격자 기준 누적 Yaw 드리프트 보정 (1ms 주기로 플랫폼 독립적 수렴)
__STATIC_INLINE void Update_IMU_Grid_Bias_Fourth(DriveFourthState_t *state, float gain) {
	uint32_t now_tick = HAL_GetTick();
	if (now_tick == state->last_grid_bias_tick) {
		return;
	}
	state->last_grid_bias_tick = now_tick;
	float corr_yaw = Wrap_Angle_Deg(imu_data.Yaw_Angle - state->imu_grid_bias);
	float nearest_45 = roundf(corr_yaw / 45.0f) * 45.0f;
	float residual = Wrap_Angle_Deg(corr_yaw - nearest_45);
	float meas_bias = Wrap_Angle_Deg(imu_data.Yaw_Angle - nearest_45);
	if (fabsf(residual) < 2.2f && fabsf(meas_bias) < 15.0f) {
		state->imu_grid_bias += gain * residual;
	}
}

// 4차 주행 전용 직선-45도 연속-직선 인코스 직선화(Cut-Through) 및 코너 인코스 영점 이동
__STATIC_INLINE void Update_Target_Pos_Fourth(DriveFourthState_t *state) {
	imu_straight_bias_track = 0;
	if (state->in_straight_cut) {
		float traveled = g_total_odom_m - state->cut_start_odom;

		float last_exit_dist = Cut_Milestone_Dist(state->cut_start_idx, state->cut_last_exit_idx);
		if (state->is_wave180_cut) {
			float total_wave_len = state->cut_straight_dist;
			uint8_t on_center_line = ((IR_Sensor.data->state & 0x1FF8) != 0) ? 1 : 0;
			if (traveled < total_wave_len && !(traveled >= total_wave_len - 0.06f && on_center_line)) {
				g_straight_cut_active = 1;
				g_suppress_marker_detect = (traveled >= total_wave_len - 0.14f) ? 0 : 1;
				IR_Sensor.data->target_pos = 0.0f;

				float half_len = (state->wave_half_len > 0.01f) ? state->wave_half_len : 0.57956f;
				int j = (int)(traveled / half_len);
				if (j < 0) j = 0;
				if (j >= (int)state->wave_count) j = (int)state->wave_count - 1;
				float u = (traveled - (float)j * half_len) / half_len;
				if (u < 0.0f) u = 0.0f;
				else if (u > 1.0f) u = 1.0f;
				float sj = (float)(((j & 1) == 0) ? state->wave_first_dir : -state->wave_first_dir);
				float grid_plus_bias = state->cut_base_yaw_ref;
				float wave_axis = Wrap_Angle_Deg(grid_plus_bias + (float)state->wave_first_dir * 90.0f);
				float u_eff = (j == 0) ? (0.14f + 0.86f * u) : u;
				float th_deg = -sj * state->wave_alpha_deg + 2.0f * sj * state->wave_alpha_deg * u_eff;
				g_straight_cut_yaw_ref = Wrap_Angle_Deg(wave_axis + th_deg);
				g_straight_cut_kappa_ref = sj / state->wave_r_arc;
			} else {
				g_straight_cut_active = 0;
				g_straight_cut_kappa_ref = 0.0f;
				IR_Sensor.data->target_pos = 0.0f;
				g_suppress_marker_detect = 0;
			}
			return;
		}
		float lock_start_dist = state->cut_lock_dist;
		float phase2_end_dist = last_exit_dist + state->cut_land_blend;
		uint8_t on_center_line = ((IR_Sensor.data->state & 0x1FF8) != 0) ? 1 : 0;
		uint8_t is_apex_leg1_cut = (!state->is_omega_cut && state->cut_end_idx == state->cut_last_exit_idx + 1 && state->cut_exit_yaw_off != 0.0f) ? 1 : 0;
		float exit_guard_dist = 0.16f;
		float max_phase2_dist = is_apex_leg1_cut ? phase2_end_dist : (state->cut_straight_dist - exit_guard_dist);
		if (max_phase2_dist < phase2_end_dist) max_phase2_dist = phase2_end_dist;
		uint8_t extend_omega = (state->is_omega_cut && !on_center_line && traveled < phase2_end_dist + 0.10f && traveled < max_phase2_dist) ? 1 : 0;

		if (traveled < lock_start_dist) {
			// Phase 1 (진입 직선 정렬 구간): 일반 라인 추종(target_pos = 0)으로 차체를 진입 직선 중심에 정렬
			g_straight_cut_active = 0;
			g_suppress_marker_detect = 0;
			g_straight_cut_yaw_ref = state->cut_base_yaw_ref;
			g_straight_cut_kappa_ref = 0.0f;
			IR_Sensor.data->target_pos = 0.0f;
			if (traveled >= 0.20f &&
			    fabsf(Sensor_Get_Last_Position()) < 0.06f &&
			    fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset) < 6.0f) {
				Update_IMU_Grid_Bias_Fourth(state, 0.04f);
				if (g_current_base_mps > 0.5f) {
					imu_straight_bias_track = 1;
				}
			}
		} else if (traveled < phase2_end_dist ||
		           extend_omega ||
		           (!state->is_omega_cut && state->cut_bulge_offset == 0.0f && !on_center_line && traveled < max_phase2_dist)) {
			// Phase 2 (시케인/단차/Ω-U턴 관통 구간)
			if (!state->cut_p1_latched) {
				float corr_yaw = imu_data.Yaw_Angle - state->imu_grid_bias;
				float grid_yaw = roundf(corr_yaw / 45.0f) * 45.0f;
				if (state->cut_start_idx < ref_log_count) {
					float exp_grid = ref_pose_th[state->cut_start_idx];
					if (fabsf(Wrap_Angle_Deg(grid_yaw - exp_grid)) > 22.5f) {
						grid_yaw = exp_grid;
					}
				}
				float target_yaw = Wrap_Angle_Deg(grid_yaw + state->cut_angle_deg + state->imu_grid_bias);
				state->cut_base_yaw_ref = target_yaw;
				state->cut_p1_latched = 1;
			}
			g_straight_cut_active = 1;
			g_suppress_marker_detect = 1;
			IR_Sensor.data->target_pos = 0.0f;

			if (state->is_omega_cut) {
				float s_p2 = traveled - lock_start_dist;
				float grid_plus_bias = Wrap_Angle_Deg(state->cut_base_yaw_ref - state->cut_angle_deg);
				float tan1_len = state->omega_tan1_len;
				float arc_len = state->omega_arc_len;
				if (s_p2 < tan1_len) {
					g_straight_cut_yaw_ref = Wrap_Angle_Deg(grid_plus_bias + state->omega_tan1_deg);
					g_straight_cut_kappa_ref = 0.0f;
				} else if (s_p2 < tan1_len + arc_len) {
					float u = (arc_len > 0.01f) ? ((s_p2 - tan1_len) / arc_len) : 1.0f;
					if (u < 0.0f) u = 0.0f;
					else if (u > 1.0f) u = 1.0f;
					float th_deg = state->omega_tan1_deg - (float)state->omega_dir * u * state->omega_sweep_deg;
					g_straight_cut_yaw_ref = Wrap_Angle_Deg(grid_plus_bias + th_deg);
					g_straight_cut_kappa_ref = -(float)state->omega_dir / state->omega_r_arc;
				} else {
					g_straight_cut_yaw_ref = Wrap_Angle_Deg(grid_plus_bias + state->omega_tan2_deg);
					g_straight_cut_kappa_ref = 0.0f;
				}
			} else if (state->cut_bulge_offset != 0.0f && phase2_end_dist > lock_start_dist + 0.10f) {
				float L = phase2_end_dist - lock_start_dist;
				float u = (traveled - lock_start_dist) / L;
				if (u < 0.0f) u = 0.0f;
				else if (u > 1.0f) u = 1.0f;
				float a = state->cut_bulge_warp;
				float tau = u + a * sinf(PI * u);
				float dtau = 1.0f + a * PI * cosf(PI * u);
				float ddtau = -a * PI * PI * sinf(PI * u);
				float two_pi_tau = 2.0f * PI * tau;
				float slope = state->cut_bulge_offset * (PI / L) * sinf(two_pi_tau) * dtau;
				float theta_off_deg = atan2f(slope, 1.0f) * (180.0f / PI);
				float yaw_ref = state->cut_base_yaw_ref + theta_off_deg;
				if (yaw_ref > 180.0f) yaw_ref -= 360.0f;
				else if (yaw_ref < -180.0f) yaw_ref += 360.0f;
				g_straight_cut_yaw_ref = yaw_ref;

				float denom = 1.0f + slope * slope;
				float d2y = state->cut_bulge_offset * (PI / (L * L)) *
				            (2.0f * PI * cosf(two_pi_tau) * dtau * dtau + sinf(two_pi_tau) * ddtau);
				g_straight_cut_kappa_ref = d2y / (denom * sqrtf(denom));
			} else {
				g_straight_cut_yaw_ref = state->cut_base_yaw_ref;
				g_straight_cut_kappa_ref = 0.0f;
			}
		} else {
			// Phase 3 (탈출 직선 재진입 구간): 라인 복귀 후 90도+ 코너 진입 전 IN 사전 시프트(0.48) 연결
			// 볼록 코사인 컷(bulge)이 코너와 반대편으로 부풀었다가 이미 인코스(-40mm)에 착지한 경우 즉시 0.48 유지해 외곽 리바운드 차단
			g_straight_cut_active = 0;
			g_straight_cut_kappa_ref = 0.0f;
			float exit_len = state->cut_straight_dist - phase2_end_dist;
			float pre_shift = 0.0f;
			if (!is_apex_leg1_cut &&
			    state->cut_end_idx + 1 < ref_log_count &&
			    seg_plan_4[state->cut_end_idx].curve_dir != 0 &&
			    seg_plan_4[state->cut_end_idx].accel_type == 0 &&
			    ref_log[state->cut_end_idx + 1].dist_from_prev_m >= 0.30f &&
			    exit_len > 0.12f) {
				float p_sign = (seg_plan_4[state->cut_end_idx].curve_dir == 1) ? 1.0f : -1.0f;
				float pu = (traveled - phase2_end_dist) / exit_len;
				if (pu < 0.0f) pu = 0.0f;
				else if (pu > 1.0f) pu = 1.0f;
				float pre_target = Drive4_Effective_Shift() * 0.96f;
				if (state->cut_bulge_offset * p_sign < 0.0f) {
					float base_hold = (pre_target > 0.32f) ? 0.32f : pre_target;
					pre_shift = p_sign * (base_hold + (pre_target - base_hold) * pu);
				} else if (pu > 0.15f) {
					pre_shift = p_sign * pre_target * ((pu - 0.15f) / 0.85f);
				}
			}
			if (pre_shift != 0.0f) {
				state->cut_p3_preshift_dir = (pre_shift > 0.0f) ? 1 : 2;
				state->cut_p3_preshift_corner = state->cut_end_idx;
			}
			IR_Sensor.data->target_pos = pre_shift;
			// 감속 슬립 시 오도메트리가 실제보다 짧게 계측되어 end 코너 마커가 억제되지 않도록 제어
			// 단, exit_len이 충분할 때는 Phase 2 가상 탈출 직후 원 마커가 중복 카운트되지 않도록 억제 창 유지
			float guard = exit_len * 0.32f;
			if (guard > 0.14f) guard = 0.14f;
			float suppress_end = phase2_end_dist + guard;
			float max_suppress = state->cut_straight_dist - 0.18f;
			if (state->braking_started && exit_len < 0.26f) suppress_end = phase2_end_dist;
			if (suppress_end > max_suppress) suppress_end = max_suppress;
			g_suppress_marker_detect = (traveled >= suppress_end) ? 0 : 1;
		}
		return;
	}

	if (state->mismatch || state->idx >= ref_log_count || state->idx == 0) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	int prev_idx = -1;
	for (int i = (int)state->idx - 1; i >= 0; i--) {
		if (ref_log[i].type != CROSS_CROSS) {
			prev_idx = i;
			break;
		}
	}
	int next_idx = (int)state->idx;
	for (int i = (int)state->idx; i < (int)ref_log_count; i++) {
		if (ref_log[i].type != CROSS_CROSS) {
			next_idx = i;
			break;
		}
	}

	if (prev_idx < 0 || next_idx >= (int)ref_log_count) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	uint8_t curve_dir = seg_plan_4[prev_idx].curve_dir;
	uint8_t a_type = seg_plan_4[prev_idx].accel_type;
	uint8_t b_dir = seg_plan_4[prev_idx].bridge_dir;

	float total_dist = 0.0f;
	for (int i = prev_idx + 1; i <= next_idx; i++) {
		total_dist += ref_log[i].dist_from_prev_m;
	}

	float traveled = g_odom_distance_m - state->marker_start_dist;
	for (int i = (int)state->idx - 1; i > prev_idx; i--) {
		traveled += ref_log[i].dist_from_prev_m;
	}

	if (total_dist < 0.001f || (curve_dir == 0 && a_type != 5)) {
		float pre_shift = 0.0f;
		if (curve_dir == 0 && total_dist >= 0.28f && !seg_plan_4[prev_idx].is_straight_cut &&
		    next_idx == (int)state->idx &&
		    g_current_base_mps <= 3.5f &&
		    (total_dist - traveled) <= 0.65f &&
		    next_idx + 1 < (int)ref_log_count &&
		    seg_plan_4[next_idx].curve_dir != 0 &&
		    seg_plan_4[next_idx].accel_type == 0 &&
		    ref_log[next_idx + 1].dist_from_prev_m >= 0.30f) {
			float p_sign = (seg_plan_4[next_idx].curve_dir == 1) ? 1.0f : -1.0f;
			float rem = total_dist - traveled;
			float win = (total_dist * 0.65f < 0.65f) ? (total_dist * 0.65f) : 0.65f;
			if (win > 0.05f && rem < win) {
				float su = 1.0f - (rem / win);
				if (su < 0.0f) su = 0.0f;
				else if (su > 1.0f) su = 1.0f;
				pre_shift = p_sign * (Drive4_Effective_Shift() * 0.96f) * su;
			}
		}
		IR_Sensor.data->target_pos = pre_shift;
		if (pre_shift == 0.0f && curve_dir == 0 && traveled > 0.15f &&
		    fabsf(Sensor_Get_Last_Position()) < 0.12f &&
		    fabsf(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset) < 8.0f) {
			Update_IMU_Grid_Bias_Fourth(state, 0.02f);
			if (fabsf(Sensor_Get_Last_Position()) < 0.06f && g_current_base_mps > 0.5f) {
				imu_straight_bias_track = 1;
			}
		}
		return;
	}

	float u = traveled / total_dist;
	if (u < 0.0f) u = 0.0f;
	if (u > 1.0f) u = 1.0f;

	float d_shift = Drive4_Effective_Shift();
	if (d_shift <= 0.0f) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	uint8_t active_dir = (curve_dir != 0) ? curve_dir : b_dir;
	if (active_dir == 0) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}
	float turn_sign = (active_dir == 1) ? 1.0f : -1.0f;
	float target = 0.0f;

	if (a_type == 5) {
		// 동일 방향 45도-직선-45도 IN-코리도 브릿징 (OUT 스윙 0, 마커 지점 <= 0.32 안전 유지로 M_L/M_R 이탈 방지)
		float edge_s = (d_shift < DRIVE4_IN_SAME45_EDGE_MAX) ? d_shift : DRIVE4_IN_SAME45_EDGE_MAX;
		if (curve_dir == 0) {
			// 중간 짧은 직선: 출구 마커 전(u=0.78)까지 edge_s로 복귀 완료해 M_L/M_R 마커 누락 방지
			float bulge = DRIVE4_IN_SAME45_BRIDGE_MID - edge_s;
			if (bulge < 0.0f) bulge = 0.0f;
			float su = u / 0.78f;
			float bump = (su < 1.0f) ? sinf(su * PI) : 0.0f;
			target = turn_sign * (edge_s + bulge * bump);
		} else {
			uint8_t next_is_bridge_straight = (next_idx < (int)ref_log_count &&
			                                   seg_plan_4[next_idx].accel_type == 5 &&
			                                   seg_plan_4[next_idx].curve_dir == 0);
			if (next_is_bridge_straight) {
				const float ramp_in = 0.28f;
				target = (u < ramp_in) ? (turn_sign * edge_s * (u / ramp_in)) : (turn_sign * edge_s);
			} else {
				const float ramp_out = 0.25f;
				target = (u > (1.0f - ramp_out)) ? (turn_sign * edge_s * ((1.0f - u) / ramp_out)) : (turn_sign * edge_s);
			}
		}
	} else if (a_type == 3) {
		// 45도 연속 시케인 고속 관통(type 3): 인코스를 파고들되 고속 위상 지연을 고려하여
		// 탈출 마커 25% 전부터 조기 복귀시켜 마커 센서(±90mm) 이탈을 원천 차단
		const float ramp_in = 0.15f;
		const float ramp_out = 0.25f;
		const float chicane_shift = d_shift * 0.85f;
		if (u < ramp_in) {
			target = turn_sign * chicane_shift * (u / ramp_in);
		} else if (u > (1.0f - ramp_out)) {
			target = turn_sign * chicane_shift * ((1.0f - u) / ramp_out);
		} else {
			target = turn_sign * chicane_shift;
		}
	} else {
		// 일반/깊은 코너(90도+): 직선 사전 시프트(corner_shift * 0.96) 연결 후 target_shift_val 그대로 인코스 유지
		float pre_dir = 0.0f;
		uint8_t prev_same_or_str = 0;
		if (prev_idx > 0) {
			int pp = prev_idx - 1;
			while (pp >= 0 && ref_log[pp].type == CROSS_CROSS) pp--;
			if (pp >= 0 && ((seg_plan_4[pp].curve_dir == 0 && !seg_plan_4[pp].is_straight_cut) ||
			                seg_plan_4[pp].curve_dir == active_dir)) {
				prev_same_or_str = 1;
			}
		}
		// 고립된 짧은 45도 커브(직전 직선/동일방향 커브 아님, 직선컷 후속 포함)가 직후 즉시 반대 방향 코너(예: -45 -> +45 -> -90)로 반전 시,
		// 역방향 +-0.58 급반전에 의한 beta > 25deg 스냅 방지를 위해 IN 시프트를 생략한다.
		float corner_shift = d_shift;
		if (total_dist < 0.28f && !prev_same_or_str &&
		    next_idx < (int)ref_log_count &&
		    seg_plan_4[next_idx].curve_dir != 0 &&
		    seg_plan_4[next_idx].curve_dir != active_dir) {
			corner_shift = 0.0f;
		}
		uint8_t prev_p3_preshift = (state->cut_p3_preshift_dir == active_dir &&
		                            prev_idx == (int)state->cut_p3_preshift_corner) ? 1 : 0;
		if (total_dist >= 0.30f && (prev_same_or_str || prev_p3_preshift)) {
			pre_dir = corner_shift * 0.96f;
		}
		float ramp_in = (total_dist >= 0.30f) ? ((pre_dir > 0.0f) ? 0.15f : 0.32f) : 0.20f;
		if (total_dist >= 0.75f && pre_dir == 0.0f && ref_log[prev_idx].dist_from_prev_m < 0.28f) {
			// 짧은(<0.28m) 비시프트 역방향 코너 직후 긴 헤어핀(>=0.75m)은 에이펙스 전(~0.13m)에 조기 IN 시프트 완료
			ramp_in = 0.13f / total_dist;
			if (ramp_in < 0.12f) ramp_in = 0.12f;
			else if (ramp_in > 0.32f) ramp_in = 0.32f;
		}
		float edge_shift = corner_shift;
		uint8_t restrict_apex = 0;
		if (next_idx < (int)ref_log_count) {
			if (seg_plan_4[next_idx].is_straight_cut && next_idx == (int)seg_plan_4[next_idx].cut_start_idx) {
				restrict_apex = 1;
			} else if (seg_plan_4[next_idx].curve_dir != 0 && seg_plan_4[next_idx].curve_dir != active_dir) {
				restrict_apex = 1;
			}
		}
		const float ramp_out = (total_dist >= 0.30f) ? (restrict_apex ? 0.34f : 0.22f) : 0.25f;
		float apex_shift = Compute_Corner_Apex_Shift(corner_shift, total_dist, restrict_apex);
		if (apex_shift < edge_shift) apex_shift = edge_shift;
		if (pre_dir > edge_shift) pre_dir = edge_shift;
		if (u < ramp_in) {
			float s_val = pre_dir + (edge_shift - pre_dir) * (u / ramp_in);
			target = turn_sign * s_val;
		} else if (u > (1.0f - ramp_out)) {
			target = turn_sign * edge_shift * ((1.0f - u) / ramp_out);
		} else {
			float um = (u - ramp_in) / (1.0f - ramp_in - ramp_out);
			target = turn_sign * (edge_shift + (apex_shift - edge_shift) * sinf(um * PI));
		}
	}

	IR_Sensor.data->target_pos = target;
}

void Drive_Fourth(void) {
	g_is_straight = 0;
	if (!Drive_Fourth_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveFourthState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;
	state.corridor_start_dist = g_odom_distance_m;
	state.end_mps = driveData.base_mps;

	if (ref_log_count > 0) {
		state.accel_active = 1;
		state.seg_len_predicted = (ref_log[0].type == CROSS_STOP && ref_log_count > 1)
				? (ref_log[0].dist_from_prev_m + ref_log[1].dist_from_prev_m)
				: ref_log[0].dist_from_prev_m;
		if (state.seg_len_predicted > 0.3f) {
			state.target_mps = driveData.max_mps;
			g_target_base_mps = state.target_mps;
		} else {
			state.target_mps = driveData.base_mps;
			g_target_base_mps = state.target_mps;
		}
	}

	while (!IR_Sensor.is_lost_position) {
		if (!state.mismatch && state.idx > 0 && state.idx < ref_log_count) {
			if (ref_log[state.idx].type == ref_log[state.idx - 1].type) {
				float min_d = ref_log[state.idx].dist_from_prev_m * 0.55f;
				if (min_d < 0.070f) min_d = 0.070f;
				else if (min_d > 0.125f) min_d = 0.125f;
				g_min_same_side_odom_m = min_d;
			} else {
				g_min_same_side_odom_m = 0.125f;
			}
		} else {
			g_min_same_side_odom_m = 0.070f;
		}

		CrossEvent_t cross = Cross_Detect_Update();
		if (cross != CROSS_NONE) {
			if (Process_Marker_Event_Fourth(cross, &state))
				break;
		}

		Check_Virtual_Markers_Fourth(&state);
		Check_Distance_And_Brake_Fourth(&state);
		Update_Target_Pos_Fourth(&state);
	}

	imu_straight_bias_track = 0;
	g_straight_cut_active = 0;
	g_suppress_marker_detect = 0;
	g_min_same_side_odom_m = 0.070f;
	g_straight_cut_kappa_ref = 0.0f;

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

	IR_Sensor.data->target_pos = 0.0f;

	float lap_time = (end_tick - start_tick) / 1000.0f;
	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End(4th)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}

	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}

// ============================================================================
// 5회차 주행 함수 모음 (4차 주행과 동일)
// ============================================================================
void Drive_Fifth(void) {
	g_is_straight = 0;
	Drive_Fourth();
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
			"iq_L,iq_R,gyro_z_dps,odom_m,marker_cnt,vbus\n");
	SD_SysID_WriteChunk(line, (UINT) len);

	uint32_t total = g_sysid_drive_count;
	if (total > SYSID_DRIVE_SAMPLE_MAX)
		total = SYSID_DRIVE_SAMPLE_MAX;

	for (uint32_t i = 0; i < total; i++) {
		const SysID_DriveSample_t *sp = &g_sysid_drive_buf[i];
		len = snprintf(line, sizeof(line),
				"%u,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%u,%.1f\n",
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
				sp->vbus_dV * 0.1f);
		SD_SysID_WriteChunk(line, (UINT) len);
	}

	return SD_SysID_Close();
}

void Drive_SysID_Test(void) {
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

	LCD_Clear();
	LCD_Printf(0, 0, "Saving SysID...");
	FRESULT res = Drive_Save_SysID_To_SD();

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
