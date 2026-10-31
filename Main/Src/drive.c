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
#include "lsm6ds3tr-c.h"

#define RAMP_TIM        (&htim14)
#define VIB_SAMPLE_MAX  10000

// @formatter:off
DriveParam_t driveData = {
    .base_mps = 1.80f,
    .max_mps = 10.00f,
    .accel = 7.5f,
    .decel = 7.5f,
    .steer_gain_p = 14.500f,
    .steer_gain_d = 260.0f,
    .pos_atten_gain = 0.25f,
    .pit_in_distance_m = 0.15f,
    .fan_en = 0,
	.target_shift_val = 0.38f,
	.yaw_gain_p = 0.35f,
	.yaw_gain_i = 0.0f,
	.lat_acc_max = 13.90f,
	.target_curve_gain = 1.42f,
	.slalom_ratio = 0.03f,
	.brake_margin_m = 0.24f,
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
		driveData.lat_acc_max = g_heal_anchor.lat_acc_max;
		driveData.brake_margin_m = g_heal_anchor.brake_margin_m;
		driveData.target_shift_val = g_heal_anchor.target_shift_val;
		driveData.target_curve_gain = g_heal_anchor.target_curve_gain;
		driveData.slalom_ratio = g_heal_anchor.slalom_ratio;
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
		g_heal_anchor.lat_acc_max = Clamp_F32(g_heal_anchor.lat_acc_max, 11.0f, 15.0f);
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

// 실패/주행 텔레메트리 비례 게인 보정 + 누적 포화(Ratchet) 방지 소프트 블렌딩
void Drive_AutoHeal_Apply(void) {
	if (!g_heal_anchor_init) {
		Drive_Diag_Reset(g_drive_diag.drive_mode);
	}

	// 1. 한쪽 쏠림(누적 포화) 방지: 현재 게인을 초기 기준점(Anchor) 쪽으로 15% 복원한 base 계산
	DriveParam_t base = driveData;
	base.steer_gain_p = 0.85f * driveData.steer_gain_p + 0.15f * g_heal_anchor.steer_gain_p;
	base.steer_gain_d = 0.85f * driveData.steer_gain_d + 0.15f * g_heal_anchor.steer_gain_d;
	base.lat_acc_max = 0.85f * driveData.lat_acc_max + 0.15f * g_heal_anchor.lat_acc_max;
	base.brake_margin_m = 0.85f * driveData.brake_margin_m + 0.15f * g_heal_anchor.brake_margin_m;
	base.target_shift_val = 0.85f * driveData.target_shift_val + 0.15f * g_heal_anchor.target_shift_val;
	base.target_curve_gain = 0.85f * driveData.target_curve_gain + 0.15f * g_heal_anchor.target_curve_gain;

	// D게인 과포화(> 390) 및 P게인 과소(< 12.2) 감지 및 해소
	uint8_t kd_oversaturated = (driveData.steer_gain_d > 390.0f) ? 1 : 0;
	if (kd_oversaturated) {
		float_t anchor_kd = (g_heal_anchor.steer_gain_d < 330.0f) ? g_heal_anchor.steer_gain_d : 330.0f;
		base.steer_gain_d = Clamp_F32(0.65f * driveData.steer_gain_d + 0.35f * anchor_kd, 240.0f, 420.0f);
	}
	if (driveData.steer_gain_p < 12.2f) {
		float_t anchor_kp = (g_heal_anchor.steer_gain_p > 14.2f) ? g_heal_anchor.steer_gain_p : 14.2f;
		base.steer_gain_p = Clamp_F32(0.65f * driveData.steer_gain_p + 0.35f * anchor_kp, 12.8f, 24.0f);
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
		float_t nom_kd = Clamp_F32(g_heal_anchor.steer_gain_d, 270.0f, 350.0f);
		healed.steer_gain_p = Clamp_F32(roundf((0.55f * driveData.steer_gain_p + 0.45f * g_heal_anchor.steer_gain_p + 0.35f * under_sev) * 1000.0f) / 1000.0f, 12.2f, 23.5f);
		healed.steer_gain_d = Clamp_F32(0.50f * driveData.steer_gain_d + 0.50f * nom_kd - 16.0f, 200.0f, 400.0f);
	} else if (speed_sev > 0.45f && g_drive_diag.drive_mode >= 2) {
		// [Entry] 직선 고속 가속 후 코너 진입 감속 부족 -> 진입 초과 속도에 비례해 브레이크 마진 증가 & 횡가속도 하향
		role = "Entry";
		float_t entry_sev = (speed_sev > slip_sev) ? speed_sev : slip_sev;
		healed.steer_gain_p = Clamp_F32(roundf((base.steer_gain_p + (g_drive_diag.was_oversteer ? -0.25f * slip_sev : +0.40f * under_sev)) * 1000.0f) / 1000.0f, 11.8f, 23.5f);
		float_t ideal_kd = Clamp_F32(healed.steer_gain_p * 22.4f, 220.0f, 440.0f);
		healed.steer_gain_d = Clamp_F32(0.60f * base.steer_gain_d + 0.40f * ideal_kd, 200.0f, 440.0f);
		healed.brake_margin_m = Clamp_F32(base.brake_margin_m + (0.015f + 0.030f * entry_sev), 0.07f, 0.24f);
		healed.lat_acc_max = Clamp_F32(base.lat_acc_max - (0.30f + 0.55f * entry_sev), 11.0f, 14.8f);
	} else if (g_drive_diag.was_oversteer) {
		// [PropOVS] 오버스티어/드리프트 과다 -> 슬립각 및 진동량에 비례해 KP 감쇠 & KD 보강
		role = "PropOVS";
		float_t dkp = -(0.35f * slip_sev + 0.45f * osc_sev);
		healed.steer_gain_p = fmaxf(0.0f, roundf((base.steer_gain_p + dkp) * 1000.0f) / 1000.0f);
		if (kd_oversaturated) {
			healed.steer_gain_d = fmaxf(0.0f, driveData.steer_gain_d - (24.0f + 36.0f * osc_sev));
		} else {
			float_t dkd = 14.0f * slip_sev + 20.0f * osc_sev;
			healed.steer_gain_d = fmaxf(0.0f, base.steer_gain_d + dkd);
		}
		healed.yaw_gain_p = Clamp_F32(base.yaw_gain_p + 0.03f * slip_sev, 0.15f, 0.60f);
		if (g_drive_diag.drive_mode >= 2) {
			healed.lat_acc_max = Clamp_F32(base.lat_acc_max - (0.25f * slip_sev + 0.35f * speed_sev), 11.2f, 15.0f);
			healed.brake_margin_m = Clamp_F32(base.brake_margin_m + (0.012f * slip_sev + 0.022f * speed_sev), 0.06f, 0.24f);
		}
	} else {
		// [PropUND] 언더스티어/조향 부족 이탈 -> 이탈 심각도에 비례해 KP 상향 & KD는 최적 댐핑비(22.4*KP)로 정렬
		role = "PropUND";
		float_t dkp = 0.65f * under_sev - 0.25f * osc_sev;
		healed.steer_gain_p = fmaxf(0.0f, roundf((base.steer_gain_p + dkp) * 1000.0f) / 1000.0f);
		float_t ideal_kd = fmaxf(0.0f, healed.steer_gain_p * 22.4f);
		healed.steer_gain_d = fmaxf(0.0f, 0.65f * base.steer_gain_d + 0.35f * ideal_kd + 10.0f * osc_sev);
		if (g_drive_diag.drive_mode >= 3) {
			healed.target_shift_val = Clamp_F32(base.target_shift_val + 0.030f * under_sev, 0.26f, 0.56f);
			healed.target_curve_gain = Clamp_F32(base.target_curve_gain + 0.050f * under_sev, 1.25f, 2.00f);
		}
	}

	// 4. 소프트 블렌딩 (65% Healed + 35% Current) 및 비포화 상하한 클램핑
	const float_t a = 0.15f;
	driveData.steer_gain_p = fmaxf(0.0f, roundf((a * healed.steer_gain_p + (1.0f - a) * driveData.steer_gain_p) * 1000.0f) / 1000.0f);
	driveData.steer_gain_d = fmaxf(0.0f, a * healed.steer_gain_d + (1.0f - a) * driveData.steer_gain_d);
	driveData.lat_acc_max = Clamp_F32(a * healed.lat_acc_max + (1.0f - a) * driveData.lat_acc_max, 11.0f, 15.0f);
	driveData.brake_margin_m = Clamp_F32(a * healed.brake_margin_m + (1.0f - a) * driveData.brake_margin_m, 0.06f, 0.24f);
	driveData.target_shift_val = Clamp_F32(a * healed.target_shift_val + (1.0f - a) * driveData.target_shift_val, 0.24f, 0.58f);
	driveData.target_curve_gain = Clamp_F32(a * healed.target_curve_gain + (1.0f - a) * driveData.target_curve_gain, 1.20f, 2.05f);
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
			LCD_Printf(0, 10, "L:%.1f B:%.2f", driveData.lat_acc_max, driveData.brake_margin_m);
		} else if (btn == INPUT_CMD_L_HOLD) {
			Drive_AutoHeal_Reset_Anchor();
			LCD_Printf(0, 8, "Reset Anchor!");
			LCD_Printf(0, 9, "P:%.3f D:%.0f", driveData.steer_gain_p, driveData.steer_gain_d);
			LCD_Printf(0, 10, "L:%.1f B:%.2f", driveData.lat_acc_max, driveData.brake_margin_m);
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
		LCD_Printf(0, 5, "LAT:%.2f BRK:%.2f", driveData.lat_acc_max, driveData.brake_margin_m);
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
static float g_smooth_target_pos = 0.0f;
static float g_last_odom_m = 0.0f;
static uint16_t ref_log_count = 0;
static SegmentPlan_t seg_plan[CROSS_LOG_MAX];

#define BRAKE_MARGIN_M 0.06f
#define ACCEL_START_MARGIN_M 0.08f

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
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
				curve_state = 1;
				is_straight = 0;
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 2;
				is_straight = 0;
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
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2)
			return 1;
	} else {
		if (cross == CROSS_LEFT)
			g_total_L++;
		else if (cross == CROSS_RIGHT)
			g_total_R++;
		else if (cross == CROSS_CROSS)
			g_total_C++;
	}

	if (state->mismatch) {
		if (cross == CROSS_CROSS) {
			for (uint16_t i = state->idx; i < ref_log_count; i++) {
				if (ref_log[i].type == CROSS_CROSS) {
					state->idx = i;
					state->mismatch = 0;
					break;
				}
			}
		}
	}

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

		if (state->idx == 0 || seg_plan[state->idx - 1].accel_ok) {
			g_target_base_mps = driveData.max_mps;
		} else {
			g_target_base_mps = driveData.base_mps;
		}

	} else {
		state->accel_active = 0;
		g_target_base_mps = driveData.base_mps;
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;

	if (!state->mismatch) {
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
			brake_dist += (v1 * v1 - v2 * v2) / (2.0f * driveData.decel);
			const float_t tau_sys = 0.045f;
			brake_dist += v1 * tau_sys;
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = driveData.base_mps;
			state->braking_started = 1;
		} else {
			if (traveled >= ACCEL_START_MARGIN_M) {
				g_target_base_mps = driveData.max_mps;
			}
		}
	}
}

void Drive_Second(void) {
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
} DriveThirdState_t;

// 3차 주행용 구간 계획 빌드 (사용자 원본 4차 알고리즘)
__STATIC_INLINE void Build_Segment_Plan_Third(void) {
	uint8_t curve_state = 0;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;

		// 1. 방향 판별 (CROSS는 투명인간 취급)
		if (cur == CROSS_LEFT) curve_state = 1;
		else if (cur == CROSS_RIGHT) curve_state = 2;
		else if (cur == CROSS_STOP) curve_state = 0;

		seg_plan_3[i].curve_dir = curve_state;

		// 2. 다음 마커까지의 가속 속성 판별
		uint8_t a_type = 0;
		if (cur == CROSS_STOP) {
			a_type = (i == 0) ? 1 : 0;
		} else if (i + 1 < ref_log_count) {
			float next_yaw_mag = fabsf(ref_log[i + 1].yaw_angle);

			if (cur == CROSS_CROSS) {
				a_type = 1; // 직선 가속
			} else if (cur == CROSS_LEFT || cur == CROSS_RIGHT) {
				if (next_yaw_mag <= 15.0f) {
					a_type = 1; // 곡선 끝 마커 뒤의 직선: 풀 가속
				} else if (next_yaw_mag < 65.0f) {
					a_type = 2; // 45도(슬라롬) 커브 가속 허용
				} else {
					a_type = 0; // 90도/180도 등 깊은 커브는 안전하게 감속
				}
			}
		}
		seg_plan_3[i].accel_type = a_type;
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

	Drive_IMU_Auto_Calibrate();

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();
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
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2) return 1;
	} else {
		if (cross == CROSS_LEFT) g_total_L++;
		else if (cross == CROSS_RIGHT) g_total_R++;
		else if (cross == CROSS_CROSS) g_total_C++;
	}

	if (state->mismatch) {
		if (cross == CROSS_CROSS) {
			for (uint16_t i = state->idx; i < ref_log_count; i++) {
				if (ref_log[i].type == CROSS_CROSS) {
					state->idx = i;
					state->mismatch = 0;
					break;
				}
			}
		}
	}

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
				float a_lat_limit = (driveData.lat_acc_max > 0.0f) ? driveData.lat_acc_max : 12.0f;
				float v_grip = sqrtf(a_lat_limit * r_eff);
				if (state->end_mps > v_grip) {
					state->end_mps = v_grip;
				}
			}
		}

		if (state->end_mps < driveData.base_mps)
			state->end_mps = driveData.base_mps;

		if (state->end_mps > state->target_mps)
			state->end_mps = state->target_mps;
		g_target_base_mps = state->target_mps;

	} else {
		state->accel_active = 0;
		state->target_mps = driveData.base_mps;
		state->end_mps = driveData.base_mps;
		g_target_base_mps = driveData.base_mps;
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;

	if (!state->mismatch) state->idx++;

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
			brake_dist += (v1 * v1 - v2 * v2) / (2.0f * driveData.decel);
			const float_t tau_sys = 0.045f;
			brake_dist += v1 * tau_sys;
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = state->end_mps;
			state->braking_started = 1;
		} else {
			if (traveled >= ACCEL_START_MARGIN_M) {
				g_target_base_mps = state->target_mps;
			}
		}
	}
}

// 곡선 구간에서 target_pos 배율 (target_curve_gain) 적용
#define TARGET_CURVE_YAW_MIN_DEG 20.0f
#define TARGET_POS_LIMIT         0.9f
__STATIC_INLINE void Apply_Target_Curve_Gain(uint16_t idx) {
	if (idx >= ref_log_count || driveData.target_curve_gain <= 0.0f)
		return;
	if (fabsf(ref_log[idx].yaw_angle) > TARGET_CURVE_YAW_MIN_DEG) {
		float t = IR_Sensor.data->target_pos * driveData.target_curve_gain;
		if (t > TARGET_POS_LIMIT) t = TARGET_POS_LIMIT;
		if (t < -TARGET_POS_LIMIT) t = -TARGET_POS_LIMIT;
		IR_Sensor.data->target_pos = t;
	}
}

// 3차 주행 전용 Target Position 예지 이동 로직 (사용자 원본 4차 로직: 마커 기반 선형 보간 & Apply_Target_Curve_Gain)
__STATIC_INLINE void Update_Target_Pos_Third(DriveThirdState_t *state) {
	if (state->mismatch || state->idx >= ref_log_count) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	int prev_idx = -1;
	for (int i = state->idx - 1; i >= 0; i--) {
		if (ref_log[i].type != CROSS_CROSS) {
			prev_idx = i;
			break;
		}
	}

	int next_idx = state->idx;
	for (int i = state->idx; i < ref_log_count; i++) {
		if (ref_log[i].type != CROSS_CROSS) {
			next_idx = i;
			break;
		}
	}

	uint8_t prev_curve = (prev_idx >= 0) ? seg_plan_3[prev_idx].curve_dir : 0;
	float current_target = 0.0f;
	if (prev_curve == 1) current_target = driveData.target_shift_val;
	else if (prev_curve == 2) current_target = -driveData.target_shift_val;

	uint8_t next_curve = (next_idx < ref_log_count) ? seg_plan_3[next_idx].curve_dir : 0;
	float next_target = 0.0f;
	if (next_curve == 1) next_target = driveData.target_shift_val;
	else if (next_curve == 2) next_target = -driveData.target_shift_val;

	float total_dist = 0.0f;
	for (int i = prev_idx + 1; i <= next_idx; i++) {
		total_dist += ref_log[i].dist_from_prev_m;
	}

	float traveled = g_odom_distance_m - state->marker_start_dist;
	for (int i = state->idx - 1; i > prev_idx; i--) {
		traveled += ref_log[i].dist_from_prev_m;
	}

	if (total_dist > 0.001f) {
		float ratio = traveled / total_dist;
		if (ratio > 1.0f) ratio = 1.0f;
		if (ratio < 0.0f) ratio = 0.0f;
		IR_Sensor.data->target_pos = current_target + (next_target - current_target) * ratio;
	} else {
		IR_Sensor.data->target_pos = current_target;
	}
	Apply_Target_Curve_Gain(state->idx);
}

void Drive_Third(void) {
	if (!Drive_Third_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveThirdState_t state = { 0 };
	state.marker_start_dist = g_odom_distance_m;
	state.end_mps = driveData.base_mps;

	// 첫 번째 직선 구간 출발 가속 처리
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

		Check_Distance_And_Brake_Third(&state); // 가속 및 감속 실행
		Update_Target_Pos_Third(&state);        // 다가오는 코너에 맞춰 센서 목표점 동적 이동
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
// 4회차 주행 함수 모음 (개선된 4차: 코사인 S-커브 부드러운 전환 & 연속 코너 직선화)
// ============================================================================

typedef struct {
	uint8_t accel_type; // 0: 감속(깊은 커브), 1: 직선 가속, 2: 45도(슬라롬) 가속
	uint8_t curve_dir;  // 0: 직선, 1: 좌(L), 2: 우(R)
} SegmentPlanFourth_t;

static SegmentPlanFourth_t seg_plan_4[CROSS_LOG_MAX];

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
	float target_mps;
	float end_mps;
} DriveFourthState_t;

// 4차 주행용 구간 계획 빌드
__STATIC_INLINE void Build_Segment_Plan_Fourth(void) {
	uint8_t curve_state = 0;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;

		// 1. 방향 판별 (CROSS는 투명인간 취급)
		if (cur == CROSS_LEFT) curve_state = 1;
		else if (cur == CROSS_RIGHT) curve_state = 2;
		else if (cur == CROSS_STOP) curve_state = 0;

		seg_plan_4[i].curve_dir = curve_state;

		// 2. 다음 마커까지의 가속 속성 판별
		uint8_t a_type = 0;
		if (cur == CROSS_STOP) {
			a_type = (i == 0) ? 1 : 0;
		} else if (i + 1 < ref_log_count) {
			float next_yaw_mag = fabsf(ref_log[i + 1].yaw_angle);

			if (cur == CROSS_CROSS) {
				a_type = 1;
			} else if (cur == CROSS_LEFT || cur == CROSS_RIGHT) {
				if (next_yaw_mag <= 15.0f) {
					a_type = 1; // 곡선 끝 마커 뒤의 직선: 풀 가속
				} else if (next_yaw_mag < 65.0f) {
					a_type = 2; // 45도(슬라롬) 가속
				} else {
					a_type = 0; // 90도 이상 급커브 안전 감속
				}
			}
		}
		seg_plan_4[i].accel_type = a_type;
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

	Drive_IMU_Auto_Calibrate();

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();
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

// 4차 주행 마커 이벤트 및 가속 구간 계산
__STATIC_INLINE uint8_t Process_Marker_Event_Fourth(CrossEvent_t cross, DriveFourthState_t *state) {
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2) return 1;
	} else {
		if (cross == CROSS_LEFT) g_total_L++;
		else if (cross == CROSS_RIGHT) g_total_R++;
		else if (cross == CROSS_CROSS) g_total_C++;
	}

	if (state->mismatch) {
		if (cross == CROSS_CROSS) {
			for (uint16_t i = state->idx; i < ref_log_count; i++) {
				if (ref_log[i].type == CROSS_CROSS) {
					state->idx = i;
					state->mismatch = 0;
					break;
				}
			}
		}
	}

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	if (!state->mismatch && state->idx < ref_log_count && seg_plan_4[state->idx].accel_type > 0) {
		state->accel_active = 1;
		const uint8_t type = seg_plan_4[state->idx].accel_type;
		float total_dist = 0.0f;
		uint8_t next_type = 0;
		uint16_t next_seg_idx = state->idx;

		for (uint16_t i = state->idx; i < ref_log_count; i++) {
			if (seg_plan_4[i].accel_type != type) {
				next_type = seg_plan_4[i].accel_type;
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

		if (next_seg_idx + 1 < ref_log_count && (next_type == 0 || next_type == 2)) {
			float next_yaw_deg = fabsf(ref_log[next_seg_idx + 1].yaw_angle);
			float next_dist = ref_log[next_seg_idx + 1].dist_from_prev_m;
			if (next_yaw_deg > 10.0f && next_dist > 0.05f) {
				float dtheta_rad = next_yaw_deg * (PI / 180.0f);
				float r_eff = next_dist / dtheta_rad;
				float a_lat_limit = (driveData.lat_acc_max > 0.0f) ? driveData.lat_acc_max : 12.0f;
				float v_grip = sqrtf(a_lat_limit * r_eff);
				if (state->end_mps > v_grip) {
					state->end_mps = v_grip;
				}
			}
		}

		if (state->end_mps < driveData.base_mps)
			state->end_mps = driveData.base_mps;

		if (state->end_mps > state->target_mps)
			state->end_mps = state->target_mps;
		g_target_base_mps = state->target_mps;

	} else {
		state->accel_active = 0;
		state->target_mps = driveData.base_mps;
		state->end_mps = driveData.base_mps;
		g_target_base_mps = driveData.base_mps;
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;

	if (!state->mismatch) state->idx++;

	return 0;
}

// 4차 주행 감속 및 가속 실행 제어
__STATIC_INLINE void Check_Distance_And_Brake_Fourth(DriveFourthState_t *state) {
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
			brake_dist += (v1 * v1 - v2 * v2) / (2.0f * driveData.decel);
			const float_t tau_sys = 0.045f;
			brake_dist += v1 * tau_sys;
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = state->end_mps;
			state->braking_started = 1;
		} else {
			if (traveled >= ACCEL_START_MARGIN_M) {
				g_target_base_mps = state->target_mps;
			}
		}
	}
}

// 4차 주행 전용 Target Position 예지 이동 로직 (개선 버전: 코사인 S-커브 전환, 긴 직선 중앙 복귀, 연속 코너 직선화)
__STATIC_INLINE void Update_Target_Pos_Fourth(DriveFourthState_t *state) {
	if (state->mismatch || state->idx >= ref_log_count) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	int prev_idx = -1;
	for (int i = state->idx - 1; i >= 0; i--) {
		if (ref_log[i].type != CROSS_CROSS) {
			prev_idx = i;
			break;
		}
	}

	int next_idx = state->idx;
	for (int i = state->idx; i < ref_log_count; i++) {
		if (ref_log[i].type != CROSS_CROSS) {
			next_idx = i;
			break;
		}
	}

	uint8_t prev_curve = (prev_idx >= 0) ? seg_plan_4[prev_idx].curve_dir : 0;
	float current_target = 0.0f;
	if (prev_curve == 1) current_target = driveData.target_shift_val;
	else if (prev_curve == 2) current_target = -driveData.target_shift_val;

	uint8_t next_curve = (next_idx < ref_log_count) ? seg_plan_4[next_idx].curve_dir : 0;
	float next_target = 0.0f;
	if (next_curve == 1) next_target = driveData.target_shift_val;
	else if (next_curve == 2) next_target = -driveData.target_shift_val;

	float total_dist = 0.0f;
	for (int i = prev_idx + 1; i <= next_idx; i++) {
		total_dist += ref_log[i].dist_from_prev_m;
	}

	float traveled = g_odom_distance_m - state->marker_start_dist;
	for (int i = state->idx - 1; i > prev_idx; i--) {
		traveled += ref_log[i].dist_from_prev_m;
	}

	// [사용자 요청: 90도 및 45도 좌우 연속 코너 완전 직선화!]
	// 좌우가 연속으로 전환되는 S자 구간(90도 S자 포함, total_dist <= 1.25m):
	// 인코스를 번갈아 타며 요동치지 않고 곧게 직선으로 통과하도록 타겟을 0.0으로 완전 평탄화
	if (prev_curve != 0 && next_curve != 0 && prev_curve != next_curve && total_dist <= 1.25f) {
		current_target = 0.0f;
		next_target = 0.0f;
	}

	if (total_dist > 0.001f) {
		float ratio = traveled / total_dist;
		if (ratio > 1.0f) ratio = 1.0f;
		if (ratio < 0.0f) ratio = 0.0f;
		IR_Sensor.data->target_pos = current_target + (next_target - current_target) * ratio;
	} else {
		IR_Sensor.data->target_pos = current_target;
	}
	Apply_Target_Curve_Gain(state->idx);
}

void Drive_Fourth(void) {
	if (!Drive_Fourth_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveFourthState_t state = { 0 };
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
			if (Process_Marker_Event_Fourth(cross, &state))
				break;
		}

		Check_Distance_And_Brake_Fourth(&state);
		Update_Target_Pos_Fourth(&state);
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
// 5회차 주행 함수 모음 (Out-In-Out 최적 레이싱 라인)
// ============================================================================

typedef struct {
	uint8_t accel_type; // 0: 감속(깊은 커브), 1: 직선 가속, 2: 45도(슬라롬) 가속
	uint8_t curve_dir;  // 0: 직선, 1: 좌(L), 2: 우(R)
} SegmentPlanFifth_t;

static SegmentPlanFifth_t seg_plan_5[CROSS_LOG_MAX];

typedef struct {
	uint16_t idx;
	uint8_t mismatch;
	uint8_t accel_active;
	uint8_t braking_started;
	float marker_start_dist;
	float seg_len_predicted;
	float target_mps;
	float end_mps;
} DriveFifthState_t;

__STATIC_INLINE void Build_Segment_Plan_Fifth(void) {
	uint8_t curve_state = 0;

	for (uint16_t i = 0; i < ref_log_count; i++) {
		CrossEvent_t cur = ref_log[i].type;
		uint8_t is_straight = 0;

		if (cur == CROSS_CROSS) {
			is_straight = (curve_state == 0) ? 1 : 0;
		} else if (cur == CROSS_LEFT) {
			if (curve_state == 1) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 1;
				is_straight = 0;
			}
		} else if (cur == CROSS_RIGHT) {
			if (curve_state == 2) {
				curve_state = 0;
				is_straight = 1;
			} else {
				curve_state = 2;
				is_straight = 0;
			}
		}

		if (cur == CROSS_STOP) {
			is_straight = (i == 0) ? 1 : 0;
			curve_state = 0;
		}

		seg_plan_5[i].curve_dir = curve_state;

		uint8_t a_type = 0;
		if (cur == CROSS_STOP) {
			a_type = (i == 0) ? 1 : 0;
		} else if (i + 1 < ref_log_count) {
			float next_yaw_mag = fabsf(ref_log[i + 1].yaw_angle);
			if (cur == CROSS_CROSS) {
				a_type = 1;
			} else if (cur == CROSS_LEFT || cur == CROSS_RIGHT) {
				if (next_yaw_mag <= 15.0f) {
					a_type = 1;
				} else if (next_yaw_mag < 65.0f) {
					a_type = 2;
				} else {
					a_type = 0;
				}
			}
		}
		seg_plan_5[i].accel_type = a_type;
	}
}

__STATIC_INLINE uint8_t Drive_Fifth_Init_Sequence(void) {
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
		Build_Segment_Plan_Fifth();
		is_ref_log_copied = 1;
	} else {
		if (ref_log_count == 0) return 0;
		Build_Segment_Plan_Fifth();
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
	Steer_Reset_State();
	Drive_Diag_Reset(5);
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

__STATIC_INLINE uint8_t Process_Marker_Event_Fifth(CrossEvent_t cross, DriveFifthState_t *state) {
	if (cross == CROSS_STOP) {
		LSM6DS3_Reset_Yaw();
		g_total_STOP++;
		if (g_total_STOP >= 2) return 1;
	} else {
		if (cross == CROSS_LEFT) g_total_L++;
		else if (cross == CROSS_RIGHT) g_total_R++;
		else if (cross == CROSS_CROSS) g_total_C++;
	}

	if (state->mismatch) {
		if (cross == CROSS_CROSS) {
			for (uint16_t i = state->idx; i < ref_log_count; i++) {
				if (ref_log[i].type == CROSS_CROSS) {
					state->idx = i;
					state->mismatch = 0;
					break;
				}
			}
		}
	}

	if (!state->mismatch) {
		if (state->idx >= ref_log_count || ref_log[state->idx].type != cross) {
			state->mismatch = 1;
		}
	}

	if (!state->mismatch && state->idx < ref_log_count && seg_plan_5[state->idx].accel_type > 0) {
		state->accel_active = 1;
		const uint8_t type = seg_plan_5[state->idx].accel_type;
		float total_dist = 0.0f;
		uint8_t next_type = 0;
		uint16_t next_seg_idx = state->idx;

		for (uint16_t i = state->idx; i < ref_log_count; i++) {
			if (seg_plan_5[i].accel_type != type) {
				next_type = seg_plan_5[i].accel_type;
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

		if (next_seg_idx + 1 < ref_log_count && (next_type == 0 || next_type == 2)) {
			float next_yaw_deg = fabsf(ref_log[next_seg_idx + 1].yaw_angle);
			float next_dist = ref_log[next_seg_idx + 1].dist_from_prev_m;
			if (next_yaw_deg > 10.0f && next_dist > 0.05f) {
				float dtheta_rad = next_yaw_deg * (PI / 180.0f);
				float r_eff = next_dist / dtheta_rad;
				float a_lat_limit = (driveData.lat_acc_max > 0.0f) ? driveData.lat_acc_max : 12.0f;
				float v_grip = sqrtf(a_lat_limit * r_eff);
				if (state->end_mps > v_grip) {
					state->end_mps = v_grip;
				}
			}
		}

		if (state->end_mps < driveData.base_mps)
			state->end_mps = driveData.base_mps;

		if (state->end_mps > state->target_mps)
			state->end_mps = state->target_mps;
		g_target_base_mps = state->target_mps;

	} else {
		state->accel_active = 0;
		state->target_mps = driveData.base_mps;
		state->end_mps = driveData.base_mps;
		g_target_base_mps = driveData.base_mps;
	}

	state->marker_start_dist = g_odom_distance_m;
	state->braking_started = 0;

	if (!state->mismatch) state->idx++;

	return 0;
}

__STATIC_INLINE void Check_Distance_And_Brake_Fifth(DriveFifthState_t *state) {
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
			brake_dist += (v1 * v1 - v2 * v2) / (2.0f * driveData.decel);
			const float_t tau_sys = 0.045f;
			brake_dist += v1 * tau_sys;
		}

		if (remaining <= brake_dist) {
			g_target_base_mps = state->end_mps;
			state->braking_started = 1;
		} else {
			if (traveled >= ACCEL_START_MARGIN_M) {
				g_target_base_mps = state->target_mps;
			}
		}
	}
}

// 5차 주행 전용 Out-In-Out 최적 레이싱 라인 (Optimal Racing Line) 영점 이동
__STATIC_INLINE void Update_Target_Pos_Fifth(DriveFifthState_t *state) {
	if (state->mismatch || state->idx >= ref_log_count || state->idx == 0) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}

	uint16_t cur_seg_idx = state->idx - 1;
	uint8_t curve_dir = seg_plan_5[cur_seg_idx].curve_dir;
	uint8_t a_type = seg_plan_5[cur_seg_idx].accel_type;

	float seg_len = ref_log[state->idx].dist_from_prev_m;
	if (seg_len < 0.05f) seg_len = 0.05f;
	float traveled = g_odom_distance_m;
	float remaining = seg_len - traveled;
	float u = traveled / seg_len;
	if (u < 0.0f) u = 0.0f;
	if (u > 1.0f) u = 1.0f;

	float d_shift = driveData.target_shift_val;
	if (d_shift < 0.02f) {
		IR_Sensor.data->target_pos = 0.0f;
		return;
	}
	if (d_shift > 0.85f) d_shift = 0.85f;

	float D_in = d_shift;
	float D_out = d_shift * 0.35f;

	float target = 0.0f;

	if (curve_dir != 0) {
		float turn_sign = (curve_dir == 1) ? 1.0f : -1.0f;
		if (a_type == 2) {
			target = turn_sign * (D_in * 0.70f) * sinf(u * PI);
		} else {
			float du = u - 0.5f;
			float val = D_in - (D_in + D_out) * 4.0f * du * du;
			target = turn_sign * val;
		}
	} else {
		uint8_t prev_curve = (cur_seg_idx > 0) ? seg_plan_5[cur_seg_idx - 1].curve_dir : 0;
		uint8_t next_curve = (state->idx < ref_log_count) ? seg_plan_5[state->idx].curve_dir : 0;

		const float TRANS_OUT = 0.30f;
		const float TRANS_IN = 0.35f;
		if (prev_curve != 0 && traveled < TRANS_OUT) {
			float prev_sign = (prev_curve == 1) ? 1.0f : -1.0f;
			float w = 0.5f * (1.0f + cosf((traveled / TRANS_OUT) * PI));
			target = -prev_sign * D_out * w;
		} else if (next_curve != 0 && remaining < TRANS_IN && remaining >= 0.0f) {
			float next_sign = (next_curve == 1) ? 1.0f : -1.0f;
			float w = 0.5f * (1.0f - cosf(((TRANS_IN - remaining) / TRANS_IN) * PI));
			target = -next_sign * D_out * w;
		} else {
			target = 0.0f;
		}
	}

	IR_Sensor.data->target_pos = target;
}

void Drive_Fifth(void) {
	if (!Drive_Fifth_Init_Sequence())
		return;

	uint32_t start_tick = HAL_GetTick();
	DriveFifthState_t state = { 0 };
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
			if (Process_Marker_Event_Fifth(cross, &state))
				break;
		}

		Check_Distance_And_Brake_Fifth(&state);
		Update_Target_Pos_Fifth(&state);
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
		LCD_Printf(0, 0, "End(5th)");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		LCD_Printf(0, 4, "T:%.2fs", lap_time);
	}

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
			"# yaw_kp=%.3f\n# yaw_ki=%.3f\n# lat_acc_max=%.2f\n"
			"# target_shift=%.2f\n# target_curve_gain=%.2f\n# brake_margin=%.2f\n"
			"# line_lost=%u\n# max_beta_deg=%.2f\n# max_lat_mm=%.1f\n# weave_dps=%.1f\n# fail_v=%.2f\n",
			driveData.base_mps, driveData.max_mps, driveData.accel, driveData.decel,
			driveData.steer_gain_p, driveData.steer_gain_d, driveData.pos_atten_gain,
			driveData.yaw_gain_p, driveData.yaw_gain_i, driveData.lat_acc_max,
			driveData.target_shift_val, driveData.target_curve_gain, driveData.brake_margin_m,
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

	LCD_Clear();
	LCD_Printf(0, 0, exit_reason_lost ? "SysID(Lost)" : "SysID(End)");
	LCD_Printf(0, 1, (res == FR_OK) ? "Saved SD OK!" : "Save Err:%d", res);
	LCD_Printf(0, 2, "/SysID_Data/");
	LCD_Printf(0, 3, "drive_sysid.csv");
	LCD_Printf(0, 4, "N:%lu T:%.2fs", (unsigned long) g_sysid_drive_count, lap_time);

	Drive_Wait_Exit_Or_Heal(1);
	LCD_Clear();
}
