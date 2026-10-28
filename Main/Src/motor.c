#include "math.h"
#include "button.h"
#include "buzzer.h"
#include "drv8316crq1.h"
#include "mt6701.h"

#include "drive.h" // 조향 제어 시 목표 속도(g_current_base_mps) 참조용
#include "lsm6ds3tr-c.h" // 캐스케이드 조향: imu_data.Gyro_Z
#include "motor.h"
#include "sensor.h"
#include "user_init.h"

#define FAN_TIM		(&htim15)
#define FAN_CHANNEL	TIM_CHANNEL_2

#define MTR_L &DRV8316C_L
#define MTR_R &DRV8316C_R

#define MTR_WAKEUP      DRV8316C_WAKEUP
#define MTR_SLEEP       DRV8316C_SLEEP
#define MTR_DRVOFF_LOW  DRV8316C_DRVOFF_LOW
#define MTR_DRVOFF_HIGH DRV8316C_DRVOFF_HIGH
#define MTR_FOC_PWM_EN  DRV8316C_FOC_PWM_EN
#define MTR_FOC_PWM_DIS DRV8316C_FOC_PWM_DIS

#define MTR_ReadRegister   	DRV8316C_ReadRegister
#define MTR_WriteRegister	DRV8316C_WriteRegister
#define MTR_UpdateRegister	DRV8316C_ApplyDefaultConfig
#define MTR_UnlockRegister	DRV8316C_UnlockRegister
#define MTR_LockRegister	DRV8316C_LockRegister

#define MTR_REG_IC_STATUS DRV_REG_IC_STATUS
#define MTR_REG_STATUS_1  DRV_REG_STATUS_1
#define MTR_REG_STATUS_2  DRV_REG_STATUS_2
#define MTR_REG_CTRL_2    DRV_REG_CTRL_2
#define MTR_REG_CTRL_3    DRV_REG_CTRL_3
#define MTR_REG_CTRL_4    DRV_REG_CTRL_4
#define MTR_REG_CTRL_5    DRV_REG_CTRL_5
#define MTR_REG_CTRL_6    DRV_REG_CTRL_6
#define MTR_REG_CTRL_10   DRV_REG_CTRL_10

void MTR_TIM_Start(FOC_Handle_t *hfoc) {
	__HAL_TIM_SET_COMPARE(hfoc->TIMx, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(hfoc->TIMx, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(hfoc->TIMx, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(hfoc->TIMx, TIM_CHANNEL_4, 4800 - ADC_READ_TIMING);

	HAL_TIM_PWM_Start(hfoc->TIMx, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(hfoc->TIMx, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(hfoc->TIMx, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(hfoc->TIMx, TIM_CHANNEL_4);
}

void MTR_TIM_Stop(FOC_Handle_t *hfoc) {
	__HAL_TIM_SET_COMPARE((hfoc->TIMx), TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE((hfoc->TIMx), TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE((hfoc->TIMx), TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop((hfoc->TIMx), TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop((hfoc->TIMx), TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop((hfoc->TIMx), TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop((hfoc->TIMx), TIM_CHANNEL_4);
}

void MTR_Start() {
	DRV8316C_FOC_PWM_EN();
	MTR_TIM_Start(&foc_L);
	MTR_TIM_Start(&foc_R);
}

void MTR_Stop() {
	DRV8316C_FOC_PWM_DIS();
	MTR_TIM_Stop(&foc_L);
	MTR_TIM_Stop(&foc_R);
}

void Encoder_Start() {
	HAL_LPTIM_Encoder_Start((foc_L.LPTIMx), UINT16_MAX);
	HAL_LPTIM_Encoder_Start((foc_R.LPTIMx), UINT16_MAX);
}

void Encoder_Stop() {
	HAL_LPTIM_Encoder_Stop(&hlptim1);
	HAL_LPTIM_Encoder_Stop(&hlptim2);
}

void MTR_Setup_And_Start(FOC_DriveMode_t mode) {
	FOC_Reset_State(&foc_L);
	FOC_Reset_State(&foc_R);

	foc_L.enc_dir = -1;
	foc_R.enc_dir = -1;
	foc_L.omega_ramp_rate = 1500;
	foc_R.omega_ramp_rate = 1500;

	foc_L.is_running = 1;
	foc_R.is_running = 1;

	foc_L.target_Id = 0.0f;
	foc_R.target_Id = 0.0f;
	foc_L.target_Iq = 0.0f;
	foc_R.target_Iq = 0.0f;
	foc_L.target_omega = 0.0f;
	foc_R.target_omega = 0.0f;
	foc_L.spd_integ = 0.0f;
	foc_R.spd_integ = 0.0f;

	Encoder_Start();

	if (mode != FOC_MODE_SVPWM_NO_SPIN) {
		MTR_Start();
		HAL_Delay(50);
		LCD_Printf(0, 0, "Aligning");
		FOC_Calibrate_Encoder_Offset_Both(&foc_L, &foc_R);
		LCD_Clear();
	}

	FOC_ADC_Start();
	HAL_Delay(50);

	if (mode == FOC_MODE_SPEED_LOOP) {
		foc_L.speed_loop_en = 1;
		foc_R.speed_loop_en = 1;
		HAL_TIM_Base_Start_IT(TIM_SPEED_LOOP);
	} else {
		foc_L.speed_loop_en = 0;
		foc_R.speed_loop_en = 0;
	}

	if (mode == FOC_MODE_NO_SVPWM_SPIN) {
		foc_L.foc_svpwm_en = 0;
		foc_R.foc_svpwm_en = 0;
	} else {
		foc_L.foc_svpwm_en = 1;
		foc_R.foc_svpwm_en = 1;
	}

}

void MTR_Safe_Stop(void) {
	FOC_Reset_State(&foc_L);
	FOC_Reset_State(&foc_R);
	HAL_TIM_Base_Stop_IT(TIM_SPEED_LOOP);
	MTR_Stop();
	FOC_ADC_Stop();
	Encoder_Stop();
	LCD_Clear();
}

void Fan_Mtr_Start() {
	HAL_TIM_PWM_Start(FAN_TIM, FAN_CHANNEL);
}

void Fan_Mtr_Set_Duty(uint8_t duty) {
	__HAL_TIM_SET_COMPARE(FAN_TIM, FAN_CHANNEL, duty);
}

void Fan_Mtr_Stop() {
	__HAL_TIM_SET_COMPARE(FAN_TIM, FAN_CHANNEL, 0);
	HAL_TIM_PWM_Stop(FAN_TIM, FAN_CHANNEL);
}

// ============================================================================
// 조향(Steer) 함수 (비대칭 필터 기반 동적 감속 적용)
// ============================================================================

arm_pid_instance_f32 steer_pid;
static volatile float32_t g_current_steer = 0.0f;
static volatile float32_t filtered_atten = 1.0f; // ★ 필터링된 atten 상태 저장 변수
static volatile float32_t g_yaw_integ = 0.0f;    // 캐스케이드 안쪽 루프 적분 [rad/s]
static volatile float32_t g_grip_atten = 1.0f;   // 접지 한계 감속 비율 (lat_acc_max)
static volatile float32_t g_prev_steer_error = 0.0f;
static volatile float32_t g_filtered_deriv = 0.0f;
static volatile float32_t g_smooth_target_pos = 0.0f; // zero position shifting 완충 필터
#define YAW_INTEG_MAX 5.0f                        // 적분 한계 [rad/s]

void Steer_Reset_State(void) {
	g_yaw_integ = 0.0f;
	g_grip_atten = 1.0f;
	filtered_atten = 1.0f;
	g_prev_steer_error = 0.0f;
	g_filtered_deriv = 0.0f;
	g_smooth_target_pos = 0.0f;
}

void Steer_Motor() {
	// 1. zero position shifting 완충: target_pos 동적 변경 시 급격한 스텝으로 인한 차체 흔들림 방지
	const float32_t target_lpf_alpha = 0.96f; // dt=0.5ms 기준 시상수 약 12ms (빠른 추종과 충격 완화의 최적점)
	g_smooth_target_pos = (target_lpf_alpha * g_smooth_target_pos) + ((1.0f - target_lpf_alpha) * IR_Sensor.data->target_pos);

	// 2. 단정밀도(Single-Precision) 강제 계측 위치 수신
	float32_t line_pos = Sensor_Get_Position();
	float32_t error = line_pos - g_smooth_target_pos;

	// 2. 미분항(Kd): 측정값(line_pos) 미분(Derivative-on-Measurement) 적용
	//    target_pos가 동적으로 이동할 때 인공적인 Kd 미분 킥(스파이크) 및 자이로 요레이트 폭주 차단
	float32_t raw_deriv = line_pos - g_prev_steer_error;
	g_prev_steer_error = line_pos;

	// LPF 차단주파수 약 80Hz (dt = 0.0005s, alpha = 0.80f)
	const float32_t lpf_alpha = 0.80f;
	g_filtered_deriv = (lpf_alpha * g_filtered_deriv) + ((1.0f - lpf_alpha) * raw_deriv);

	// 3. fabsf()와 1.0f 등 단정밀도 전용 수학 함수/리터럴 사용
	float32_t raw_atten = 1.0f - (fabsf(error) * driveData.pos_atten_gain);

	if (raw_atten < 0.4f) {
		raw_atten = 0.4f;
	}

	if (raw_atten < filtered_atten) {
		filtered_atten = raw_atten;
	} else {
#define LPF_ALPHA 0.998f
		filtered_atten = (LPF_ALPHA * filtered_atten)
				+ ((1.0f - LPF_ALPHA) * raw_atten);
	}

	// 필터가 적용된 실제 주행 속도
	float32_t active_mps = g_current_base_mps * filtered_atten;

	// Gyro_Z [dps]는 좌회전(CCW)이 + 이므로 부호를 뒤집는다: yaw_meas [rad/s], + = 우회전
	float32_t yaw_meas = -(imu_data.Gyro_Z - imu_data.Gyro_Z_Offset) * (PI / 180.0f);

	// 4. 접지 한계 감속: 속도 x 요레이트(= 횡가속도)가 lat_acc_max를 넘으면 그만큼 속도를 낮춘다.
	//    곡선에서 미끄러지지 않게(엔코더 거리, 마커 인식이 깨끗하게). 빨리 내리고 천천히(약 0.1 s) 올린다.
	if (driveData.lat_acc_max > 0.0f) {
		float32_t a_lat = active_mps * fabsf(yaw_meas);
		float32_t raw = (a_lat > driveData.lat_acc_max) ? driveData.lat_acc_max / a_lat : 1.0f;
		if (raw < g_grip_atten)
			g_grip_atten = raw;
		else
			g_grip_atten = 0.995f * g_grip_atten + 0.005f * raw;
		active_mps *= g_grip_atten;
	} else {
		g_grip_atten = 1.0f;
	}

	// 5. 속도 스케일링: 고속 가속 구간(max_mps)에서 루프 이득이 과도해져 발진하지 않도록 보정
	//    (이동거리 미분 de/ds 및 Pure Pursuit 곡률 기반 보정)
	float32_t p_term = steer_pid.Kp * error;
	float32_t d_term = steer_pid.Kd * g_filtered_deriv;

	float32_t v_base = driveData.base_mps > 0.5f ? driveData.base_mps : 1.0f;
	if (active_mps > v_base) {
		float32_t speed_ratio = v_base / active_mps;
		p_term *= speed_ratio;
		d_term *= speed_ratio;
	}

	g_current_steer = p_term + d_term;

	// 6. 캐스케이드 조향: 라인 PD 출력(steer, 곡률)을 목표 요레이트로 바꾸고 자이로로 추종
	//    yaw_ref [rad/s], + = 우회전(왼쪽 바퀴가 빠름). yaw_gain_p = yaw_gain_i = 0이면 기존 PD와 동일.
	float32_t yaw_ref = active_mps * g_current_steer;
	float32_t yaw_cmd = yaw_ref;
	if (driveData.yaw_gain_p > 0.0f || driveData.yaw_gain_i > 0.0f) {
		float32_t yaw_err = yaw_ref - yaw_meas;
		if (active_mps > 0.05f) {
			g_yaw_integ += driveData.yaw_gain_i * yaw_err * RAMP_DT;
			g_yaw_integ = fminf(fmaxf(g_yaw_integ, -YAW_INTEG_MAX), YAW_INTEG_MAX);
		} else {
			g_yaw_integ = 0.0f;
		}
		yaw_cmd = yaw_ref + driveData.yaw_gain_p * yaw_err + g_yaw_integ;
	} else {
		g_yaw_integ = 0.0f;
	}

	// 양쪽 모터 목표 속도 산출: 바퀴 속도차 = 요레이트 x 바퀴 간격
	float32_t mps_L = active_mps + yaw_cmd * THREAD_DIV2;
	float32_t mps_R = active_mps - yaw_cmd * THREAD_DIV2;

	foc_L.target_omega = mps_L * MPS_TO_OMEGA;
	foc_R.target_omega = -mps_R * MPS_TO_OMEGA;

	foc_L.omega_setpoint = foc_L.target_omega;
	foc_R.omega_setpoint = foc_R.target_omega;

	// 7. 실기(Sim2Real) 텔레메트리 실시간 수집 (추정 슬립각 beta, 자이로 진동 에너지 weave, 이탈 직전 속도)
	Drive_Diag_Update_ISR(line_pos, active_mps, yaw_meas, yaw_ref,
			imu_data.Gyro_Z - imu_data.Gyro_Z_Offset);
}

// ====================================================================
// 드라이버 설정 및 상태 읽기
// ====================================================================

void MTR_Read_Register() {
	uint8_t lcd_left_x_bias = 6;
	uint8_t lcd_right_x_bias = 9;
	LCD_Printf(lcd_left_x_bias, 0, "L");
	LCD_Printf(lcd_right_x_bias, 0, "R");
	LCD_Printf(0, 2, "IC:");
	LCD_Printf(0, 3, "ST1");
	LCD_Printf(0, 4, "ST2");
	LCD_Printf(0, 5, "CTR2");
	LCD_Printf(0, 6, "CTR3");
	LCD_Printf(0, 7, "CTR4");
	LCD_Printf(0, 8, "CTR5");
	LCD_Printf(0, 9, "CTR6");
	LCD_Printf(0, 10, "CTR10");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		uint8_t mtr_L_pData, mtr_R_pData;

		MTR_ReadRegister(MTR_L, MTR_REG_IC_STATUS, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 2, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_STATUS_1, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 3, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_STATUS_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 4, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 5, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_3, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 6, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_4, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 7, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_5, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 8, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_6, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 9, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_10, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 10, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_IC_STATUS, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 2, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_STATUS_1, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 3, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_STATUS_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 4, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 5, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_3, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 6, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_4, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 7, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_5, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 8, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_6, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 9, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_10, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 10, "%02x", mtr_R_pData);
	}
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}

void MTR_Update_Setup() {
	MTR_FOC_PWM_DIS()
	;
	FOC_Init_Motor(&foc_L, &htim3, &hadc2, &hlptim2);
	FOC_Init_Motor(&foc_R, &htim4, &hadc1, &hlptim1);

	MTR_SLEEP(MTR_L);
	MTR_SLEEP(MTR_R);

	LCD_Printf(0, 0, "Diver Sleep");
	HAL_Delay(100);
	MTR_WAKEUP(MTR_L);
	MTR_WAKEUP(MTR_R);

	LCD_Printf(0, 1, "Driver Wakeup");
	HAL_Delay(100);

	DRV8316C_UnlockRegister(MTR_L);
	DRV8316C_UnlockRegister(MTR_R);
	MTR_UpdateRegister(MTR_L);
	MTR_UpdateRegister(MTR_R);
	DRV8316C_LockRegister(MTR_L);
	DRV8316C_LockRegister(MTR_R);

	LCD_Printf(0, 2, "Driver Update");
	HAL_Delay(1000);

	LCD_Clear();
	MTR_FOC_PWM_EN()
	;
}

void MTR_Simple_Control() {
	UserInput_t bt = INPUT_CMD_NONE;
	MTR_Setup_And_Start(FOC_MODE_NO_SVPWM_SPIN);

	static int16_t angle = 0;
	static uint16_t half_pwm = 2400;
	static uint16_t max_pwm = 1000;

	int16_t prev_angle = angle;
	uint16_t enc_L_prev = (uint16_t) hlptim2.Instance->CNT;
	uint16_t enc_R_prev = (uint16_t) hlptim1.Instance->CNT;
	int8_t dir_L_ok = 1;
	int8_t dir_R_ok = 1;

	while (1) {
		bt = Button_Get_Input();
		switch (bt) {
		case INPUT_CMD_K_HOLD:
			LCD_Clear();
			return;
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			angle -= 10;
			break;
		case INPUT_CMD_R_SINGLE:
		case INPUT_CMD_R_HOLD:
			angle += 10;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			max_pwm += 10;
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			max_pwm -= 10;
			break;
		default:
		}

		float rad = (float) angle * M_PI / 180.0f;
		float rad120 = 120.0f * M_PI / 180.0f;

		float su = sinf(rad);
		float sv = sinf(rad - rad120);
		float sw = sinf(rad + rad120);

		float min_s = fminf(su, fminf(sv, sw));
		float max_s = fmaxf(su, fmaxf(sv, sw));
		float zss = (min_s + max_s) / 2.0f;
		su -= zss;
		sv -= zss;
		sw -= zss;

		int32_t u_calc = half_pwm + (int32_t) (su * (float) max_pwm);
		int32_t v_calc = half_pwm + (int32_t) (sv * (float) max_pwm);
		int32_t w_calc = half_pwm + (int32_t) (sw * (float) max_pwm);

		u_calc = u_calc < 0 ? 0 : (u_calc > 4800 ? 4800 : u_calc);
		v_calc = v_calc < 0 ? 0 : (v_calc > 4800 ? 4800 : v_calc);
		w_calc = w_calc < 0 ? 0 : (w_calc > 4800 ? 4800 : w_calc);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t )u_calc);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t )v_calc);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t )w_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t )u_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t )v_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t )w_calc);

		int16_t angle_delta = angle - prev_angle;
		if (angle_delta != 0) {
			uint16_t enc_L_now = (uint16_t) hlptim2.Instance->CNT;
			uint16_t enc_R_now = (uint16_t) hlptim1.Instance->CNT;

			int16_t enc_L_delta = (int16_t) (enc_L_now - enc_L_prev);
			int16_t enc_R_delta = (int16_t) (enc_R_now - enc_R_prev);

			if (enc_L_delta != 0) {
				dir_L_ok = ((angle_delta > 0) == (enc_L_delta > 0)) ? 1 : -1;
			}
			if (enc_R_delta != 0) {
				dir_R_ok = ((angle_delta > 0) == (enc_R_delta > 0)) ? 1 : -1;
			}

			enc_L_prev = enc_L_now;
			enc_R_prev = enc_R_now;
			prev_angle = angle;
		}

		LCD_Printf(0, 0, "Ang:%3d", angle);
		LCD_Printf(0, 1, "Max:%4d", max_pwm);
		LCD_Printf(0, 2, "DirL:%s", dir_L_ok > 0 ? "OK " : "REV");
		LCD_Printf(0, 3, "DirR:%s", dir_R_ok > 0 ? "OK " : "REV");
	}
}

void MTR_Simple_FOC() {
	UserInput_t bt = INPUT_CMD_NONE;
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);

	foc_L.target_Iq = 0.001f;
	foc_R.target_Iq = 0.001f;

	float omega = 0.0f;

	while (1) {
		bt = Button_Get_Input();

		switch (bt) {
		case INPUT_CMD_R_SINGLE:
		case INPUT_CMD_R_HOLD:
			omega += 10.0f;
			break;
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			omega -= 10.0f;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			foc_L.target_Iq += 0.1f;
			foc_R.target_Iq += 0.1f;
			if (foc_L.target_Iq > 5.0f) {
				foc_L.target_Iq = 5.0f;
				foc_R.target_Iq = 5.0f;
			}
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			foc_L.target_Iq -= 0.1f;
			foc_R.target_Iq -= 0.1f;
			if (foc_L.target_Iq < -5.0f) {
				foc_L.target_Iq = -5.0f;
				foc_R.target_Iq = -5.0f;
			}
			break;
		case INPUT_CMD_K_HOLD:
			MTR_Safe_Stop();
			omega = 0.0f;
			return;
		default:
			break;
		}

		if (omega > 2000.0f)
			omega = 2000.0f;
		if (omega < -2000.0f)
			omega = -2000.0f;

		foc_L.omega_e = omega;
		foc_R.omega_e = omega;

		LCD_Printf(0, 0, "w:%6.1f Iq:%.2f", omega, foc_L.target_Iq);
		LCD_Printf(0, 1, "thL:%6.2f", foc_L.theta_e);
		LCD_Printf(0, 2, "thR:%6.2f", foc_R.theta_e);
		LCD_Printf(0, 3, "IdL:%6.3f", foc_L.I_d);
		LCD_Printf(0, 4, "IqL:%6.3f", foc_L.I_q);
		LCD_Printf(0, 5, "IdR:%6.3f", foc_R.I_d);
		LCD_Printf(0, 6, "IqR:%6.3f", foc_R.I_q);
		LCD_Printf(0, 7, "IaL:%6.3f", foc_L.I_a);
		LCD_Printf(0, 8, "IbL:%6.3f", foc_L.I_b);
		LCD_Printf(0, 9, "IaR:%6.3f", foc_R.I_a);
		LCD_Printf(0, 10, "IbR:%6.3f", foc_R.I_b);
		LCD_Printf(0, 11, "r1:%5d %5d", (uint16_t) ADC1->JDR1,
				(uint16_t) ADC1->JDR2);
		LCD_Printf(0, 12, "r2:%5d %5d", (uint16_t) ADC2->JDR1,
				(uint16_t) ADC2->JDR2);
		LCD_Printf(0, 13, "wmeasL:%6.1f", foc_L.omega_e_meas);
	}
}

void MTR_Encoder_Test() {
	MTR_Setup_And_Start(FOC_MODE_SVPWM_NO_SPIN);

	while (1) {
		LCD_Printf(0, 0, "L:%5d R:%5d", (uint16_t) hlptim2.Instance->CNT,
				(uint16_t) hlptim1.Instance->CNT);
		LCD_Printf(0, 1, "eL:%6.3f", foc_L.theta_e);
		LCD_Printf(0, 2, "eR:%6.3f", foc_R.theta_e);
		LCD_Printf(0, 3, "AL:%5d", (uint16_t) ADC2->JDR1);
		LCD_Printf(0, 4, "AR:%5d", (uint16_t) ADC1->JDR1);
		LCD_Printf(0, 5, "NFL: %d NFR: %d",
				HAL_GPIO_ReadPin(MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin),
				HAL_GPIO_ReadPin(MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin));
		if (Button_Get_Input() == INPUT_CMD_K_HOLD) {
			MTR_Safe_Stop();
			return;
		}

		FOC_Update_Theta_Encoder(&foc_L);
		FOC_Update_Theta_Encoder(&foc_R);
		HAL_Delay(50);
	}
}

#define TUNE_TEST_CURRENT 1.0f

void MTR_Current_Tune_Loop() {
	UserInput_t bt = INPUT_CMD_NONE;
	MTR_Setup_And_Start(FOC_MODE_SVPWM_SPIN);
	foc_L.pid_iq.Kp = 0.f;
	foc_L.pid_iq.Ki = 0.f;
	foc_R.pid_iq.Kp = 0.f;
	foc_R.pid_iq.Ki = 0.f;

	uint32_t last_toggle_time = HAL_GetTick();
	uint8_t toggle_state = 0;

	uint8_t sel = 0;
	const float step_kp = 0.000001f;
	const float step_ki = 0.000001f;

	while (1) {
		bt = Button_Get_Input();

		if (HAL_GetTick() - last_toggle_time > 500) {
			last_toggle_time = HAL_GetTick();
			toggle_state = !toggle_state;
			foc_L.target_Id = toggle_state ? TUNE_TEST_CURRENT : 0.0f;
		}

		switch (bt) {
		case INPUT_CMD_K_SINGLE:
			sel = (sel + 1) % 2;
			break;

		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			if (sel == 0)
				foc_L.pid_id.Kp += step_kp;
			else
				foc_L.pid_id.Ki += step_ki;
			arm_pid_init_f32(&foc_L.pid_id, 0);
			break;

		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			if (sel == 0) {
				foc_L.pid_id.Kp -= step_kp;
				if (foc_L.pid_id.Kp < 0.0f)
					foc_L.pid_id.Kp = 0.0f;
			} else {
				foc_L.pid_id.Ki -= step_ki;
				if (foc_L.pid_id.Ki < 0.0f)
					foc_L.pid_id.Ki = 0.0f;
			}
			arm_pid_init_f32(&foc_L.pid_id, 0);
			break;

		case INPUT_CMD_K_HOLD:
			foc_L.pid_iq.Kp = foc_L.pid_id.Kp;
			foc_L.pid_iq.Ki = foc_L.pid_id.Ki;
			foc_R.pid_id.Kp = foc_L.pid_id.Kp;
			foc_R.pid_id.Ki = foc_L.pid_id.Ki;
			foc_R.pid_iq.Kp = foc_L.pid_id.Kp;
			foc_R.pid_iq.Ki = foc_L.pid_id.Ki;

			arm_pid_init_f32(&foc_L.pid_iq, 0);
			arm_pid_init_f32(&foc_R.pid_id, 0);
			arm_pid_init_f32(&foc_R.pid_iq, 0);

			MTR_Safe_Stop();
			return;

		default:
			break;
		}

		LCD_Printf(0, 0, "Id Step Tune");
		LCD_Printf(0, 1, "%cKp:%6.4f", sel == 0 ? '>' : ' ', foc_L.pid_id.Kp);
		LCD_Printf(0, 2, "%cKi:%6.4f", sel == 1 ? '>' : ' ', foc_L.pid_id.Ki);
		LCD_Printf(0, 4, "Tgt Id:%6.2f", foc_L.target_Id);
		LCD_Printf(0, 5, "Cur Id:%6.2f", foc_L.I_d);
		LCD_Printf(0, 6, "Cur Iq:%6.2f", foc_L.I_q);
	}
}

void MTR_Speed_FOC() {
	UserInput_t bt = INPUT_CMD_NONE;
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);

	float omega = 0.0f;
	uint8_t sel = 0;

	const float step_iq_kp = 0.05f;
	const float step_iq_ki = 0.025f;
	const float step_spd_kp = 0.0001f;
	const float step_spd_ki = 0.1f;
	const float step_spd_kd = 0.000001f;

	while (1) {
		bt = Button_Get_Input();

		switch (bt) {
		case INPUT_CMD_R_SINGLE:
		case INPUT_CMD_R_HOLD:
			omega += 50.0f;
			if (omega > 2000.0f)
				omega = 2000.0f;
			break;
		case INPUT_CMD_R_DOUBLE:
			omega += 250.0f;
			if (omega > 2000.0f)
				omega = 2000.0f;
			break;
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			omega -= 50.0f;
			if (omega < -2000.0f)
				omega = -2000.0f;
			break;
		case INPUT_CMD_L_DOUBLE:
			omega -= 250.0f;
			if (omega < -2000.0f)
				omega = -2000.0f;
			break;
		case INPUT_CMD_K_SINGLE:
			sel = (sel + 1) % 5;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			switch (sel) {
			case 0:
				foc_L.pid_iq.Kp += step_iq_kp;
				foc_R.pid_iq.Kp += step_iq_kp;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 1:
				foc_L.pid_iq.Ki += step_iq_ki;
				foc_R.pid_iq.Ki += step_iq_ki;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 2:
				foc_L.spd_Kp += step_spd_kp;
				foc_R.spd_Kp += step_spd_kp;
				break;
			case 3:
				foc_L.spd_Ki += step_spd_ki;
				foc_R.spd_Ki += step_spd_ki;
				break;
			case 4:
				foc_L.spd_Kd += step_spd_kd;
				foc_R.spd_Kd += step_spd_kd;
				break;
			}
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			switch (sel) {
			case 0:
				foc_L.pid_iq.Kp -= step_iq_kp;
				if (foc_L.pid_iq.Kp < 0.0f)
					foc_L.pid_iq.Kp = 0.0f;
				foc_R.pid_iq.Kp = foc_L.pid_iq.Kp;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 1:
				foc_L.pid_iq.Ki -= step_iq_ki;
				if (foc_L.pid_iq.Ki < 0.0f)
					foc_L.pid_iq.Ki = 0.0f;
				foc_R.pid_iq.Ki = foc_L.pid_iq.Ki;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 2:
				foc_L.spd_Kp -= step_spd_kp;
				if (foc_L.spd_Kp < 0.0f)
					foc_L.spd_Kp = 0.0f;
				foc_R.spd_Kp = foc_L.spd_Kp;
				break;
			case 3:
				foc_L.spd_Ki -= step_spd_ki;
				if (foc_L.spd_Ki < 0.0f)
					foc_L.spd_Ki = 0.0f;
				foc_R.spd_Ki = foc_L.spd_Ki;
				break;
			case 4:
				foc_L.spd_Kd -= step_spd_kd;
				if (foc_L.spd_Kd < 0.0f)
					foc_L.spd_Kd = 0.0f;
				foc_R.spd_Kd = foc_L.spd_Kd;
				break;
			}
			break;
		case INPUT_CMD_K_HOLD:
			MTR_Safe_Stop();
			return;
		default:
			break;
		}

		foc_L.target_omega = omega;
		foc_R.target_omega = omega;

		LCD_Printf(0, 0, "%cIqKp:%6.3f", sel == 0 ? '>' : ' ',
				foc_L.pid_iq.Kp);
		LCD_Printf(0, 1, "%cIqKi:%6.3f", sel == 1 ? '>' : ' ',
				foc_L.pid_iq.Ki);
		LCD_Printf(0, 2, "%cSpKp:%6.3f", sel == 2 ? '>' : ' ',
				foc_L.spd_Kp * 1000);
		LCD_Printf(0, 3, "%cSpKi:%6.3f", sel == 3 ? '>' : ' ',
				foc_L.spd_Ki);
		LCD_Printf(0, 4, "%cSpKd:%6.3f", sel == 4 ? '>' : ' ',
				foc_L.spd_Kd * 1000);
		LCD_Printf(0, 6, "ref:%6.1f", omega);
		LCD_Printf(0, 7, "SpdL:%6.1f", foc_L.omega_e_meas);
		LCD_Printf(0, 8, "SpdR :%6.1f", foc_R.omega_e_meas);
		LCD_Printf(0, 9, "IqL:%8.5f", foc_L.target_Iq);
		LCD_Printf(0, 10, "IqcL:%8.5f", foc_L.I_q);
		LCD_Printf(0, 11, "IqR:%8.5f", foc_R.target_Iq);
		LCD_Printf(0, 12, "IqcR:%8.5f", foc_R.I_q);
		LCD_Printf(0, 13, "Vbus:%5.3f", FOC_Get_VBus());
	}
}

void Fan_Test() {
	Fan_Mtr_Start();
	uint16_t duty = 100;
	UserInput_t bt;
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, duty);
		switch (bt) {
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			duty += 10;
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			duty -= 10;
			break;
		default:
			break;
		}
		LCD_Printf(0, 0, "%4d", duty);
	}
	Fan_Mtr_Stop();
}

void Magnet_Encoder_Test() {
	UserInput_t bt;
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		MT6701_ReadSSI(&encDataL);
		MT6701_ReadSSI(&encDataR);
		LCD_Printf(0, 0, "L: %6.3f", encDataL.motor_elec_angle);
		LCD_Printf(0, 1, "R: %6.3f", encDataR.motor_elec_angle);
	}
}


void Battery_Check_Safe(void) {
	// 1. ★ 가장 중요: 모터 드라이버 하드웨어 출력 완전 차단 (소음/진동 원천 차단)
	MTR_FOC_PWM_DIS();

	// 2. 모터 상태 초기화 (제어 루프 차단)
	FOC_Reset_State(&foc_L);
	FOC_Reset_State(&foc_R);
	foc_L.speed_loop_en = 0;
	foc_R.speed_loop_en = 0;

	// 3. ★ 핵심: 모터는 꺼져있지만, ADC를 깨우기 위해 MCU 내부 PWM 타이머 가동
	MTR_TIM_Start(&foc_L);
	MTR_TIM_Start(&foc_R);

	// 4. 전압 측정을 위한 ADC 가동
	FOC_ADC_Start();

	// 5. Vbus 필터링 루프를 돌리기 위해 2kHz 속도 제어 타이머 가동
	HAL_TIM_Base_Start_IT(TIM_SPEED_LOOP);

	LCD_Clear();
	LCD_Printf(0, 0, "=== Battery ===");

	// K 버튼(Hold)을 길게 누를 때까지 실시간 전압 갱신
	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 3, "V: %8.6f V", FOC_Get_VBus());
		HAL_Delay(50);
	}

	// 6. 끄고 나갈 때 사용했던 타이머와 ADC 안전하게 원상복구
	HAL_TIM_Base_Stop_IT(TIM_SPEED_LOOP);
	FOC_ADC_Stop();
	MTR_TIM_Stop(&foc_L);
	MTR_TIM_Stop(&foc_R);
	LCD_Clear();
}


#include "sd_ui.h"

void MTR_SysID_Test(void) {
	LCD_Clear();
	LCD_Printf(0, 0, "Motor SysID");
	LCD_Printf(0, 1, "Lift Wheels!");
	HAL_Delay(500);

	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	foc_L.omega_ramp_rate = 100000.0f;
	foc_R.omega_ramp_rate = 100000.0f;

	uint32_t n = 0;
	uint32_t t0 = HAL_GetTick();
	uint8_t aborted = 0;

	// Phase 0: 전류 루프 스텝 응답 (80 샘플, 1ms 간격)
	foc_L.speed_loop_en = 0;
	foc_R.speed_loop_en = 0;
	foc_L.target_Iq = 0.0f;
	foc_R.target_Iq = 0.0f;
	for (int k = 0; k < 80 && n < SYSID_DRIVE_SAMPLE_MAX; k++) {
		float id_cmd = (k < 40) ? 1.5f : 0.0f;
		foc_L.target_Id = id_cmd;
		foc_R.target_Id = id_cmd;

		SysID_DriveSample_t *sp = &g_sysid_drive_buf[n++];
		sp->t_ms = (uint16_t) (HAL_GetTick() - t0);
		sp->line_pos_x10k = 0;
		sp->tgt_mps_mmps = 0;
		sp->mps_L_mmps = (int16_t) foc_L.omega_e_meas;
		sp->mps_R_mmps = (int16_t) (-foc_R.omega_e_meas);
		sp->tgt_iq_L_mA = (int16_t) (foc_L.target_Id * 1000.0f);
		sp->tgt_iq_R_mA = (int16_t) (foc_R.target_Id * 1000.0f);
		sp->iq_L_mA = (int16_t) (foc_L.I_d * 1000.0f);
		sp->iq_R_mA = (int16_t) (foc_R.I_d * 1000.0f);
		sp->gyro_z_ddps = (int16_t) (foc_L.V_d * 100.0f);
		sp->odom_mm = (uint16_t) (int16_t) (foc_R.V_d * 100.0f);
		sp->marker_cnt = 0;
		sp->vbus_dV = (uint8_t) (FOC_Get_VBus() * 10.0f);
		HAL_Delay(1);
	}
	foc_L.target_Id = 0.0f;
	foc_R.target_Id = 0.0f;

	// Phase 1..7: 무부하 속도 계단 스윕 (쿨롱 마찰 + 점성 마찰 + 속도 스텝 응답)
	static const float test_omegas[7] = {
		200.0f, 400.0f, 600.0f, 800.0f, 1000.0f, 1200.0f, 1500.0f
	};
	float step_w[7] = { 0 };
	float step_iq[7] = { 0 };
	int valid_steps = 0;

	foc_L.speed_loop_en = 1;
	foc_R.speed_loop_en = 1;

	for (int s = 0; s < 7 && !aborted; s++) {
		float w = test_omegas[s];
		foc_L.target_omega = w;
		foc_R.target_omega = -w;
		foc_L.omega_setpoint = w;
		foc_R.omega_setpoint = -w;

		LCD_Printf(0, 2, "Step %d/7:%4.0f", s + 1, w);

		float sum_w = 0.0f;
		float sum_iq = 0.0f;
		int steady_cnt = 0;

		for (int k = 0; k < 120 && n < SYSID_DRIVE_SAMPLE_MAX; k++) {
			if (Button_Get_Input() == INPUT_CMD_K_HOLD) {
				aborted = 1;
				break;
			}
			float iq_l_meas = (fabsf(foc_L.I_q) > 1e-4f) ? foc_L.I_q : foc_L.target_Iq;
			float iq_r_meas = (fabsf(foc_R.I_q) > 1e-4f) ? (-foc_R.I_q) : (-foc_R.target_Iq);

			SysID_DriveSample_t *sp = &g_sysid_drive_buf[n++];
			sp->t_ms = (uint16_t) (HAL_GetTick() - t0);
			sp->line_pos_x10k = (int16_t) (s + 1);
			sp->tgt_mps_mmps = (int16_t) w;
			sp->mps_L_mmps = (int16_t) foc_L.omega_e_meas;
			sp->mps_R_mmps = (int16_t) (-foc_R.omega_e_meas);
			sp->tgt_iq_L_mA = (int16_t) (foc_L.target_Iq * 1000.0f);
			sp->tgt_iq_R_mA = (int16_t) (-foc_R.target_Iq * 1000.0f);
			sp->iq_L_mA = (int16_t) (iq_l_meas * 1000.0f);
			sp->iq_R_mA = (int16_t) (iq_r_meas * 1000.0f);
			sp->gyro_z_ddps = (int16_t) (foc_L.V_q * 100.0f);
			sp->odom_mm = (uint16_t) (int16_t) (-foc_R.V_q * 100.0f);
			sp->marker_cnt = (uint8_t) (s + 1);
			sp->vbus_dV = (uint8_t) (FOC_Get_VBus() * 10.0f);

			if (k >= 60) {
				sum_w += 0.5f * (fabsf(foc_L.omega_e_meas) + fabsf(foc_R.omega_e_meas));
				sum_iq += 0.5f * (fabsf(iq_l_meas) + fabsf(iq_r_meas));
				steady_cnt++;
			}
			HAL_Delay(5);
		}
		if (!aborted && steady_cnt > 0) {
			step_w[valid_steps] = sum_w / (float) steady_cnt;
			step_iq[valid_steps] = sum_iq / (float) steady_cnt;
			valid_steps++;
		}
	}

	float vbus_now = FOC_Get_VBus();
	MTR_Safe_Stop();

	// 선형 회귀: tau = Kt * Iq = friction + viscous * omega
	float friction_Nm = 0.00025f;
	float viscous_Nms = 2.0e-8f;
	if (valid_steps >= 2) {
		float sw = 0.0f, st = 0.0f, sww = 0.0f, swt = 0.0f;
		for (int i = 0; i < valid_steps; i++) {
			float w = step_w[i];
			float tau = MOTOR_TORQUE_CONSTANT * step_iq[i];
			sw += w;
			st += tau;
			sww += w * w;
			swt += w * tau;
		}
		float denom = (float) valid_steps * sww - sw * sw;
		if (fabsf(denom) > 1e-6f) {
			viscous_Nms = ((float) valid_steps * swt - sw * st) / denom;
			friction_Nm = (st - viscous_Nms * sw) / (float) valid_steps;
			if (viscous_Nms < 0.0f) viscous_Nms = 0.0f;
			if (friction_Nm < 0.0f) friction_Nm = 0.0f;
		}
	}

	LCD_Clear();
	LCD_Printf(0, 0, "Saving MotorID..");
	FRESULT res = SD_SysID_Open(SYSID_MOTOR_PATH);
	if (res == FR_OK) {
		char line[256];
		int len = snprintf(line, sizeof(line),
				"# kt=%.6f\n# r_phase=%.4f\n# j_rotor=%.4e\n# gear=%.7f\n"
				"# vbus=%.3f\n# fitted_friction_Nm=%.6e\n# fitted_viscous_Nms=%.6e\n"
				"step,t_ms,target_omega,meas_omega_L,meas_omega_R,"
				"target_iq_L,target_iq_R,meas_iq_L,meas_iq_R,vq_L,vq_R,vbus\n",
				(double) MOTOR_TORQUE_CONSTANT,
				(double) MOTOR_PARAM_PHASE_RESISTOR,
				(double) MOTOR_PARAM_ROTOR_INERTIA,
				(double) GEAR_RATIO,
				(double) vbus_now,
				(double) friction_Nm,
				(double) viscous_Nms);
		SD_SysID_WriteChunk(line, (UINT) len);

		for (uint32_t i = 0; i < n; i++) {
			const SysID_DriveSample_t *sp = &g_sysid_drive_buf[i];
			int16_t vq_r_cV = (int16_t) sp->odom_mm;
			len = snprintf(line, sizeof(line),
					"%d,%u,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f\n",
					(int) sp->line_pos_x10k,
					(unsigned) sp->t_ms,
					(int) sp->tgt_mps_mmps,
					(int) sp->mps_L_mmps,
					(int) sp->mps_R_mmps,
					sp->tgt_iq_L_mA * 0.001f,
					sp->tgt_iq_R_mA * 0.001f,
					sp->iq_L_mA * 0.001f,
					sp->iq_R_mA * 0.001f,
					sp->gyro_z_ddps * 0.01f,
					vq_r_cV * 0.01f,
					sp->vbus_dV * 0.1f);
			SD_SysID_WriteChunk(line, (UINT) len);
		}
		res = SD_SysID_Close();
	}

	LCD_Clear();
	LCD_Printf(0, 0, "Motor SysID");
	LCD_Printf(0, 1, (res == FR_OK) ? "Saved SD OK!" : "Save Err:%d", res);
	LCD_Printf(0, 2, "/SysID_Data/");
	LCD_Printf(0, 3, "motor_sysid.csv");
	LCD_Printf(0, 5, "Fric:%.3fmNm", friction_Nm * 1000.0f);
	LCD_Printf(0, 6, "Visc:%.2e", viscous_Nms);
	LCD_Printf(0, 7, "Vbus:%.2fV", vbus_now);
	LCD_Printf(0, 9, "[K Hold] Exit");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD)
		;
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}
