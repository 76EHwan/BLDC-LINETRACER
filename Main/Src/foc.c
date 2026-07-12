#include "foc.h"
#include "adc.h"
#include "SDcard.h"
#include <stdio.h>

#define FOC_PARAM_PATH "/FOC_DATA/foc_param.txt"

// 모터 핸들 전역 인스턴스 (좌/우)
FOC_Handle_t foc_L;
FOC_Handle_t foc_R;

#pragma pack(push, 1)
typedef struct {
	uint32_t magic_number;
	float L_offset_a, L_offset_c, L_theta_offset;
	float L_id_Kp, L_id_Ki, L_iq_Kp, L_iq_Ki;
	float L_spd_Kp, L_spd_Ki, L_iq_limit;
	int8_t L_enc_dir;

	float R_offset_a, R_offset_c, R_theta_offset;
	float R_id_Kp, R_id_Ki, R_iq_Kp, R_iq_Ki;
	float R_spd_Kp, R_spd_Ki, R_iq_limit;
	int8_t R_enc_dir;
} FOC_SaveData_t;
#pragma pack(pop)

// regular(배터리) DMA 버퍼. 상전류는 injected(JDR)로 읽으므로 여기 안 들어감.
// 배터리만 쓰면 FOC_ADC_DMA_LENGTH는 1로 줄여도 됨.
__attribute__((section(".ram_d2_nocache"), aligned(32)))          uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
__attribute__((section(".ram_d2_nocache"), aligned(32)))          uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];

// 속도 루프 파라미터
#define SPD_IQ_LIMIT     2.5f        // Iq 지령 상한 (A)

// 엔코더 방향 보정: 반전 시 (RES - raw)로 미러링한 카운트 반환
static inline float32_t FOC_Enc_Cnt(FOC_Handle_t *hfoc) {
	uint16_t raw = (uint16_t) hfoc->LPTIMx->Instance->CNT;
	if (hfoc->enc_dir < 0) {
		return (float32_t) ENCODER_RESOLUTION - (float32_t) raw;
	}
	return (float32_t) raw;
}

// 1. regular(배터리) DMA + injected(상전류) IT 동시 구동 시작
void FOC_ADC_Start() {
	// regular DMA 먼저 시작 후, 돌고 있는 ADC에 injected IT를 얹는 순서

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_dma_buf, FOC_ADC_DMA_LENGTH);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc2_dma_buf, FOC_ADC_DMA_LENGTH);

	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
}

void FOC_Reset_State(FOC_Handle_t *hfoc) {
	hfoc->is_running = 0;
	hfoc->foc_svpwm_en = 0;
	hfoc->speed_loop_en = 0;

	// 2. 현재 상태 및 타겟 변수 초기화
	hfoc->target_Id = 0.0f;
	hfoc->target_Iq = 0.0f;
	hfoc->omega_e = 0.0f;
	hfoc->theta_e = 0.0f;

	hfoc->omega_e_meas = 0.0f;
	hfoc->target_omega = 0.0f;

	// 3. I항(적분기) 누적 오차 리셋
	hfoc->spd_integ = 0.0f;

	// [중요] 4. 인코더 카운트 동기화
	// 모터가 꺼져있는 동안 외부 힘에 의해 축이 돌아갔을 수 있으므로,
	// 현재 타이머의 CNT 값을 읽어와 prev_cnt를 맞춰주어야 속도 측정 시 튀지 않습니다.
	hfoc->enc_prev_cnt = (uint16_t) hfoc->LPTIMx->Instance->CNT;

	// [중요] 5. CMSIS DSP PID 내부 상태(State) 리셋
	// arm_pid_init_f32의 두 번째 인자를 1로 주면 내부의 에러 누적 메모리를 0으로 지워줍니다.
	arm_pid_init_f32(&hfoc->pid_id, 1);
	arm_pid_init_f32(&hfoc->pid_iq, 1);

}

// 2. 모터 제어기 구조체 초기화
void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_HandleTypeDef *TIMx,
		ADC_HandleTypeDef *ADCx, LPTIM_HandleTypeDef *LPTIMx) {
	// 1. 하드웨어 포인터 할당
	hfoc->TIMx = TIMx;
	hfoc->ADCx = ADCx;
	hfoc->LPTIMx = LPTIMx;

	FOC_Reset_State(hfoc);

	// 2. 디폴트 파라미터 (오프셋 및 방향)
	hfoc->offset_a = 32768.0f;
	hfoc->offset_c = 32768.0f;
	hfoc->theta_offset = 0.0f;
	hfoc->enc_dir = +1;

	// 3. 제어기 게인(Gain) 및 리밋(Limit) 설정
	hfoc->pid_id.Kp = DEFAULT_ID_KP;
	hfoc->pid_id.Ki = DEFAULT_ID_KI;
	hfoc->pid_id.Kd = 0.f;

	hfoc->pid_iq.Kp = DEFAULT_IQ_KP;
	hfoc->pid_iq.Ki = DEFAULT_IQ_KI;
	hfoc->pid_iq.Kd = 0.f;

	hfoc->spd_Kp = 0.00016f;
	hfoc->spd_Ki = 0.00035f;
	hfoc->iq_limit = SPD_IQ_LIMIT;

	// SD 카드에서 Kp, Ki를 불러왔으므로 PID 구조체에 한 번 반영해 줍니다.
	arm_pid_init_f32(&hfoc->pid_id, 1);
	arm_pid_init_f32(&hfoc->pid_iq, 1);
}

// 3. 전류 센서(ADC) 영점 캘리브레이션 (무부하, injected JDR 사용)
void FOC_Calibrate_Offset(FOC_Handle_t *hfoc) {
	uint32_t sum_a = 0, sum_c = 0;
	const int N = 500;

	for (int i = 0; i < N; i++) {
		sum_a += (uint16_t) hfoc->ADCx->Instance->JDR1;   // rank1 = 상A
		sum_c += (uint16_t) hfoc->ADCx->Instance->JDR2;   // rank2 = 상C
		HAL_Delay(1);
	}
	hfoc->offset_a = (float32_t) (sum_a / N);
	hfoc->offset_c = (float32_t) (sum_c / N);
}

// 4. 엔코더 영점(D축) 정렬 캘리브레이션
void FOC_Calibrate_Encoder_Offset(FOC_Handle_t *hfoc) {
	// 4-1. 강제 정렬을 위한 직류 전압 인가 (정격의 10% 수준)
	float32_t align_voltage = MOTOR_RATED_VOLTAGE * 0.1f;
	float32_t v_a, v_b, v_c;

	arm_inv_clarke_f32(align_voltage, 0.0f, &v_a, &v_b);
	v_c = -(v_a + v_b);

	float32_t duty_a = (v_a / MOTOR_RATED_VOLTAGE) * PWM_PERIOD
			+ PWM_HALF_PERIOD;
	float32_t duty_b = (v_b / MOTOR_RATED_VOLTAGE) * PWM_PERIOD
			+ PWM_HALF_PERIOD;
	float32_t duty_c = (v_c / MOTOR_RATED_VOLTAGE) * PWM_PERIOD
			+ PWM_HALF_PERIOD;

	hfoc->TIMx->Instance->CCR1 = (uint32_t) duty_a;
	hfoc->TIMx->Instance->CCR2 = (uint32_t) duty_b;
	hfoc->TIMx->Instance->CCR3 = (uint32_t) duty_c;

	hfoc->foc_svpwm_en = 1;

	// 4-2. 회전자가 끌려와 물리적으로 멈출 때까지 대기
	HAL_Delay(500);

	// 4-3. 정렬된 상태(전기각 0도)에서 LPTIM 카운터 가독 (방향 보정 적용)
	float32_t cnt = FOC_Enc_Cnt(hfoc);
	float32_t theta_m = (cnt / ENCODER_RESOLUTION) * (2.0f * M_PI);
	float32_t theta_e_raw = theta_m * MOTOR_POLE_PAIRS;

	while (theta_e_raw >= (float32_t) (2.0 * M_PI))
		theta_e_raw -= (float32_t) (2.0 * M_PI);
	while (theta_e_raw < 0.0f)
		theta_e_raw += (float32_t) (2.0 * M_PI);

	// 4-4. 오프셋 편차 저장
	hfoc->theta_offset = (float32_t) (2.0 * M_PI) - theta_e_raw;
	if (hfoc->theta_offset >= (float32_t) (2.0 * M_PI)) {
		hfoc->theta_offset -= (float32_t) (2.0 * M_PI);
	}

	// 4-5. 정렬 종료 및 출력 중립(0V) 복구
	hfoc->TIMx->Instance->CCR1 = (uint32_t) PWM_HALF_PERIOD;
	hfoc->TIMx->Instance->CCR2 = (uint32_t) PWM_HALF_PERIOD;
	hfoc->TIMx->Instance->CCR3 = (uint32_t) PWM_HALF_PERIOD;
	hfoc->foc_svpwm_en = 0;

	HAL_Delay(500);

	// 정렬 중 변한 CNT를 속도 루프 기준으로 동기화 (raw 기준, 차분과 짝 맞춤)
	hfoc->enc_prev_cnt = (uint16_t) hfoc->LPTIMx->Instance->CNT;
}

// 5. 실시간 LPTIM 엔코더 전기각 갱신 함수 (방향 보정 적용)
void FOC_Update_Theta_Encoder(FOC_Handle_t *hfoc) {
	float32_t cnt = FOC_Enc_Cnt(hfoc);

	float32_t theta_m = (cnt / ENCODER_RESOLUTION) * (2.0f * M_PI);
	float32_t theta_e = (theta_m * MOTOR_POLE_PAIRS) + hfoc->theta_offset;

	while (theta_e >= (float32_t) (2.0 * M_PI))
		theta_e -= (float32_t) (2.0 * M_PI);
	while (theta_e < 0.0f)
		theta_e += (float32_t) (2.0 * M_PI);

	hfoc->theta_e = theta_e;
}

// 6. 메인 FOC 실행 루프 (injected 변환 완료 IRQ에서 주기적 호출)
void FOC_Execute_Loop(FOC_Handle_t *hfoc) {
	if (hfoc->is_running == 0)
		return;

	// [1] 각도 갱신
	FOC_Update_Theta_Encoder(hfoc);

	// [2] injected 변환 결과(JDR)에서 상전류 측정 및 스케일링 (rank1=A, rank2=C)
	hfoc->I_a = ((float32_t) (uint16_t) hfoc->ADCx->Instance->JDR1
			- hfoc->offset_a) * CURRENT_SCALE;
	hfoc->I_c = ((float32_t) (uint16_t) hfoc->ADCx->Instance->JDR2
			- hfoc->offset_c) * CURRENT_SCALE;
	hfoc->I_b = -(hfoc->I_a + hfoc->I_c);

	// [3] Clarke Transform
	arm_clarke_f32(hfoc->I_a, hfoc->I_b, &hfoc->I_alpha, &hfoc->I_beta);

	// [4] Park Transform
	float32_t cos_theta = arm_cos_f32(hfoc->theta_e);
	float32_t sin_theta = arm_sin_f32(hfoc->theta_e);
	arm_park_f32(hfoc->I_alpha, hfoc->I_beta, &hfoc->I_d, &hfoc->I_q, sin_theta,
			cos_theta);

	// [5] PI 전류 제어 및 전압 포화 제한
	hfoc->V_d = arm_pid_f32(&hfoc->pid_id, hfoc->target_Id - hfoc->I_d);
	hfoc->V_q = arm_pid_f32(&hfoc->pid_iq, hfoc->target_Iq - hfoc->I_q);

	if (hfoc->V_d > MOTOR_RATED_VOLTAGE)
		hfoc->V_d = MOTOR_RATED_VOLTAGE;
	else if (hfoc->V_d < -MOTOR_RATED_VOLTAGE)
		hfoc->V_d = -MOTOR_RATED_VOLTAGE;

	if (hfoc->V_q > MOTOR_RATED_VOLTAGE)
		hfoc->V_q = MOTOR_RATED_VOLTAGE;
	else if (hfoc->V_q < -MOTOR_RATED_VOLTAGE)
		hfoc->V_q = -MOTOR_RATED_VOLTAGE;

	// [6] Inverse Park Transform
	arm_inv_park_f32(hfoc->V_d, hfoc->V_q, &hfoc->V_alpha, &hfoc->V_beta,
			sin_theta, cos_theta);

	// [7] Inverse Clarke Transform
	float32_t v_a, v_b;
	arm_inv_clarke_f32(hfoc->V_alpha, hfoc->V_beta, &v_a, &v_b);
	float32_t v_c = -(v_a + v_b);

	// [8] SVPWM 기반 Duty 연산 (0 ~ PWM_PERIOD)
	float32_t duty_a = (v_a / MOTOR_RATED_VOLTAGE) * PWM_PERIOD
			+ PWM_HALF_PERIOD;
	float32_t duty_b = (v_b / MOTOR_RATED_VOLTAGE) * PWM_PERIOD
			+ PWM_HALF_PERIOD;
	float32_t duty_c = (v_c / MOTOR_RATED_VOLTAGE) * PWM_PERIOD
			+ PWM_HALF_PERIOD;

	// 하드웨어 타이머 범위를 벗어나지 않도록 클램핑
	duty_a = duty_a < 0.0f ? 0.0f : (duty_a > PWM_PERIOD ? PWM_PERIOD : duty_a);
	duty_b = duty_b < 0.0f ? 0.0f : (duty_b > PWM_PERIOD ? PWM_PERIOD : duty_b);
	duty_c = duty_c < 0.0f ? 0.0f : (duty_c > PWM_PERIOD ? PWM_PERIOD : duty_c);

	// [9] 하드웨어 타이머 레지스터 적용
	if (hfoc->foc_svpwm_en) {
		hfoc->TIMx->Instance->CCR1 = (uint32_t) duty_a;
		hfoc->TIMx->Instance->CCR2 = (uint32_t) duty_b;
		hfoc->TIMx->Instance->CCR3 = (uint32_t) duty_c;
	}
}

// 7. 속도 PI 루프 (1kHz 타이머에서 모터별 호출, 위치형 + anti-windup)
void FOC_Speed_Loop(FOC_Handle_t *hfoc) {
	if (hfoc->speed_loop_en == 0)
		return;

	// [1] raw CNT 차분 (16비트 wrap 자동 처리) 후 방향 부호 적용
	uint16_t cnt = (uint16_t) hfoc->LPTIMx->Instance->CNT;
	int16_t delta = (int16_t) (cnt - hfoc->enc_prev_cnt);
	hfoc->enc_prev_cnt = cnt;
	if (hfoc->enc_dir < 0)
		delta = -delta;

	// [2] 기계각속도 -> 전기각속도 (rad/s)
	float32_t omega_m = ((float32_t) delta / ENCODER_RESOLUTION)
			* (2.0f * M_PI)/ SPD_DT;
	hfoc->omega_e_meas = omega_m * MOTOR_POLE_PAIRS;

	// [3] 속도 PI
	float32_t err = hfoc->target_omega - hfoc->omega_e_meas;
	hfoc->spd_integ += hfoc->spd_Ki * err * SPD_DT;

	// 적분 windup 방지: 적분항을 Iq 한계로 클램프
	if (hfoc->spd_integ > hfoc->iq_limit)
		hfoc->spd_integ = hfoc->iq_limit;
	if (hfoc->spd_integ < -hfoc->iq_limit)
		hfoc->spd_integ = -hfoc->iq_limit;

	float32_t iq_cmd = hfoc->spd_Kp * err + hfoc->spd_integ;

	// 출력 클램프
	if (iq_cmd > hfoc->iq_limit)
		iq_cmd = hfoc->iq_limit;
	if (iq_cmd < -hfoc->iq_limit)
		iq_cmd = -hfoc->iq_limit;

	// [4] 전류 루프에 Iq 지령 전달
	hfoc->target_Iq = iq_cmd;
}

// =========================================================
// [인터럽트 핸들러]
// =========================================================
// injected 변환 완료(JEOS) 콜백이 FOC loop를 구동한다.
// ADC1=foc_R, ADC2=foc_L
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		FOC_Execute_Loop(&foc_R);
	} else if (hadc->Instance == ADC2) {
		FOC_Execute_Loop(&foc_L);
	}
}

// 2kHz 타이머 IRQ. 두 모터 속도 루프를 같은 dt로 처리.
void Speed_TIM_IRQ_Handler() {
	FOC_Speed_Loop(&foc_L);
	FOC_Speed_Loop(&foc_R);
}

// =========================================================
// [SD카드 저장 및 불러오기]
// =========================================================
static const SDCard_ConfigEntry foc_param_table[] = {
	{ "L_offset_a",     &foc_L.offset_a,      SDCFG_FLOAT },
	{ "L_offset_c",     &foc_L.offset_c,      SDCFG_FLOAT },
	{ "L_theta_offset", &foc_L.theta_offset,  SDCFG_FLOAT },
	{ "L_id_Kp",        &foc_L.pid_id.Kp,     SDCFG_FLOAT },
	{ "L_id_Ki",        &foc_L.pid_id.Ki,     SDCFG_FLOAT },
	{ "L_iq_Kp",        &foc_L.pid_iq.Kp,     SDCFG_FLOAT },
	{ "L_iq_Ki",        &foc_L.pid_iq.Ki,     SDCFG_FLOAT },
	{ "L_spd_Kp",       &foc_L.spd_Kp,        SDCFG_FLOAT },
	{ "L_spd_Ki",       &foc_L.spd_Ki,        SDCFG_FLOAT },
	{ "L_iq_limit",     &foc_L.iq_limit,      SDCFG_FLOAT },
	{ "L_enc_dir",      &foc_L.enc_dir,       SDCFG_INT8  },

	{ "R_offset_a",     &foc_R.offset_a,      SDCFG_FLOAT },
	{ "R_offset_c",     &foc_R.offset_c,      SDCFG_FLOAT },
	{ "R_theta_offset", &foc_R.theta_offset,  SDCFG_FLOAT },
	{ "R_id_Kp",        &foc_R.pid_id.Kp,     SDCFG_FLOAT },
	{ "R_id_Ki",        &foc_R.pid_id.Ki,     SDCFG_FLOAT },
	{ "R_iq_Kp",        &foc_R.pid_iq.Kp,     SDCFG_FLOAT },
	{ "R_iq_Ki",        &foc_R.pid_iq.Ki,     SDCFG_FLOAT },
	{ "R_spd_Kp",       &foc_R.spd_Kp,        SDCFG_FLOAT },
	{ "R_spd_Ki",       &foc_R.spd_Ki,        SDCFG_FLOAT },
	{ "R_iq_limit",     &foc_R.iq_limit,      SDCFG_FLOAT },
	{ "R_enc_dir",      &foc_R.enc_dir,       SDCFG_INT8  },
};

FRESULT Save_FOC_Parameters(void) {
	return SDCard_SaveConfig(FOC_PARAM_PATH, foc_param_table, FOC_PARAM_COUNT);
}

FRESULT Load_FOC_Parameters(void) {
	FRESULT res = SDCard_LoadConfig(FOC_PARAM_PATH, foc_param_table, FOC_PARAM_COUNT);
	if (res != FR_OK)
		return res;

	arm_pid_init_f32(&foc_L.pid_id, 1);
	arm_pid_init_f32(&foc_L.pid_iq, 1);
	arm_pid_init_f32(&foc_R.pid_id, 1);
	arm_pid_init_f32(&foc_R.pid_iq, 1);

	return FR_OK;
}
