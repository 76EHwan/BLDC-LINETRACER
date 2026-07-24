#include <stdio.h>

#include "adc.h"

#include "motor.h"
#include "foc.h"

// 모터 핸들 전역 인스턴스 (좌/우)
FOC_Handle_t foc_L;
FOC_Handle_t foc_R;

#pragma pack(push, 1)
typedef struct {
	uint32_t magic_number;
	float L_offset_a, L_offset_c, L_theta_offset;
	float L_id_Kp, L_id_Ki, L_iq_Kp, L_iq_Ki;
	float L_spd_Kp, L_spd_Ki, L_spd_Kd, L_iq_limit;
	int8_t L_enc_dir;

	float R_offset_a, R_offset_c, R_theta_offset;
	float R_id_Kp, R_id_Ki, R_iq_Kp, R_iq_Ki;
	float R_spd_Kp, R_spd_Ki, R_spd_Kd, R_iq_limit;
	int8_t R_enc_dir;
} FOC_SaveData_t;
#pragma pack(pop)

// regular(배터리) DMA 버퍼. 상전류는 injected(JDR)로 읽으므로 여기 안 들어감.
// 배터리만 쓰면 FOC_ADC_DMA_LENGTH는 1로 줄여도 됨.
__attribute__((section(".ram_d2_nocache"), aligned(32)))                     uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
__attribute__((section(".ram_d2_nocache"), aligned(32)))                     uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];

// 엔코더 방향 보정: 반전 시 (RES - raw)로 미러링한 카운트 반환
static inline float32_t FOC_Enc_Cnt(FOC_Handle_t *hfoc) {
	uint16_t raw = (uint16_t) hfoc->LPTIMx->Instance->CNT;
	if (hfoc->enc_dir < 0) {
		return (float32_t) ENCODER_RESOLUTION - (float32_t) raw;
	}
	return (float32_t) raw;
}

volatile float32_t g_vbus_filt = MOTOR_RATED_VOLTAGE;

__STATIC_INLINE void FOC_Update_VBus(void) {
	// adc1_dma_buf, adc2_dma_buf 둘 다 배터리를 보고 있다면 평균, 아니면 하나만 사용
	uint32_t raw = (adc1_dma_buf[0] + adc2_dma_buf[0]) / 2; // 실제로 배터리를 읽는 채널로 교체
	float32_t vbus_raw = (float32_t) raw * VBUS_ADC_SCALE;

	const float32_t alpha = 0.05f; // 필터 계수 (2kHz 기준 튜닝)
	g_vbus_filt += alpha * (vbus_raw - g_vbus_filt);

	// 안전 클램프: 너무 낮으면 0으로 나누기 방지, 너무 높으면 이상값 방지
	if (g_vbus_filt < 3.0f)
		g_vbus_filt = 3.0f;
	if (g_vbus_filt > 30.0f)
		g_vbus_filt = 30.0f;
}

float32_t FOC_Get_VBus(void) {
	return g_vbus_filt;
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

	HAL_Delay(50);

	g_vbus_filt = (float32_t) (adc1_dma_buf[0] + adc2_dma_buf[0])
			/ 2.f* VBUS_ADC_SCALE;
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

	// D항 상태 리셋 (계단 지령/재기동 시 미분 스파이크 방지)
	hfoc->spd_prev_meas = 0.0f;
	hfoc->spd_deriv_filt = 0.0f;

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

	hfoc->spd_Kp = 0.0007f;
	hfoc->spd_Ki = 0.0015f;
	hfoc->spd_Kd = 0.000001f;      // 기본은 0에서 시작, 필요 시 튜닝
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

void FOC_Calibrate_Encoder_Offset_Both(FOC_Handle_t *hfoc_L, FOC_Handle_t *hfoc_R) {
	// 4-1. 강제 정렬을 위한 직류 전압 인가 (정격의 10% 수준)
	float32_t align_voltage = MOTOR_RATED_VOLTAGE * 0.1f;
	float32_t v_a, v_b, v_c;

	arm_inv_clarke_f32(align_voltage, 0.0f, &v_a, &v_b);
	v_c = -(v_a + v_b);

	float32_t duty_a = (v_a / MOTOR_RATED_VOLTAGE) * PWM_PERIOD + PWM_HALF_PERIOD;
	float32_t duty_b = (v_b / MOTOR_RATED_VOLTAGE) * PWM_PERIOD + PWM_HALF_PERIOD;
	float32_t duty_c = (v_c / MOTOR_RATED_VOLTAGE) * PWM_PERIOD + PWM_HALF_PERIOD;

	// [좌측 모터] PWM 인가
	hfoc_L->TIMx->Instance->CCR1 = (uint32_t) duty_a;
	hfoc_L->TIMx->Instance->CCR2 = (uint32_t) duty_b;
	hfoc_L->TIMx->Instance->CCR3 = (uint32_t) duty_c;
	hfoc_L->foc_svpwm_en = 1;

	// [우측 모터] PWM 인가
	hfoc_R->TIMx->Instance->CCR1 = (uint32_t) duty_a;
	hfoc_R->TIMx->Instance->CCR2 = (uint32_t) duty_b;
	hfoc_R->TIMx->Instance->CCR3 = (uint32_t) duty_c;
	hfoc_R->foc_svpwm_en = 1;

	// 4-2. 두 모터의 회전자가 끌려와 물리적으로 멈출 때까지 대기
	HAL_Delay(500);

	// 4-3. 정렬된 상태(전기각 0도)에서 양쪽 LPTIM 카운터 가독 및 오프셋 계산

	// --- [좌측 모터 오프셋 계산] ---
	float32_t cnt_L = FOC_Enc_Cnt(hfoc_L);
	float32_t theta_m_L = (cnt_L / ENCODER_RESOLUTION) * (2.0f * M_PI);
	float32_t theta_e_raw_L = theta_m_L * MOTOR_POLE_PAIRS;

	while (theta_e_raw_L >= (float32_t) (2.0 * M_PI))
		theta_e_raw_L -= (float32_t) (2.0 * M_PI);
	while (theta_e_raw_L < 0.0f)
		theta_e_raw_L += (float32_t) (2.0 * M_PI);

	hfoc_L->theta_offset = (float32_t) (2.0 * M_PI) - theta_e_raw_L;
	if (hfoc_L->theta_offset >= (float32_t) (2.0 * M_PI)) {
		hfoc_L->theta_offset -= (float32_t) (2.0 * M_PI);
	}

	// --- [우측 모터 오프셋 계산] ---
	float32_t cnt_R = FOC_Enc_Cnt(hfoc_R);
	float32_t theta_m_R = (cnt_R / ENCODER_RESOLUTION) * (2.0f * M_PI);
	float32_t theta_e_raw_R = theta_m_R * MOTOR_POLE_PAIRS;

	while (theta_e_raw_R >= (float32_t) (2.0 * M_PI))
		theta_e_raw_R -= (float32_t) (2.0 * M_PI);
	while (theta_e_raw_R < 0.0f)
		theta_e_raw_R += (float32_t) (2.0 * M_PI);

	hfoc_R->theta_offset = (float32_t) (2.0 * M_PI) - theta_e_raw_R;
	if (hfoc_R->theta_offset >= (float32_t) (2.0 * M_PI)) {
		hfoc_R->theta_offset -= (float32_t) (2.0 * M_PI);
	}

	// 4-5. 정렬 종료 및 출력 중립(0V) 복구
	hfoc_L->TIMx->Instance->CCR1 = (uint32_t) PWM_HALF_PERIOD;
	hfoc_L->TIMx->Instance->CCR2 = (uint32_t) PWM_HALF_PERIOD;
	hfoc_L->TIMx->Instance->CCR3 = (uint32_t) PWM_HALF_PERIOD;
	hfoc_L->foc_svpwm_en = 0;

	hfoc_R->TIMx->Instance->CCR1 = (uint32_t) PWM_HALF_PERIOD;
	hfoc_R->TIMx->Instance->CCR2 = (uint32_t) PWM_HALF_PERIOD;
	hfoc_R->TIMx->Instance->CCR3 = (uint32_t) PWM_HALF_PERIOD;
	hfoc_R->foc_svpwm_en = 0;

	HAL_Delay(500);

	// 정렬 중 변한 양쪽 CNT를 속도 루프 기준으로 동기화
	hfoc_L->enc_prev_cnt = (uint16_t) hfoc_L->LPTIMx->Instance->CNT;
	hfoc_R->enc_prev_cnt = (uint16_t) hfoc_R->LPTIMx->Instance->CNT;
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


float_t g_odom_distance_m = 0.f;

float_t FOC_Meas_Mps(FOC_Handle_t *hfoc) {
	return fabsf(hfoc->omega_e_meas) / (INV_TIRE_RADIUS * MOTOR_POLE_PAIRS * GEAR_RATIO);
}

void Odom_Reset(void) {
	g_odom_distance_m = 0.f;
}

void Odom_Accumulate(float dt_sec) {
	float_t mps = 0.5f * (FOC_Meas_Mps(&foc_L) + FOC_Meas_Mps(&foc_R));
	g_odom_distance_m += mps * dt_sec;
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

	// [5] PI 전류 제어
	float32_t Vd_pi = arm_pid_f32(&hfoc->pid_id, hfoc->target_Id - hfoc->I_d);
	float32_t Vq_pi = arm_pid_f32(&hfoc->pid_iq, hfoc->target_Iq - hfoc->I_q);

	// [5-1] 역기전력 및 상호 간섭(cross-coupling) 디커플링 feedforward
	//   coreless 모터는 돌극성이 없어 Ld = Lq = L 이 정확히 성립하므로
	//   Vd_ff = -we * L * Iq
	//   Vq_ff =  we * L * Id + we * λ   (λ: 자속쇄교수, 토크상수에서 역산)
	//   omega_e_meas는 속도 루프(2kHz)에서 갱신되는 값을 그대로 재사용합니다.

	float32_t we = hfoc->omega_e_meas;

	// 1. 역기전력(Back-EMF) 항: 회전 속도에 비례
	float32_t Vq_bemf = we * MOTOR_FLUX_LINKAGE;

	// 2. 인덕턴스 디커플링(Cross-coupling) 항
	float32_t Vd_cross = -we * MOTOR_PHASE_INDUCTANCE_H * hfoc->I_q;
	float32_t Vq_cross = we * MOTOR_PHASE_INDUCTANCE_H * hfoc->I_d;

	// [디버깅 스위치] 처음에는 ff_gain을 0.05 ~ 0.1 정도로 매우 작게 주고 시작합니다.
	// 안정적이면 1.0까지 서서히 올립니다. 만약 0.1만 넣었는데도 덜덜거리면 부호나 파라미터가 틀린 것입니다.
	float32_t ff_bemf_gain = 0.01f; // TODO: 0.1f로 올려서 테스트
	float32_t ff_cross_gain = 0.01f; // BEMF가 완벽해지면 시도

	float32_t Vd_ff = Vd_cross * ff_cross_gain;
	float32_t Vq_ff = (Vq_bemf * ff_bemf_gain) + (Vq_cross * ff_cross_gain);

	hfoc->V_d = Vd_pi + Vd_ff;
	hfoc->V_q = Vq_pi + Vq_ff;

//	hfoc->V_d = Vd_pi;
//	hfoc->V_q = Vq_pi;

	// [5-2] 전압 포화 제한
	float32_t v_limit = g_vbus_filt;
	if (hfoc->V_d > v_limit)
		hfoc->V_d = v_limit;
	else if (hfoc->V_d < -v_limit)
		hfoc->V_d = -v_limit;

	if (hfoc->V_q > v_limit)
		hfoc->V_q = v_limit;
	else if (hfoc->V_q < -v_limit)
		hfoc->V_q = -v_limit;

	// [6] Inverse Park Transform
	arm_inv_park_f32(hfoc->V_d, hfoc->V_q, &hfoc->V_alpha, &hfoc->V_beta,
			sin_theta, cos_theta);

	// [7] Inverse Clarke Transform
	float32_t v_a, v_b;
	arm_inv_clarke_f32(hfoc->V_alpha, hfoc->V_beta, &v_a, &v_b);
	float32_t v_c = -(v_a + v_b);

	// [8] SVPWM 기반 Duty 연산 (0 ~ PWM_PERIOD)
	float32_t duty_a = (v_a / g_vbus_filt) * PWM_PERIOD + PWM_HALF_PERIOD;
	float32_t duty_b = (v_b / g_vbus_filt) * PWM_PERIOD + PWM_HALF_PERIOD;
	float32_t duty_c = (v_c / g_vbus_filt) * PWM_PERIOD + PWM_HALF_PERIOD;

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

// 7. 속도 PID 루프 (2kHz 타이머에서 모터별 호출, 위치형 + anti-windup)
void FOC_Speed_Loop(FOC_Handle_t *hfoc) {
	if (hfoc->speed_loop_en == 0)
		return;

	// [1] raw CNT 차분 (16비트 wrap 자동 처리) 후 방향 부호 적용
	uint16_t cnt = (uint16_t) hfoc->LPTIMx->Instance->CNT;
	int16_t delta = (int16_t) (cnt - hfoc->enc_prev_cnt);
	hfoc->enc_prev_cnt = cnt;
	if (hfoc->enc_dir < 0)
		delta = -delta;

	// [2] 기계각속도 -> 전기각속도 (rad/s) 계산
	float32_t omega_m_raw = ((float32_t) delta / ENCODER_RESOLUTION)
			* (2.0f * M_PI) / SPD_DT;
	float32_t omega_e_raw = omega_m_raw * MOTOR_POLE_PAIRS;

	hfoc->spd_history[hfoc->spd_hist_idx] = omega_e_raw;
		hfoc->spd_hist_idx = (hfoc->spd_hist_idx + 1) % SPD_MA_WINDOW;

		float32_t sum = 0.0f;
		for(uint8_t i = 0; i < SPD_MA_WINDOW; i++) {
			sum += hfoc->spd_history[i];
		}
		hfoc->omega_e_meas = sum / (float32_t)SPD_MA_WINDOW;

	// [3] 속도 PID - P항, I항 계산
	float32_t err = hfoc->target_omega - hfoc->omega_e_meas;
	hfoc->spd_integ += hfoc->spd_Ki * err * SPD_DT;

	// 적분 windup 방지: 적분항을 Iq 한계로 클램프
	if (hfoc->spd_integ > hfoc->iq_limit)
		hfoc->spd_integ = hfoc->iq_limit;
	if (hfoc->spd_integ < -hfoc->iq_limit)
		hfoc->spd_integ = -hfoc->iq_limit;

	// =======================================================
	// [3-1] D항: 측정값 미분 (Derivative-on-measurement) + 전용 LPF
	// =======================================================
	// 이미 LPF를 거친 omega_e_meas를 사용하여 미분 (안정성 증가)
	float32_t raw_deriv = -(hfoc->omega_e_meas - hfoc->spd_prev_meas) / SPD_DT;

	// D항 전용 필터 계수 계산 (변수명 d_alpha로 변경)
	float32_t d_alpha = SPD_DT / (SPD_D_TAU + SPD_DT);
	hfoc->spd_deriv_filt += d_alpha * (raw_deriv - hfoc->spd_deriv_filt);

	hfoc->spd_prev_meas = hfoc->omega_e_meas; // 다음 스텝을 위해 현재 속도 저장

	float32_t d_term = hfoc->spd_Kd * hfoc->spd_deriv_filt;
	// =======================================================

	// 최종 Iq 지령 계산 (P + I + D)
	float32_t iq_cmd = hfoc->spd_Kp * err + hfoc->spd_integ + d_term;

	// 출력 클램프
	if (iq_cmd > hfoc->iq_limit)
		iq_cmd = hfoc->iq_limit;
	if (iq_cmd < -hfoc->iq_limit)
		iq_cmd = -hfoc->iq_limit;

	// [4] 전류 루프에 Iq 지령 전달
	hfoc->target_Iq = iq_cmd;
	hfoc->err = err;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		FOC_Execute_Loop(&foc_R);
	} else if (hadc->Instance == ADC2) {
		FOC_Execute_Loop(&foc_L);
	}
}

// 2kHz 타이머 IRQ. 두 모터 속도 루프를 같은 dt로 처리.
void Speed_TIM_IRQ_Handler() {
	FOC_Update_VBus();
	FOC_Speed_Loop(&foc_L);
	FOC_Speed_Loop(&foc_R);
}

