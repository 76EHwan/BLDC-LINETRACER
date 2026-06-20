#include "foc.h"
#include "adc.h"

FOC_Handle_t foc_L;
FOC_Handle_t foc_R;

#define FOC_ADC_DMA_LENGTH 3
uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH]; // 왼쪽 모터 센서
uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH]; // 오른쪽 모터 센서

// 데시메이션 (건너뛰기) 상수: 25kHz -> 12.5kHz로 연산 부하 절반 감소
#define FOC_DECIMATION 2
static uint8_t foc_tick_L = 0;
static uint8_t foc_tick_R = 0;

// 출력 전압(Duty) 포화 제한용 상수
#define V_OUT_MAX  (PWM_HALF_PERIOD)
#define V_OUT_MIN -(PWM_HALF_PERIOD)

void FOC_ADC_Start(){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc1_dma_buf, FOC_ADC_DMA_LENGTH);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) adc2_dma_buf, FOC_ADC_DMA_LENGTH);
}

// 1. 초기화 함수 (main에서 1회 실행)
void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_TypeDef *TIMx) {
	hfoc->TIMx = TIMx;
	hfoc->is_running = 0; // 초기 상태: 정지

	hfoc->target_Id = 0.0f;
	hfoc->target_Iq = 0.0f; // 목표 전류 0A

	hfoc->pid_id.Kp = 1.0f;
	hfoc->pid_id.Ki = 0.01f;
	hfoc->pid_id.Kd = 0.0f;
	arm_pid_init_f32(&hfoc->pid_id, 1);

	hfoc->pid_iq.Kp = 1.0f;
	hfoc->pid_iq.Ki = 0.01f;
	hfoc->pid_iq.Kd = 0.0f;
	arm_pid_init_f32(&hfoc->pid_iq, 1);
}

// 2. 센서리스 각도 추정 (임시 - 실제 옵저버나 엔코더 코드로 교체 필요)
static float32_t Estimate_Position(FOC_Handle_t *hfoc) {
	static float32_t virtual_angle = 0.0f;
	virtual_angle += 0.005f; // 가상의 속도로 계속 회전
	if (virtual_angle > 2.0f * PI)
		virtual_angle -= 2.0f * PI;
	return virtual_angle;
}

// 3. FOC 메인 루프 (ADC DMA 콜백에서 호출됨)
void FOC_Execute_Loop(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf) {

    if (hfoc->is_running == 0) return;

    // ==========================================================
    // 1. 배터리 전압 모니터링 및 LiPo 보호 (LVC)
    // ==========================================================
    float32_t V_bat = (float32_t)adc_buf[0] * BAT_VOLTAGE_SCALE;

    // ==========================================================
    // 2. 전류 읽기 및 FOC 변환 (기존과 동일)
    // ==========================================================
    hfoc->I_a = ((float32_t)adc_buf[1] - hfoc->offset_a) * CURRENT_SCALE;
    hfoc->I_c = ((float32_t)adc_buf[2] - hfoc->offset_c) * CURRENT_SCALE;
    hfoc->I_b = -(hfoc->I_a + hfoc->I_c);

    hfoc->theta_e = Estimate_Position(hfoc);
    arm_clarke_f32(hfoc->I_a, hfoc->I_b, &hfoc->I_alpha, &hfoc->I_beta);

    float32_t cos_theta = arm_cos_f32(hfoc->theta_e);
    float32_t sin_theta = arm_sin_f32(hfoc->theta_e);
    arm_park_f32(hfoc->I_alpha, hfoc->I_beta, &hfoc->I_d, &hfoc->I_q, sin_theta, cos_theta);

    // ==========================================================
    // 3. PID 제어 및 모터 소손 방지 (Voltage Clamping)
    // ==========================================================
    hfoc->V_d = arm_pid_f32(&hfoc->pid_id, hfoc->target_Id - hfoc->I_d);
    hfoc->V_q = arm_pid_f32(&hfoc->pid_iq, hfoc->target_Iq - hfoc->I_q);

    // V_d와 V_q의 벡터합(실제 모터에 인가되는 총 전압) 크기 계산
    float32_t v_out_mag = sqrtf(hfoc->V_d * hfoc->V_d + hfoc->V_q * hfoc->V_q);

    // 만약 PID가 7.4V 이상을 요구하면, 7.4V에 맞춰서 비율을 축소! (모터 보호)
    if (v_out_mag > MOTOR_RATED_VOLTAGE) {
        float32_t scale = MOTOR_RATED_VOLTAGE / v_out_mag;
        hfoc->V_d *= scale;
        hfoc->V_q *= scale;
    }

    // ==========================================================
    // 4. 역변환 (기존과 동일)
    // ==========================================================
    arm_inv_park_f32(hfoc->V_d, hfoc->V_q, &hfoc->V_alpha, &hfoc->V_beta, sin_theta, cos_theta);

    float32_t v_a, v_b;
    arm_inv_clarke_f32(hfoc->V_alpha, hfoc->V_beta, &v_a, &v_b);
    float32_t v_c = -(v_a + v_b);

    // ==========================================================
    // 5. [핵심] 전압 피드포워드 (배터리 전압 비례 Duty 변환)
    // ==========================================================
    // v_a, v_b, v_c는 현재 -7.4V ~ +7.4V 사이의 물리적 '전압'입니다.
    // 이를 실시간 배터리 전압(V_bat)으로 나누어 Duty Ratio(-0.5 ~ +0.5)로 바꿉니다.
    // 그리고 PWM 주기(4800)를 곱한 뒤 중심점(2400)을 더해줍니다.

    float32_t duty_a = (v_a / V_bat) * PWM_PERIOD + PWM_HALF_PERIOD;
    float32_t duty_b = (v_b / V_bat) * PWM_PERIOD + PWM_HALF_PERIOD;
    float32_t duty_c = (v_c / V_bat) * PWM_PERIOD + PWM_HALF_PERIOD;

    // 최종 Duty Limit (하드웨어 보호용 절대 한계치)
    if (duty_a > PWM_PERIOD) duty_a = PWM_PERIOD;
    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_b > PWM_PERIOD) duty_b = PWM_PERIOD;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_c > PWM_PERIOD) duty_c = PWM_PERIOD;
    if (duty_c < 0.0f) duty_c = 0.0f;

    // 타이머 레지스터 업데이트
    hfoc->TIMx->CCR1 = (uint32_t)duty_a;
    hfoc->TIMx->CCR2 = (uint32_t)duty_b;
    hfoc->TIMx->CCR3 = (uint32_t)duty_c;
}

void ADC1_IRQ_Handler() {
	foc_tick_L++;
	if (foc_tick_L >= FOC_DECIMATION) {
		foc_tick_L = 0;
		FOC_Execute_Loop(&foc_L, adc1_dma_buf);
	}
}

void ADC2_IRQ_Handler() {
	foc_tick_R++;
	if (foc_tick_R >= FOC_DECIMATION) {
		foc_tick_R = 0;
		FOC_Execute_Loop(&foc_R, adc2_dma_buf);
	}
}

