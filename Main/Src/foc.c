#include "foc.h"
#include "adc.h"

FOC_Handle_t foc_L;
FOC_Handle_t foc_R;

__attribute__((section(".ram_d2_nocache"), aligned(32))) uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
__attribute__((section(".ram_d2_nocache"), aligned(32))) uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];

#define FOC_DECIMATION 2
static uint8_t foc_tick_L = 0;
static uint8_t foc_tick_R = 0;

void FOC_ADC_Start() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buf, FOC_ADC_DMA_LENGTH);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buf, FOC_ADC_DMA_LENGTH);
}

void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_TypeDef *TIMx) {
    hfoc->TIMx       = TIMx;
    hfoc->is_running = 0;

    hfoc->offset_a = (float32_t)(UINT16_MAX / 2);
    hfoc->offset_c = (float32_t)(UINT16_MAX / 2);

    hfoc->target_Id = 0.0f;
    hfoc->target_Iq = 0.0f;
    hfoc->omega_e   = 0.0f;
    hfoc->theta_e   = 0.0f;

    hfoc->pid_id.Kp = 1.0f;
    hfoc->pid_id.Ki = 0.01f;
    hfoc->pid_id.Kd = 0.0f;
    arm_pid_init_f32(&hfoc->pid_id, 1);

    hfoc->pid_iq.Kp = 1.0f;
    hfoc->pid_iq.Ki = 0.01f;
    hfoc->pid_iq.Kd = 0.0f;
    arm_pid_init_f32(&hfoc->pid_iq, 1);

    hfoc->foc_svpwm_en = 0;
}

void FOC_Calibrate_Offset(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf) {
    uint32_t sum_a = 0, sum_c = 0;
    const int N = 1000;
    for (int i = 0; i < N; i++) {
        sum_a += adc_buf[1];
        sum_c += adc_buf[2];
        HAL_Delay(1);
    }
    hfoc->offset_a = (float32_t)(sum_a / N);
    hfoc->offset_c = (float32_t)(sum_c / N);
}

static void FOC_Update_Theta(FOC_Handle_t *hfoc) {
    hfoc->theta_e += hfoc->omega_e / 12500.0f;
    if (hfoc->theta_e > (float32_t)(2.0 * M_PI)) hfoc->theta_e -= (float32_t)(2.0 * M_PI);
    if (hfoc->theta_e < 0.0f)                     hfoc->theta_e += (float32_t)(2.0 * M_PI);
}

void FOC_Execute_Loop(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf) {
    if (hfoc->is_running == 0) return;

    FOC_Update_Theta(hfoc);

    hfoc->I_a = ((float32_t)adc_buf[1] - hfoc->offset_a) * CURRENT_SCALE;
    hfoc->I_c = ((float32_t)adc_buf[2] - hfoc->offset_c) * CURRENT_SCALE;
    hfoc->I_b = -(hfoc->I_a + hfoc->I_c);

    arm_clarke_f32(hfoc->I_a, hfoc->I_b, &hfoc->I_alpha, &hfoc->I_beta);

    float32_t cos_theta = arm_cos_f32(hfoc->theta_e);
    float32_t sin_theta = arm_sin_f32(hfoc->theta_e);
    arm_park_f32(hfoc->I_alpha, hfoc->I_beta, &hfoc->I_d, &hfoc->I_q,
                 sin_theta, cos_theta);

    // 오픈루프: target_Iq를 직접 전압으로
    hfoc->V_d = 0.0f;
    hfoc->V_q = hfoc->target_Iq;

    arm_inv_park_f32(hfoc->V_d, hfoc->V_q, &hfoc->V_alpha, &hfoc->V_beta,
                     sin_theta, cos_theta);

    float32_t v_a, v_b;
    arm_inv_clarke_f32(hfoc->V_alpha, hfoc->V_beta, &v_a, &v_b);
    float32_t v_c = -(v_a + v_b);

    float32_t duty_a = (v_a / MOTOR_RATED_VOLTAGE) * PWM_PERIOD + PWM_HALF_PERIOD;
    float32_t duty_b = (v_b / MOTOR_RATED_VOLTAGE) * PWM_PERIOD + PWM_HALF_PERIOD;
    float32_t duty_c = (v_c / MOTOR_RATED_VOLTAGE) * PWM_PERIOD + PWM_HALF_PERIOD;

    duty_a = duty_a < 0.0f ? 0.0f : (duty_a > PWM_PERIOD ? PWM_PERIOD : duty_a);
    duty_b = duty_b < 0.0f ? 0.0f : (duty_b > PWM_PERIOD ? PWM_PERIOD : duty_b);
    duty_c = duty_c < 0.0f ? 0.0f : (duty_c > PWM_PERIOD ? PWM_PERIOD : duty_c);

    if (hfoc->foc_svpwm_en) {
        hfoc->TIMx->CCR1 = (uint32_t)duty_a;
        hfoc->TIMx->CCR2 = (uint32_t)duty_b;
        hfoc->TIMx->CCR3 = (uint32_t)duty_c;
    }
}

void ADC1_IRQ_Handler() {
    foc_tick_R++;
    if (foc_tick_R >= FOC_DECIMATION) {
        foc_tick_R = 0;
        FOC_Execute_Loop(&foc_R, adc1_dma_buf);
    }
}

void ADC2_IRQ_Handler() {
    foc_tick_L++;
    if (foc_tick_L >= FOC_DECIMATION) {
        foc_tick_L = 0;
        FOC_Execute_Loop(&foc_L, adc2_dma_buf);
    }
}
