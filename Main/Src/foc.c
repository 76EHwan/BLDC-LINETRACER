#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "arm_math.h"

#define PWM_PERIOD 1000.0f
#define V_OUT_MAX  1000.0f
#define V_OUT_MIN -1000.0f

arm_pid_instance_f32 pid_id;
arm_pid_instance_f32 pid_iq;

float32_t I_a, I_b, I_c;
float32_t I_alpha, I_beta;
float32_t I_d, I_q;
float32_t V_d, V_q;
float32_t V_alpha, V_beta;
float32_t theta_e = 0.0f;

float32_t target_Id = 0.0f;
float32_t target_Iq = 5.0f;

void FOC_Init(void) {
    pid_id.Kp = 1.0f;
    pid_id.Ki = 0.01f;
    pid_id.Kd = 0.0f;
    arm_pid_init_f32(&pid_id, 1);

    pid_iq.Kp = 1.0f;
    pid_iq.Ki = 0.01f;
    pid_iq.Kd = 0.0f;
    arm_pid_init_f32(&pid_iq, 1);
}

float32_t Estimate_Position(float32_t v_alpha, float32_t v_beta, float32_t i_alpha, float32_t i_beta) {
    static float32_t virtual_angle = 0.0f;
    virtual_angle += 0.005f;
    if (virtual_angle > 2.0f * PI) virtual_angle -= 2.0f * PI;
    return virtual_angle;
}

void FOC_Control_Loop(void) {
	I_a =
    I_b = -(I_a + I_c);

    theta_e = Estimate_Position(V_alpha, V_beta, I_alpha, I_beta);

    arm_clarke_f32(I_a, I_b, &I_alpha, &I_beta);

    float32_t cos_theta = arm_cos_f32(theta_e);
    float32_t sin_theta = arm_sin_f32(theta_e);

    arm_park_f32(I_alpha, I_beta, &I_d, &I_q, sin_theta, cos_theta);

    V_d = arm_pid_f32(&pid_id, target_Id - I_d);
    V_q = arm_pid_f32(&pid_iq, target_Iq - I_q);

    if (V_d > V_OUT_MAX) V_d = V_OUT_MAX;
    else if (V_d < V_OUT_MIN) V_d = V_OUT_MIN;

    if (V_q > V_OUT_MAX) V_q = V_OUT_MAX;
    else if (V_q < V_OUT_MIN) V_q = V_OUT_MIN;

    arm_inv_park_f32(V_d, V_q, &V_alpha, &V_beta, sin_theta, cos_theta);

    float32_t v_a, v_b;
    arm_inv_clarke_f32(V_alpha, V_beta, &v_a, &v_b);

    float32_t duty_a = v_a + (PWM_PERIOD / 2.0f);
    float32_t duty_b = v_b + (PWM_PERIOD / 2.0f);
    float32_t duty_c = -(v_a + v_b) + (PWM_PERIOD / 2.0f);

    if (duty_a > PWM_PERIOD) duty_a = PWM_PERIOD;
    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_b > PWM_PERIOD) duty_b = PWM_PERIOD;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_c > PWM_PERIOD) duty_c = PWM_PERIOD;
    if (duty_c < 0.0f) duty_c = 0.0f;

    TIM3->CCR1 = (uint32_t)duty_a;
    TIM3->CCR2 = (uint32_t)duty_b;
    TIM3->CCR3 = (uint32_t)duty_c;
}
