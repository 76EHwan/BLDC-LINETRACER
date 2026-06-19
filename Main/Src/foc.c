#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "arm_math.h" // CMSIS-DSP 라이브러리 포함 (FPU 최적화)

// 상수 정의
#define SQRT3      1.73205081f
#define PWM_PERIOD 1000.0f

// CMSIS-DSP 고속 PID 인스턴스
arm_pid_instance_f32 pid_id;
arm_pid_instance_f32 pid_iq;

// 모터 상태 변수
float32_t I_a, I_b, I_c;
float32_t I_alpha, I_beta;
float32_t I_d, I_q;
float32_t V_d, V_q;
float32_t V_alpha, V_beta;
float32_t theta_e = 0.0f;

// 목표 값
float32_t target_Id = 0.0f;
float32_t target_Iq = 5.0f;

// 전압 제한 (Anti-windup 및 출력 포화용)
#define V_OUT_MAX  1000.0f
#define V_OUT_MIN -1000.0f

// 1. 초기화 함수 (main 함수 초기에 1회 실행)
void FOC_Init(void) {
    // Id PID 제어기 게인 설정 및 초기화
    pid_id.Kp = 1.0f;
    pid_id.Ki = 0.01f;
    pid_id.Kd = 0.0f;
    arm_pid_init_f32(&pid_id, 1);

    // Iq PID 제어기 게인 설정 및 초기화
    pid_iq.Kp = 1.0f;
    pid_iq.Ki = 0.01f;
    pid_iq.Kd = 0.0f;
    arm_pid_init_f32(&pid_iq, 1);
}

// 2. 센서리스 위치 추정기 (간이)
float32_t Estimate_Position(float32_t v_alpha, float32_t v_beta, float32_t i_alpha, float32_t i_beta) {
    static float32_t virtual_angle = 0.0f;
    virtual_angle += 0.005f;
    if (virtual_angle > 2.0f * PI) virtual_angle -= 2.0f * PI;
    return virtual_angle;
}

// 3. 고속 FOC 제어 루프 (TIM4 인터럽트에서 주기적 실행)
void FOC_Control_Loop(void) {
    // 1. 전류 측정
//    I_a = Get_PhaseCurrent_A();
//    I_b = Get_PhaseCurrent_B();
    I_c = -(I_a + I_b);

    // 2. 전기각 추정
    theta_e = Estimate_Position(V_alpha, V_beta, I_alpha, I_beta);

    // 3. Clarke Transform (사칙연산은 STM32H7의 FPU가 1사이클에 하드웨어 처리)
    I_alpha = I_a;
    I_beta = (I_a + 2.0f * I_b) / SQRT3;

    // 4. 고속 삼각함수 연산 (CMSIS-DSP 활용)
    // 일반 math.h의 cosf(), sinf()보다 압도적으로 빠릅니다.
    float32_t cos_theta = arm_cos_f32(theta_e);
    float32_t sin_theta = arm_sin_f32(theta_e);

    // 5. Park Transform
    I_d = I_alpha * cos_theta + I_beta * sin_theta;
    I_q = -I_alpha * sin_theta + I_beta * cos_theta;

    // 6. 고속 PID 제어 (CMSIS-DSP 활용)
    // 오차(Error)를 계산하여 PID 함수에 넣으면 바로 제어값이 출력됩니다.
    V_d = arm_pid_f32(&pid_id, target_Id - I_d);
    V_q = arm_pid_f32(&pid_iq, target_Iq - I_q);

    // 출력 전압 포화(Saturation) 처리
    if (V_d > V_OUT_MAX) V_d = V_OUT_MAX;
    else if (V_d < V_OUT_MIN) V_d = V_OUT_MIN;

    if (V_q > V_OUT_MAX) V_q = V_OUT_MAX;
    else if (V_q < V_OUT_MIN) V_q = V_OUT_MIN;

    // 7. Inverse Park Transform
    V_alpha = V_d * cos_theta - V_q * sin_theta;
    V_beta = V_d * sin_theta + V_q * cos_theta;

    // 8. SVPWM (Inverse Clarke & Duty 계산)
    float32_t duty_a = (V_alpha) + (PWM_PERIOD / 2.0f);
    float32_t duty_b = (-0.5f * V_alpha + (SQRT3 / 2.0f) * V_beta) + (PWM_PERIOD / 2.0f);
    float32_t duty_c = (-0.5f * V_alpha - (SQRT3 / 2.0f) * V_beta) + (PWM_PERIOD / 2.0f);

    // Duty Limit
    if(duty_a > PWM_PERIOD) duty_a = PWM_PERIOD;
    if(duty_a < 0.0f) duty_a = 0.0f;
    if(duty_b > PWM_PERIOD) duty_b = PWM_PERIOD;
    if(duty_b < 0.0f) duty_b = 0.0f;
    if(duty_c > PWM_PERIOD) duty_c = PWM_PERIOD;
    if(duty_c < 0.0f) duty_c = 0.0f;

    // 9. TIM3 PWM 레지스터 업데이트
    TIM3->CCR1 = (uint32_t)duty_a;
    TIM3->CCR2 = (uint32_t)duty_b;
    TIM3->CCR3 = (uint32_t)duty_c;
}
