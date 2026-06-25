#ifndef FOC_H
#define FOC_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"

// 하드웨어 스펙에 따른 상수 설정
#define PWM_PERIOD      4800.0f  // TIM3, TIM4의 ARR 값 (25kHz 중앙 정렬)
#define PWM_HALF_PERIOD 2400.0f  // Duty 50% (0V 출력 기준점)
#define CURRENT_SCALE   (3.3f / 65535.f / 0.15f)   // [예시] ADC 1비트당 실제 전류[A] 변환 상수

#define MOTOR_RATED_VOLTAGE 7.4f  // 모터 정격 전압 [V]
#define LIPO_4S_CUTOFF     12.0f  // 배터리 보호 컷오프 전압 [V]
#define BAT_VOLTAGE_SCALE  (1.f / (1.f + 10.f))

#define FOC_ADC_DMA_LENGTH 3

// 개별 모터 제어를 위한 FOC 인스턴스 구조체
typedef struct {
    // 하드웨어 연결
    TIM_TypeDef *TIMx;

    // 제어 상태
    uint8_t is_running;

    // PID 제어기
    arm_pid_instance_f32 pid_id;
    arm_pid_instance_f32 pid_iq;
    float32_t omega_e;

    // 전류 센서 오프셋 (캘리브레이션으로 구한 값)
    float32_t offset_a;
    float32_t offset_c; // b상은 -(a+c)로 계산하므로 a, c만 읽음

    // 측정 및 연산 변수
    float32_t I_a, I_b, I_c;
    float32_t I_alpha, I_beta;
    float32_t I_d, I_q;
    float32_t V_d, V_q;
    float32_t V_alpha, V_beta;
    float32_t theta_e;

    // 목표치 (User Input)
    float32_t target_Id;
    float32_t target_Iq;

    uint8_t foc_svpwm_en;
} FOC_Handle_t;

// 외부에서 사용할 전역 인스턴스 선언
extern FOC_Handle_t foc_L;
extern FOC_Handle_t foc_R;

extern __attribute__((section(".ram_d2_nocache"), aligned(32))) uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
extern __attribute__((section(".ram_d2_nocache"), aligned(32))) uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];


// 함수 프로토타입
void FOC_Calibrate_Offset(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf);
void FOC_ADC_Start();
void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_TypeDef *TIMx);
void FOC_Execute_Loop(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf);

#endif // FOC_H
