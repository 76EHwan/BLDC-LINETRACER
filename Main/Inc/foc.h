#ifndef __FOC_H
#define __FOC_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"

#define Speed_TIM_IRQ_Handler TIM13_IRQ_Handler

// =========================================================
// [하드웨어 및 모터 파라미터 설정]
// =========================================================
#define MOTOR_POLE_PAIRS        1           // 모터 극쌍수 (Pole Pairs = 1)
#define ENCODER_RESOLUTION      2048.0f     // 엔코더 1회전 펄스 수
#define MOTOR_RATED_VOLTAGE     12.0f       // 시스템 전압 (V, 배터리에 맞게 수정)

#define PWM_PERIOD              4800.0f     // 타이머 ARR 주기 (Center-aligned)
#define PWM_HALF_PERIOD         (PWM_PERIOD / 2.0f)

// 전류 센싱 스케일 팩터 (ADC Raw 값 -> 실제 전류 A 로 변환)
// 공식: VREF / ADC_MAX / CSA_GAIN (또는 Shunt값에 따른 통합 계수)
#define CURRENT_SCALE           (3.3f / 65536.0f / 0.15f)
#define FOC_ADC_DMA_LENGTH      3           // DMA 버퍼 길이

// =========================================================
// [FOC 제어 핸들 구조체]
// =========================================================
typedef struct {
	// 1. 하드웨어 포인터
	TIM_TypeDef *TIMx;      // PWM 타이머 (TIM3, TIM4)
	ADC_TypeDef *ADCx;      // 전류 센싱 ADC (ADC1, ADC2)
	LPTIM_TypeDef *LPTIMx;    // 엔코더 타이머 (LPTIM1, LPTIM2)

	// 2. 제어 상태 및 플래그
	uint8_t is_running;   // 제어 루프 구동 여부
	uint8_t foc_svpwm_en; // 하드웨어 PWM 레지스터 출력 허용 여부

	// 3. 캘리브레이션 오프셋
	float32_t offset_a;     // A상 전류 센서 영점
	float32_t offset_c;     // C상 전류 센서 영점
	float32_t theta_offset; // 엔코더 전기각 0도 정렬 오프셋

	// 4. 지령치 (목표값)
	float32_t target_Id;    // 자속 제어 지령 (기본 0A)
	float32_t target_Iq;    // 토크 제어 지령 (A)

	// 5. 상태 변수
	float32_t omega_e;      // 전기각 속도
	float32_t theta_e;      // 현재 전기각 (라디안)
	float32_t omega_e_meas;   // 측정 전기각속도 (rad/s)
	float32_t target_omega;   // 속도 지령 (rad/s)
	uint16_t enc_prev_cnt;   // 직전 엔코더 CNT
	uint8_t speed_loop_en;  // 속도 루프 on/off
	int8_t enc_dir;        // 엔코더 방향: +1 정방향, -1 반전

	float32_t spd_Kp;
	float32_t spd_Ki;
	float32_t spd_integ;      // 속도 PI 적분항
	float32_t iq_limit;       // Iq 지령 상한

	// 6. 전류 피드백 변수
	float32_t I_a, I_b, I_c;
	float32_t I_alpha, I_beta;
	float32_t I_d, I_q;

	// 7. 전압 출력 변수
	float32_t V_d, V_q;
	float32_t V_alpha, V_beta;

	// 8. CMSIS-DSP PID 제어기 인스턴스
	arm_pid_instance_f32 pid_id;
	arm_pid_instance_f32 pid_iq;

} FOC_Handle_t;

// =========================================================
// [전역 변수 및 함수 프로토타입]
// =========================================================
extern FOC_Handle_t foc_L;
extern FOC_Handle_t foc_R;

extern uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
extern uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];

void FOC_ADC_Start(void);
void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_TypeDef *TIMx, ADC_TypeDef *ADCx,
		LPTIM_TypeDef *LPTIMx);
void FOC_Calibrate_Offset(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf);
void FOC_Calibrate_Encoder_Offset(FOC_Handle_t *hfoc);
void FOC_Execute_Loop(FOC_Handle_t *hfoc, volatile uint16_t *adc_buf);
void FOC_Speed_Loop(FOC_Handle_t *hfoc);
void Speed_TIM_IRQ_Handler(void);

#endif /* __FOC_H */
