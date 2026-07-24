#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "tim.h"
#include "arm_math.h"

#define TIM_SPEED_LOOP	&htim13
#define Speed_TIM_IRQ_Handler TIM13_IRQ_Handler

// =========================================================
// [하드웨어 및 모터 파라미터 설정]
// =========================================================
#define MOTOR_POLE_PAIRS        1           // 모터 극쌍수 (Pole Pairs = 1)
#define ENCODER_RESOLUTION      2048.0f     // 엔코더 1회전 펄스 수
#define MOTOR_RATED_VOLTAGE     16.8f       // 시스템 전압 (V, 배터리에 맞게 수정)

#define PWM_PERIOD              4800.0f     // 타이머 ARR 주기 (Center-aligned)
#define PWM_HALF_PERIOD         (PWM_PERIOD / 2.0f)

// 전류 센싱 스케일 팩터 (ADC Raw 값 -> 실제 전류 A 로 변환)
// 공식: VREF / ADC_MAX / CSA_GAIN (또는 Shunt값에 따른 통합 계수)
#define CURRENT_CSA_GAIN_MA		300
#define CURRENT_SCALE           (3.3f / 65536.0f / CURRENT_CSA_GAIN_MA / 1000.f)
#define FOC_ADC_DMA_LENGTH      1           // DMA 버퍼 길이

#define SPD_DT         	0.0005f
#define SPD_D_TAU       0.001f       // D항 LPF 시정수 (2kHz 대비 4샘플 정도)

#define SPD_IQ_LIMIT     2.f        // Iq 지령 상한 (A)

// =========================================================
// [모터 전기적 파라미터 - maxon ECX SPEED 16 M, 36V 권선 기준]
// =========================================================
#define MOTOR_PARAM_TERMINAL_RESISTOR	1.92f	// [Ω] 단자간 저항
#define MOTOR_PARAM_TERMINAL_INDUCTANCE	0.129	// [mH] 단자간 인덕턴스
#define MOTOR_PARAM_PHASE_RESISTOR		(MOTOR_PARAM_TERMINAL_RESISTOR / 2.0f)
#define MOTOR_PARAM_PHASE_INDUCTANCE	(MOTOR_PARAM_TERMINAL_INDUCTANCE / 2.0f)  // [mH]

// 인덕턴스는 계산 편의상 mH로 정의되어 있으므로, 물리 계산 시 H 단위로 변환해서 사용
#define MOTOR_PHASE_INDUCTANCE_H		(MOTOR_PARAM_PHASE_INDUCTANCE / 1000.0f)  // [H]

// 토크상수(카탈로그, 36V 권선) -> 자속쇄교수 역산
// coreless 모터는 Ld = Lq (돌극성 없음) 이므로 Te = 1.5 * P * λ * Iq 로 정확히 성립
#define MOTOR_TORQUE_CONSTANT_MNM_A		6.46f                          // [mNm/A]
#define MOTOR_TORQUE_CONSTANT			(MOTOR_TORQUE_CONSTANT_MNM_A / 1000.0f)  // [Nm/A]
#define MOTOR_FLUX_LINKAGE				(MOTOR_TORQUE_CONSTANT / (1.5f * MOTOR_POLE_PAIRS))  // [Wb]

#define FOC_CONTROL_FREQUENCY 			(240000000.0f / (PWM_PERIOD) / 2.0f)
#define FOC_CONTROL_DT					(1.0f / (FOC_CONTROL_FREQUENCY))
#define FOC_CURRENT_BW_RATIO			(20.f)
#define CURRENT_CONTROL_BANDWIDTH		(((FOC_CONTROL_FREQUENCY) / (FOC_CURRENT_BW_RATIO)) * 2.0f * PI)
#define DEFAULT_ID_KP					((CURRENT_CONTROL_BANDWIDTH) * (MOTOR_PARAM_PHASE_INDUCTANCE) / 1000.0f)
#define DEFAULT_ID_KI					((CURRENT_CONTROL_BANDWIDTH) * (MOTOR_PARAM_PHASE_RESISTOR) * (FOC_CONTROL_DT))
#define DEFAULT_IQ_KP					DEFAULT_ID_KP
#define DEFAULT_IQ_KI					DEFAULT_ID_KI

#define VBUS_DIVIDER_RATIO   19.0f
#define VBUS_ADC_VREF        3.3f
#define VBUS_ADC_SCALE       (VBUS_ADC_VREF / 65536.0f * VBUS_DIVIDER_RATIO)

#define SPD_MA_WINDOW 4  // 4~8 정도의 작은 값 추천 (지연과 노이즈의 타협점)

// =========================================================
// [FOC 제어 핸들 구조체]
// =========================================================
typedef struct {
	// 1. 하드웨어 포인터
	TIM_HandleTypeDef *TIMx;      // PWM 타이머 (TIM3, TIM4)
	ADC_HandleTypeDef *ADCx;      // 전류 센싱 ADC (ADC1, ADC2)
	LPTIM_HandleTypeDef *LPTIMx;    // 엔코더 타이머 (LPTIM1, LPTIM2)

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
	float_t err;

	float32_t spd_Kp;
	float32_t spd_Ki;
	float32_t spd_Kd;
	float32_t spd_integ;      // 속도 PI 적분항
	float32_t iq_limit;       // Iq 지령 상한

	float32_t spd_prev_meas;   // 이전 스텝 측정 속도 (D항용)
	float32_t spd_deriv_filt;  // 필터링된 미분값

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

	float32_t omega_setpoint;
	float32_t omega_ramp_rate;

	float32_t spd_history[SPD_MA_WINDOW];
	uint8_t spd_hist_idx;

	float32_t pll_theta_est;    // PLL로 추정된 기계각 위치 (rad, 0 ~ 2*PI)
	float32_t pll_omega_integ;  // PLL 루프 필터의 적분항 (rad/s)
	float32_t pll_omega_est;    // PLL로 추정된 기계각속도 (rad/s)

	float32_t pll_kp;           // PLL 비례 게인 (추천 초기값: 200.0f)
	float32_t pll_ki;           // PLL 적분 게인 (추천 초기값: 10000.0f)

} FOC_Handle_t;

// =========================================================
// [전역 변수 및 함수 프로토타입]
// =========================================================
extern FOC_Handle_t foc_L;
extern FOC_Handle_t foc_R;

extern uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
extern uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];

extern float_t g_odom_distance_m;

float32_t FOC_Get_VBus(void);

void FOC_ADC_Start(void);
void FOC_Reset_State(FOC_Handle_t *hfoc);
void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_HandleTypeDef *TIMx,
		ADC_HandleTypeDef *ADCx, LPTIM_HandleTypeDef *LPTIMx);
void FOC_Calibrate_Offset(FOC_Handle_t *hfoc);
void FOC_Calibrate_Encoder_Offset(FOC_Handle_t *hfoc);
void FOC_Calibrate_Encoder_Offset_Both(FOC_Handle_t *hfoc_L, FOC_Handle_t *hfoc_R);
void FOC_Update_Theta_Encoder(FOC_Handle_t *hfoc);

float_t FOC_Meas_Mps(FOC_Handle_t *hfoc);
void Odom_Reset(void);
void Odom_Accumulate(float dt_sec);

void FOC_Execute_Loop(FOC_Handle_t *hfoc);
void FOC_Speed_Loop(FOC_Handle_t *hfoc);
void Speed_TIM_IRQ_Handler(void);

#endif /* __FOC_H */
