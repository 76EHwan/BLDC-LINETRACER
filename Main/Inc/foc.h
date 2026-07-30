#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "tim.h"
#include "arm_math.h"

#define TIM_SPEED_LOOP	&htim13
#define Speed_TIM_IRQ_Handler TIM13_IRQ_Handler

// =========================================================
// [íëì¨ì´ ë° ëª¨í° íë¼ë¯¸í° ì¤ì ]
// =========================================================
#define MOTOR_POLE_PAIRS        1           // ëª¨í° ê·¹ìì (Pole Pairs = 1)
#define ENCODER_RESOLUTION      2048.0f     // ìì½ë 1íì  íì¤ ì
#define MOTOR_RATED_VOLTAGE     16.8f       // ìì¤í ì ì (V, ë°°í°ë¦¬ì ë§ê² ìì )

#define PWM_PERIOD              4800.0f     // íì´ë¨¸ ARR ì£¼ê¸° (Center-aligned)
#define PWM_HALF_PERIOD         (PWM_PERIOD / 2.0f)

// ì ë¥ ì¼ì± ì¤ì¼ì¼ í©í° (ADC Raw ê° -> ì¤ì  ì ë¥ A ë¡ ë³í)
// ê³µì: VREF / ADC_MAX / CSA_GAIN (ëë Shuntê°ì ë°ë¥¸ íµí© ê³ì)
#define CURRENT_CSA_GAIN_V_MA		300
#define CURRENT_CSA_GAIN_V_A		(CURRENT_CSA_GAIN_V_MA / 1000.f)
#define CURRENT_SCALE           (3.3f / 65536.0f / CURRENT_CSA_GAIN_V_A)
#define FOC_ADC_DMA_LENGTH      1           // DMA ë²í¼ ê¸¸ì´

#define SPD_DT         	0.0005f
#define SPD_D_TAU       0.001f       // Dí­ LPF ìì ì (2kHz ëë¹ 4ìí ì ë)

#define SPD_IQ_LIMIT     5.f        // Iq ì§ë ¹ ìí (A)

// =========================================================
// [ëª¨í° ì ê¸°ì  íë¼ë¯¸í° - maxon ECX SPEED 16 M, 36V ê¶ì  ê¸°ì¤]
// =========================================================
#define MOTOR_PARAM_TERMINAL_RESISTOR	1.92f	// [Î©] ë¨ìê° ì í­
#define MOTOR_PARAM_TERMINAL_INDUCTANCE	0.129	// [mH] ë¨ìê° ì¸ëí´ì¤
#define MOTOR_PARAM_PHASE_RESISTOR		(MOTOR_PARAM_TERMINAL_RESISTOR / 2.0f)
#define MOTOR_PARAM_PHASE_INDUCTANCE	(MOTOR_PARAM_TERMINAL_INDUCTANCE / 2.0f)  // [mH]

// ì¸ëí´ì¤ë ê³ì° í¸ìì mHë¡ ì ìëì´ ìì¼ë¯ë¡, ë¬¼ë¦¬ ê³ì° ì H ë¨ìë¡ ë³íí´ì ì¬ì©
#define MOTOR_PHASE_INDUCTANCE_H		(MOTOR_PARAM_PHASE_INDUCTANCE / 1000.0f)  // [H]

// í í¬ìì(ì¹´íë¡ê·¸, 36V ê¶ì ) -> ìììêµì ì­ì°
// coreless ëª¨í°ë Ld = Lq (ëê·¹ì± ìì) ì´ë¯ë¡ Te = 1.5 * P * Î» * Iq ë¡ ì íí ì±ë¦½
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

#define SPD_MA_WINDOW 2  // 4~8 ì ëì ìì ê° ì¶ì² (ì§ì°ê³¼ ë¸ì´ì¦ì ííì )

// =========================================================
// [FOC ì ì´ í¸ë¤ êµ¬ì¡°ì²´]
// =========================================================
typedef struct {
	// 1. íëì¨ì´ í¬ì¸í°
	TIM_HandleTypeDef *TIMx;      // PWM íì´ë¨¸ (TIM3, TIM4)
	ADC_HandleTypeDef *ADCx;      // ì ë¥ ì¼ì± ADC (ADC1, ADC2)
	LPTIM_HandleTypeDef *LPTIMx;    // ìì½ë íì´ë¨¸ (LPTIM1, LPTIM2)

	// 2. ì ì´ ìí ë° íëê·¸
	uint8_t is_running;   // ì ì´ ë£¨í êµ¬ë ì¬ë¶
	uint8_t foc_svpwm_en; // íëì¨ì´ PWM ë ì§ì¤í° ì¶ë ¥ íì© ì¬ë¶

	// 3. ìºë¦¬ë¸ë ì´ì ì¤íì
	float32_t offset_a;     // Aì ì ë¥ ì¼ì ìì 
	float32_t offset_c;     // Cì ì ë¥ ì¼ì ìì 
	float32_t theta_offset; // ìì½ë ì ê¸°ê° 0ë ì ë ¬ ì¤íì

	// 4. ì§ë ¹ì¹ (ëª©íê°)
	float32_t target_Id;    // ìì ì ì´ ì§ë ¹ (ê¸°ë³¸ 0A)
	float32_t target_Iq;    // í í¬ ì ì´ ì§ë ¹ (A)

	// 5. ìí ë³ì
	float32_t omega_e;      // ì ê¸°ê° ìë
	float32_t theta_e;      // íì¬ ì ê¸°ê° (ë¼ëì)
	float32_t omega_e_meas;   // ì¸¡ì  ì ê¸°ê°ìë (rad/s)
	float32_t target_omega;   // ìë ì§ë ¹ (rad/s)
	uint16_t enc_prev_cnt;   // ì§ì  ìì½ë CNT
	uint8_t speed_loop_en;  // ìë ë£¨í on/off
	int8_t enc_dir;        // ìì½ë ë°©í¥: +1 ì ë°©í¥, -1 ë°ì 
	float_t err;

	float32_t spd_Kp;
	float32_t spd_Ki;
	float32_t spd_Kd;
	float32_t spd_integ;      // ìë PI ì ë¶í­
	float32_t iq_limit;       // Iq ì§ë ¹ ìí

	float32_t spd_prev_meas;   // ì´ì  ì¤í ì¸¡ì  ìë (Dí­ì©)
	float32_t spd_deriv_filt;  // íí°ë§ë ë¯¸ë¶ê°

	// 6. ì ë¥ í¼ëë°± ë³ì
	float32_t I_a, I_b, I_c;
	float32_t I_alpha, I_beta;
	float32_t I_d, I_q;

	// 7. ì ì ì¶ë ¥ ë³ì
	float32_t V_d, V_q;
	float32_t V_alpha, V_beta;

	// 8. CMSIS-DSP PID ì ì´ê¸° ì¸ì¤í´ì¤
	arm_pid_instance_f32 pid_id;
	arm_pid_instance_f32 pid_iq;

	float32_t omega_setpoint;
	float32_t omega_ramp_rate;

	float32_t spd_history[SPD_MA_WINDOW];
	uint8_t spd_hist_idx;

	float32_t pll_theta_est; // PLLë¡ ì¶ì ë ê¸°ê³ê° ìì¹ (rad, 0 ~ 2*PI)
	float32_t pll_omega_integ;  // PLL ë£¨í íí°ì ì ë¶í­ (rad/s)
	float32_t pll_omega_est;    // PLLë¡ ì¶ì ë ê¸°ê³ê°ìë (rad/s)

	float32_t pll_kp;           // PLL ë¹ë¡ ê²ì¸ (ì¶ì² ì´ê¸°ê°: 200.0f)
	float32_t pll_ki;          // PLL ì ë¶ ê²ì¸ (ì¶ì² ì´ê¸°ê°: 10000.0f)

	float32_t ramped_omega;  // 램프 처리가 적용된 내부 제어용 목표 속도

} FOC_Handle_t;

// =========================================================
// [ì ì­ ë³ì ë° í¨ì íë¡í íì]
// =========================================================
extern FOC_Handle_t foc_L;
extern FOC_Handle_t foc_R;

extern uint16_t adc1_dma_buf[FOC_ADC_DMA_LENGTH];
extern uint16_t adc2_dma_buf[FOC_ADC_DMA_LENGTH];

extern float_t g_odom_distance_m;

float32_t FOC_Get_VBus(void);

void FOC_ADC_Start(void);
void FOC_ADC_Stop(void);

void FOC_Reset_State(FOC_Handle_t *hfoc);
void FOC_Init_Motor(FOC_Handle_t *hfoc, TIM_HandleTypeDef *TIMx,
		ADC_HandleTypeDef *ADCx, LPTIM_HandleTypeDef *LPTIMx);
void FOC_Calibrate_Offset(FOC_Handle_t *hfoc);
void FOC_Calibrate_Encoder_Offset(FOC_Handle_t *hfoc);
void FOC_Calibrate_Encoder_Offset_Both(FOC_Handle_t *hfoc_L,
		FOC_Handle_t *hfoc_R);
void FOC_Update_Theta_Encoder(FOC_Handle_t *hfoc);

float_t FOC_Meas_Mps(FOC_Handle_t *hfoc);
void Odom_Reset(void);
void Odom_Accumulate(float dt_sec);

void FOC_Execute_Loop(FOC_Handle_t *hfoc);
void FOC_Speed_Loop(FOC_Handle_t *hfoc);
void Speed_TIM_IRQ_Handler(void);

#endif /* __FOC_H */
