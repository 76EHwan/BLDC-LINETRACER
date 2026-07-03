#include "user_init.h"
#include "button.h"
#include "motor.h"
#include "foc.h"
#include "lptim.h"
#include "math.h"

#ifdef FOC_CONTROL
#include "drv8316crq1.h"

#define MTR_L &DRV8316C_L
#define MTR_R &DRV8316C_R

#define MTR_WAKEUP      DRV8316C_WAKEUP
#define MTR_SLEEP       DRV8316C_SLEEP
#define MTR_DRVOFF_LOW  DRV8316C_DRVOFF_LOW
#define MTR_DRVOFF_HIGH DRV8316C_DRVOFF_HIGH
#define MTR_FOC_PWM_EN  DRV8316C_FOC_PWM_EN
#define MTR_FOC_PWM_DIS DRV8316C_FOC_PWM_DIS

#define MTR_ReadRegister   	DRV8316C_ReadRegister
#define MTR_WriteRegister	DRV8316C_WriteRegister
#define MTR_UpdateRegister	DRV8316C_ApplyDefaultConfig
#define MTR_UnlockRegister	DRV8316C_UnlockRegister
#define MTR_LockRegister	DRV8316C_LockRegister

#define MTR_REG_IC_STATUS DRV_REG_IC_STATUS
#define MTR_REG_STATUS_1  DRV_REG_STATUS_1
#define MTR_REG_STATUS_2  DRV_REG_STATUS_2
#define MTR_REG_CTRL_2    DRV_REG_CTRL_2
#define MTR_REG_CTRL_3    DRV_REG_CTRL_3
#define MTR_REG_CTRL_4    DRV_REG_CTRL_4
#define MTR_REG_CTRL_5    DRV_REG_CTRL_5
#define MTR_REG_CTRL_6    DRV_REG_CTRL_6
#define MTR_REG_CTRL_10   DRV_REG_CTRL_10
#endif

#ifdef SENSOR_TRAP_CONTROL
#include "mct8316z.h"

#define MTR_L &MCT8316Z_L
#define MTR_R &MCT8316Z_R

#define MTR_ReadRegister   MCT8316Z_ReadRegister
#define MTR_UpdateRegister MCT8316Z_ApplyDefaultConfig
#define MTR_REG_IC_STATUS  MCT_REG_IC_STATUS
#define MTR_REG_STATUS_1   MCT_REG_STATUS_1
#define MTR_REG_STATUS_2   MCT_REG_STATUS_2
#define MTR_REG_CTRL_2     MCT_REG_CTRL_2
#define MTR_REG_CTRL_3     MCT_REG_CTRL_3
#define MTR_REG_CTRL_4     MCT_REG_CTRL_4
#define MTR_REG_CTRL_5     MCT_REG_CTRL_5
#define MTR_REG_CTRL_6     MCT_REG_CTRL_6
#define MTR_REG_CTRL_7     MCT_REG_CTRL_7
#define MTR_REG_CTRL_8     MCT_REG_CTRL_8
#define MTR_REG_CTRL_9     MCT_REG_CTRL_9
#endif

// ====================================================================
// 공통 제어 함수 모음
// ====================================================================

void MTR_L_TIM_Start() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 4790);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void MTR_L_TIM_Stop() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

void MTR_R_TIM_Start() {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 4790);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void MTR_R_TIM_Stop() {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void MTR_Start() {
#ifdef FOC_CONTROL
	DRV8316C_FOC_PWM_EN();
	MTR_L_TIM_Start();
	MTR_R_TIM_Start();
#endif
#ifdef SENSOR_TRAP_CONTROL
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
#endif
}

void MTR_Stop() {
#ifdef FOC_CONTROL
	DRV8316C_FOC_PWM_DIS();
	MTR_L_TIM_Stop();
	MTR_R_TIM_Stop();
#endif
#ifdef SENSOR_TRAP_CONTROL
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    MCT8316Z_SLEEP(MTR_L);
    MCT8316Z_SLEEP(MTR_R);
#endif
}

void Encoder_Start() {
	HAL_LPTIM_Encoder_Start(&hlptim1, UINT16_MAX);
	HAL_LPTIM_Encoder_Start(&hlptim2, UINT16_MAX);
}

void Encoder_Stop() {
	HAL_LPTIM_Encoder_Stop(&hlptim1);
	HAL_LPTIM_Encoder_Stop(&hlptim2);
}

// ====================================================================
// FOC 통합 구동 및 안전 정지 함수
// ====================================================================
void MTR_Setup_And_Start(FOC_DriveMode_t mode) {
#ifdef FOC_CONTROL
	FOC_Init_Motor(&foc_L, TIM3, ADC2, LPTIM2);
	FOC_Init_Motor(&foc_R, TIM4, ADC1, LPTIM1);

	arm_pid_init_f32(&foc_L.pid_id, 1);
	arm_pid_init_f32(&foc_L.pid_iq, 1);
	arm_pid_init_f32(&foc_R.pid_id, 1);
	arm_pid_init_f32(&foc_R.pid_iq, 1);

	foc_L.enc_dir = -1;
	foc_R.enc_dir = -1;
#endif

	Encoder_Start();
	FOC_ADC_Start();

	// [핵심 해결 2] ADC 및 내부 하드웨어 안정화 대기
	HAL_Delay(50);

	if (mode != FOC_MODE_SVPWM_NO_SPIN) {
		MTR_Start();

		// PWM 출력 인가 후 전류 증폭기(CSA) 영점 안정화 대기
		HAL_Delay(50);

#ifdef FOC_CONTROL
		LCD_Printf(0, 0, "Calibrating...");
		FOC_Calibrate_Offset(&foc_L, adc2_dma_buf);
		FOC_Calibrate_Offset(&foc_R, adc1_dma_buf);

		LCD_Printf(0, 0, "Aligning L... ");
		FOC_Calibrate_Encoder_Offset(&foc_L);
		LCD_Printf(0, 0, "Aligning R... ");
		FOC_Calibrate_Encoder_Offset(&foc_R);
		LCD_Clear();
	}
#endif
	MTR_Stop();

#ifdef FOC_CONTROL
	// 플래그 및 지령치 초기화 로직 유지
	if (mode == FOC_MODE_NO_SVPWM_SPIN) {
		foc_L.foc_svpwm_en = 0;
		foc_R.foc_svpwm_en = 0;
	} else {
		foc_L.foc_svpwm_en = 1;
		foc_R.foc_svpwm_en = 1;
	}

	foc_L.is_running = 1;
	foc_R.is_running = 1;

	if (mode == FOC_MODE_SPEED_LOOP) {
		foc_L.speed_loop_en = 1;
		foc_R.speed_loop_en = 1;
	} else {
		foc_L.speed_loop_en = 0;
		foc_R.speed_loop_en = 0;
	}

	foc_L.target_Id = 0.0f;
	foc_R.target_Id = 0.0f;
	foc_L.target_Iq = 0.0f;
	foc_R.target_Iq = 0.0f;
	foc_L.target_omega = 0.0f;
	foc_R.target_omega = 0.0f;
	foc_L.spd_integ = 0.0f;
	foc_R.spd_integ = 0.0f;

	foc_L.enc_prev_cnt = (uint16_t) foc_L.LPTIMx->CNT;
	foc_R.enc_prev_cnt = (uint16_t) foc_R.LPTIMx->CNT;
	// enc_dir은 위에서 이미 -1로 확정됨 (여기서 재대입하지 않음)

	MTR_Start();
	if (mode == FOC_MODE_SPEED_LOOP) {
		HAL_TIM_Base_Start_IT(&htim13);
	}
#endif
}

void MTR_Safe_Stop(void) {
	// 1. 속도 루프 인터럽트 타이머 선 정지
	HAL_TIM_Base_Stop_IT(&htim13);

#ifdef FOC_CONTROL
	// 2. FOC 제어 플래그 차단
	foc_L.is_running = 0;
	foc_R.is_running = 0;
	foc_L.foc_svpwm_en = 0;
	foc_R.foc_svpwm_en = 0;
	foc_L.speed_loop_en = 0;
	foc_R.speed_loop_en = 0;

	// 3. 지령치 초기화
	foc_L.target_Id = 0.0f;
	foc_R.target_Id = 0.0f;
	foc_L.target_Iq = 0.0f;
	foc_R.target_Iq = 0.0f;
	foc_L.target_omega = 0.0f;
	foc_R.target_omega = 0.0f;
	foc_L.spd_integ = 0.0f;
	foc_R.spd_integ = 0.0f;
#endif

	// 4. 하드웨어 출력 차단
	MTR_Stop();         // MTR_Stop() 내에서 DRV8316C_FOC_PWM_DIS() 호출됨
	Encoder_Stop();

	// 5. UI 정리
	LCD_Clear();
}

// ====================================================================
// 드라이버 설정 및 상태 읽기
// ====================================================================

void MTR_Read_Register() {
	uint8_t lcd_left_x_bias = 6;
	uint8_t lcd_right_x_bias = 9;
	LCD_Printf(lcd_left_x_bias, 0, "L");
	LCD_Printf(lcd_right_x_bias, 0, "R");
	LCD_Printf(0, 2, "IC:");
	LCD_Printf(0, 3, "ST1");
	LCD_Printf(0, 4, "ST2");
	LCD_Printf(0, 5, "CTR2");
	LCD_Printf(0, 6, "CTR3");
	LCD_Printf(0, 7, "CTR4");
	LCD_Printf(0, 8, "CTR5");
	LCD_Printf(0, 9, "CTR6");
	LCD_Printf(0, 10, "CTR7");
	LCD_Printf(0, 11, "CTR8");
	LCD_Printf(0, 12, "CTR9");
	LCD_Printf(0, 13, "CTR10");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		uint8_t mtr_L_pData, mtr_R_pData;

		MTR_ReadRegister(MTR_L, MTR_REG_IC_STATUS, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 2, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_STATUS_1, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 3, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_STATUS_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 4, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 5, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_3, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 6, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_4, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 7, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_5, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 8, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_6, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 9, "%02x", mtr_L_pData);
#ifdef FOC_CONTROL
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_10, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 13, "%02x", mtr_L_pData);
#endif
#ifdef SENSOR_TRAP_CONTROL
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_7, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 10, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_8, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 11, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_9, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 12, "%02x", mtr_L_pData);
#endif

		MTR_ReadRegister(MTR_R, MTR_REG_IC_STATUS, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 2, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_STATUS_1, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 3, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_STATUS_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 4, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 5, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_3, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 6, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_4, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 7, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_5, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 8, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_6, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 9, "%02x", mtr_R_pData);
#ifdef FOC_CONTROL
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_10, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 13, "%02x", mtr_R_pData);
#endif
#ifdef SENSOR_TRAP_CONTROL
        MTR_ReadRegister(MTR_R, MTR_REG_CTRL_7, &mtr_R_pData);
        LCD_Printf(lcd_right_x_bias, 10, "%02x", mtr_R_pData);
        MTR_ReadRegister(MTR_R, MTR_REG_CTRL_8, &mtr_R_pData);
        LCD_Printf(lcd_right_x_bias, 11, "%02x", mtr_R_pData);
        MTR_ReadRegister(MTR_R, MTR_REG_CTRL_9, &mtr_R_pData);
        LCD_Printf(lcd_right_x_bias, 12, "%02x", mtr_R_pData);
#endif
	}
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}

void MTR_Update_Setup() {
#ifdef FOC_CONTROL
	MTR_FOC_PWM_DIS()
	;
#endif
	MTR_SLEEP(MTR_L);
	MTR_SLEEP(MTR_R);
	LCD_Printf(0, 0, "Diver Sleep");
	HAL_Delay(100);
	MTR_WAKEUP(MTR_L);
	MTR_WAKEUP(MTR_R);
	LCD_Printf(0, 1, "Driver Wakeup");
	HAL_Delay(100);

	DRV8316C_UnlockRegister(MTR_L);
	DRV8316C_UnlockRegister(MTR_R);
	MTR_UpdateRegister(MTR_L);
	MTR_UpdateRegister(MTR_R);
	DRV8316C_LockRegister(MTR_L);
	DRV8316C_LockRegister(MTR_R);

	LCD_Printf(0, 2, "Driver Update");
	HAL_Delay(1000);
#ifdef FOC_CONTROL
	MTR_FOC_PWM_EN()
	;
#endif
}

// ====================================================================
// 각종 테스트 및 제어 루프 (통합 함수 적용)
// ====================================================================

void MTR_Simple_Control() {
	UserInput_t bt = INPUT_CMD_NONE;

#ifdef FOC_CONTROL
	// 모터는 돌지만, 내부 루프에서 수동 PWM 조작을 하므로 SVPWM 제어기는 정지
	MTR_Setup_And_Start(FOC_MODE_NO_SVPWM_SPIN);

	static int16_t angle = 0;
	static uint16_t half_pwm = 2400;
	static uint16_t max_pwm = 500;

	while (1) {
		bt = Button_Get_Input();

		switch (bt) {
		case INPUT_CMD_L_HOLD:
		case INPUT_CMD_L_SINGLE:
			angle -= 10;
			if (angle < 0)
				angle += 360;
			break;
		case INPUT_CMD_R_HOLD:
		case INPUT_CMD_R_SINGLE:
			angle += 10;
			if (angle >= 360)
				angle -= 360;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			max_pwm += 10;
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			if (max_pwm >= 10)
				max_pwm -= 10;
			break;
		case INPUT_CMD_K_HOLD:
			MTR_Safe_Stop();
			return;
		default:
			break;
		}

		float rad = (float) angle * M_PI / 180.0f;
		float rad120 = 120.0f * M_PI / 180.0f;

		float su = sinf(rad);
		float sv = sinf(rad - rad120);
		float sw = sinf(rad + rad120);

		float min_s = fminf(su, fminf(sv, sw));
		float max_s = fmaxf(su, fmaxf(sv, sw));
		float zss = (min_s + max_s) / 2.0f;
		su -= zss;
		sv -= zss;
		sw -= zss;

		int32_t u_calc = half_pwm + (int32_t) (su * (float) max_pwm);
		int32_t v_calc = half_pwm + (int32_t) (sv * (float) max_pwm);
		int32_t w_calc = half_pwm + (int32_t) (sw * (float) max_pwm);

		u_calc = u_calc < 0 ? 0 : (u_calc > 4800 ? 4800 : u_calc);
		v_calc = v_calc < 0 ? 0 : (v_calc > 4800 ? 4800 : v_calc);
		w_calc = w_calc < 0 ? 0 : (w_calc > 4800 ? 4800 : w_calc);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t )u_calc);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t )v_calc);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t )w_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t )u_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t )v_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t )w_calc);

		LCD_Printf(0, 0, "Ang:%3d Max:%4d", angle, max_pwm);
	}
#endif

#ifdef SENSOR_TRAP_CONTROL
    uint16_t duty = 2000;
    MTR_Start();
    for (uint16_t i = 0; i < 1000; i += 5) {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2 * i);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 2 * i);
        HAL_Delay(1);
    }
    while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
        LCD_Printf(0, 0, "%5d", duty);
        if (bt == INPUT_CMD_L_HOLD) duty -= (duty < 1000) ? 0 : 20;
        if (bt == INPUT_CMD_R_HOLD) duty += (duty > 8000) ? 0 : 20;
        if (bt == INPUT_CMD_U_HOLD) {
            HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin, GPIO_PIN_SET);
        }
        if (bt == INPUT_CMD_D_HOLD) {
            HAL_GPIO_WritePin(MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin, GPIO_PIN_RESET);
            MCT8316Z_ClearFaults(MTR_L);
            HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin, GPIO_PIN_RESET);
            MCT8316Z_ClearFaults(MTR_R);
            HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, duty);
    }
    Button_Wait_Release(&btn_k);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    MTR_Safe_Stop();
#endif
}

void MTR_Simple_FOC() {
#ifdef FOC_CONTROL
	UserInput_t bt = INPUT_CMD_NONE;

	// 모터 구동 및 SVPWM 활성화 모드
	MTR_Setup_And_Start(FOC_MODE_SVPWM_SPIN);

	// 기본 Iq 값 부여
	foc_L.target_Iq = 0.5f;
	foc_R.target_Iq = 0.5f;

	float omega = 0.0f;

	while (1) {
		bt = Button_Get_Input();

		switch (bt) {
		case INPUT_CMD_R_SINGLE:
		case INPUT_CMD_R_HOLD:
			omega += 10.0f;
			break;
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			omega -= 10.0f;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			foc_L.target_Iq += 0.1f;
			foc_R.target_Iq += 0.1f;
			if (foc_L.target_Iq > 5.0f) {
				foc_L.target_Iq = 5.0f;
				foc_R.target_Iq = 5.0f;
			}
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			foc_L.target_Iq -= 0.1f;
			foc_R.target_Iq -= 0.1f;
			if (foc_L.target_Iq < -5.0f) {
				foc_L.target_Iq = -5.0f;
				foc_R.target_Iq = -5.0f;
			}
			break;
		case INPUT_CMD_K_HOLD:
			MTR_Safe_Stop();
			omega = 0.0f;
			return;
		default:
			break;
		}

		if (omega > 2000.0f)
			omega = 2000.0f;
		if (omega < -2000.0f)
			omega = -2000.0f;

		foc_L.omega_e = omega;
		foc_R.omega_e = omega;

		LCD_Printf(0, 0, "w:%6.1f Iq:%.2f", omega, foc_L.target_Iq);
		LCD_Printf(0, 1, "thL:%6.2f", foc_L.theta_e);
		LCD_Printf(0, 2, "thR:%6.2f", foc_R.theta_e);
		LCD_Printf(0, 3, "IdL:%6.3f", foc_L.I_d);
		LCD_Printf(0, 4, "IqL:%6.3f", foc_L.I_q);
		LCD_Printf(0, 5, "IdR:%6.3f", foc_R.I_d);
		LCD_Printf(0, 6, "IqR:%6.3f", foc_R.I_q);
		LCD_Printf(0, 7, "IaL:%6.3f", foc_L.I_a);
		LCD_Printf(0, 8, "IbL:%6.3f", foc_L.I_b);
		LCD_Printf(0, 9, "IaR:%6.3f", foc_R.I_a);
		LCD_Printf(0, 10, "IbR:%6.3f", foc_R.I_b);
		LCD_Printf(0, 11, "r1:%5d %5d", adc1_dma_buf[1], adc1_dma_buf[2]);
		LCD_Printf(0, 12, "r2:%5d %5d", adc2_dma_buf[1], adc2_dma_buf[2]);
	}
#endif
}

void MTR_Encoder_Test() {
	// 센서/전기각 등 연산은 수행하되, 모터로 향하는 출력(PWM)은 차단하는 모드
	MTR_Setup_And_Start(FOC_MODE_SVPWM_NO_SPIN);

	while (1) {
		LCD_Printf(0, 0, "L:%5d R:%5d", (uint16_t) hlptim2.Instance->CNT,
				(uint16_t) hlptim1.Instance->CNT);
		LCD_Printf(0, 1, "eL:%6.3f eR:%6.3f", foc_L.theta_e, foc_R.theta_e);
		LCD_Printf(0, 2, "AL:%5d AR:%5d", adc2_dma_buf[1], adc1_dma_buf[1]);
		LCD_Printf(0, 3, "NFL: %d NFR: %d",
				HAL_GPIO_ReadPin(MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin),
				HAL_GPIO_ReadPin(MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin));
		if (Button_Get_Input() == INPUT_CMD_K_HOLD) {
			MTR_Safe_Stop();
			return;
		}

#ifdef FOC_CONTROL
		FOC_Update_Theta_Encoder(&foc_L);
		FOC_Update_Theta_Encoder(&foc_R);
#endif
		HAL_Delay(50);
	}
}

// 튜닝 전용 스텝 전류 크기
#define TUNE_TEST_CURRENT 1.0f

void MTR_Current_Tune_Loop() {
#ifdef FOC_CONTROL
	UserInput_t bt = INPUT_CMD_NONE;

	// 전류 제어를 위한 FOC 구동 모드
	MTR_Setup_And_Start(FOC_MODE_SVPWM_SPIN);

	uint32_t last_toggle_time = HAL_GetTick();
	uint8_t toggle_state = 0;

	uint8_t sel = 0; // 0: Kp, 1: Ki
	const float step_kp = 0.001f;
	const float step_ki = 0.001f;

	while (1) {
		bt = Button_Get_Input();

		// 1. 스텝 지령 생성 (500ms 마다 0A <-> 1A 토글)
		if (HAL_GetTick() - last_toggle_time > 500) {
			last_toggle_time = HAL_GetTick();
			toggle_state = !toggle_state;
			foc_L.target_Id = toggle_state ? TUNE_TEST_CURRENT : 0.0f;
		}

		// 2. 버튼 입력으로 Id 게인 조절
		switch (bt) {
		case INPUT_CMD_K_SINGLE:
			sel = (sel + 1) % 2;
			break;

		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			if (sel == 0)
				foc_L.pid_id.Kp += step_kp;
			else
				foc_L.pid_id.Ki += step_ki;
			arm_pid_init_f32(&foc_L.pid_id, 0);
			break;

		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			if (sel == 0) {
				foc_L.pid_id.Kp -= step_kp;
				if (foc_L.pid_id.Kp < 0.0f)
					foc_L.pid_id.Kp = 0.0f;
			} else {
				foc_L.pid_id.Ki -= step_ki;
				if (foc_L.pid_id.Ki < 0.0f)
					foc_L.pid_id.Ki = 0.0f;
			}
			arm_pid_init_f32(&foc_L.pid_id, 0);
			break;

		case INPUT_CMD_K_HOLD:
			// 종료 전 찾은 Id 게인을 Iq 및 우측 모터 제어기에도 동일하게 적용
			foc_L.pid_iq.Kp = foc_L.pid_id.Kp;
			foc_L.pid_iq.Ki = foc_L.pid_id.Ki;
			foc_R.pid_id.Kp = foc_L.pid_id.Kp;
			foc_R.pid_id.Ki = foc_L.pid_id.Ki;
			foc_R.pid_iq.Kp = foc_L.pid_id.Kp;
			foc_R.pid_iq.Ki = foc_L.pid_id.Ki;

			arm_pid_init_f32(&foc_L.pid_iq, 0);
			arm_pid_init_f32(&foc_R.pid_id, 0);
			arm_pid_init_f32(&foc_R.pid_iq, 0);

			MTR_Safe_Stop();
			return;

		default:
			break;
		}

		LCD_Printf(0, 0, "Id Step Tune");
		LCD_Printf(0, 1, "%cKp:%6.4f", sel == 0 ? '>' : ' ', foc_L.pid_id.Kp);
		LCD_Printf(0, 2, "%cKi:%6.4f", sel == 1 ? '>' : ' ', foc_L.pid_id.Ki);
		LCD_Printf(0, 4, "Tgt Id:%6.2f", foc_L.target_Id);
		LCD_Printf(0, 5, "Cur Id:%6.2f", foc_L.I_d);
		LCD_Printf(0, 6, "Cur Iq:%6.2f", foc_L.I_q);
	}
#endif
}

void MTR_Speed_FOC() {
#ifdef FOC_CONTROL
	UserInput_t bt = INPUT_CMD_NONE;

	// 통합 시작 함수: 정상 FOC 속도 제어 모드
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);

	// 속도 루프 제어를 위해 IT 타이머 및 루프 플래그 활성화

	float omega = 0.0f;
	uint8_t sel = 0;

	const float step_iq_kp = 0.05f;
	const float step_iq_ki = 0.005f;
	const float step_spd_kp = 0.01f;
	const float step_spd_ki = 0.01f;

	while (1) {
		bt = Button_Get_Input();

		switch (bt) {
		case INPUT_CMD_R_SINGLE:
		case INPUT_CMD_R_HOLD:
			omega += 25.0f;
			if (omega > 1000.0f)
				omega = 1000.0f;
			break;
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			omega -= 25.0f;
			if (omega < -1000.0f)
				omega = -1000.0f;
			break;
		case INPUT_CMD_K_SINGLE:
			sel = (sel + 1) % 4;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			switch (sel) {
			case 0:
				foc_L.pid_iq.Kp += step_iq_kp;
				foc_R.pid_iq.Kp += step_iq_kp;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 1:
				foc_L.pid_iq.Ki += step_iq_ki;
				foc_R.pid_iq.Ki += step_iq_ki;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 2:
				foc_L.spd_Kp += step_spd_kp;
				foc_R.spd_Kp += step_spd_kp;
				break;
			case 3:
				foc_L.spd_Ki += step_spd_ki;
				foc_R.spd_Ki += step_spd_ki;
				break;
			}
			break;
		case INPUT_CMD_D_SINGLE:
		case INPUT_CMD_D_HOLD:
			switch (sel) {
			case 0:
				foc_L.pid_iq.Kp -= step_iq_kp;
				if (foc_L.pid_iq.Kp < 0.0f)
					foc_L.pid_iq.Kp = 0.0f;
				foc_R.pid_iq.Kp = foc_L.pid_iq.Kp;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 1:
				foc_L.pid_iq.Ki -= step_iq_ki;
				if (foc_L.pid_iq.Ki < 0.0f)
					foc_L.pid_iq.Ki = 0.0f;
				foc_R.pid_iq.Ki = foc_L.pid_iq.Ki;
				arm_pid_init_f32(&foc_L.pid_iq, 0);
				arm_pid_init_f32(&foc_R.pid_iq, 0);
				break;
			case 2:
				foc_L.spd_Kp -= step_spd_kp;
				if (foc_L.spd_Kp < 0.0f)
					foc_L.spd_Kp = 0.0f;
				foc_R.spd_Kp = foc_L.spd_Kp;
				break;
			case 3:
				foc_L.spd_Ki -= step_spd_ki;
				if (foc_L.spd_Ki < 0.0f)
					foc_L.spd_Ki = 0.0f;
				foc_R.spd_Ki = foc_L.spd_Ki;
				break;
			}
			break;
		case INPUT_CMD_K_HOLD:
			MTR_Safe_Stop();
			return;
		default:
			break;
		}

		foc_L.target_omega = omega;
		foc_R.target_omega = omega;

		LCD_Printf(0, 0, "%cIqKp:%6.3f", sel == 0 ? '>' : ' ', foc_L.pid_iq.Kp);
		LCD_Printf(0, 1, "%cIqKi:%6.3f", sel == 1 ? '>' : ' ', foc_L.pid_iq.Ki);
		LCD_Printf(0, 2, "%cSpKp:%6.3f", sel == 2 ? '>' : ' ', foc_L.spd_Kp);
		LCD_Printf(0, 3, "%cSpKi:%6.3f", sel == 3 ? '>' : ' ', foc_L.spd_Ki);
		LCD_Printf(0, 5, "ref:%6.1f", omega);
		LCD_Printf(0, 6, "wL :%6.1f", foc_L.omega_e_meas);
		LCD_Printf(0, 7, "wR :%6.1f", foc_R.omega_e_meas);
		LCD_Printf(0, 8, "IqL:%6.3f", foc_L.target_Iq);
		LCD_Printf(0, 9, "IqcL:%6.3f", foc_L.I_q);
		LCD_Printf(0, 10, "IqR:%6.3f", foc_R.target_Iq);
		LCD_Printf(0, 11, "IqcR:%6.3f", foc_R.I_q);
	}
#endif
}

void MTR_Set_Speed(float mps_L, float mps_R) {
	foc_L.omega_setpoint = mps_L * INV_TIRE_RADIUS;
	foc_R.omega_setpoint = mps_R * INV_TIRE_RADIUS;
}
