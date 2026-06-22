/*
 * motor.c
 *
 *  Created on: 2026. 6. 9.
 *      Author: kth59
 */

#include "user_init.h"
#include "button.h"
#include "motor.h"
#include "math.h"

#ifdef FOC_CONTROL
#include "drv8316crq1.h"

#define MTR_L &DRV8316C_L
#define MTR_R &DRV8316C_R

#define MTR_ReadRegister DRV8316C_ReadRegister
#define MTR_UpdateRegister DRV8316C_ApplyDefaultConfig
#define MTR_REG_IC_STATUS DRV_REG_IC_STATUS
#define MTR_REG_STATUS_1 DRV_REG_STATUS_1
#define MTR_REG_STATUS_2 DRV_REG_STATUS_2
#define MTR_REG_CTRL_2 DRV_REG_CTRL_2
#define MTR_REG_CTRL_2 DRV_REG_CTRL_2
#define MTR_REG_CTRL_3 DRV_REG_CTRL_3
#define MTR_REG_CTRL_4 DRV_REG_CTRL_4
#define MTR_REG_CTRL_5 DRV_REG_CTRL_5
#define MTR_REG_CTRL_6 DRV_REG_CTRL_6
#define MTR_REG_CTRL_10 DRV_REG_CTRL_10

#endif

#ifdef SENSOR_TRAP_CONTROL
#include "mct8316z.h"

#define MTR_L &MCT8316Z_L
#define MTR_R &MCT8316Z_R

#define MTR_ReadRegister MCT8316Z_ReadRegister
#define MTR_UpdateRegister MCT8316Z_ApplyDefaultConfig
#define MTR_REG_IC_STATUS MCT_REG_IC_STATUS
#define MTR_REG_STATUS_1 MCT_REG_STATUS_1
#define MTR_REG_STATUS_2 MCT_REG_STATUS_2
#define MTR_REG_CTRL_2 MCT_REG_CTRL_2
#define MTR_REG_CTRL_3 MCT_REG_CTRL_3
#define MTR_REG_CTRL_4 MCT_REG_CTRL_4
#define MTR_REG_CTRL_5 MCT_REG_CTRL_5
#define MTR_REG_CTRL_6 MCT_REG_CTRL_6
#define MTR_REG_CTRL_7 MCT_REG_CTRL_7
#define MTR_REG_CTRL_8 MCT_REG_CTRL_8
#define MTR_REG_CTRL_9 MCT_REG_CTRL_9

#endif

void MTR_L_TIM_Start() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void MTR_L_TIM_Stop() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

void MTR_R_TIM_Start() {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void MTR_R_TIM_Stop() {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

void MTR_Start() {
#ifdef FOC_CONTROL
//	DRV8316C_DRVOFF_LOW(MTR_L);
//	DRV8316C_DRVOFF_LOW(MTR_R);
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
		LCD_Printf(lcd_left_x_bias, 1, "%d", HAL_GPIO_ReadPin(*MTR_L.nFAULT_Port, *MTR_L.nFAULT_Pin));
		LCD_Printf(lcd_right_x_bias, 1, "%d", HAL_GPIO_ReadPin(*MTR_R.nFAULT_Port, *MTR_R.nFAULT_Pin));

		uint8_t mtr_L_pData;
		uint8_t mtr_R_pData;

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
#ifdef SENSOR_TRAP_CONTROL
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_7, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 10, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_8, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 11, "%02x", mtr_L_pData);
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_9, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 12, "%02x", mtr_L_pData);
#endif
#ifdef FOC_CONTROL
		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_10, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 13, "%02x", mtr_L_pData);
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
#ifdef SENSOR_TRAP_CONTROL
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_7, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 10, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_8, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 11, "%02x", mtr_R_pData);
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_9, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 12, "%02x", mtr_R_pData);
#endif
#ifdef FOC_CONTROL
		MTR_ReadRegister(MTR_R, MTR_REG_CTRL_10, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 13, "%02x", mtr_R_pData);
#endif

	}
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}

void MTR_Simple_Control() {
	UserInput_t bt = INPUT_CMD_NONE;
	MTR_Start();

#ifdef FOC_CONTROL
	while (1) {
		// idx 대신 전기각(Angle) 사용: 0 ~ 359도
		static int16_t angle = 0;
		static uint16_t half_pwm = 2400;
		static uint16_t max_pwm = 500;

		switch (bt) {
		case INPUT_CMD_L_HOLD:
		case INPUT_CMD_L_SINGLE:
			// 15도씩 부드럽게 감소
			angle -= 2;
			if (angle < 0)
				angle += 360;
			break;
		case INPUT_CMD_R_HOLD:
		case INPUT_CMD_R_SINGLE:
			// 15도씩 부드럽게 증가
			angle += 2;
			if (angle >= 360)
				angle -= 360;
			break;
		case INPUT_CMD_U_SINGLE:
		case INPUT_CMD_U_HOLD:
			max_pwm += 10;
			break;
		case INPUT_CMD_K_HOLD:
			DRV8316C_FOC_PWM_DIS()
			;
			MTR_Stop();
			LCD_Clear();
			return;
		default:
			break;
		}

		LCD_Printf(0, 0, "Ang:%3d", angle);
		LCD_Printf(0, 1, "Max:%4d", max_pwm);

		// ==========================================
		// [SVPWM 수학적 연산 파트]
		// ==========================================
		// 1. 각도를 라디안으로 변환
		float rad = (float) angle * M_PI / 180.0f;
		float rad120 = 120.0f * M_PI / 180.0f;

		// 2. 순수 3상 정현파(Sine Wave) 생성 (-1.0 ~ +1.0)
		float su = sinf(rad);
		float sv = sinf(rad - rad120);
		float sw = sinf(rad + rad120);

		// 3. SVPWM의 핵심: 공통 모드 전압(Zero Sequence Signal) 추출
		// 3개의 상 중 가장 큰 값과 가장 작은 값의 평균을 구함
		float min_s = fminf(su, fminf(sv, sw));
		float max_s = fmaxf(su, fmaxf(sv, sw));
		float zss = (min_s + max_s) / 2.0f;

		// 4. 원래 정현파에서 ZSS를 빼주면 완벽한 SVPWM(안장 형태) 파형 완성
		su -= zss;
		sv -= zss;
		sw -= zss;

		// 5. PWM 듀티비로 변환 (half_pwm을 중심으로 진동)
		int32_t u_calc = half_pwm + (int32_t) (su * (float) max_pwm);
		int32_t v_calc = half_pwm + (int32_t) (sv * (float) max_pwm);
		int32_t w_calc = half_pwm + (int32_t) (sw * (float) max_pwm);

		// 6. 안전을 위한 클램핑 처리 (0 ~ 4800)
		if (u_calc < 0)
			u_calc = 0;
		else if (u_calc > 4800)
			u_calc = 4800;
		if (v_calc < 0)
			v_calc = 0;
		else if (v_calc > 4800)
			v_calc = 4800;
		if (w_calc < 0)
			w_calc = 0;
		else if (w_calc > 4800)
			w_calc = 4800;

		// ==========================================

		// 타이머 레지스터에 인가
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t )u_calc);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t )v_calc);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t )w_calc);

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t )u_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t )v_calc);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t )w_calc);

		LCD_Printf(0, 2, "%4d    ",
				__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1));
		LCD_Printf(0, 3, "%4d    ",
				__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2));
		LCD_Printf(0, 4, "%4d    ",
				__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3));
		LCD_Printf(0, 5, "%4d    ",
				__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1));
		LCD_Printf(0, 6, "%4d    ",
				__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2));
		LCD_Printf(0, 7, "%4d    ",
				__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3));

		uint8_t mtr_L_pData;
		MTR_ReadRegister(MTR_L, MTR_REG_IC_STATUS, &mtr_L_pData);
		LCD_Printf(0, 8, "%02x", mtr_L_pData);

		MTR_ReadRegister(MTR_L, MTR_REG_CTRL_2, &mtr_L_pData);
		if (mtr_L_pData == 0x60) {
			DRV8316C_FOC_PWM_DIS();
			DRV8316C_ApplyDefaultConfig(MTR_L);
			DRV8316C_ApplyDefaultConfig(MTR_R);
			DRV8316C_FOC_PWM_EN();
		}

		while (!(bt = Button_Get_Input()))
			;
	}
	MTR_Stop();
	LCD_Clear();
#endif
#ifdef SENSOR_TRAP_CONTROL
	uint16_t duty = 2000;
	MTR_Start();
	for (uint16_t i = 0; i < 1000; i = i + 5) {
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2 * i);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 2 * i);
		HAL_Delay(1);
	}
	while ((bt = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		LCD_Printf(0, 0, "%5d", duty);
		if (bt == INPUT_CMD_L_HOLD) {
			duty = duty - ((duty < 1000) ? 0 : 20);
		}
		if (bt == INPUT_CMD_R_HOLD) {
			duty = duty + ((duty > 8000) ? 0 : 20);
		}
		if (bt == INPUT_CMD_U_HOLD) {
			HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin, GPIO_PIN_SET);
		}
		if (bt == INPUT_CMD_D_HOLD) {

			HAL_GPIO_WritePin(MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin, GPIO_PIN_RESET);

			MCT8316Z_ClearFaults(MTR_L);
			HAL_GPIO_WritePin(MTR_BRAKE_L_GPIO_Port, MTR_BRAKE_L_Pin,
					GPIO_PIN_RESET);
			MCT8316Z_ClearFaults(MTR_R);
			HAL_GPIO_WritePin(MTR_BRAKE_R_GPIO_Port, MTR_BRAKE_R_Pin,
					GPIO_PIN_RESET);
		}
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, duty);
	}
	Button_Wait_Release(&btn_k);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
#endif

}

void MTR_Update_Setup() {
////	DRV8316C_FOC_PWM_DIS();
//
//	DRV8316C_SLEEP(MTR_L);
//	DRV8316C_SLEEP(MTR_R);
//	LCD_Printf(0, 0, "Diver Sleep");
//	HAL_Delay(100);
//	DRV8316C_WAKEUP(MTR_L);
//	DRV8316C_WAKEUP(MTR_R);
//	LCD_Printf(0, 1, "Driver Wakeup");
//	HAL_Delay(100);

	MTR_UpdateRegister(MTR_L);
	MTR_UpdateRegister(MTR_R);
}
