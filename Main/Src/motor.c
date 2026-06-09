/*
 * motor.c
 *
 *  Created on: 2026. 6. 9.
 *      Author: kth59
 */

#include "user_init.h"
#include "button.h"
#include "motor.h"

#ifdef FOC_CONTROL
#include "drv8316crq1.h"

#define MTR_L DRV8316C_L
#define MTR_R DRV8316C_R

#define MTR_ReadRegister DRV8316C_ReadRegister
#define MTR_REG_IC_STATUS DRV_REG_IC_STATUS
#define MTR_REG_STATUS_1 DRV_REG_STATUS_1
#define MTR_REG_STATUS_2 DRV_REG_STATUS_2
#define MTR_REG_CTRL_2 DRV_REG_CTRL_2

#endif

#ifdef SENSOR_TRAP_CONTROL
#include "mct8316z.h"

#define MTR_L MCT8316Z_L
#define MTR_R MCT8316Z_R

#define MTR_ReadRegister MCT8316Z_ReadRegister
#define MTR_REG_IC_STATUS MCT_REG_IC_STATUS
#define MTR_REG_STATUS_1 MCT_REG_STATUS_1
#define MTR_REG_STATUS_2 MCT_REG_STATUS_2
#define MTR_REG_CTRL_2 MCT_REG_CTRL_2

#endif

void MTR_Read_Register() {
	uint8_t lcd_left_x_bias = 0;
	uint8_t lcd_right_x_bias = 6;
	LCD_Printf(lcd_left_x_bias, 0, "Left");
	LCD_Printf(lcd_right_x_bias, 0, "Right");

	while (Button_Get_Input() != INPUT_CMD_K_HOLD) {
		uint8_t mtr_L_pData;
		uint8_t mtr_R_pData;

		MTR_ReadRegister(&MTR_L, MTR_REG_IC_STATUS, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 2, "%02x", mtr_L_pData);
		MTR_ReadRegister(&MTR_L, MTR_REG_STATUS_1, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 3, "%02x", mtr_L_pData);
		MTR_ReadRegister(&MTR_L, MTR_REG_STATUS_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 4, "%02x", mtr_L_pData);
		MTR_ReadRegister(&MTR_L, MTR_REG_CTRL_2, &mtr_L_pData);
		LCD_Printf(lcd_left_x_bias, 5, "%02x", mtr_L_pData);

		MTR_ReadRegister(&MTR_R, MTR_REG_IC_STATUS, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 2, "%02x", mtr_R_pData);
		MTR_ReadRegister(&MTR_R, MTR_REG_STATUS_1, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 3, "%02x", mtr_R_pData);
		MTR_ReadRegister(&MTR_R, MTR_REG_STATUS_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 4, "%02x", mtr_R_pData);
		MTR_ReadRegister(&MTR_R, MTR_REG_CTRL_2, &mtr_R_pData);
		LCD_Printf(lcd_right_x_bias, 5, "%02x", mtr_R_pData);
	}
	Button_Wait_Release(&btn_k);
	LCD_Clear();
}
