/*
 * user_init.c
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */
#include "main.h"
#include "rng.h"

#include "user_init.h"
#include "st7789_lcd.h"
#include "w25qxx.h"
#include "SDcard.h"
#include "button.h"

#define TEST_ADDRESS 0x000000

void Button_init() {
	Button_Init_Internal(&btn_k, KEY_GPIO_Port, KEY_Pin, GPIO_PIN_SET);
	Button_Init_Internal(&btn_l, SWITCH_LEFT_GPIO_Port, SWITCH_LEFT_Pin,
			GPIO_PIN_SET);
	Button_Init_Internal(&btn_r, SWITCH_RIGHT_GPIO_Port, SWITCH_RIGHT_Pin,
			GPIO_PIN_SET);
	Button_Init_Internal(&btn_u, SWITCH_UP_GPIO_Port, SWITCH_UP_Pin,
			GPIO_PIN_SET);
	Button_Init_Internal(&btn_d, SWITCH_DOWN_GPIO_Port, SWITCH_DOWN_Pin,
			GPIO_PIN_SET);
}

void SDCard_Test(void) {
	FRESULT res;
	char writeData[] = "STM32 FATFS Write & Read Test Success!";
	char readBuffer[50];

	LCD7789_Printf(0, 0, "[SD Test]");

	res = SDCard_Mount();
	if (res != FR_OK) {
		LCD7789_Printf(0, 1, "FAIL: Mount %d", res);
		return;
	}

	res = SDCard_Write("test.txt", writeData);
	if (res != FR_OK) {
		LCD7789_Printf(0, 1, "FAIL: Write %d", res);
		SDCard_Unmount();
		return;
	}

	res = SDCard_Read("test.txt", readBuffer, sizeof(readBuffer));
	if (res != FR_OK) {
		LCD7789_Printf(0, 1, "FAIL: Read %d", res);
		SDCard_Unmount();
		return;
	}

	if (strcmp(writeData, readBuffer) == 0) {
		LCD7789_Printf(0, 1, "SUCCESS!");
	} else {
		LCD7789_Printf(0, 1, "FAIL: Mismatch");
	}

	LCD7789_Printf(0, 2, readBuffer);

	SDCard_Unmount();
}

void W25QXX_Test(void) {
	W25Qx_Init();
	uint16_t id;
	W25Qx_Read_ID(&id);
	LCD_Printf(0, 0, "%04X", id);
	char build_time_str[32];
	char read_buffer[32];
	sprintf(build_time_str, "%s %s", __DATE__, __TIME__);
	uint16_t data_length = strlen(build_time_str) + 1;
	W25Qx_Erase_Block(0);
	W25Qx_Write((uint8_t*) build_time_str, TEST_ADDRESS, data_length);
	memset(read_buffer, 0, sizeof(read_buffer));
	W25Qx_Read((uint8_t*) read_buffer, TEST_ADDRESS, data_length);
	LCD_Printf(0, 1, "Last Update:");
	LCD_Printf(0, 2, "%s", read_buffer);
	while(Button_Get_Input() != INPUT_CMD_K_DOUBLE);
	LCD_Clear();
}

#ifdef FOC_CONTROL
void Test_DRV8316C_Read_Status(DRV8316C_Handle_t *hdrv) {
	uint8_t status;
	// 디버깅용: 상태 레지스터 읽기
	DRV8316C_ReadRegister(hdrv, DRV_REG_IC_STATUS, &status);
	LCD7789_Printf(0, 4, "IC STATUS: %02X", status);

	DRV8316C_ReadRegister(hdrv, DRV_REG_STATUS_1, &status);
	LCD7789_Printf(0, 5, "STATUS 1: %02X", status);

	DRV8316C_ReadRegister(hdrv, DRV_REG_STATUS_2, &status);
	LCD7789_Printf(0, 6, "STATUS 2: %02X", status);

	DRV8316C_ReadRegister(hdrv, DRV_REG_CTRL_2, &status);
	LCD7789_Printf(0, 7, "CTRL 2: %02X", status);

}
#endif
