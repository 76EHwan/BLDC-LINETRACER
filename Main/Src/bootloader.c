/*
 * bootloader.c
 *
 *  Created on: 2026. 5. 1.
 *      Author: kth59
 */

#include "bootloader.h"
#include "main.h"
#include "user_init.h"
#include "stm32h7xx_hal.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "button.h"

#define SYSTEM_MEMORY_BASE  0x1FF09800UL
#define BOOTLOADER_FLAG     0xDEADBEEFUL

extern USBD_HandleTypeDef hUsbDeviceFS;

void Check_Bootloader_Request(void) {
	HAL_PWR_EnableBkUpAccess();

	if (RTC->BKP0R != BOOTLOADER_FLAG) {
		return;
	}

	RTC->BKP0R = 0;

	void (*SysMemBootJump)(void);
	SysMemBootJump = (void (*)(void)) (*((uint32_t*) (SYSTEM_MEMORY_BASE + 4)));
	__set_MSP(*((uint32_t*) SYSTEM_MEMORY_BASE));
	SysMemBootJump();

	while (1) {
	}
}

void JumpToBootloader(void) {
	HAL_PWR_EnableBkUpAccess();
	RTC->BKP0R = BOOTLOADER_FLAG;

	NVIC_SystemReset();
}

void Boot_Loading(void) {
	LCD_Clear();
	LCD_Printf(0, 0, "Boot mode");
	LCD_Printf(0, 1, "Hold KEY Button");
	HAL_Delay(1000);
	uint8_t boot_time = 0;
	while (HAL_GPIO_ReadPin(btn_k.port, btn_k.pin) == btn_k.active_state) {
		LCD_Printf(10, 0, "%d", 3 - boot_time / 10);
		HAL_Delay(100);
		boot_time++;
		if (boot_time == 40) {
			LCD_Printf(0, 0, "Loading Boot");
			LCD_Printf(0, 1, "[              ]");
			HAL_Delay(1000);
			for (uint8_t i = 1; i < 15; i++) {
				LCD_Printf(i, 1, "=");
				HAL_Delay(50);
			}
			HAL_Delay(500);
			LCD_Clear();
			JumpToBootloader();
		}
	}
	LCD_Clear();
}
