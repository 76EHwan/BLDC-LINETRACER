/*
 * user_init.c
 *
 *  Created on: 2026. 4. 16.
 *      Author: kth59
 */
#include "main.h"
#include "rng.h"
#include <stdio.h> // sprintf 사용을 위해 추가
#include "ff.h"    // f_unlink (FatFs 파일 삭제) 사용을 위해 추가

#include "user_init.h"
#include "button.h"
#include "buzzer.h"
#include "lsm6ds3tr-c.h"
#include "SDcard.h"
#include "mt6701.h"
#include "menu.h"

#ifdef FOC_CONTROL
#include "drv8316crq1.h"
#include "foc.h"
#elif defined(SENSOR_TRAP_CONTROL)
#include "mct8316z.h"
#include "trap.h"
#endif

// ==========================================================
// ★ 주행 마커 txt 파일 전체 삭제 함수
// ==========================================================
void Delete_All_Marker_Logs(void) {
    char filepath[64];
    FRESULT res;

    // 1. SD 카드 마운트 (SDFatFS_NC 전역 버퍼 사용)
    if (SDCard_Mount() != FR_OK) {
        return; // 마운트 실패 시 함수 종료
    }

    // 2. 파일 삭제 루프 진행 (save_slot_1 ~ save_slot_10)
    for (int i = 1; i <= 10; i++) {
        sprintf(filepath, "0:/Drive_Data/save_slot_%d.txt", i);
        res = f_unlink(filepath);
        (void)res;
    }

    // 3. 안전을 위해 삭제 작업 완료 후 마운트 해제
    SDCard_Unmount();
}

void User_Init() {
	Button_init();
	Buzzer_Init();
	Buzzer_Start();
	LCD_Test();

	HAL_StatusTypeDef imu_status = LSM6DS_Init();
	if (imu_status == HAL_OK) {
		LCD_Printf(0, 4, "%s OK(0x%02X)", LSM6DS_GetModelName(), imu_data.Chip_ID);
	} else {
		LCD_Printf(0, 4, "IMU Fail");
	}
	Buzzer_Stop();

	MX_DRV8316C_Init();
	LCD_Printf(0, 5, "DRV8316 SET OK");
	FOC_Init_Motor(&foc_L, &htim3, &hadc2, &hlptim2);
	FOC_Init_Motor(&foc_R, &htim4, &hadc1, &hlptim1);

	FOC_ADC_Start();

	LCD_Printf(0, 6, "FOC L ADC Cali");
	FOC_Calibrate_Offset(&foc_L);
	LCD_Printf(0, 7, "%5.f %5.f", foc_L.offset_a, foc_L.offset_c);

	LCD_Printf(0, 8, "FOC R ADC Cali");
	FOC_Calibrate_Offset(&foc_R);
	LCD_Printf(0, 9, "%5.f %5.f", foc_R.offset_a, foc_R.offset_c);
	LCD_Printf(0, 10, "VBAT: %5.3f V", g_vbus_filt);
	FOC_ADC_Stop();

	if (imu_status == HAL_OK) {
		LCD_Printf(0, 11, "IMU Cali...");
		LSM6DS3_Gyro_Calibrate_Z_Only();
		LCD_Printf(0, 11, "IMU Cali OK");
	} else {
		LCD_Printf(0, 11, "IMU Cali SKIP");
	}

	FRESULT res = SDCard_Test();
	if (res == FR_OK) {
		sdcard_err = 0;
		LCD_Printf(0, 12, "SD:OK 0x%08lX", (unsigned long)sd_test_rng_val);
		Delete_All_Marker_Logs();
	} else if (res == FR_INT_ERR) {
		sdcard_err = 1;
		LCD_Printf(0, 12, "SD:MISMATCH");
	} else {
		sdcard_err = 1;
		LCD_Printf(0, 12, "SD:SKIP(%d)", res);
	}

	// =========================================================================
	// 초기화 결과 확인: KEY 버튼을 누르고 있으면 화면 일시정지 유지
	// =========================================================================
	uint32_t wait_start = HAL_GetTick();
	while ((HAL_GetTick() - wait_start < 700) || (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)) {
		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
			LCD_Printf(0, 14, "PAUSED (Rel KEY)");
			// KEY 버튼에서 손을 뗄 때까지 화면 정지 상태 유지 및 LED 점멸
			while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
				LED_TOGGLE;
				HAL_Delay(100);
			}
			LED_ON;
			HAL_Delay(200);
			break;
		}
		HAL_Delay(10);
	}

	Button_init(); // 메뉴 진입 전 버튼 상태 초기화
}
