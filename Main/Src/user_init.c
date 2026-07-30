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

#include "drv8316crq1.h"
#include "button.h"
#include "buzzer.h"
#include "mct8316z.h"
#include "lsm6ds3tr-c.h"
#include "SDcard.h"
#include "mt6701.h"

#include "user_init.h"
#include "menu.h"
#include "foc.h"

// ==========================================================
// ★ 주행 마커 txt 파일 전체 삭제 함수
// ==========================================================
void Delete_All_Marker_Logs(void) {
    char filepath[64];
    FATFS fs; // 로컬 파일 시스템 객체 생성
    FRESULT res;

    // 1. SD 카드 논리 드라이브("0:") 강제 마운트 (1 = 즉시 마운트)
    if (f_mount(&fs, "0:", 1) != FR_OK) {
        LCD_Printf(0, 8, "Del Mount Fail");
        HAL_Delay(1000);
        return; // 마운트 실패 시 함수 종료
    }

    // 2. 파일 삭제 루프 진행
    for (int i = 1; i <= 10; i++) {
        sprintf(filepath, "0:/Drive_Data/save_slot_%d.txt", i);
        res = f_unlink(filepath);

        // FR_NO_FILE(E4)은 파일이 없는 정상이므로 무시
        if (res != FR_OK && res != FR_NO_FILE) {
            LCD_Printf(0, 8, "Del Fail %-2d: E%d", i, res);
            HAL_Delay(500);
        }
    }

    // 3. 안전을 위해 삭제 작업 완료 후 마운트 해제
    f_mount(NULL, "0:", 0);
}

void User_Init() {
	Button_init();
	Buzzer_Init();
//	Buzzer_Start();
	LCD7789_Test();
	LSM6DS3_Init();
	Buzzer_Stop();
	MX_DRV8316C_Init();
	LCD_Printf(0, 5, "DRV8316 SET OK");
	FOC_Init_Motor(&foc_L, &htim3, &hadc2, &hlptim2);
	FOC_Init_Motor(&foc_R, &htim4, &hadc1, &hlptim1);

	FOC_ADC_Start();

	LCD_Printf(0, 6, "FOC L ADC Cali");
	FOC_Calibrate_Offset(&foc_L);

	LCD_Printf(0, 7, "FOC R ADC Cali");
	FOC_Calibrate_Offset(&foc_R);

	FOC_ADC_Stop();

//	uint8_t encBuffer[3] = { 0 };
//	MT6701_Init(&encDataL, encBuffer);
//	LCD_Printf(0, 8, "ENC %02X%02X%02X", encBuffer[2], encBuffer[1], encBuffer[0]);

	LCD_Printf(0, 9, "IMU Cali");
	LSM6DS3_Gyro_Calibrate_Z_Only();

//	SDCard_DebugTest();

	FRESULT res;
	if ((res = SDCard_Test()) != FR_OK) {
		LCD_Printf(0, 6, "SDcard Fail");
		while (1) {
			LED_Blink(250);
		}
	}

	// ★ SD 카드 정상 인식 후 마커 기록 파일들 삭제 함수 호출
	Delete_All_Marker_Logs();

//	if ((res = FOC_Parameters_InitOrLoad()) != FR_OK) {
//		LCD_Printf(0, 7, "FOC param save Fail");
//	}

	HAL_Delay(500);

}
