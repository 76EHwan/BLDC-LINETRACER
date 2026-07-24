/*
 * sd_ui.h
 *
 *  Created on: 2026. 7. 24.
 *      Author: kth59
 */

#ifndef INC_SD_UI_H_
#define INC_SD_UI_H_

#include "main.h"
#include "ff.h"

#define CALIBRATION_PATH	"/Sensor_Data/calibration_result.txt"

#define FOC_PARAM_PATH		"/Foc_Data/foc_param.txt"


// SD카드 저장/불러오기 통합 함수
FRESULT Sensor_Save_Calibration(void);
FRESULT Sensor_Load_Calibration(void);
FRESULT Save_FOC_Parameters(void);
FRESULT Load_FOC_Parameters(void);
void Save_MarkerLog_To_SD(uint8_t slot_number);

// UI 함수
uint8_t Select_Save_Slot(void);

#endif /* INC_SD_UI_H_ */
