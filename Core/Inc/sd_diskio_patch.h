/*
 * sd_diskio_patch.h
 *
 *  Created on: 2026. 7. 13.
 *      Author: kth59
 */

#ifndef INC_SD_DISKIO_PATCH_H_
#define INC_SD_DISKIO_PATCH_H_

#include "ff_gen_drv.h"

/* fatfs.c의 USER CODE BEGIN Init 에서 이 드라이버로 교체해서 쓴다 */
extern const Diskio_drvTypeDef SD_Driver_Fixed;

/* 진단용: SD_write_Fixed가 WaitReady 타임아웃 났을 때의 마지막 상태값들 */
extern uint32_t SD_Diag_LastCardState;
extern uint32_t SD_Diag_LastHalState;
extern uint32_t SD_Diag_LastHalError;
extern uint32_t SD_Diag_HandleInstance;
extern uint32_t SD_Diag_HandleStateEntry;

#endif /* INC_SD_DISKIO_PATCH_H_ */
