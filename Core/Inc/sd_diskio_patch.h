/*
 * sd_diskio_patch.h
 *
<<<<<<< HEAD
 * CubeMX가 "Generate Code"할 때마다 sd_diskio.c를 스켈레톤으로 덮어써서
 * SD_write()의 캐시 버그 수정이 계속 사라지는 문제를 피하기 위한 파일.
 * 이 파일은 CubeMX가 전혀 알지 못하므로 재생성해도 절대 지워지지 않는다.
 *  Created on: 2026. 7. 12.
 *      Author: kth59
 */

#ifndef INC_SD_DISKIO_PATCH_H_
#define INC_SD_DISKIO_PATCH_H_

#include "ff_gen_drv.h"

/* fatfs.c의 USER CODE BEGIN Init 에서 이 드라이버로 교체해서 쓴다 */
extern const Diskio_drvTypeDef SD_Driver_Fixed;
=======
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
>>>>>>> refs/heads/Bug/SDcard

#endif /* INC_SD_DISKIO_PATCH_H_ */
