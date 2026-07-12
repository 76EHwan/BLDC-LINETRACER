/*
 * sd_diskio_patch.c
 *
<<<<<<< HEAD
 *  Created on: 2026. 7. 12.
 *      Author: kth59
 */


/*
 * sd_diskio_patch.c
 *
 * CubeMX가 매번 sd_diskio.c를 스켈레톤으로 덮어써서 SD_write()의
 * 캐시 클린(clean) 누락 버그 수정이 계속 사라지는 문제 때문에 만든 파일.
 *
 * 이 파일은 CubeMX가 전혀 알지 못하는 "수동 추가 파일"이므로
 * Generate Code를 아무리 눌러도 절대 덮어써지거나 사라지지 않는다.
 *
 * - SD_initialize / SD_status / SD_read / SD_ioctl 은 sd_diskio.c에 있는
 *   원본 함수를 extern 선언해서 그대로 재사용한다 (읽기는 이미 정상 동작 확인됨).
 * - SD_write 만 새로 구현한다. sd_diskio.c의 static WriteStatus/scratch를
 *   건드릴 수 없으므로(파일 스코프), 완료 대기는 BSP_SD_GetCardState() 폴링으로
 *   대체하고, 자체 32바이트 정렬 scratch 버퍼를 따로 둔다.
 */

#include "sd_diskio_patch.h"
#include "sd_diskio.h"
#include <string.h>

#define SD_FIXED_TIMEOUT   (30 * 1000)
#define SD_FIXED_BLOCKSIZE 512

/* sd_diskio.c에 있는 원본 함수들 (static이 아니므로 외부에서 링크 가능) */
extern DSTATUS SD_initialize(BYTE lun);
extern DSTATUS SD_status(BYTE lun);
extern DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
#if _USE_IOCTL == 1
extern DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff);
#endif

/* 이 파일 전용 32바이트 정렬 scratch 버퍼 (sd_diskio.c의 것과 별개) */
__attribute__((aligned(32))) static uint8_t scratch_fixed[SD_FIXED_BLOCKSIZE];

static int SD_Fixed_WaitReady(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	while ((HAL_GetTick() - tick) < timeout) {
		if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
			return 0;
	}
	return -1;
}

static DRESULT SD_write_Fixed(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
=======
 * Created on: 2026. 7. 13.
 * Author: kth59
 */


/*
 * sd_diskio_patch.c
 *
 * CubeMX가 매번 sd_diskio.c를 스켈레톤으로 덮어써서 SD_write()의
 * 캐시 클린(clean) 누락 버그 수정이 계속 사라지는 문제 때문에 만든 파일.
 *
 * 이 파일은 CubeMX가 전혀 알지 못하는 "수동 추가 파일"이므로
 * Generate Code를 아무리 눌러도 절대 덮어써지거나 사라지지 않는다.
 *
 * - SD_initialize / SD_status / SD_read / SD_ioctl 은 sd_diskio.c에 있는
 * 원본 함수를 extern 선언해서 그대로 재사용한다 (읽기는 이미 정상 동작 확인됨).
 * - SD_write 만 새로 구현한다. sd_diskio.c의 static WriteStatus/scratch를
 * 건드릴 수 없으므로(파일 스코프), 완료 대기는 BSP_SD_GetCardState() 폴링으로
 * 대체하고, 자체 32바이트 정렬 scratch 버퍼를 따로 둔다.
 */

#include "sd_diskio_patch.h"
#include "sd_diskio.h"
#include <string.h>

#define SD_FIXED_TIMEOUT   (30 * 1000)
#define SD_FIXED_BLOCKSIZE 512

/* sd_diskio.c에 있는 원본 함수들 (static이 아니므로 외부에서 링크 가능) */
extern DSTATUS SD_initialize(BYTE lun);
extern DSTATUS SD_status(BYTE lun);
extern DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
#if _USE_IOCTL == 1
extern DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff);
#endif

/* 이 파일 전용 32바이트 정렬 scratch 버퍼 (sd_diskio.c의 것과 별개) */
__attribute__((aligned(32))) static uint8_t scratch_fixed[SD_FIXED_BLOCKSIZE];

/* 진단용: WaitReady가 실패했을 때 마지막으로 관찰된 상태값들을 저장.
 * SDCard_DebugTest() 등에서 extern으로 가져다 LCD에 찍어보면
 * 정확히 어떤 상태에서 멈췄는지 확인 가능. */
uint32_t SD_Diag_LastCardState = 0xFFFFFFFF;
uint32_t SD_Diag_LastHalState  = 0xFFFFFFFF;
uint32_t SD_Diag_LastHalError  = 0xFFFFFFFF;

extern SD_HandleTypeDef hsd1;

static int SD_Fixed_WaitReady(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	uint32_t state;

	/* 1. STM32 내부의 DMA 전송 완료 대기 (명령어 전송 금지) */
	while (HAL_SD_GetState(&hsd1) != HAL_SD_STATE_READY) {
		if ((HAL_GetTick() - tick) >= timeout) {
			SD_Diag_LastHalState = (uint32_t)HAL_SD_GetState(&hsd1);
			SD_Diag_LastHalError = HAL_SD_GetError(&hsd1);
			return -1;
		}
	}

	/* 2. SD 카드의 내부 플래시 쓰기 완료 대기 */
	while ((HAL_GetTick() - tick) < timeout) {
		state = BSP_SD_GetCardState();
		if (state == SD_TRANSFER_OK)
			return 0;
	}

	/* 타임아웃 직전 마지막 상태를 저장 */
	SD_Diag_LastCardState = BSP_SD_GetCardState();
	SD_Diag_LastHalState  = (uint32_t)HAL_SD_GetState(&hsd1);
	SD_Diag_LastHalError  = HAL_SD_GetError(&hsd1);
	return -1;
}

uint32_t SD_Diag_HandleInstance = 0;
uint32_t SD_Diag_HandleStateEntry = 0;

static DRESULT SD_write_Fixed(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
	/* 진단: 아무것도 하기 전에 hsd1 핸들이 이미 깨져있는지 확인.
	 * Instance는 반드시 SDMMC1 베이스 주소(0x52007000대)여야 정상. */
	SD_Diag_HandleInstance   = (uint32_t)hsd1.Instance;
	SD_Diag_HandleStateEntry = (uint32_t)hsd1.State;

>>>>>>> refs/heads/Bug/SDcard
	if (SD_Fixed_WaitReady(SD_FIXED_TIMEOUT) < 0)
		return RES_ERROR;

	for (UINT i = 0; i < count; i++) {
		memcpy(scratch_fixed, buff, SD_FIXED_BLOCKSIZE);

		/*
		 * 핵심 수정: CPU가 memcpy로 scratch_fixed에 쓴 내용은 D-Cache에만
		 * 있을 수 있다. DMA는 캐시를 거치지 않고 물리 메모리를 직접 읽으므로,
		 * DMA를 시작하기 전에 반드시 캐시를 물리 메모리로 flush(clean)해야
		 * DMA가 방금 쓴 실제 데이터를 카드로 전송한다.
		 */
		SCB_CleanDCache_by_Addr((uint32_t*)scratch_fixed,
				(SD_FIXED_BLOCKSIZE + 31) & ~31);

		if (BSP_SD_WriteBlocks_DMA((uint32_t*)scratch_fixed,
				(uint32_t)(sector + i), 1) != MSD_OK) {
			return RES_ERROR;
		}

		if (SD_Fixed_WaitReady(SD_FIXED_TIMEOUT) < 0)
			return RES_ERROR;

		buff += SD_FIXED_BLOCKSIZE;
	}

	return RES_OK;
}

const Diskio_drvTypeDef SD_Driver_Fixed = {
	SD_initialize,
	SD_status,
	SD_read,
#if _USE_WRITE == 1
	SD_write_Fixed,
#endif
#if _USE_IOCTL == 1
	SD_ioctl,
#endif
};
