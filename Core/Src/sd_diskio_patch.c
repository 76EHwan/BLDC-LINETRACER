/*
 * sd_diskio_patch.c
 *
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
