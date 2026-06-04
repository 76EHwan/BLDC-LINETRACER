/*
 * SDcard.h
 *
 *  Created on: 2026. 4. 15.
 *      Author: kth59
 */

#ifndef BSP_SDCARD_SDCARD_H_
#define BSP_SDCARD_SDCARD_H_

#include "fatfs.h"
#include <string.h>

void SDCard_Test(void);

extern FATFS fs;

// =====================
// SD카드 마운트
// =====================
FRESULT SDCard_Mount(void);
void SDCard_Unmount(void);
FRESULT SDCard_Write(const char* filename, const char* data);
FRESULT SDCard_Read(const char* filename, char* buffer, UINT bufSize);

#endif /* BSP_SDCARD_SDCARD_H_ */
