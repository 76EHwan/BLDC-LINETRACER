/*
 * SDcard.h
 *
 *  Created on: 2026. 4. 15.
 *      Author: kth59
 */

#ifndef BSP_SDCARD_SDCARD_H_
#define BSP_SDCARD_SDCARD_H_

#include "ff.h"
#include <string.h>

void SDCard_Test(void);

extern uint8_t sdcard_err;


// =====================
// SD카드 마운트
// =====================
FRESULT SDCard_Mount(void);
void SDCard_Unmount(void);
FRESULT SDCard_Write(const char* filename, const char* data);
FRESULT SDCard_Read(const char* filename, char* buffer, UINT bufSize);

FRESULT SDCard_Mkdir(const char* dirname);
FRESULT SDCard_WriteBinary(const char* filename, const void* data, UINT size);
FRESULT SDCard_ReadBinary(const char* filename, void* buffer, UINT size);

FRESULT SDCard_Save(const char* path, const void* data, UINT size);
FRESULT SDCard_Load(const char* path, void* buffer, UINT size);

void SDCard_Test(void);
void SDCard_DebugTest(void);

#endif /* BSP_SDCARD_SDCARD_H_ */
