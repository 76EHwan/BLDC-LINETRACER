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

FRESULT SDCard_Test(void);

extern uint8_t sdcard_err;

typedef enum {
    SDCFG_FLOAT,
    SDCFG_INT8,
    SDCFG_UINT16,
} SDCard_ConfigType;

typedef struct {
    const char *key;       // "L_offset_a" 같은 이름
    volatile void *ptr;     // 실제 변수 주소 (float*, int8_t*, uint16_t* 등). volatile 변수도 그대로 받도록 volatile void*로 선언
    SDCard_ConfigType type;
} SDCard_ConfigEntry;

FRESULT SDCard_Mount(void);
void SDCard_Unmount(void);
FRESULT SDCard_Write(const char* filename, const char* data);
FRESULT SDCard_Read(const char* filename, char* buffer, UINT bufSize);

FRESULT SDCard_Mkdir(const char* dirname);
FRESULT SDCard_WriteBinary(const char* filename, const void* data, UINT size);
FRESULT SDCard_ReadBinary(const char* filename, void* buffer, UINT size);

FRESULT SDCard_Save(const char* path, const void* data, UINT size);
FRESULT SDCard_Load(const char* path, void* buffer, UINT size);
uint8_t SDCard_FileExists(const char* path);

FRESULT SDCard_Test(void);
void SDCard_DebugTest(void);

FRESULT SDCard_SaveConfig(const char *path, const SDCard_ConfigEntry *entries, int count);
FRESULT SDCard_LoadConfig(const char *path, const SDCard_ConfigEntry *entries, int count);

#endif /* BSP_SDCARD_SDCARD_H_ */
