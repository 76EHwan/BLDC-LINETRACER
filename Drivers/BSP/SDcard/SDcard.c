#include "SDcard.h"
#include "fatfs.h"
#include "user_init.h"

uint8_t sdcard_err = 1;

FRESULT SDCard_Mount(void) {
	return f_mount(&SDFatFS_NC, "", 1);
}

void SDCard_Unmount(void) {
	f_mount(NULL, "", 0);
}

FRESULT SDCard_Write(const char *filename, const char *data) {
	FRESULT res, res_close;
	UINT bytesWritten;

	res = f_open(&SDFile_NC, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK)
		return res;

	res = f_write(&SDFile_NC, data, strlen(data), &bytesWritten);
<<<<<<< HEAD
	res_close = f_close(&SDFile_NC);

	/* f_close()에서 실제 디스크 커밋(FAT/디렉토리 엔트리 flush)이 실패할 수 있으므로
	 * f_write()의 결과보다 우선해서 확인해야 진짜 실패를 놓치지 않는다. */
	if (res_close != FR_OK)
		return res_close;
=======
	f_close(&SDFile_NC);
>>>>>>> refs/heads/Bug/SDcard

	if (res == FR_OK && bytesWritten == 0)
		return FR_DENIED;
	return res;
}

FRESULT SDCard_Read(const char *filename, char *buffer, UINT bufSize) {
	FRESULT res;
	UINT bytesRead;

	res = f_open(&SDFile_NC, filename, FA_READ);
	if (res != FR_OK)
		return res;

	memset(buffer, 0, bufSize);
	res = f_read(&SDFile_NC, buffer, bufSize - 1, &bytesRead);
	f_close(&SDFile_NC);

	if (res == FR_OK && bytesRead == 0)
		return FR_DENIED;
	return res;
}

uint8_t SDCard_FileExists(const char *path) {
	FRESULT res = SDCard_Mount();
	if (res != FR_OK)
		return 0;

	FILINFO fno;
	res = f_stat(path, &fno);
	SDCard_Unmount();

	return (res == FR_OK) ? 1 : 0;
}

FRESULT SDCard_Test(void) {
	FRESULT res;
	char writeData[] = "STM32 FATFS Write & Read Test Success!";
	char readBuffer[50];

	LCD_Printf(0, 0, "[SD Test]");

	res = SDCard_Mount();
	if (res != FR_OK) {
		LCD_Printf(0, 1, "FAIL: Mount %d", res);
		return res;
	}

	res = SDCard_Write("test.txt", writeData);
	if (res != FR_OK) {
		LCD_Printf(0, 1, "FAIL: Write %d", res);
		SDCard_Unmount();
		return res;
	}

	res = SDCard_Read("test.txt", readBuffer, sizeof(readBuffer));
	if (res != FR_OK) {
		LCD_Printf(0, 1, "FAIL: Read %d", res);
		SDCard_Unmount();
		return res;
	}

	if (strcmp(writeData, readBuffer) != 0) {
		LCD_Printf(0, 1, "FAIL: Mismatch");
		return FR_NOT_ENABLED;
	}

	LCD_Printf(0, 2, readBuffer);

	SDCard_Unmount();
	return FR_OK;
}

FRESULT SDCard_Mkdir(const char *dirname) {
	FRESULT res = f_mkdir(dirname);
	if (res == FR_EXIST)
		return FR_OK;
	return res;
}

FRESULT SDCard_WriteBinary(const char *filename, const void *data, UINT size) {
	FRESULT res, res_close;
	UINT bytesWritten;

	res = f_open(&SDFile_NC, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK)
		return res;

	res = f_write(&SDFile_NC, data, size, &bytesWritten);
<<<<<<< HEAD
	res_close = f_close(&SDFile_NC);

	if (res_close != FR_OK)
		return res_close;
=======
	f_close(&SDFile_NC);
>>>>>>> refs/heads/Bug/SDcard

	if (res == FR_OK && bytesWritten != size)
		return FR_DENIED;
	return res;
}

FRESULT SDCard_ReadBinary(const char *filename, void *buffer, UINT size) {
	FRESULT res;
	UINT bytesRead;

	res = f_open(&SDFile_NC, filename, FA_READ);
	if (res != FR_OK)
		return res;

	res = f_read(&SDFile_NC, buffer, size, &bytesRead);
	f_close(&SDFile_NC);

	if (res == FR_OK && bytesRead != size)
		return FR_DENIED;
	return res;
}

static FRESULT SDCard_MkdirRecursive(const char *path) {
	char buf[128];
	strncpy(buf, path, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	char *lastSlash = strrchr(buf, '/');
	if (lastSlash == NULL || lastSlash == buf) {
		return FR_OK;
	}
	*lastSlash = '\0';

	char dirPath[128] = { 0 };
	char *token = strtok(buf, "/");
	FRESULT res = FR_OK;

	while (token != NULL) {
		strcat(dirPath, "/");
		strcat(dirPath, token);
		res = f_mkdir(dirPath);
		if (res != FR_OK && res != FR_EXIST) {
			return res;
		}
		token = strtok(NULL, "/");   // 반드시 추가
	}
	return FR_OK;
}
// path에 data를 size만큼 바이너리로 저장.
// 상위 디렉토리가 없으면 자동 생성, mount/unmount도 알아서 처리.
FRESULT SDCard_Save(const char *path, const void *data, UINT size) {
	FRESULT res = SDCard_Mount();
	if (res != FR_OK)
		return res;

	res = SDCard_MkdirRecursive(path);
	if (res != FR_OK) {
		SDCard_Unmount();
		return res;
	}

	res = SDCard_WriteBinary(path, data, size);
	SDCard_Unmount();
	return res;
}

void SDCard_DebugTest(void) {
	FRESULT res = SDCard_Mount();
	LCD_Printf(0, 3, "Mount: %d", res);

	static uint8_t buf[512];
	memset(buf, 0xAA, sizeof(buf));
	memset(&SDFile_NC, 0, sizeof(FIL));

	res = f_open(&SDFile_NC, "split2.bin", FA_CREATE_ALWAYS | FA_WRITE);
	LCD_Printf(0, 4, "Open: %d", res);

	UINT bw;
	FRESULT r1 = f_write(&SDFile_NC, buf, 512, &bw);
	LCD_Printf(0, 5, "W1: res=%d bw=%u", r1, bw);   // bw 값도 확인

	FRESULT r_sync = f_sync(&SDFile_NC);
	LCD_Printf(0, 9, "Sync: %d", r_sync);

	FRESULT rc = f_close(&SDFile_NC);
	LCD_Printf(0, 6, "Close: %d", rc);              // 반드시 확인
	// 언마운트 없이 같은 세션에서 바로 확인
	FRESULT chk = f_stat("split2.bin", &fno);
	LCD_Printf(0, 7, "StatSame: %d sz=%lu", chk, fno.fsize);

	// 여유 클러스터 확인
	DWORD free_clust;
	FATFS *fs = &SDFatFS_NC;
	FRESULT gf = f_getfree("", &free_clust, &fs);
	LCD_Printf(0, 11, "Free: %d clust=%lu", gf, free_clust);

	SDCard_Unmount();
	while (1) {
	}
}
// path에서 size만큼 읽어 buffer에 덮어씀.
// mount/unmount는 알아서 처리.
FRESULT SDCard_Load(const char *path, void *buffer, UINT size) {
	FRESULT res = SDCard_Mount();
	if (res != FR_OK)
		return res;

	res = SDCard_ReadBinary(path, buffer, size);
	SDCard_Unmount();
	return res;
}

FRESULT SDCard_SaveConfig(const char *path, const SDCard_ConfigEntry *entries, int count) {
	char buf[1024];
	int len = 0;

	for (int i = 0; i < count; i++) {
		switch (entries[i].type) {
		case SDCFG_FLOAT:
			len += sprintf(buf + len, "%s=%f\n", entries[i].key, *(float*)entries[i].ptr);
			break;
		case SDCFG_INT8:
			len += sprintf(buf + len, "%s=%d\n", entries[i].key, *(int8_t*)entries[i].ptr);
			break;
		}
		if (len >= (int)sizeof(buf) - 64) break; // 버퍼 오버플로우 방지
	}

	return SDCard_Save(path, buf, len);
}

static int SDCard_FindValue(const char *text, const char *key, float *out) {
	size_t keylen = strlen(key);
	const char *p = text;

	while ((p = strstr(p, key)) != NULL) {
		// key 앞이 줄 시작(첫 글자 또는 개행 직후)인지 확인해서 부분 일치 오탐 방지
		int atLineStart = (p == text) || (*(p - 1) == '\n');
		if (atLineStart && p[keylen] == '=') {
			*out = strtof(p + keylen + 1, NULL);
			return 1;
		}
		p += keylen;
	}
	return 0;
}

FRESULT SDCard_LoadConfig(const char *path, const SDCard_ConfigEntry *entries, int count) {
	static char buf[1024];
	FRESULT res = SDCard_Load(path, buf, sizeof(buf) - 1);
	if (res != FR_OK)
		return res;
	buf[sizeof(buf) - 1] = '\0';

	float v;
	for (int i = 0; i < count; i++) {
		if (!SDCard_FindValue(buf, entries[i].key, &v))
			continue;

		switch (entries[i].type) {
		case SDCFG_FLOAT:
			*(float*)entries[i].ptr = v;
			break;
		case SDCFG_INT8:
			*(int8_t*)entries[i].ptr = (int8_t)v;
			break;
		}
	}
	return FR_OK;
}
