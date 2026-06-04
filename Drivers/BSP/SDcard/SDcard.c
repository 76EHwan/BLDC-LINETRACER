#include "SDcard.h"

extern FATFS SDFatFS;
FIL file;

// =====================
// SD카드 마운트
// =====================
FRESULT SDCard_Mount(void) {
	return f_mount(&SDFatFS, "", 1);
}

void SDCard_Unmount(void) {
	f_mount(NULL, "", 0);
}

// =====================
// 파일 쓰기
// =====================
FRESULT SDCard_Write(const char *filename, const char *data) {
	FRESULT res;
	UINT bytesWritten;

	res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK)
		return res;

	res = f_write(&file, data, strlen(data), &bytesWritten);
	f_close(&file);

	if (res == FR_OK && bytesWritten == 0)
		return FR_DENIED;
	return res;
}

// =====================
// 파일 읽기
// =====================
FRESULT SDCard_Read(const char *filename, char *buffer, UINT bufSize) {
	FRESULT res;
	UINT bytesRead;

	res = f_open(&file, filename, FA_READ);
	if (res != FR_OK)
		return res;

	memset(buffer, 0, bufSize);
	res = f_read(&file, buffer, bufSize - 1, &bytesRead);
	f_close(&file);

	if (res == FR_OK && bytesRead == 0)
		return FR_DENIED;
	return res;
}
