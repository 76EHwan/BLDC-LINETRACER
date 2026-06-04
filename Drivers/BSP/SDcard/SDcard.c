#include "SDcard.h"
#include "st7789_lcd.h"

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

void SDCard_Test(void) {
	FRESULT res;
	char writeData[] = "STM32 FATFS Write & Read Test Success!";
	char readBuffer[50];

	LCD7789_Printf(0, 0, "[SD Test]");

	res = SDCard_Mount();
	if (res != FR_OK) {
		LCD7789_Printf(0, 1, "FAIL: Mount %d", res);
		return;
	}

	res = SDCard_Write("test.txt", writeData);
	if (res != FR_OK) {
		LCD7789_Printf(0, 1, "FAIL: Write %d", res);
		SDCard_Unmount();
		return;
	}

	res = SDCard_Read("test.txt", readBuffer, sizeof(readBuffer));
	if (res != FR_OK) {
		LCD7789_Printf(0, 1, "FAIL: Read %d", res);
		SDCard_Unmount();
		return;
	}

	if (strcmp(writeData, readBuffer) == 0) {
		LCD7789_Printf(0, 1, "SUCCESS!");
	} else {
		LCD7789_Printf(0, 1, "FAIL: Mismatch");
	}

	LCD7789_Printf(0, 2, readBuffer);

	SDCard_Unmount();
}
