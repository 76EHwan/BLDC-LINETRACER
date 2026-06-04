#include "st7735.h"
#include "st7735_lcd.h"
#include "font.h"
#include "spi.h"
#include "tim.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TFT96

//SPI 통신 인터페이스
//LCD_RST
#define LCD_RST_SET     
#define LCD_RST_RESET  
//LCD_RS//dc  
#define LCD_RS_SET      HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_SET)
#define LCD_RS_RESET    HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_RESET)
//LCD_CS  
#define LCD_CS_SET      HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)
#define LCD_CS_RESET    HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)
//SPI Driver
#define SPI spi4
#define SPI_Drv (&hspi4)
#define delay_ms HAL_Delay
#define get_tick HAL_GetTick
//LCD_Brightness timer
#define LCD_Brightness_timer &htim1
#define LCD_Brightness_channel TIM_CHANNEL_2

static int32_t lcd7735_init(void);
static int32_t lcd7735_gettick(void);
static int32_t lcd7735_writereg(uint8_t reg, uint8_t *pdata, uint32_t length);
static int32_t lcd7735_readreg(uint8_t reg, uint8_t *pdata);
static int32_t lcd7735_senddata(uint8_t *pdata, uint32_t length);
static int32_t lcd7735_recvdata(uint8_t *pdata, uint32_t length);

uint16_t LCD7735_BACK_BRIGHT = 600;

ST7735_IO_t st7735_pIO = { lcd7735_init,
0, 0, lcd7735_writereg, lcd7735_readreg, lcd7735_senddata, lcd7735_recvdata, lcd7735_gettick };

ST7735_Object_t st7735_pObj;
uint32_t st7735_id;

void LCD7735_Test(void) {
	uint8_t text[20];

#if defined(TFT96)
	ST7735Ctx.Orientation = ST7735_ORIENTATION_PORTRAIT;
	ST7735Ctx.Panel = HannStar_Panel;
	ST7735Ctx.Type = ST7735_0_9_inch_screen;
#elif defined(TFT18)
	ST7735Ctx.Orientation = ST7735_ORIENTATION_LANDSCAPE_ROT180;
	ST7735Ctx.Panel = HannStar_Panel;
	ST7735Ctx.Type = ST7735_1_8a_inch_screen;
#else
	#error "Unknown Screen"
#endif

	ST7735_RegisterBusIO(&st7735_pObj, &st7735_pIO);
	ST7735_LCD_Driver.Init(&st7735_pObj, ST7735_FORMAT_RBG565, &ST7735Ctx);
	ST7735_LCD_Driver.ReadID(&st7735_pObj, &st7735_id);

	LCD7735_SetBrightness(0);

#ifdef TFT96
	extern unsigned char WeActStudiologo_160_80[];
	ST7735_LCD_Driver.DrawBitmap(&st7735_pObj, 0, 0, WeActStudiologo_160_80);
#elif defined(TFT18)
	extern unsigned char WeActStudiologo_128_160[];
	ST7735_LCD_Driver.DrawBitmap(&st7735_pObj,0,0,WeActStudiologo_128_160);	
#endif

	uint32_t tick = get_tick();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		delay_ms(10);

		if (get_tick() - tick <= 1000)
			LCD7735_SetBrightness((get_tick() - tick) * LCD7735_BACK_BRIGHT / 1000);
		else if (get_tick() - tick <= 3000) {
			sprintf((char*) &text, "%03ld", (get_tick() - tick - 1000) / 10);
			LCD7735_ShowString(ST7735Ctx.Width - 20, 1, ST7735Ctx.Width, 12, 12, text);
			ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, ST7735Ctx.Height - 3,
					(get_tick() - tick - 1000) * ST7735Ctx.Width / 2000, 3, 0xFFFF);
		} else if (get_tick() - tick > 3000)
			break;
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
		delay_ms(10);
	}
	LCD7735_Light(0, 300);

	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width,
			ST7735Ctx.Height, BLACK);

	LCD7735_Light(LCD7735_BACK_BRIGHT, 300);
}

static uint32_t LCD7735_LightSet;
static uint8_t IsLCD7735_SoftPWM = 0;

void LCD7735_SetBrightness(uint32_t Brightness) {
	LCD7735_LightSet = Brightness;
	if (!IsLCD7735_SoftPWM)
		__HAL_TIM_SetCompare(LCD_Brightness_timer, LCD_Brightness_channel, Brightness);
}

uint32_t LCD7735_GetBrightness(void) {
	if (IsLCD7735_SoftPWM)
		return LCD7735_LightSet;
	else
		return __HAL_TIM_GetCompare(LCD_Brightness_timer, LCD_Brightness_channel);
}

void LCD7735_SoftPWMEnable(uint8_t enable) {
	IsLCD7735_SoftPWM = enable;
	if (!enable)
		LCD7735_SetBrightness(LCD7735_LightSet);
}

uint8_t LCD7735_SoftPWMIsEnable(void) {
	return IsLCD7735_SoftPWM;
}

void LCD7735_SoftPWMCtrlInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	MX_TIM16_Init(); // Freq: 10K
	HAL_TIM_Base_Start_IT(&htim16);

	LCD7735_SoftPWMEnable(1);
}

void LCD7735_SoftPWMCtrlDeInit(void) {
	HAL_TIM_Base_DeInit(&htim16);
	HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);
}

void LCD7735_SoftPWMCtrlRun(void) {
	static uint32_t timecount;

	if (timecount > 1000)
		timecount = 0;
	else
		timecount += 10;

	if (timecount >= LCD7735_LightSet)
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
}

/* * HAL_TIM_PeriodElapsedCallback 은 메인 인터럽트 파일(stm32h7xx_it.c 등)에
 * 전역으로 하나만 존재해야 하므로 이곳에서는 주석 처리합니다.
 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// 	if (htim->Instance == TIM16) {
// 		LCD7735_SoftPWMCtrlRun();
// 	}
// }

void LCD7735_Light(uint32_t Brightness_Dis, uint32_t time) {
	uint32_t Brightness_Now;
	uint32_t time_now;
	float temp1, temp2;
	float k, set;

	Brightness_Now = LCD7735_GetBrightness();
	time_now = 0;
	if (Brightness_Now == Brightness_Dis)
		return;

	if (time == time_now)
		return;

	temp1 = Brightness_Now;
	temp1 = temp1 - Brightness_Dis;
	temp2 = time_now;
	temp2 = temp2 - time;

	k = temp1 / temp2;

	uint32_t tick = get_tick();
	while (1) {
		delay_ms(1);
		time_now = get_tick() - tick;
		temp2 = time_now - 0;
		set = temp2 * k + Brightness_Now;
		LCD7735_SetBrightness((uint32_t) set);
		if (time_now >= time)
			break;
	}
}

uint16_t LCD7735_POINT_COLOR = 0xFFFF;
uint16_t LCD7735_BACK_COLOR = BLACK;

void LCD7735_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode) {
	uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint16_t x0 = x;
	uint16_t colortemp = LCD7735_POINT_COLOR;
	uint32_t h, w;

	uint16_t write[size][size == 12 ? 6 : 8];
	uint16_t count;

	ST7735_GetXSize(&st7735_pObj, &w);
	ST7735_GetYSize(&st7735_pObj, &h);

	num = num - ' ';
	count = 0;

	if (!mode) {
		for (t = 0; t < size; t++) {
			if (size == 12)
				temp = asc2_1206[num][t];
			else
				temp = asc2_1608[num][t];

			for (t1 = 0; t1 < 8; t1++) {
				if (temp & 0x80)
					LCD7735_POINT_COLOR = (colortemp & 0xFF) << 8 | colortemp >> 8;
				else
					LCD7735_POINT_COLOR = (LCD7735_BACK_COLOR & 0xFF) << 8 | LCD7735_BACK_COLOR >> 8;

				write[count][t / 2] = LCD7735_POINT_COLOR;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y >= h) {
					LCD7735_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						LCD7735_POINT_COLOR = colortemp;
						return;
					}
					break;
				}
			}
		}
	} else {
		for (t = 0; t < size; t++) {
			if (size == 12)
				temp = asc2_1206[num][t];
			else
				temp = asc2_1608[num][t];
			for (t1 = 0; t1 < 8; t1++) {
				if (temp & 0x80)
					write[count][t / 2] = (LCD7735_POINT_COLOR & 0xFF) << 8 | LCD7735_POINT_COLOR >> 8;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y >= h) {
					LCD7735_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						LCD7735_POINT_COLOR = colortemp;
						return;
					}
					break;
				}
			}
		}
	}
	ST7735_FillRGBRect(&st7735_pObj, x0, y0, (uint8_t*) &write,
			size == 12 ? 6 : 8, size);
	LCD7735_POINT_COLOR = colortemp;
}

void LCD7735_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p) {
	uint8_t x0 = x;
	width += x;
	height += y;
	while ((*p <= '~') && (*p >= ' ')) {
		if (x + size / 2 > width) {
			x = x0;
			y += size;
		}
		if (y >= height)
			break;
		LCD7735_ShowChar(x, y, *p, size, 0);
		x += size / 2;
		p++;
	}
}

static int32_t lcd7735_init(void) {
	int32_t result = ST7735_OK;
	HAL_TIMEx_PWMN_Start(LCD_Brightness_timer, LCD_Brightness_channel);
	return result;
}

static int32_t lcd7735_gettick(void) {
	return HAL_GetTick();
}

static int32_t lcd7735_writereg(uint8_t reg, uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;
	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, 100);
	LCD_RS_SET;
	if (length > 0)
		result += HAL_SPI_Transmit(SPI_Drv, pdata, length, 500);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

static int32_t lcd7735_readreg(uint8_t reg, uint8_t *pdata) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;

	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, 100);
	LCD_RS_SET;
	result += HAL_SPI_Receive(SPI_Drv, pdata, 1, 500);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

static int32_t lcd7735_senddata(uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	result = HAL_SPI_Transmit(SPI_Drv, pdata, length, 100);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

static int32_t lcd7735_recvdata(uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	result = HAL_SPI_Receive(SPI_Drv, pdata, length, 500);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

void LCD7735_Printf(uint8_t x, uint8_t y, const char *text, ...) {
	char txt[512] = { 0 };
	va_list args;
	va_start(args, text);
	vsprintf(txt, text, args);
	va_end(args);

	uint16_t px = 6 * x + 1;
	uint16_t py = 14 * y + 3;

	LCD7735_ShowString(px, py, ST7735Ctx.Width - px - 1, ST7735Ctx.Height - py - 1,
			12, (uint8_t*) txt);
}

void LCD7735_Clear() {
	LCD7735_Light(0, 250);

	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width,
			ST7735Ctx.Height, BLACK);

	LCD7735_Light(900, 250);
}

void LCD7735_Display_Random_BMP_From_SD(const TCHAR *address) {
	FRESULT res;
	DIR dir;
	FILINFO fno;
	int bmp_count = 0;

	LCD7735_Clear();
	LCD7735_Printf(0, 0, "[BMP Load Test]");

	if (SDCard_Mount() != FR_OK) {
		LCD7735_Printf(0, 1, "ERR: SD Mount");
		return;
	}

	res = f_opendir(&dir, address);
	if (res != FR_OK) {
		LCD7735_Printf(0, 1, "ERR: Open Dir %d", res);
		SDCard_Unmount();
		return;
	}

	while (1) {
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0)
			break;
		if (strstr(fno.fname, ".bmp") || strstr(fno.fname, ".BMP")) {
			bmp_count++;
		}
	}
	f_closedir(&dir);

	LCD7735_Printf(0, 1, "BMP Count: %d", bmp_count);
	if (bmp_count == 0) {
		LCD7735_Printf(0, 2, "ERR: No BMPs");
		SDCard_Unmount();
		return;
	}

	srand(HAL_GetTick());
	int target_idx = rand() % bmp_count;
	char target_filename[30] = "";

	res = f_opendir(&dir, address);
	if (res == FR_OK) {
		int current_idx = 0;
		while (1) {
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0)
				break;

			if (strstr(fno.fname, ".bmp") || strstr(fno.fname, ".BMP")) {
				if (current_idx == target_idx) {
					strcpy(target_filename, fno.fname);
					break;
				}
				current_idx++;
			}
		}
		f_closedir(&dir);
	}

	char full_path[64];
	sprintf(full_path, "%s/%s", address, target_filename);

	char short_name[15] = { 0 };
	strncpy(short_name, target_filename, 10);
	LCD7735_Printf(0, 2, "File: %s", short_name);

	FIL file;
	UINT bytesRead;
	uint8_t header[54];

	res = f_open(&file, full_path, FA_READ);
	if (res != FR_OK) {
		LCD7735_Printf(0, 3, "ERR: File Open %d", res);
		SDCard_Unmount();
		return;
	}

	f_read(&file, header, 54, &bytesRead);

	uint32_t dataOffset = header[10] | (header[11] << 8) | (header[12] << 16)
			| (header[13] << 24);
	int32_t width = header[18] | (header[19] << 8) | (header[20] << 16)
			| (header[21] << 24);
	int32_t height = header[22] | (header[23] << 8) | (header[24] << 16)
			| (header[25] << 24);
	uint16_t bitDepth = header[28] | (header[29] << 8);

	LCD7735_Printf(0, 3, "W:%d H:%d B:%d", (int) width, (int) height,
			(int) bitDepth);

	if (bitDepth != 24) {
		LCD7735_Printf(0, 4, "ERR: Not 24bit!");
		f_close(&file);
		SDCard_Unmount();
		return;
	}

	LCD7735_Printf(0, 4, "Drawing...");
	HAL_Delay(1000);
	LCD7735_Clear();

	f_lseek(&file, dataOffset);
	uint8_t rowBuffer[160 * 3];
	uint16_t lcdBuffer[160];
	int padding = (4 - ((width * 3) % 4)) % 4;

	for (int y = height - 1; y >= 0; y--) {
		f_read(&file, rowBuffer, (width * 3) + padding, &bytesRead);
		for (int x = 0; x < width; x++) {
			uint8_t b = rowBuffer[x * 3];
			uint8_t g = rowBuffer[x * 3 + 1];
			uint8_t r = rowBuffer[x * 3 + 2];

			uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
			lcdBuffer[x] = (color >> 8) | (color << 8);
		}
		if (y < 160 && width <= 160) {
			ST7735_FillRGBRect(&st7735_pObj, 0, y, (uint8_t*) lcdBuffer, width,
					1);
		}
	}

	f_close(&file);
	SDCard_Unmount();
}
