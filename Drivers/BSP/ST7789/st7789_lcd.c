#include "st7789.h"
#include "st7789_lcd.h"
#include "spi.h"
#include "tim.h"
#include "rng.h"
#include "fatfs.h"
#include "font.h"
#include "SDcard.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TFT135x240

#define LCD_RST_SET
#define LCD_RST_RESET
#define LCD_RS_SET      HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port, LCD_WR_RS_Pin, GPIO_PIN_SET)
#define LCD_RS_RESET    HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port, LCD_WR_RS_Pin, GPIO_PIN_RESET)
#define LCD_CS_SET      HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_CS_RESET    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)

#define SPI spi4
#define SPI_Drv (&hspi4)
#define delay_ms HAL_Delay
#define get_tick HAL_GetTick

#define LCD_Brightness_timer &htim1
#define LCD_Brightness_channel TIM_CHANNEL_2

static int32_t lcd7789_init(void);
static int32_t lcd7789_gettick(void);
static int32_t lcd7789_writereg(uint8_t reg, uint8_t *pdata, uint32_t length);
static int32_t lcd7789_readreg(uint8_t reg, uint8_t *pdata);
static int32_t lcd7789_senddata(uint8_t *pdata, uint32_t length);
static int32_t lcd7789_recvdata(uint8_t *pdata, uint32_t length);

uint16_t LCD7789_BACK_BRIGHT = 600;

ST7789_IO_t st7789_pIO = { lcd7789_init, 0, 0, lcd7789_writereg,
		lcd7789_readreg, lcd7789_senddata, lcd7789_recvdata, lcd7789_gettick };

ST7789_Object_t st7789_pObj;

/* SPI4 BDMA 전용 D3 메모리 버퍼 (캐시 불가 영역) */
#define SPI_DMA_BUF_SIZE 1024
__attribute__((section(".ram_d3"), aligned(32))) uint8_t spi4_tx_buf[SPI_DMA_BUF_SIZE];
__attribute__((section(".ram_d3"), aligned(32))) uint8_t spi4_rx_buf[SPI_DMA_BUF_SIZE];

/* SD 카드 파일 읽기 전용 버퍼 (D2 캐시 불가 영역에 강제 할당) */
__attribute__((section(".ram_d2_nocache"), aligned(32))) uint8_t sd_row_buffer[240 * 3];
__attribute__((section(".ram_d2_nocache"), aligned(32))) uint16_t sd_lcd_buffer[240];

void LCD7789_Test(void) {

#if defined(TFT135x240)
	ST7789Ctx.Orientation = ST7789_ORIENTATION_PORTRAIT;
	ST7789Ctx.Type = ST7789_135x240_screen;
#elif defined(TFT240x240)
	ST7789Ctx.Orientation = ST7789_ORIENTATION_PORTRAIT;
	ST7789Ctx.Type = ST7789_240x240_screen;
#else
	#error "Unknown Screen"
#endif

	ST7789_RegisterBusIO(&st7789_pObj, &st7789_pIO);
	ST7789_LCD_Driver.Init(&st7789_pObj, ST7789_FORMAT_RBG565, &ST7789Ctx);
	ST7789_SetBrightness(&st7789_pObj, 0);

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0, 0, ST7789Ctx.Width,
			ST7789Ctx.Height, BLACK);

	FRESULT res = SDCard_Mount();
	if (res != FR_OK) {
		LCD7789_Printf(0, 0, "Mount Fail: %d", res);
		while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET)
			;
	} else {
		LCD7789_Printf(0, 0, "No Picture");
		LCD7789_Display_Random_BMP_From_SD("/display");

		uint32_t tick = get_tick();
		while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
			delay_ms(10);

			uint32_t elapsed = get_tick() - tick;

			if (elapsed <= 1000) {
				LCD7789_SetBrightness(elapsed * LCD7789_BACK_BRIGHT / 1000);
			} else if (elapsed <= 3000) {
				LCD7789_SetBrightness(LCD7789_BACK_BRIGHT);
				ST7789_LCD_Driver.FillRect(&st7789_pObj, 0,
						ST7789Ctx.Height - 5,
						(elapsed - 1000) * ST7789Ctx.Width / 2000, 5, 0xFFFF);
			} else if (elapsed > 3000) {
				break;
			}
		}
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
		delay_ms(10);
	}

	LCD7789_Light(0, 300);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0, 0, ST7789Ctx.Width,
			ST7789Ctx.Height, BLACK);
	LCD7789_Light(LCD7789_BACK_BRIGHT, 300);
}

static uint32_t LCD7789_LightSet;
static uint8_t IsLCD7789_SoftPWM = 0;

void LCD7789_SetBrightness(uint32_t Brightness) {
	LCD7789_LightSet = Brightness;
	if (!IsLCD7789_SoftPWM)
		__HAL_TIM_SetCompare(LCD_Brightness_timer, LCD_Brightness_channel, Brightness);
}

uint32_t LCD7789_GetBrightness(void) {
	if (IsLCD7789_SoftPWM)
		return LCD7789_LightSet;
	else
		return __HAL_TIM_GetCompare(LCD_Brightness_timer, LCD_Brightness_channel);
}

void LCD7789_SoftPWMEnable(uint8_t enable) {
	IsLCD7789_SoftPWM = enable;
	if (!enable)
		LCD7789_SetBrightness(LCD7789_LightSet);
}

uint8_t LCD7789_SoftPWMIsEnable(void) {
	return IsLCD7789_SoftPWM;
}

void LCD7789_SoftPWMCtrlInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	MX_TIM16_Init();
	HAL_TIM_Base_Start_IT(&htim16);
	LCD7789_SoftPWMEnable(1);
}

void LCD7789_SoftPWMCtrlDeInit(void) {
	HAL_TIM_Base_DeInit(&htim16);
	HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);
}

void LCD7789_SoftPWMCtrlRun(void) {
	static uint32_t timecount;
	if (timecount > 1000)
		timecount = 0;
	else
		timecount += 10;

	if (timecount >= LCD7789_LightSet)
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
}

void LCD7789_Light(uint32_t Brightness_Dis, uint32_t time) {
	uint32_t Brightness_Now;
	uint32_t time_now;
	float temp1, temp2;
	float k, set;

	Brightness_Now = LCD7789_GetBrightness();
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
		LCD7789_SetBrightness((uint32_t) set);
		if (time_now >= time)
			break;
	}
}

uint16_t LCD7789_POINT_COLOR = 0xFFFF;
uint16_t LCD7789_BACK_COLOR = BLACK;

void LCD7789_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size,
		uint8_t mode) {
	uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint16_t x0 = x;
	uint16_t colortemp = LCD7789_POINT_COLOR;
	uint32_t h, w;

	uint16_t write[size][size == 12 ? 6 : 8];
	uint16_t count;

	ST7789_GetXSize(&st7789_pObj, &w);
	ST7789_GetYSize(&st7789_pObj, &h);

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
					LCD7789_POINT_COLOR = (colortemp & 0xFF) << 8
							| colortemp >> 8;
				else
					LCD7789_POINT_COLOR = (LCD7789_BACK_COLOR & 0xFF) << 8
							| LCD7789_BACK_COLOR >> 8;

				write[count][t / 2] = LCD7789_POINT_COLOR;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y > h) {
					LCD7789_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						LCD7789_POINT_COLOR = colortemp;
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
					write[count][t / 2] = (LCD7789_POINT_COLOR & 0xFF) << 8
							| LCD7789_POINT_COLOR >> 8;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y > h) {
					LCD7789_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						LCD7789_POINT_COLOR = colortemp;
						return;
					}
					break;
				}
			}
		}
	}
	ST7789_FillRGBRect(&st7789_pObj, x0, y0, (uint8_t*) &write,
			size == 12 ? 6 : 8, size);
	LCD7789_POINT_COLOR = colortemp;
}

void LCD7789_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
		uint8_t size, uint8_t *p) {
	uint8_t x0 = x;
	width += x;
	height += y;
	while ((*p <= '~') && (*p >= ' ')) {
		if (x + size / 2 > width) {
			x = x0;
			y += size;
		}
		if (y > height)
			break;
		LCD7789_ShowChar(x, y, *p, size, 0);
		x += size / 2;
		p++;
	}
}

void LCD7789_Printf(uint16_t x, uint16_t y, const char *text, ...) {
	char txt[512] = { 0 };
	va_list args;
	va_start(args, text);
	vsprintf(txt, text, args);
	va_end(args);

// 고정 크기 지정 (ST7735 시절의 고정 렌더링 스타일. 필요시 12나 16으로 변경 가능)
	static uint8_t fixed_size = 16;
	static uint8_t x_bias;
	static uint8_t y_bias;
	static uint8_t offset_x = 0;
	static uint8_t offset_y = 0;

	switch (fixed_size) {
	case 12:
		x_bias = 6;
		y_bias = 12;
		offset_x = 1;
		offset_y = 0;
		break;
	default:
		x_bias = 8;
		y_bias = 16;
		offset_x = 3;
		offset_y = 0;
	}

	LCD7789_ShowString(x_bias * x + offset_x, y_bias * y + offset_y,
			ST7789Ctx.Width - x, ST7789Ctx.Height - y, fixed_size,
			(uint8_t*) txt);
}

static int32_t lcd7789_init(void) {
	HAL_TIMEx_PWMN_Start(LCD_Brightness_timer, LCD_Brightness_channel);
	return ST7789_OK;
}

static int32_t lcd7789_gettick(void) {
	return HAL_GetTick();
}

static int32_t lcd7789_writereg(uint8_t reg, uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;

	// 레지스터 주소는 1바이트이므로 폴링 전송
	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, 100);
	LCD_RS_SET;

	if (length > 0) {
		uint32_t remaining = length;
		uint8_t *ptr = pdata;

		while (remaining > 0) {
			uint16_t chunk =
					(remaining > SPI_DMA_BUF_SIZE) ?
					SPI_DMA_BUF_SIZE :
														(uint16_t) remaining;

			// 1. 데이터를 D3 버퍼로 복사 (이때 데이터는 D-Cache에 머물러 있을 수 있음)
			memcpy(spi4_tx_buf, ptr, chunk);

			// 2. [핵심 추가] 캐시의 내용을 물리 메모리(SRAM4)로 강제 밀어내기!
			// Cortex-M7의 캐시 라인은 32바이트 단위이므로, 크기를 32의 배수로 올림 처리합니다.
			SCB_CleanDCache_by_Addr((uint32_t*) spi4_tx_buf,
					(chunk + 31) & ~31);

			// 3. 물리 메모리에 데이터가 확실히 쓰였으므로 BDMA 전송 시작
			HAL_SPI_Transmit_DMA(SPI_Drv, spi4_tx_buf, chunk);

			while (HAL_SPI_GetState(SPI_Drv) != HAL_SPI_STATE_READY) {
			}

			ptr += chunk;
			remaining -= chunk;
		}
	}
	LCD_CS_SET;
	return result > 0 ? -1 : 0;
}

static int32_t lcd7789_readreg(uint8_t reg, uint8_t *pdata) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;
	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, 100);
	LCD_RS_SET;

	// 읽기는 보통 1바이트이므로 폴링 유지 (성능상 이득)
	result += HAL_SPI_Receive(SPI_Drv, pdata, 1, 100);

	LCD_CS_SET;
	return result > 0 ? -1 : 0;
}

static int32_t lcd7789_senddata(uint8_t *pdata, uint32_t length) {
	LCD_CS_RESET;
	uint32_t remaining = length;
	uint8_t *ptr = pdata;

	while (remaining > 0) {
		uint16_t chunk =
				(remaining > SPI_DMA_BUF_SIZE) ?
				SPI_DMA_BUF_SIZE :
													(uint16_t) remaining;

		// 1. 데이터를 D3 버퍼로 복사 (이때 데이터는 D-Cache에 머물러 있을 수 있음)
		memcpy(spi4_tx_buf, ptr, chunk);

		// 2. [핵심 추가] 캐시의 내용을 물리 메모리(SRAM4)로 강제 밀어내기!
		// Cortex-M7의 캐시 라인은 32바이트 단위이므로, 크기를 32의 배수로 올림 처리합니다.
		SCB_CleanDCache_by_Addr((uint32_t*) spi4_tx_buf, (chunk + 31) & ~31);

		// 3. 물리 메모리에 데이터가 확실히 쓰였으므로 BDMA 전송 시작
		HAL_SPI_Transmit_DMA(SPI_Drv, spi4_tx_buf, chunk);

		while (HAL_SPI_GetState(SPI_Drv) != HAL_SPI_STATE_READY) {
		}

		ptr += chunk;
		remaining -= chunk;
	}
	LCD_CS_SET;
	return 0;
}

static int32_t lcd7789_recvdata(uint8_t *pdata, uint32_t length) {
	LCD_CS_RESET;
	uint32_t remaining = length;
	uint8_t *ptr = pdata;

	while (remaining > 0) {
		uint16_t chunk =
				(remaining > SPI_DMA_BUF_SIZE) ?
				SPI_DMA_BUF_SIZE :
													(uint16_t) remaining;

		// 1. 데이터를 D3 버퍼로 복사 (이때 데이터는 D-Cache에 머물러 있을 수 있음)
		memcpy(spi4_tx_buf, ptr, chunk);

		// 2. [핵심 추가] 캐시의 내용을 물리 메모리(SRAM4)로 강제 밀어내기!
		// Cortex-M7의 캐시 라인은 32바이트 단위이므로, 크기를 32의 배수로 올림 처리합니다.
		SCB_CleanDCache_by_Addr((uint32_t*) spi4_tx_buf, (chunk + 31) & ~31);

		// 3. 물리 메모리에 데이터가 확실히 쓰였으므로 BDMA 전송 시작
		HAL_SPI_Transmit_DMA(SPI_Drv, spi4_tx_buf, chunk);

		while (HAL_SPI_GetState(SPI_Drv) != HAL_SPI_STATE_READY) {
		}

		ptr += chunk;
		remaining -= chunk;
	}
	LCD_CS_SET;
	return 0;
}

void LCD7789_Clear() {
	LCD7789_Light(0, 250);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0, 0, ST7789Ctx.Width,
			ST7789Ctx.Height, BLACK);
	LCD7789_Light(900, 250);
}

void LCD7789_Display_Random_BMP_From_SD(const TCHAR *address) {
	FRESULT res;
	int bmp_count = 0;
	uint16_t y_pos = 5;

	if (SDCard_Mount() != FR_OK) {
		LCD7789_Printf(0, y_pos, "ERR: SD Mount");
		return;
	}

	res = f_opendir(&dir, address);
	if (res != FR_OK) {
		LCD7789_Printf(0, y_pos, "ERR: Open Dir %d", res);
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

	if (bmp_count == 0) {
		LCD7789_Printf(0, y_pos, "ERR: No BMPs");
		SDCard_Unmount();
		return;
	}

	uint32_t random_val = 0;
	if (HAL_RNG_GenerateRandomNumber(&hrng, &random_val) != HAL_OK) {
		random_val = HAL_GetTick();
	}

	int target_idx = random_val % bmp_count;

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

	y_pos += 15;

	UINT bytesRead;
	uint8_t header[54];

	res = f_open(&file, full_path, FA_READ);
	if (res != FR_OK) {
		LCD7789_Printf(0, y_pos, "ERR: File Open %d", res);
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

	if (bitDepth != 24) {
		LCD7789_Printf(0, y_pos, "ERR: Not 24bit!");
		f_close(&file);
		SDCard_Unmount();
		return;
	}

	f_lseek(&file, dataOffset);

	int padding = (4 - ((width * 3) % 4)) % 4;

	for (int y = height - 1; y >= 0; y--) {
		f_read(&file, sd_row_buffer, (width * 3) + padding, &bytesRead);

		for (int x = 0; x < width; x++) {
			uint8_t b = sd_row_buffer[x * 3];
			uint8_t g = sd_row_buffer[x * 3 + 1];
			uint8_t r = sd_row_buffer[x * 3 + 2];

			uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
			sd_lcd_buffer[x] = (color >> 8) | (color << 8);
		}
		if (y < ST7789Ctx.Height && width <= ST7789Ctx.Width) {
			ST7789_LCD_Driver.FillRGBRect(&st7789_pObj, 0, y,
					(uint8_t*) sd_lcd_buffer, width, 1);
		}
	}

	f_close(&file);
	SDCard_Unmount();
}

void LCD7789_Set_Color(uint16_t point, uint16_t back) {
	LCD7789_POINT_COLOR = point;
	LCD7789_BACK_COLOR = back;
}
