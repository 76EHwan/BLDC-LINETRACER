#include "w25qxx.h"
#include "user_init.h"
#include "button.h"
#include <math.h>

#define delay(ms) HAL_Delay(ms-1)
#define get_tick() HAL_GetTick()

W25Qx_Parameter W25Qx_Para;
/**
 * @brief  Initializes the W25QXXXX interface.
 * @retval None
 */
uint8_t W25Qx_Init(void) {
	uint8_t state;
	/* Reset W25Qxxx */
	W25Qx_Reset();
	delay(5);
	state = W25Qx_Get_Parameter(&W25Qx_Para);

	return state;
}

/**
 * @brief  This function reset the W25Qx.
 * @retval None
 */
void W25Qx_Reset(void) {
	uint8_t cmd[2] = { RESET_ENABLE_CMD, RESET_MEMORY_CMD };

	W25Qx_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(&hspi1, cmd, 2, W25QXXXX_TIMEOUT_VALUE);
	W25Qx_Disable();

}

/**
 * @brief  Reads current status of the W25QXXXX.
 * @retval W25QXXXX memory status
 */
uint8_t W25Qx_GetStatus(void) {
	uint8_t tx[2] = { READ_STATUS_REG1_CMD, 0xFF };
	uint8_t rx[2] = { 0 };

	W25Qx_Enable();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, W25QXXXX_TIMEOUT_VALUE);
	W25Qx_Disable();

	return ((rx[1] & W25QXXXX_FSR_BUSY) != 0) ? W25Qx_BUSY : W25Qx_OK;
}

/**
 * @brief  This function send a Write Enable and wait it is effective.
 * @retval None
 */
uint8_t W25Qx_WriteEnable(void) {
	uint8_t cmd[] = { WRITE_ENABLE_CMD };
	uint32_t tickstart = get_tick();

	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 1, W25QXXXX_TIMEOUT_VALUE);
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();

	/* Wait the end of Flash writing */
	while (W25Qx_GetStatus() == W25Qx_BUSY) {
		/* Check for the Timeout */
		if ((get_tick() - tickstart) > W25QXXXX_TIMEOUT_VALUE) {
			return W25Qx_TIMEOUT;
		}
		delay(1);
	}

	return W25Qx_OK;
}

/**
 * @brief  Read Manufacture/Device ID.
 * @param  return value address
 /   ����ֵ����:
 /   0XEF13,��ʾоƬ�ͺ�ΪW25Q80
 /   0XEF14,��ʾоƬ�ͺ�ΪW25Q16
 /   0XEF15,��ʾоƬ�ͺ�ΪW25Q32
 /   0XEF16,��ʾоƬ�ͺ�ΪW25Q64
 * @retval None
 */
void W25Qx_Read_ID(uint16_t *ID) {
	uint8_t tx[6] = { READ_ID_CMD, 0x00, 0x00, 0x00, 0xFF, 0xFF };
	uint8_t rx[6] = { 0 };

	W25Qx_Enable();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 6, W25QXXXX_TIMEOUT_VALUE);
	W25Qx_Disable();

	*ID = (rx[4] << 8) | rx[5];
}

uint8_t W25Qx_Get_Parameter(W25Qx_Parameter *Para) {
	uint16_t id;
	uint8_t device_id;
	uint32_t size;

	Para->PAGE_SIZE = 256;
	Para->W25Qx_SUBSECTOR_SIZE = 4096;
	Para->W25Qx_SECTOR_SIZE = 0x10000;

	W25Qx_Read_ID(&id);
	device_id = id & 0xff;
	if (device_id < x25Q80 || device_id > x25Q128)
		return W25Qx_ERROR;

	size = (uint32_t) powf(2, (device_id - x25Q80)) * 1024 * 1024;

	Para->W25Qx_Id = id;
	Para->W25Qx_Size = size;
	Para->W25Qx_SUBSECTOR_COUNT = Para->W25Qx_Size / Para->W25Qx_SUBSECTOR_SIZE;
	Para->W25Qx_SECTOR_COUNT = Para->W25Qx_Size / Para->W25Qx_SECTOR_SIZE;

	return W25Qx_OK;
}
/**
 * @brief  Reads an amount of data from the QSPI memory.
 * @param  pData: Pointer to data to be read
 * @param  ReadAddr: Read start address
 * @param  Size: Size of data to read
 * @retval QSPI memory status
 */
uint8_t W25Qx_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size) {
	uint8_t cmd_tx[4] = {
	READ_CMD, (uint8_t) (ReadAddr >> 16), (uint8_t) (ReadAddr >> 8),
			(uint8_t) (ReadAddr) };
	uint8_t cmd_rx[4];

	W25Qx_Enable();
	// 커맨드 4바이트: TX/RX 동시 (H7는 이렇게 해야 안전)
	HAL_SPI_TransmitReceive(&hspi1, cmd_tx, cmd_rx, 4, W25QXXXX_TIMEOUT_VALUE);

	// 데이터 수신: dummy TX 버퍼 필요
	uint8_t dummy[Size];
	memset(dummy, 0xFF, Size);
	HAL_SPI_TransmitReceive(&hspi1, dummy, pData, Size, W25QXXXX_TIMEOUT_VALUE);
	W25Qx_Disable();

	return W25Qx_OK;
}

/**
 * @brief  Writes an amount of data to the QSPI memory.
 * @param  pData: Pointer to data to be written
 * @param  WriteAddr: Write start address
 * @param  Size: Size of data to write,No more than 256byte.
 * @retval QSPI memory status
 */
uint8_t W25Qx_Write(uint8_t *pData, uint32_t WriteAddr, uint32_t Size) {
	uint8_t cmd[4];
	uint32_t end_addr, current_size, current_addr;
	uint32_t tickstart = get_tick();

	/* Calculation of the size between the write address and the end of the page */
	current_addr = 0;

	while (current_addr <= WriteAddr) {
		current_addr += W25QXXXX_PAGE_SIZE;
	}
	current_size = current_addr - WriteAddr;

	/* Check if the size of the data is less than the remaining place in the page */
	if (current_size > Size) {
		current_size = Size;
	}

	/* Initialize the adress variables */
	current_addr = WriteAddr;
	end_addr = WriteAddr + Size;

	/* Perform the write page by page */
	do {
		/* Configure the command */
		cmd[0] = PAGE_PROG_CMD;
		cmd[1] = (uint8_t) (current_addr >> 16);
		cmd[2] = (uint8_t) (current_addr >> 8);
		cmd[3] = (uint8_t) (current_addr);

		/* Enable write operations */
		W25Qx_WriteEnable();

		W25Qx_Enable();
		/* Send the command */
		if (HAL_SPI_Transmit(&hspi1, cmd, 4, W25QXXXX_TIMEOUT_VALUE)
				!= HAL_OK) {
			return W25Qx_ERROR;
		}

		/* Transmission of the data */
		if (HAL_SPI_Transmit(&hspi1, pData, current_size,
				W25QXXXX_TIMEOUT_VALUE) != HAL_OK) {
			return W25Qx_ERROR;
		}
		W25Qx_Disable();
		/* Wait the end of Flash writing */
		while (W25Qx_GetStatus() == W25Qx_BUSY) {
			/* Check for the Timeout */
			if ((get_tick() - tickstart) > W25QXXXX_TIMEOUT_VALUE) {
				return W25Qx_TIMEOUT;
			}
			//delay(1);
		}

		/* Update the address and size variables for next page programming */
		current_addr += current_size;
		pData += current_size;
		current_size =
				((current_addr + W25QXXXX_PAGE_SIZE) > end_addr) ?
						(end_addr - current_addr) : W25QXXXX_PAGE_SIZE;
	} while (current_addr < end_addr);

	return W25Qx_OK;
}

/**
 * @brief  Erases the specified block of the QSPI memory.
 * @param  BlockAddress: Block address to erase
 * @retval QSPI memory status
 */
uint8_t W25Qx_Erase_Block(uint32_t Address) {
	uint8_t cmd[4];
	uint32_t tickstart = get_tick();
	cmd[0] = SECTOR_ERASE_CMD;
	cmd[1] = (uint8_t) (Address >> 16);
	cmd[2] = (uint8_t) (Address >> 8);
	cmd[3] = (uint8_t) (Address);

	/* Enable write operations */
	W25Qx_WriteEnable();

	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 4, W25QXXXX_TIMEOUT_VALUE);
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();

	/* Wait the end of Flash writing */
	while (W25Qx_GetStatus() == W25Qx_BUSY) {
		/* Check for the Timeout */
		if ((get_tick() - tickstart) > W25QXXXX_SECTOR_ERASE_MAX_TIME) {
			return W25Qx_TIMEOUT;
		}
		//delay(1);
	}
	return W25Qx_OK;
}

/**
 * @brief  Erases the entire QSPI memory.This function will take a very long time.
 * @retval QSPI memory status
 */
uint8_t W25Qx_Erase_Chip(void) {
	uint8_t cmd[4];
	uint32_t tickstart = get_tick();
	cmd[0] = CHIP_ERASE_CMD;

	/* Enable write operations */
	W25Qx_WriteEnable();

	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 1, W25QXXXX_TIMEOUT_VALUE);
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();

	/* Wait the end of Flash writing */
	while (W25Qx_GetStatus() == W25Qx_BUSY) {
		/* Check for the Timeout */
		if ((get_tick() - tickstart) > W25QXXXX_BULK_ERASE_MAX_TIME) {
			return W25Qx_TIMEOUT;
		}
		//delay(1);
	}
	return W25Qx_OK;
}

#define TEST_ADDRESS 0x000000

void W25QXX_Test(void) {
	W25Qx_Init();
	uint16_t id;
	W25Qx_Read_ID(&id);
	LCD_Printf(0, 0, "%04X", id);
	char build_time_str[32];
	char read_buffer[32];
	sprintf(build_time_str, "%s %s", __DATE__, __TIME__);
	uint16_t data_length = strlen(build_time_str) + 1;
	W25Qx_Erase_Block(0);
	W25Qx_Write((uint8_t*) build_time_str, TEST_ADDRESS, data_length);
	memset(read_buffer, 0, sizeof(read_buffer));
	W25Qx_Read((uint8_t*) read_buffer, TEST_ADDRESS, data_length);
	LCD_Printf(0, 1, "Last Update:");
	LCD_Printf(0, 2, "%s", read_buffer);
	while(Button_Get_Input() != INPUT_CMD_K_DOUBLE);
	LCD_Clear();
}
