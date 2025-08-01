/*
 * sdcard.c
 *
 *  Created on: Jul 18, 2025
 *      Author: kth59
 */

#include "sdcard.h"
#include "lcd.h"
#include <stdio.h>
#include "string.h"

FATFS fs_;
FIL fil_;
FRESULT fresult;
char buffer_[BUFF_SIZE];
UINT br_, bw_;

