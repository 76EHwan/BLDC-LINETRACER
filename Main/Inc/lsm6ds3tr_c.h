/*
 * lsm6ds3tr_c.h
 *
 *  Created on: Jul 15, 2025
 *      Author: kth59
 */

#ifndef INC_LSM6DS3TR_C_H_
#define INC_LSM6DS3TR_C_H_

#define LSM6DS3TR_C_WHO_AM_I_REG    0x0F

#include "spi.h"
#include "lcd.h"
#include "math.h"
#include <stdint.h>

void LSM6DS3TR_C_Init();
void LSM6DS3TR_C_Routine();

#endif /* INC_LSM6DS3TR_C_H_ */
