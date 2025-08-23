/*
 * drive.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_DRIVE_H_
#define MAIN_INC_DRIVE_H_

#define DRIVE_TIM	&hlptim5
#define DRIVE_PSC	0

#include "main.h"
#include "sensor.h"
#include "motor.h"
#include "mcf8316c.h"
#include "math.h"

void Drive_LPTIM5_IRQ(void);

void Drive_First(void);
void Drive_Second(void);
void Drive_Third(void);
void Drive_Forth(void);


#endif /* MAIN_INC_DRIVE_H_ */
