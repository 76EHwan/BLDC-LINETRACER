/*
 * drive.h
 *
 *  Created on: 2026. 7. 2.
 *      Author: kth59
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "tim.h"

#define RAMP_TIM	(&htim14)
#define Ramp_TIM_IRQ_Handler TIM14_IRQ_Handler

typedef struct {
	float mps;
} DriveParam_t;

void Line_Follow_Drive(void);

#endif /* INC_DRIVE_H_ */
