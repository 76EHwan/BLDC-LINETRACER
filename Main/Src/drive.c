/*
 * drive.c
 *
 *  Created on: 2026. 7. 2.
 *      Author: kth59
 */

#include "main.h"

#include "foc.h"

#include "sensor.h"
#include "motor.h"
#include "drive.h"

DriveParam_t driveData[2];

__STATIC_INLINE void Ramp_Omega(FOC_Handle_t *hfoc) {
	float d = hfoc->omega_setpoint - hfoc->target_omega;
	float step = hfoc->omega_ramp_rate * SPD_DT;
	if (d > step)
		hfoc->target_omega += step;
	else if (d < -step)
		hfoc->target_omega -= step;
	else
		hfoc->target_omega = hfoc->omega_setpoint;
}

void Ramp_TIM_IRQ_Handler() {


	MTR_Set_Speed((driveData + 0)->mps, (driveData + 1)->mps);
	Ramp_Omega(&foc_L);
	Ramp_Omega(&foc_R);

}

void Ramp_Start() {
	foc_L.omega_setpoint = 0.f;
	foc_R.omega_setpoint = 0.f;

	HAL_TIM_Base_Start_IT(RAMP_TIM);
}

void Ramp_Stop() {
	foc_L.omega_setpoint = 0.f;
	foc_R.omega_setpoint = 0.f;

	HAL_TIM_Base_Stop_IT(RAMP_TIM);
}

