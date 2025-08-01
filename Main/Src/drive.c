/*
 * drive.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#define dt	0.0005f

#include "drive.h"

float accel;
float pit_accel;
float t_v;	// target velocity
float c_v;	// current velocity

void Drive_LPTIM5_IRQ() {
	c_v = (fabsf(t_v - c_v) < accel * dt) ?
			t_v : ((t_v - c_v > 0) ? c_v + accel * dt : c_v - accel * dt);

}

void Drive_First() {

}

void Drive_Second() {

}

void Drive_Third() {

}

void Drive_Forth() {

}
