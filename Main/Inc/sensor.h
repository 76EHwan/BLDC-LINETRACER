/*
 * sensor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */
#include "main.h"

#ifndef MAIN_INC_SENSOR_H_
#define MAIN_INC_SENSOR_H_

#define SENSOR_NUM 14

void Sensor_LPTIM3_IRQ(void);
void Marker_LPTIM3_IRQ(void);

void Sensor_Test_Raw(void);
void Sensor_Test_Menu(void);
void Sensor_Calibration(void);
void Sensor_Test_Normalized(void);
void Sensor_Test_State(void);
void Sensor_Test_Position(void);

void ADC_Battery_LPTIM5_IRQ(void);

typedef struct {
	int32_t raw[SENSOR_NUM];
	int32_t whiteMax[SENSOR_NUM];
	int32_t blackMax[SENSOR_NUM];
	int32_t normalizeCoef[SENSOR_NUM];
	int32_t normalized[SENSOR_NUM];
	int16_t state;
	float_t position;
	float_t windowPos;
	uint8_t threshold;
	float_t voltage;
} sensor_t;

extern sensor_t sensor;

void Sensor_Test_Voltage();


#endif /* MAIN_INC_SENSOR_H_ */
