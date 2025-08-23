/*
 * sensor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_SENSOR_H_
#define MAIN_INC_SENSOR_H_

#include "main.h"
#include "init.h"
#include "adc.h"
#include "motor.h"
#include "lcd.h"
#include "tim.h"
#include "lptim.h"
#include "math.h"
#include "switch.h"

#define SENSOR_NUM 16

#define ADC_SENSOR_CHANNEL	(&hadc1)
#define ADC_BATTERY_CHANNEL	(&hadc2)
#define ADC_MARKER_CHANNEL	(&hadc3)

#define ADC_SENSOR_TIM		(&hlptim3)
#define ADC_BATTERY_TIM		(&hlptim5)

#define VARIANCE			0.4f
#define PDF_COEFF			(1.f / VARIANCE / M_SQRTPI / M_SQRT2)
#define PDF_EXP(x)			(-powf(x,2.f) / (2.f * powf(VARIANCE, 2.f)))

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
	uint16_t state;
	float_t position;
	float_t windowPos;
	uint8_t threshold;
	float_t voltage;
} sensor_t;

extern sensor_t sensor;

void Sensor_Start();
void Sensor_Stop();

void Sensor_Test_Voltage();


#endif /* MAIN_INC_SENSOR_H_ */
