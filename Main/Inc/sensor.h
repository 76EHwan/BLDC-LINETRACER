/*
 * sensor.h
 *
 *  Created on: 2026. 5. 2.
 *      Author: kth59
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "adc.h"
#include "tim.h"

#define NUM_SENSORS 18
#define LEFT_MARK_SENSOR_INDEX 16
#define RIGHT_MARK_SENSOR_INDEX 17


typedef struct {
	volatile uint8_t idx;
	uint16_t raw[NUM_SENSORS];
	uint16_t whitemax[NUM_SENSORS];
	uint16_t blackmax[NUM_SENSORS];
	uint16_t normalized_coef_bias[NUM_SENSORS];
	uint16_t normalized[NUM_SENSORS];
	uint32_t state;
	float_t line_position;
	uint16_t threshold;
} SensorDataTypeDef;

typedef struct {
	uint32_t is_calibration;
	volatile SensorDataTypeDef data;
} Sensor_TypeDef;

extern volatile Sensor_TypeDef IR_Sensor;


void Sensor_Calibration();
void Sensor_Raw_Printf();

void TIM7_IRQ_Handler(void);
void ADC3_IRQ_Handler(void);

void  Sensor_Line_LUT_Init(void);
float Sensor_Line_Estimate(void);

void IMU_Test(void);


#endif /* INC_SENSOR_H_ */
