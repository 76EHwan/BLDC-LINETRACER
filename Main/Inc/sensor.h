#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "adc.h"
#include "tim.h"

#define NUM_SENSORS 18
#define LEFT_MARK_SENSOR_INDEX 16
#define RIGHT_MARK_SENSOR_INDEX 17

#define POS_WINDOW_HALF   	3
#define POS_WINDOW_SIZE     (POS_WINDOW_HALF * 2)

#define LINE_N_SENSORS      16
#define SCAN_GROUP_LEN 		10
#define SCAN_SLOT_MARK_L    8
#define SCAN_SLOT_MARK_R    9
#define SCAN_CYCLE_LEN      20
#define SCAN_CYCLE_LEN_HALF	(SCAN_CYCLE_LEN / 2)

#define CROSS_LOG_BUFFER_SIZE	8192
#define CROSS_LOG_MAX 			256

typedef enum {
	CROSS_NONE = 0,
	CROSS_LEFT,
	CROSS_RIGHT,
	CROSS_CROSS,
	CROSS_STOP,
} CrossEvent_t;

typedef struct {
	volatile uint8_t idx;
	uint16_t raw[NUM_SENSORS];
	uint16_t whitemax[NUM_SENSORS];
	uint16_t blackmax[NUM_SENSORS];
	uint16_t normalized_coef_bias[NUM_SENSORS];
	uint16_t normalized[NUM_SENSORS];
	uint32_t state;
	uint16_t threshold;
	uint8_t line_lost_sum_min;

	uint8_t mark_left;
	uint8_t mark_right;
} SensorData_TypeDef;

typedef struct {
	uint8_t scan_group;
	uint8_t is_calibration;
	uint8_t is_lost_position;
	uint8_t is_position;
	volatile SensorData_TypeDef *data;
} Sensor_TypeDef;

typedef struct {
	CrossEvent_t type;
	float dist_from_prev_m;
} CrossMarkerLog_t;

extern volatile SensorData_TypeDef sensorData;
extern volatile Sensor_TypeDef IR_Sensor;
extern uint16_t adc3_buffer[1];
extern volatile uint32_t count_sensor_irq;
extern const float line_sensor_pos[LINE_N_SENSORS];

// sensor.c에서 관리되는 마커 기록용 로그 배열과 카운터
extern CrossMarkerLog_t g_cross_log[CROSS_LOG_MAX];
extern uint16_t g_cross_log_count;

extern volatile uint16_t buzzer_timer_count;
extern float_t g_buzzer_duration;

void Sensor_Start();
void Sensor_Stop();

void Sensor_Printf(uint8_t idx, volatile uint16_t *sensor_data);

float Sensor_Get_Position(void);

void Cross_Detect_Reset(void);
CrossEvent_t Cross_Detect_Update(void);
void Cross_Log_Push(CrossEvent_t type);

void Sensor_Calibration();
void Sensor_Raw_Printf();
void Sensor_Normalize_Printf();
void Sensor_State_Printf();
void Sensor_Position_Printf();

void TIM7_IRQ_Handler(void);
void ADC3_IRQ_Half_Handler(void);
void ADC3_IRQ_Cplt_Handler(void);

void  Sensor_Line_LUT_Init(void);
float Sensor_Line_Estimate_Pass1(void);
float Sensor_Line_Estimate_Pass2(void);

void IMU_Test(void);

#endif /* INC_SENSOR_H_ */
