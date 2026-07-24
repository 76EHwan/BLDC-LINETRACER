/*
 * sd_ui.c
 *
 *  Created on: 2026. 7. 24.
 *      Author: kth59
 */

#include <stdio.h>

#include "button.h"
#include "SDcard.h"

#include "drive.h"
#include "foc.h"
#include "sd_ui.h"
#include "sensor.h"
#include "user_init.h"

// =========================================================
// [SD카드 저장 및 불러오기]
// =========================================================
// @formatter:off
static const SDCard_ConfigEntry sensor_calib_table[] = {
        { "whitemax_00",    (void*)&sensorData.whitemax[0],                 SDCFG_UINT16 },
        { "whitemax_01",    (void*)&sensorData.whitemax[1],                 SDCFG_UINT16 },
        { "whitemax_02",    (void*)&sensorData.whitemax[2],                 SDCFG_UINT16 },
        { "whitemax_03",    (void*)&sensorData.whitemax[3],                 SDCFG_UINT16 },
        { "whitemax_04",    (void*)&sensorData.whitemax[4],                 SDCFG_UINT16 },
        { "whitemax_05",    (void*)&sensorData.whitemax[5],                 SDCFG_UINT16 },
        { "whitemax_06",    (void*)&sensorData.whitemax[6],                 SDCFG_UINT16 },
        { "whitemax_07",    (void*)&sensorData.whitemax[7],                 SDCFG_UINT16 },
        { "whitemax_08",    (void*)&sensorData.whitemax[8],                 SDCFG_UINT16 },
        { "whitemax_09",    (void*)&sensorData.whitemax[9],                 SDCFG_UINT16 },
        { "whitemax_10",    (void*)&sensorData.whitemax[10],                SDCFG_UINT16 },
        { "whitemax_11",    (void*)&sensorData.whitemax[11],                SDCFG_UINT16 },
        { "whitemax_12",    (void*)&sensorData.whitemax[12],                SDCFG_UINT16 },
        { "whitemax_13",    (void*)&sensorData.whitemax[13],                SDCFG_UINT16 },
        { "whitemax_14",    (void*)&sensorData.whitemax[14],                SDCFG_UINT16 },
        { "whitemax_15",    (void*)&sensorData.whitemax[15],                SDCFG_UINT16 },
        { "whitemax_16",    (void*)&sensorData.whitemax[16],                SDCFG_UINT16 },
        { "whitemax_17",    (void*)&sensorData.whitemax[17],                SDCFG_UINT16 },
        { "blackmax_00",    (void*)&sensorData.blackmax[0],                 SDCFG_UINT16 },
        { "blackmax_01",    (void*)&sensorData.blackmax[1],                 SDCFG_UINT16 },
        { "blackmax_02",    (void*)&sensorData.blackmax[2],                 SDCFG_UINT16 },
        { "blackmax_03",    (void*)&sensorData.blackmax[3],                 SDCFG_UINT16 },
        { "blackmax_04",    (void*)&sensorData.blackmax[4],                 SDCFG_UINT16 },
        { "blackmax_05",    (void*)&sensorData.blackmax[5],                 SDCFG_UINT16 },
        { "blackmax_06",    (void*)&sensorData.blackmax[6],                 SDCFG_UINT16 },
        { "blackmax_07",    (void*)&sensorData.blackmax[7],                 SDCFG_UINT16 },
        { "blackmax_08",    (void*)&sensorData.blackmax[8],                 SDCFG_UINT16 },
        { "blackmax_09",    (void*)&sensorData.blackmax[9],                 SDCFG_UINT16 },
        { "blackmax_10",    (void*)&sensorData.blackmax[10],                SDCFG_UINT16 },
        { "blackmax_11",    (void*)&sensorData.blackmax[11],                SDCFG_UINT16 },
        { "blackmax_12",    (void*)&sensorData.blackmax[12],                SDCFG_UINT16 },
        { "blackmax_13",    (void*)&sensorData.blackmax[13],                SDCFG_UINT16 },
        { "blackmax_14",    (void*)&sensorData.blackmax[14],                SDCFG_UINT16 },
        { "blackmax_15",    (void*)&sensorData.blackmax[15],                SDCFG_UINT16 },
        { "blackmax_16",    (void*)&sensorData.blackmax[16],                SDCFG_UINT16 },
        { "blackmax_17",    (void*)&sensorData.blackmax[17],                SDCFG_UINT16 },
        { "coef_bias_00",   (void*)&sensorData.normalized_coef_bias[0],     SDCFG_UINT16 },
        { "coef_bias_01",   (void*)&sensorData.normalized_coef_bias[1],     SDCFG_UINT16 },
        { "coef_bias_02",   (void*)&sensorData.normalized_coef_bias[2],     SDCFG_UINT16 },
        { "coef_bias_03",   (void*)&sensorData.normalized_coef_bias[3],     SDCFG_UINT16 },
        { "coef_bias_04",   (void*)&sensorData.normalized_coef_bias[4],     SDCFG_UINT16 },
        { "coef_bias_05",   (void*)&sensorData.normalized_coef_bias[5],     SDCFG_UINT16 },
        { "coef_bias_06",   (void*)&sensorData.normalized_coef_bias[6],     SDCFG_UINT16 },
        { "coef_bias_07",   (void*)&sensorData.normalized_coef_bias[7],     SDCFG_UINT16 },
        { "coef_bias_08",   (void*)&sensorData.normalized_coef_bias[8],     SDCFG_UINT16 },
        { "coef_bias_09",   (void*)&sensorData.normalized_coef_bias[9],     SDCFG_UINT16 },
        { "coef_bias_10",   (void*)&sensorData.normalized_coef_bias[10],    SDCFG_UINT16 },
        { "coef_bias_11",   (void*)&sensorData.normalized_coef_bias[11],    SDCFG_UINT16 },
        { "coef_bias_12",   (void*)&sensorData.normalized_coef_bias[12],    SDCFG_UINT16 },
        { "coef_bias_13",   (void*)&sensorData.normalized_coef_bias[13],    SDCFG_UINT16 },
        { "coef_bias_14",   (void*)&sensorData.normalized_coef_bias[14],    SDCFG_UINT16 },
        { "coef_bias_15",   (void*)&sensorData.normalized_coef_bias[15],    SDCFG_UINT16 },
        { "coef_bias_16",   (void*)&sensorData.normalized_coef_bias[16],    SDCFG_UINT16 },
        { "coef_bias_17",   (void*)&sensorData.normalized_coef_bias[17],    SDCFG_UINT16 },
};
// @formatter:on

#define SENSOR_CALIB_COUNT	(sizeof(sensor_calib_table) / sizeof(sensor_calib_table[0]))

FRESULT Sensor_Save_Calibration(void) {
	return SDCard_SaveConfig(CALIBRATION_PATH, sensor_calib_table, SENSOR_CALIB_COUNT);
}

FRESULT Sensor_Load_Calibration(void) {
	FRESULT res = SDCard_LoadConfig(CALIBRATION_PATH, sensor_calib_table, SENSOR_CALIB_COUNT);
	if (res == FR_OK) IR_Sensor.is_calibration = 1;
	return res;
}


// =========================================================
// FOC 파라미터 SD 카드 저장/로드
// =========================================================

#define FOC_PARAM_COUNT	(sizeof(foc_param_table) / sizeof(foc_param_table[0]))

// @formatter:off
static const SDCard_ConfigEntry foc_param_table[] = {
		{ "L_offset_a",		&foc_L.offset_a, 		SDCFG_FLOAT },
		{ "L_offset_c", 	&foc_L.offset_c, 		SDCFG_FLOAT },
		{ "L_theta_offset", &foc_L.theta_offset, 	SDCFG_FLOAT },
		{ "L_id_Kp", 		&foc_L.pid_id.Kp, 		SDCFG_FLOAT },
		{ "L_id_Ki", 		&foc_L.pid_id.Ki, 		SDCFG_FLOAT },
		{ "L_iq_Kp", 		&foc_L.pid_iq.Kp, 		SDCFG_FLOAT },
		{ "L_iq_Ki", 		&foc_L.pid_iq.Ki, 		SDCFG_FLOAT },
		{ "L_spd_Kp",		&foc_L.spd_Kp, 			SDCFG_FLOAT },
		{ "L_spd_Ki",		&foc_L.spd_Ki, 			SDCFG_FLOAT },
		{ "L_spd_Kd", 		&foc_L.spd_Kd, 			SDCFG_FLOAT },
		{ "L_iq_limit", 	&foc_L.iq_limit, 		SDCFG_FLOAT },
		{ "L_enc_dir", 		&foc_L.enc_dir, 		SDCFG_INT8 },

		{ "R_offset_a",		&foc_R.offset_a, 		SDCFG_FLOAT },
		{ "R_offset_c", 	&foc_R.offset_c, 		SDCFG_FLOAT },
		{ "R_theta_offset", &foc_R.theta_offset, 	SDCFG_FLOAT },
		{ "R_id_Kp", 		&foc_R.pid_id.Kp, 		SDCFG_FLOAT },
		{ "R_id_Ki", 		&foc_R.pid_id.Ki, 		SDCFG_FLOAT },
		{ "R_iq_Kp", 		&foc_R.pid_iq.Kp, 		SDCFG_FLOAT },
		{ "R_iq_Ki", 		&foc_R.pid_iq.Ki, 		SDCFG_FLOAT },
		{ "R_spd_Kp",		&foc_R.spd_Kp, 			SDCFG_FLOAT },
		{ "R_spd_Ki",		&foc_R.spd_Ki, 			SDCFG_FLOAT },
		{ "R_spd_Kd", 		&foc_R.spd_Kd, 			SDCFG_FLOAT },
		{ "R_iq_limit", 	&foc_R.iq_limit, 		SDCFG_FLOAT },
		{ "R_enc_dir", 		&foc_R.enc_dir, 		SDCFG_INT8 },
};

// @formatter:on

FRESULT Save_FOC_Parameters(void) {
	return SDCard_SaveConfig(FOC_PARAM_PATH, foc_param_table, FOC_PARAM_COUNT);
}

FRESULT Load_FOC_Parameters(void) {
	FRESULT res = SDCard_LoadConfig(FOC_PARAM_PATH, foc_param_table,
	FOC_PARAM_COUNT);
	if (res != FR_OK)
		return res;

	arm_pid_init_f32(&foc_L.pid_id, 1);
	arm_pid_init_f32(&foc_L.pid_iq, 1);
	arm_pid_init_f32(&foc_R.pid_id, 1);
	arm_pid_init_f32(&foc_R.pid_iq, 1);

	return FR_OK;
}

// ============================================================================
// SD 카드에 마커 및 주행 설정 기록 저장 (세이브 슬롯 기능 적용)
// ============================================================================

uint8_t Select_Save_Slot(void) {
	static uint8_t slot = 1;
	UserInput_t btn = INPUT_CMD_NONE;
	LCD_Printf(0, 4, "Select Save Slot");
	LCD_Printf(0, 5, "[K Hold] to Save");

	while ((btn = Button_Get_Input()) != INPUT_CMD_K_HOLD) {
		if (slot > 10)
				slot = 1;
			if (slot < 1)
				slot = 10;
		LCD_Printf(0, 2, "Slot: %-2d", slot);
		switch (btn) {
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			if (slot > 1)
				slot--;
			else
				slot = 10;
			break;

		case INPUT_CMD_R_SINGLE:
		case INPUT_CMD_R_HOLD:
			if (slot < 10)
				slot++;
			else
				slot = 1;
			break;

		default:
			break;
		}
	}
	return slot++;
}

void Save_MarkerLog_To_SD(uint8_t slot_number) {
	static char log_buf[CROSS_LOG_BUFFER_SIZE];
	int len = 0;
	char filepath[64];

	if (slot_number < 1 || slot_number > 10) {
		slot_number = 1;
	}
	sprintf(filepath, "/Drive_Data/save_slot_%d.txt", slot_number);

	// 주행 파라미터 정보 기록
	// @formatter:off
	len += sprintf(log_buf + len, "base mps = %.2f\n", driveData.base_mps);
	len += sprintf(log_buf + len, "max mps = %.2f\n", driveData.max_mps);
	len += sprintf(log_buf + len, "accel = %.2f\n", driveData.accel);
	len += sprintf(log_buf + len, "decel = %.2f\n", driveData.decel);
	len += sprintf(log_buf + len, "steer kp = %.2f\n", driveData.steer_gain_p);
	len += sprintf(log_buf + len, "steer kd = %.2f\n", driveData.steer_gain_d);
	len += sprintf(log_buf + len, "pos atten gain = %.2f\n", driveData.pos_atten_gain);

	// 구분선 및 마커 헤더 기록
	len += sprintf(log_buf + len, "===================\n");
	len += sprintf(log_buf + len, "IDX\tTYPE\tDIST\n");
	// @formatter:on

	uint8_t count = (g_cross_log_count < CROSS_LOG_MAX) ? g_cross_log_count : CROSS_LOG_MAX;

	for (uint16_t i = 0; i < count; i++) {
		CrossEvent_t type = g_cross_log[i].type;
		float dist = g_cross_log[i].dist_from_prev_m;

		const char *type_str = (type == CROSS_LEFT) ? "L" :
								(type == CROSS_RIGHT) ? "R" :
								(type == CROSS_CROSS) ? "C" : "U";

		len += sprintf(log_buf + len, "%d\t%s\t%.3f m\n", i, type_str, dist);

		if (len >= (int) sizeof(log_buf) - 64)
			break;
	}

	SDCard_Save(filepath, log_buf, len);
}
