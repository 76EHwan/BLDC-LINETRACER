#include "main.h"
#include "drive.h"
#include "sensor.h"
#include "foc.h"
#include "motor.h"
#include "user_init.h"
#include "button.h"
#include "sd_ui.h" // UI 및 SD 카드 함수 사용

// @foramtter:off
DriveParam_t driveData = {
		.base_mps = 1.f,
		.max_mps = 6.f,
		.accel = 4.f,
		.decel = 8.f,
		.steer_gain_p = 1.35f,
		.steer_gain_d = 0.0f,
		.pos_atten_gain = 0.45f,
		.pit_in_distance_m = 0.15f,
		.fan_en = 0,
};
// @formatter:on


void Drive_Stop_At_Distance(float target_distance_m) {
	if (g_current_base_mps <= 0.0f || target_distance_m <= 0.0f) {
		g_target_base_mps = 0.0f;
		HAL_Delay(500);
		return;
	}

	decel = (g_current_base_mps * g_current_base_mps)
			/ (2.0f * target_distance_m);
	g_target_base_mps = 0.0f;
	g_is_braking = 1;

	while (g_current_base_mps > 0.001f) {
		if (IR_Sensor.is_lost_position)
			break;
	}

	g_current_base_mps = 0.0f;
	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < 500)
		;
	g_is_braking = 0;
}
// 상단 전역 변수 선언 구역에 추가
uint8_t g_total_L = 0;
uint8_t g_total_R = 0;
uint8_t g_total_C = 0;
uint8_t g_total_STOP = 0; // ★ 정지 마커 카운트용 변수 추가

// ============================================================================
// 1. 주행 초기화 함수 (성공 시 1, 실패 시 0 반환)
// ============================================================================
static uint8_t Drive_Init_Sequence(void) {
	if (!IR_Sensor.is_calibration) {
		if (Sensor_Load_Calibration() != FR_OK) {
			LCD_Printf(0, 0, "Fail");
			HAL_Delay(1000);
			return 0; // 초기화 실패
		}
	}

	if (driveData.fan_en) {
		Fan_Mtr_Start();
		Fan_Mtr_Set_Duty(driveData.fan_en * 100);
		HAL_Delay(1000);
	}

	// 파라미터 및 제어 변수 초기화
	accel = driveData.accel;
	decel = driveData.decel;
	Odom_Reset();
	g_cross_log_count = 0;
	Cross_Detect_Reset();

	// ★ 추가: 이전 주행의 Line Lost 상태 및 마커 상태 찌꺼기 완벽 초기화
	IR_Sensor.is_lost_position = 0;
	IR_Sensor.data->mark_left = 0;
	IR_Sensor.data->mark_right = 0;

	// ★ 전역 마커 카운트 초기화
	g_total_L = 0;
	g_total_R = 0;
	g_total_C = 0;
	g_total_STOP = 0; // 정지 카운트 초기화

	// PID 세팅
	steer_pid.Kp = driveData.steer_gain_p;
	steer_pid.Ki = 0.0f;
	steer_pid.Kd = driveData.steer_gain_d;
	arm_pid_init_f32(&steer_pid, 1);

	// 하드웨어 타이머 및 제어기 구동 시작
	Sensor_Start();
	HAL_Delay(10);
	MTR_Setup_And_Start(FOC_MODE_SPEED_LOOP);
	Ramp_Start();

	// 목표 속도 인가
	g_target_base_mps = driveData.base_mps;

	return 1; // 초기화 성공
}
// ============================================================================
// 2. 마커 이벤트 처리 및 카운트 함수 (정지 신호 발생 시 1 반환)
// ============================================================================
static uint8_t Process_Marker_Event(CrossEvent_t cross) {
	if (cross == CROSS_STOP) {
		g_total_STOP++; // ★ 정지 마커 인식 횟수 증가

		if (g_total_STOP >= 2) {
			return 1; // ★ 두 번째 정지 마커 감지 시에만 루프 탈출 신호 반환
		}

		return 0; // 첫 번째 정지 마커는 무시하고 계속 주행
	}

	// 전역 변수를 직접 증가
	if (cross == CROSS_LEFT) {
		g_total_L++;
	} else if (cross == CROSS_RIGHT) {
		g_total_R++;
	} else if (cross == CROSS_CROSS) {
		g_total_C++;
	}

	return 0; // 계속 주행
}

// ============================================================================
// 3. 메인 라인 트레이싱 주행 함수
// ============================================================================
void Drive_First(void) {
	if (!Drive_Init_Sequence()) {
		return;
	}

	while (!IR_Sensor.is_lost_position) {
		CrossEvent_t cross = Cross_Detect_Update();

		if (cross != CROSS_NONE) {

			if (Process_Marker_Event(cross)) {
				break;
			}
		}
	}

	Drive_Stop_At_Distance(driveData.pit_in_distance_m);
	Ramp_Stop();
	MTR_Safe_Stop();
	Sensor_Stop();
	Fan_Mtr_Stop();

	if (IR_Sensor.is_lost_position) {
		LCD_Printf(0, 0, "Line Lost");
	} else {
		LCD_Printf(0, 0, "End");
		LCD_Printf(0, 1, "L:%d", g_total_L);
		LCD_Printf(0, 2, "R:%d", g_total_R);
		LCD_Printf(0, 3, "C:%d", g_total_C);
		uint8_t slot = Select_Save_Slot();
		Save_MarkerLog_To_SD(slot);
	}

	while (Button_Get_Input() != INPUT_CMD_K_HOLD);
	LCD_Clear();
}
