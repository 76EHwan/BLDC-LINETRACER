/*
 * menu.c
 *
 *  Created on: 2026. 5. 1.
 *      Author: kth59
 */

#include "main.h"
#include "menu.h"
#include "st7789_lcd.h"
#include "bootloader.h"
#include "button.h"
#include "w25qxx.h"
#include "sensor.h"
//#include "motor.h"
//#include "drive.h"

__STATIC_INLINE void GoTo_Sensor_Menu(void);

__STATIC_INLINE void GoTo_Motor_Menu(void);

__STATIC_INLINE void GoTo_Drive_Menu(void);

__STATIC_INLINE void GoTo_Param_Menu(void);

MenuItem_t main_menu_items[] = { { .name = "Boot Load", .pfnActionCallback =
		Boot_Loading }, { .name = "Sensor Menu", .pfnActionCallback =
		GoTo_Sensor_Menu }, { .name = "Motor Menu", .pfnActionCallback =
		GoTo_Motor_Menu }, { .name = "Drive Menu", .pfnActionCallback =
		GoTo_Drive_Menu }, { .name = "Param Menu", .pfnActionCallback =
		GoTo_Param_Menu }, };

MenuItem_t sensor_menu_items[] = { { .name = "Calibration", .pfnActionCallback =
		Sensor_Calibration }, { .name = "Raw", .pfnActionCallback =
		Sensor_Raw_Printf },
		{ .name = "Normalized", .pfnActionCallback = NULL }, { .name = "State",
				.pfnActionCallback =
				NULL }, { .name = "Update Thres", .pfnActionCallback = NULL }, };

MenuItem_t motor_menu_items[] = { { .name = "Driver Setup", .pfnActionCallback =
NULL }, { .name = "Update Setup", .pfnActionCallback = NULL }, { .name =
		"Simple PWM", .pfnActionCallback = NULL }, { .name = "Simple 6-STEP",
		.pfnActionCallback = NULL }, { .name = "Simple FOC",
		.pfnActionCallback = NULL }, { .name = "Update PI", .pfnActionCallback =
NULL }, };

MenuItem_t drive_menu_items[] = { { .name = "1st Drive", .pfnActionCallback =
NULL }, { .name = "2nd Drive", .pfnActionCallback = NULL }, { .name =
		"3rd Drive", .pfnActionCallback = NULL }, { .name = "4th Drive",
		.pfnActionCallback =
		NULL }, { .name = "Update Param", .pfnActionCallback =
NULL }, { .name = "View Marker", .pfnActionCallback =
NULL }, { .name = "Save Flash", .pfnActionCallback =
NULL }, { .name = "Save MicroSD", .pfnActionCallback =
NULL }, };

MenuItem_t param_menu_items[] = { { .name = "LED Test", .pfnActionCallback =
		LED_Test }, { .name = "LCD Test", .pfnActionCallback = LCD7789_Test }, {
		.name = "Flash Test", .pfnActionCallback = W25QXX_Test }, { .name =
		"SD Card Test", .pfnActionCallback =
NULL }, };

MenuContext_t main_menu = { .category_name = "Main Menu", .pMenuItems =
		main_menu_items, .item_count = 5, .prev_index = 0, .cursor_index = 0 };

MenuContext_t sensor_menu = { .category_name = "Sensor Menu", .pMenuItems =
		sensor_menu_items, .item_count = 5, .prev_index = 0, .cursor_index = 0 };

MenuContext_t motor_menu = { .category_name = "Motor Menu", .pMenuItems =
		motor_menu_items, .item_count = 6, .prev_index = 0, .cursor_index = 0 };

MenuContext_t drive_menu = { .category_name = "Drive Menu", .pMenuItems =
		drive_menu_items, .item_count = 8, .prev_index = 0, .cursor_index = 0 };

MenuContext_t param_menu = { .category_name = "Param Menu", .pMenuItems =
		param_menu_items, .item_count = 4, .prev_index = 0, .cursor_index = 0 };

MenuContext_t *current_menu = &main_menu;

/* 구조체 복사를 방지하기 위해 포인터로 넘겨받습니다. */
__STATIC_INLINE void Show_Menu(MenuContext_t *pCtx) {
	LCD_Printf(0, 0, "%-16s", pCtx->category_name); // -16s 로 공백 채움
	LCD_Printf(0, 1, "----------------");

	for (uint8_t i = 0; i < pCtx->item_count; i++) {
		if (i == pCtx->cursor_index) {
			LCD_Set_Color(BLACK, WHITE);
		} else {
			LCD_Set_Color(WHITE, BLACK);
		}
		// "%-12s" 를 사용하여 짧은 글자 뒤에 공백을 강제로 넣어줍니다.
		LCD_Printf(0, 2 + i, "%X. %-12s", i + 1, pCtx->pMenuItems[i].name);
	}
	LCD_Set_Color(WHITE, BLACK);
}
__STATIC_INLINE void Select_Menu(MenuContext_t *pCtx) {
	UserInput_t bt = Button_Get_Input();

	switch (bt) {
	case INPUT_CMD_D_SINGLE:
		pCtx->cursor_index =
				(pCtx->cursor_index == (pCtx->item_count - 1)) ?
						0 : (pCtx->cursor_index + 1);
		break;
	case INPUT_CMD_U_SINGLE:
		pCtx->cursor_index =
				(pCtx->cursor_index == 0) ?
						(pCtx->item_count - 1) : (pCtx->cursor_index - 1);
		break;

	case INPUT_CMD_L_SINGLE:
		if (current_menu != &main_menu) {
			current_menu = &main_menu;
			current_menu->cursor_index = 0;
			LCD_Clear();
		}
		break;

	case INPUT_CMD_R_SINGLE:
		while (HAL_GPIO_ReadPin(btn_r.port, btn_r.pin) == btn_k.active_state)
			;
		LCD_Clear();
		uint8_t selected = pCtx->cursor_index;
		if (pCtx->pMenuItems[selected].pfnActionCallback != NULL) {
			pCtx->pMenuItems[selected].pfnActionCallback();
		}
		break;

	default:
		break;
	}
}

/* 메인 루틴에서 지속적으로 호출될 함수 (이름도 ProcessLoop으로 더 명확하게 변경) */
void Menu_ProcessLoop(void) {
	// 1. 버튼 입력 확인 및 상태 전환
	// (이 함수 내부에서 current_menu가 다른 메뉴로 바뀔 수 있습니다)
	Select_Menu(current_menu);

	// 2. 항상 '최신' 상태의 current_menu를 렌더링
	Show_Menu(current_menu);

	// 3. 과거 상태 업데이트
	current_menu->prev_index = current_menu->cursor_index;
}

__STATIC_INLINE void GoTo_Sensor_Menu(void) {
	current_menu = &sensor_menu;
	current_menu->cursor_index = 0;
}

__STATIC_INLINE void GoTo_Motor_Menu(void) {
	current_menu = &motor_menu;
	current_menu->cursor_index = 0;
}

__STATIC_INLINE void GoTo_Drive_Menu(void) {
	current_menu = &drive_menu;
	current_menu->cursor_index = 0;
}

__STATIC_INLINE void GoTo_Param_Menu(void) {
	current_menu = &param_menu;
	current_menu->cursor_index = 0;
}
