/*
 * menu.c
 */

#include "main.h"
#include "menu.h"
#include "user_init.h"
#include "bootloader.h"
#include "button.h"
#include "w25qxx.h"
#include "sensor.h"
#include "motor.h"

// 하위 메뉴들을 배열에서 미리 참조할 수 있도록 전방 선언
extern MenuContext_t sensor_menu;
extern MenuContext_t motor_menu;
extern MenuContext_t drive_menu;
extern MenuContext_t param_menu;

// =========================================================
// 1. 메뉴 아이템 배열 정의 (자동 개수 산정을 위해 먼저 정의)
// =========================================================
// @formatter:off
MenuItem_t main_menu_items[] = {
    { .name = "Boot Load",   .pfnActionCallback = Boot_Loading, .child_menu = NULL },
    { .name = "Sensor Menu", .pfnActionCallback = NULL,         .child_menu = &sensor_menu },
    { .name = "Motor Menu",  .pfnActionCallback = NULL,         .child_menu = &motor_menu },
    { .name = "Drive Menu",  .pfnActionCallback = NULL,         .child_menu = &drive_menu },
    { .name = "Param Menu",  .pfnActionCallback = NULL,         .child_menu = &param_menu }
};

MenuItem_t sensor_menu_items[] = {
    { .name = "Calibration",  .pfnActionCallback = Sensor_Calibration },
    { .name = "Raw",          .pfnActionCallback = Sensor_Raw_Printf },
    { .name = "Normalized",   .pfnActionCallback = NULL },
    { .name = "State",        .pfnActionCallback = NULL },
    { .name = "Update Thres", .pfnActionCallback = NULL },
    { .name = "IMU Test",     .pfnActionCallback = IMU_Test }
};

MenuItem_t motor_menu_items[] = {
    { .name = "Driver Setup",  .pfnActionCallback = MTR_Read_Register },
    { .name = "Update Setup",  .pfnActionCallback = MTR_Update_Setup },
    { .name = "Simple PWM",    .pfnActionCallback = MTR_Simple_Control },
    { .name = "Simple 6-STEP", .pfnActionCallback = NULL },
    { .name = "Simple FOC",    .pfnActionCallback = MTR_Simple_FOC },
    { .name = "Update PI",     .pfnActionCallback = MTR_Current_Tune_Loop },
    { .name = "Encoder",       .pfnActionCallback = MTR_Encoder_Test },
    { .name = "Encoder FOC",   .pfnActionCallback = MTR_Speed_FOC }
};

MenuItem_t drive_menu_items[] = {
    { .name = "1st Drive",    .pfnActionCallback = NULL },
    { .name = "2nd Drive",    .pfnActionCallback = NULL },
    { .name = "3rd Drive",    .pfnActionCallback = NULL },
    { .name = "4th Drive",    .pfnActionCallback = NULL },
    { .name = "Update Param", .pfnActionCallback = NULL },
    { .name = "View Marker",  .pfnActionCallback = NULL },
    { .name = "Save Flash",   .pfnActionCallback = NULL },
    { .name = "Save MicroSD", .pfnActionCallback = NULL }
};

MenuItem_t param_menu_items[] = {
    { .name = "LED Test",     .pfnActionCallback = LED_Test },
    { .name = "LCD Test",     .pfnActionCallback = LCD7789_Test },
    { .name = "Flash Test",   .pfnActionCallback = W25QXX_Test },
    { .name = "SD Card Test", .pfnActionCallback = NULL }
};

// =========================================================
// 2. 메뉴 컨텍스트 정의 (MENU_ITEM_COUNT 매크로로 자동 산정)
// =========================================================
MenuContext_t main_menu = {
    .category_name = "Main Menu",
    .pMenuItems = main_menu_items,
    .item_count = MENU_ITEM_COUNT(main_menu_items),
    .parent_menu = NULL,  // 최상위 메뉴
    .cursor_index = 0
};

MenuContext_t sensor_menu = {
    .category_name = "Sensor Menu",
    .pMenuItems = sensor_menu_items,
    .item_count = MENU_ITEM_COUNT(sensor_menu_items),
    .parent_menu = &main_menu,
    .cursor_index = 0
};

MenuContext_t motor_menu = {
    .category_name = "Motor Menu",
    .pMenuItems = motor_menu_items,
    .item_count = MENU_ITEM_COUNT(motor_menu_items),
    .parent_menu = &main_menu,
    .cursor_index = 0
};

MenuContext_t drive_menu = {
    .category_name = "Drive Menu",
    .pMenuItems = drive_menu_items,
    .item_count = MENU_ITEM_COUNT(drive_menu_items),
    .parent_menu = &main_menu,
    .cursor_index = 0
};

MenuContext_t param_menu = {
    .category_name = "Param Menu",
    .pMenuItems = param_menu_items,
    .item_count = MENU_ITEM_COUNT(param_menu_items),
    .parent_menu = &main_menu,
    .cursor_index = 0
};

MenuContext_t *current_menu = &main_menu;
// @formatter:on

// =========================================================
// 3. UI 렌더링 및 입력 처리 함수
// =========================================================
__STATIC_INLINE void Show_Menu(MenuContext_t *pCtx) {
	LCD_Printf(0, 0, "%-16s", pCtx->category_name);
	LCD_Printf(0, 1, "----------------");

	for (uint8_t i = 0; i < pCtx->item_count; i++) {
		if (i == pCtx->cursor_index) {
			LCD_Set_Color(BLACK, WHITE);
		} else {
			LCD_Set_Color(WHITE, BLACK);
		}
		LCD_Printf(0, 2 + i, "%X. %-12s", i + 1, pCtx->pMenuItems[i].name);
	}
	LCD_Set_Color(WHITE, BLACK);
}

__STATIC_INLINE void Select_Menu(MenuContext_t *pCtx) {
	UserInput_t bt = Button_Get_Input();

	switch (bt) {
	case INPUT_CMD_D_SINGLE:
	case INPUT_CMD_D_HOLD:
		pCtx->cursor_index =
				(pCtx->cursor_index == (pCtx->item_count - 1)) ?
						0 : (pCtx->cursor_index + 1);
		break;

	case INPUT_CMD_U_SINGLE:
	case INPUT_CMD_U_HOLD:
		pCtx->cursor_index =
				(pCtx->cursor_index == 0) ?
						(pCtx->item_count - 1) : (pCtx->cursor_index - 1);
		break;

	case INPUT_CMD_L_SINGLE:
		// parent_menu가 등록되어 있다면 상위 메뉴로 이동
		if (pCtx->parent_menu != NULL) {
			current_menu = pCtx->parent_menu;
			current_menu->cursor_index = 0; // 뒤로 갈 때 커서 초기화 원하면 유지
			LCD_Clear();
		}
		break;

	case INPUT_CMD_R_SINGLE:
		while (HAL_GPIO_ReadPin(btn_r.port, btn_r.pin) == btn_k.active_state)
			;
		LCD_Clear();

		uint8_t selected = pCtx->cursor_index;

		// 1. 하위 메뉴가 연결되어 있으면 메뉴 진입
		if (pCtx->pMenuItems[selected].child_menu != NULL) {
			current_menu = pCtx->pMenuItems[selected].child_menu;
			current_menu->cursor_index = 0;
		}
		// 2. 실행할 함수가 연결되어 있으면 콜백 실행
		else if (pCtx->pMenuItems[selected].pfnActionCallback != NULL) {
			pCtx->pMenuItems[selected].pfnActionCallback();
		}
		break;

	default:
		break;
	}
}

void Menu_ProcessLoop() {
	Select_Menu(current_menu);
	Show_Menu(current_menu);
	current_menu->prev_index = current_menu->cursor_index;
}
