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

/* ===== Last-Used (STM32H743 internal flash, Bank2 last sector) ===== */
#define LU_FLASH_ADDR     0x081E0000UL          /* Bank2 / Sector7 start */
#define LU_FLASH_END      0x08200000UL          /* end of 2MB flash */
#define LU_FLASH_BANK     FLASH_BANK_2
#define LU_FLASH_SECTOR   FLASH_SECTOR_7
#define LU_SLOT_SIZE      32U                    /* one 256-bit flash word */
#define LU_SLOT_COUNT     ((LU_FLASH_END - LU_FLASH_ADDR) / LU_SLOT_SIZE)  /* 4096 */
#define LU_MAGIC          0xA5C35A3CUL

typedef struct {
	uint32_t magic;
	uint32_t func;         /* callback 주소 */
	uint32_t reserved[6];  /* 32바이트 맞춤 */
} LU_Record_t;

_Static_assert(sizeof(LU_Record_t) == LU_SLOT_SIZE, "record must be 32 bytes");

static void LastUsed_Execute(void);
static void LastUsed_Save(MenuContext_t *pCtx, uint8_t index);

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
    { .name = "Boot Load",   .pfnActionCallback = Boot_Loading,     .child_menu = NULL },
    { .name = "Sensor Menu", .pfnActionCallback = NULL,             .child_menu = &sensor_menu },
    { .name = "Motor Menu",  .pfnActionCallback = NULL,             .child_menu = &motor_menu },
    { .name = "Drive Menu",  .pfnActionCallback = NULL,             .child_menu = &drive_menu },
    { .name = "Param Menu",  .pfnActionCallback = NULL,             .child_menu = &param_menu },
    { .name = "Last Used",   .pfnActionCallback = LastUsed_Execute, .child_menu = NULL }
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
// 2-1. Last-Used 처리
// =========================================================
static MenuContext_t * const lu_all_menus[] = {
	&main_menu, &sensor_menu, &motor_menu, &drive_menu, &param_menu
};

/* 섹터 전체를 훑어 가장 마지막 유효 기록을 반환 (append 특성상 = 최신) */
static const LU_Record_t *LastUsed_Latest(void) {
	const LU_Record_t *s = (const LU_Record_t *)LU_FLASH_ADDR;
	const LU_Record_t *latest = NULL;
	for (uint32_t i = 0; i < LU_SLOT_COUNT; i++) {
		if (s[i].magic == LU_MAGIC)
			latest = &s[i];
	}
	return latest;
}

/* 처음으로 지워진(0xFF) 칸의 주소. 없으면 0(가득 참) */
static uint32_t LastUsed_FreeAddr(void) {
	const LU_Record_t *s = (const LU_Record_t *)LU_FLASH_ADDR;
	for (uint32_t i = 0; i < LU_SLOT_COUNT; i++) {
		if (s[i].magic == 0xFFFFFFFFUL)
			return LU_FLASH_ADDR + i * LU_SLOT_SIZE;
	}
	return 0;
}

/* 저장된 func 주소가 현재 펌웨어의 실제 콜백인지 검증 겸 조회 */
static MenuItem_t *LastUsed_FindItem(uint32_t func) {
	for (uint32_t m = 0; m < MENU_ITEM_COUNT(lu_all_menus); m++) {
		MenuContext_t *mc = lu_all_menus[m];
		for (uint8_t i = 0; i < mc->item_count; i++) {
			void (*cb)(void) = mc->pMenuItems[i].pfnActionCallback;
			if (cb != NULL && cb != LastUsed_Execute && (uint32_t)cb == func)
				return &mc->pMenuItems[i];
		}
	}
	return NULL;
}

/* 메인 메뉴에서 Last Used 항목 자체를 찾는다 */
static MenuItem_t *LastUsed_Slot(void) {
	for (uint8_t i = 0; i < main_menu.item_count; i++) {
		if (main_menu.pMenuItems[i].pfnActionCallback == LastUsed_Execute)
			return &main_menu.pMenuItems[i];
	}
	return NULL;
}

/* Last Used 항목의 표시 이름을 최근 실행 함수 이름으로 교체.
 * item->name은 rodata 문자열 리터럴이라 포인터만 바꿔도 안전하다. */
static void LastUsed_RefreshLabel(void) {
	MenuItem_t *slot = LastUsed_Slot();
	if (slot == NULL)
		return;
	const LU_Record_t *rec = LastUsed_Latest();
	MenuItem_t *item = (rec != NULL) ? LastUsed_FindItem(rec->func) : NULL;
	slot->name = (item != NULL) ? item->name : "Last Used";
}

static void LastUsed_Save(MenuContext_t *pCtx, uint8_t index) {
	uint32_t func = (uint32_t)pCtx->pMenuItems[index].pfnActionCallback;
	const LU_Record_t *last = LastUsed_Latest();
	if (last != NULL && last->func == func)
		return; /* 직전과 동일하면 기록 안 함 */

	__attribute__((aligned(32))) LU_Record_t rec;
	rec.magic = LU_MAGIC;
	rec.func  = func;
	for (int k = 0; k < 6; k++)
		rec.reserved[k] = 0xFFFFFFFFUL;

	uint32_t addr = LastUsed_FreeAddr();

	HAL_FLASH_Unlock();
	if (addr == 0) { /* 가득 차면 섹터 지우고 처음부터 */
		FLASH_EraseInitTypeDef er = { 0 };
		er.TypeErase = FLASH_TYPEERASE_SECTORS;
		er.Banks = LU_FLASH_BANK;
		er.Sector = LU_FLASH_SECTOR;
		er.NbSectors = 1;
		er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		uint32_t serr = 0;
		HAL_FLASHEx_Erase(&er, &serr);
		addr = LU_FLASH_ADDR;
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*) LU_FLASH_ADDR,
				(int32_t) (LU_FLASH_END - LU_FLASH_ADDR));
#endif
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t) &rec);
	HAL_FLASH_Lock();

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*) addr, (int32_t) LU_SLOT_SIZE);
#endif

	LastUsed_RefreshLabel(); /* 방금 실행한 항목으로 메뉴 라벨 갱신 */
}

static void LastUsed_Execute(void) {
	const LU_Record_t *rec = LastUsed_Latest();
	MenuItem_t *item = (rec != NULL) ? LastUsed_FindItem(rec->func) : NULL;

	if (item == NULL) {                    /* 기록 없음 / 펌웨어 바뀜 → 무시 */
		LCD_Clear();
		LCD_Printf(0, 0, "%-16s", "No valid history");
		HAL_Delay(1000);
		LCD_Clear();
		return;
	}
	item->pfnActionCallback();
}

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
			if (pCtx->pMenuItems[selected].pfnActionCallback != LastUsed_Execute)
				LastUsed_Save(pCtx, selected);   /* 실행 전에 기록 */
			pCtx->pMenuItems[selected].pfnActionCallback();
		}
		break;

	default:
		break;
	}
}

void Menu_ProcessLoop() {
	static uint8_t lu_inited = 0;
	if (!lu_inited) {                      /* 부팅 후 flash 기록으로 라벨 1회 복원 */
		LastUsed_RefreshLabel();
		lu_inited = 1;
	}
	Select_Menu(current_menu);
	Show_Menu(current_menu);
	current_menu->prev_index = current_menu->cursor_index;
}
