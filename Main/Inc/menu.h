/*
 * menu.h
 *
 *  Created on: 2026. 5. 1.
 *      Author: kth59
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "main.h"

/* 1. 직관적이고 있어 보이는 콜백 함수 포인터 타입 정의 */
typedef void (*MenuActionCb_t)(void);

/* 2. 메뉴 단일 항목 구조체 */
typedef struct {
    char name[20];
    MenuActionCb_t pfnActionCallback; // void * 대신 명시적 함수 포인터 및 세련된 네이밍
} MenuItem_t;

/* 3. 메뉴 카테고리(컨텍스트) 구조체 */
typedef struct {
    char category_name[20];
    MenuItem_t *pMenuItems;           // category -> pMenuItems (포인터임을 명확히 표시)
    uint8_t item_count;               // max_menu_cnt -> item_count
    uint8_t prev_index;               // past_num -> prev_index
    uint8_t cursor_index;             // select_num -> cursor_index (커서 위치)
} MenuContext_t;                      // category_t -> MenuContext_t (상태를 관리하므로 Context가 더 적합)

/* 전역 변수 외부 선언 */
extern MenuItem_t boot_menu_items[];
extern MenuContext_t *current_menu;

/* 함수 원형 선언 (Call by Reference) */
void Menu_ProcessLoop();
void Sensor_Menu();
void Motor_Menu();
void Drive_Menu();

#endif /* INC_MENU_H_ */
