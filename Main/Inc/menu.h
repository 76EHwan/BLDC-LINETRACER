// menu.h
#ifndef MENU_H_
#define MENU_H_

#include <stdint.h>

// 전방 선언 (상호 참조를 위해 필요)
typedef struct MenuContext_t MenuContext_t;

typedef struct {
    const char *name;
    void (*pfnActionCallback)(void);
    MenuContext_t *child_menu;        // 하위 메뉴로 이동하기 위한 포인터 추가
} MenuItem_t;

struct MenuContext_t {
    const char *category_name;
    MenuItem_t *pMenuItems;
    uint8_t item_count;
    MenuContext_t *parent_menu;       // 'Left' 버튼을 눌렀을 때 돌아갈 상위 메뉴
    uint8_t prev_index;
    uint8_t cursor_index;
};

// 배열 크기를 자동으로 계산하는 매크로
#define MENU_ITEM_COUNT(arr) (sizeof(arr) / sizeof((arr)[0]))
void Menu_ProcessLoop(void);

#endif /* MENU_H_ */
