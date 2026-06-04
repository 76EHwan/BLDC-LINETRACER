#include "button.h"

/* --- 내부 전용 설정 값 --- */
#define BTN_DEBOUNCE_TIME       20   // 노이즈 제거 시간 (ms)
#define BTN_LONG_PRESS_TIME     500  // 길게 누르기로 인식할 시간 (ms)
#define BTN_DOUBLE_CLICK_GAP    250  // 더블 클릭 대기 시간 (ms)
#define BTN_LONG_PRESS_REPEAT   100  // 길게 누를 때 이벤트 반복 주기 (ms)

/* --- 내부 전용 버튼 인스턴스 (5방향 스위치) --- */
ButtonHandle_t btn_u;
ButtonHandle_t btn_d;
ButtonHandle_t btn_l;
ButtonHandle_t btn_r;
ButtonHandle_t btn_k;

/* ========================================================== */
/*                   내부 상태 머신 로직 (숨김)                   */
/* ========================================================== */

void Button_Init_Internal(ButtonHandle_t *btn, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_state) {
    btn->state = BTN_STATE_IDLE;
    btn->start_time = 0;
    btn->last_repeat_time = 0;
    btn->port = port;
    btn->pin = pin;
    btn->active_state = active_state;
}

static ButtonEvent_t Button_Get_Event(ButtonHandle_t *btn) {
    uint32_t now = HAL_GetTick();

    // 설정된 active_state와 현재 핀 상태가 일치하면 '눌림'으로 판단
    bool is_pressed = (HAL_GPIO_ReadPin(btn->port, btn->pin) == btn->active_state);

    ButtonEvent_t event = BTN_EVENT_NONE;

    switch (btn->state) {
        case BTN_STATE_IDLE:
            if (is_pressed) {
                btn->state = BTN_STATE_PRESSED;
                btn->start_time = now;
            }
            break;

        case BTN_STATE_PRESSED:
            if (!is_pressed) {
                if (now - btn->start_time >= BTN_DEBOUNCE_TIME) {
                    btn->state = BTN_STATE_WAIT_DOUBLE;
                    btn->start_time = now;
                } else {
                    btn->state = BTN_STATE_IDLE;
                }
            } else if (now - btn->start_time >= BTN_LONG_PRESS_TIME) {
                btn->state = BTN_STATE_LONG_PRESS;
                btn->last_repeat_time = now;
                event = BTN_EVENT_LONG_PRESS_HOLD;
            }
            break;

        case BTN_STATE_WAIT_DOUBLE:
            if (is_pressed) {
                // 두 번째로 눌리는 순간 이벤트 발생 대신 상태만 전환하고 시작 시간 기록
                btn->state = BTN_STATE_SECOND_PRESSED;
                btn->start_time = now;
            } else if (now - btn->start_time > BTN_DOUBLE_CLICK_GAP) {
                event = BTN_EVENT_SINGLE_CLICK;
                btn->state = BTN_STATE_IDLE;
            }
            break;

        case BTN_STATE_SECOND_PRESSED:
            // 두 번째 누른 상태에서 손을 뗄 때 더블 클릭 판정
            if (!is_pressed) {
                if (now - btn->start_time >= BTN_DEBOUNCE_TIME) {
                    event = BTN_EVENT_DOUBLE_CLICK;
                }
                // 노이즈(디바운스 미달)이든 정상 클릭이든 뗐으므로 IDLE로 돌아감
                btn->state = BTN_STATE_IDLE;
            }
            break;

        case BTN_STATE_LONG_PRESS:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
            } else {
                if (now - btn->last_repeat_time >= BTN_LONG_PRESS_REPEAT) {
                    event = BTN_EVENT_LONG_PRESS_HOLD;
                    btn->last_repeat_time = now;
                }
            }
            break;

        case BTN_STATE_WAIT_RELEASE:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
            }
            break;
    }
    return event;
}

UserInput_t Button_Get_Input(void) {
    // 1. 모든 버튼의 상태 머신을 갱신 (누락 방지)
    ButtonEvent_t evt_u = Button_Get_Event(&btn_u);
    ButtonEvent_t evt_d = Button_Get_Event(&btn_d);
    ButtonEvent_t evt_l = Button_Get_Event(&btn_l);
    ButtonEvent_t evt_r = Button_Get_Event(&btn_r);
    ButtonEvent_t evt_k = Button_Get_Event(&btn_k);

    // 2. 우선순위에 따라 커맨드 반환 (K > U/D > L/R)
    if (evt_k == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_K_SINGLE;
    if (evt_k == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_K_DOUBLE;
    if (evt_k == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_K_HOLD;

    if (evt_u == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_U_SINGLE;
    if (evt_u == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_U_DOUBLE;
    if (evt_u == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_U_HOLD;

    if (evt_d == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_D_SINGLE;
    if (evt_d == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_D_DOUBLE;
    if (evt_d == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_D_HOLD;

    if (evt_l == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_L_SINGLE;
    if (evt_l == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_L_DOUBLE;
    if (evt_l == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_L_HOLD;

    if (evt_r == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_R_SINGLE;
    if (evt_r == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_R_DOUBLE;
    if (evt_r == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_R_HOLD;

    return INPUT_CMD_NONE;
}

void Button_Wait_Release(ButtonHandle_t *btn) {
	while (HAL_GPIO_ReadPin(btn->port, btn->pin) == btn->active_state);
}
