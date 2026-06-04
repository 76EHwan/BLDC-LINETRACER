/*
 * button.h
 *
 *  Created on: 2026. 4. 30.
 *      Author: kth59
 */

#ifndef BSP_BUTTON_BUTTON_H_
#define BSP_BUTTON_BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef enum {
    INPUT_CMD_NONE = 0,

    // Up (u)
    INPUT_CMD_U_SINGLE,
    INPUT_CMD_U_DOUBLE,
    INPUT_CMD_U_HOLD,

    // Down (d)
    INPUT_CMD_D_SINGLE,
    INPUT_CMD_D_DOUBLE,
    INPUT_CMD_D_HOLD,

    // Left (l)
    INPUT_CMD_L_SINGLE,
    INPUT_CMD_L_DOUBLE,
    INPUT_CMD_L_HOLD,

    // Right (r)
    INPUT_CMD_R_SINGLE,
    INPUT_CMD_R_DOUBLE,
    INPUT_CMD_R_HOLD,

    // OK / Select (k)
    INPUT_CMD_K_SINGLE,
    INPUT_CMD_K_DOUBLE,
    INPUT_CMD_K_HOLD,
} UserInput_t;

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_LONG_PRESS_HOLD
} ButtonEvent_t;

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_PRESSED,
    BTN_STATE_WAIT_DOUBLE,
    BTN_STATE_SECOND_PRESSED,
    BTN_STATE_LONG_PRESS,
    BTN_STATE_WAIT_RELEASE
} ButtonState_t;

typedef struct {
    ButtonState_t state;
    uint32_t start_time;
    uint32_t last_repeat_time;

    // 하드웨어 정보
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state; // 눌렸을 때의 핀 상태 (High/Low)
} ButtonHandle_t;
extern ButtonHandle_t btn_u;
extern ButtonHandle_t btn_d;
extern ButtonHandle_t btn_l;
extern ButtonHandle_t btn_r;
extern ButtonHandle_t btn_k;

void Button_Init_Internal(ButtonHandle_t *btn, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_state);
UserInput_t Button_Get_Input(void);
void Button_Wait_Release(ButtonHandle_t *btn);
void Button_init(void);

#endif /* BSP_BUTTON_BUTTON_H_ */
