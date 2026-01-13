#include "button.h"

#define DEBOUNCE_TIME     20   // th?i gian ch?ng d?i
#define HOLD_TIME         2000 // th?i gian nh?n gi? (1 giây)
#define REPEAT_INTERVAL   50  // kho?ng th?i gian l?p l?i khi gi? (0.2 giây)

typedef struct {
    uint32_t last_time;     // th?i di?m thay d?i tr?ng thái cu?i
    uint32_t hold_start;    // th?i di?m b?t d?u gi?
    uint8_t last_state;     // tr?ng thái tru?c
    uint8_t is_held;        // c? dang gi?
} ButtonState;

static ButtonState btn1    = {0, 0, 1, 0};
static ButtonState btn2    = {0, 0, 1, 0};
static ButtonState btn3    = {0, 0, 1, 0};


Button_State Button_Pressing(void){

	struct {
        GPIO_TypeDef *port;
        uint16_t pin;
        ButtonState *state;
        Button_State press_evt;
        Button_State hold_evt;
    } buttons[] = {
        {BUTTON_START_GPIO_Port, BUTTON_START_Pin, &btn1, BUTTON_START_PRESS, BUTTON_START_HOLD},
        {BUTTON_STOP_GPIO_Port, BUTTON_STOP_Pin, &btn2, BUTTON_STOP_PRESS, BUTTON_STOP_HOLD},
        {ENCODER_SW_GPIO_Port, ENCODER_SW_Pin, &btn3, BUTTON_ENCODER_PRESS, BUTTON_ENCODER_HOLD}
    };
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < 3; i++) {
        uint8_t state = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
        ButtonState *btn = buttons[i].state;

        if (state != btn->last_state) {

            if (now - btn->last_time > DEBOUNCE_TIME) {
                btn->last_time  = now;
                btn->last_state = state;

                if (state == GPIO_PIN_RESET) {
                    btn->hold_start = now;
                    btn->is_held    = 0;
                }
                else {
                    if (!btn->is_held) {
                        return buttons[i].press_evt;
                    }
                    btn->is_held = 0;
                }
            }
        }

        else if (state == GPIO_PIN_RESET) {
            if (buttons[i].hold_evt == 0) continue;
            if (!btn->is_held && (now - btn->hold_start >= HOLD_TIME)) {
                btn->is_held   = 1;
                btn->last_time = now;
                return buttons[i].hold_evt;
            }
            if (btn->is_held && (now - btn->last_time >= REPEAT_INTERVAL)) {
                btn->last_time = now;
                return buttons[i].hold_evt;
            }
        }
    }

    return BUTTON_NONE;
}
