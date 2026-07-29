#include "button_input.h"
#include "board_hardware.h"

#define DEBOUNCE_MS         (20U)
#define DOUBLE_CLICK_MS     (300U)
#define CLICK_TIMEOUT_MS    (400U)

typedef struct {
    uint8_t  raw_active;       /* Current debounced state: 1=pressed */
    uint8_t  last_raw;
    uint8_t  stable;
    uint16_t debounce_cnt;
    uint8_t  click_count;
    uint32_t last_release_ms;
    ButtonEvent pending_event;
} ButtonState;

static ButtonState s_buttons[BTN_COUNT];

static uint8_t read_raw(ButtonId id)
{
    (void)id;
    /* BLS: active HIGH when pressed */
    return HW_GPIO_READ(HW_BLS_KEY_PORT, HW_BLS_KEY_PIN) ? 1U : 0U;
}

void Button_Init(void)
{
    uint8_t i;
    for (i = 0; i < BTN_COUNT; ++i) {
        s_buttons[i].raw_active = 0;
        s_buttons[i].last_raw = 0;
        s_buttons[i].stable = 0;
        s_buttons[i].debounce_cnt = 0;
        s_buttons[i].click_count = 0;
        s_buttons[i].last_release_ms = 0;
        s_buttons[i].pending_event = BTN_EVENT_NONE;
    }
}

void Button_Update(uint32_t now_ms)
{
    uint8_t i;
    for (i = 0; i < BTN_COUNT; ++i) {
        ButtonState *b = &s_buttons[i];
        uint8_t current = read_raw((ButtonId)i);

        /* Debounce */
        if (current != b->last_raw) {
            b->last_raw = current;
            b->debounce_cnt = 0;
        } else {
            if (b->debounce_cnt < DEBOUNCE_MS / 10U) {
                b->debounce_cnt++;
            }
            if (b->debounce_cnt >= DEBOUNCE_MS / 10U) {
                if (current != b->stable) {
                    uint8_t was_pressed = b->stable;
                    b->stable = current;

                    /* Rising edge: press */
                    if (current && !was_pressed) {
                        /* nothing on press */
                    }
                    /* Falling edge: release */
                    if (!current && was_pressed) {
                        b->click_count++;
                        b->last_release_ms = now_ms;
                    }
                }
            }
        }

        /* Evaluate click pattern after timeout */
        if (b->click_count > 0 && !b->stable) {
            uint32_t elapsed = now_ms - b->last_release_ms;
            if (elapsed >= CLICK_TIMEOUT_MS) {
                if (b->click_count >= 2) {
                    b->pending_event = BTN_EVENT_DOUBLE_CLICK;
                } else {
                    b->pending_event = BTN_EVENT_SINGLE_CLICK;
                }
                b->click_count = 0;
            }
        }
    }
}

ButtonEvent Button_GetEvent(ButtonId id)
{
    ButtonEvent ev;
    if (id >= BTN_COUNT) return BTN_EVENT_NONE;
    ev = s_buttons[id].pending_event;
    s_buttons[id].pending_event = BTN_EVENT_NONE;
    return ev;
}
