#include "button_input.h"
#include "board_hardware.h"

/*
 * Click detection is driven entirely by timestamps, not by call counts, so
 * it behaves correctly even if Button_Update is called irregularly.
 *
 * Sequence for one press:
 *   raw goes active  -> candidate change, wait DEBOUNCE_MS of stability
 *   stable active    -> press registered
 *   raw goes idle    -> candidate change, wait DEBOUNCE_MS of stability
 *   stable idle      -> click_count++, start/restart the multi-click window
 *   window expires   -> report SINGLE (1 click) or DOUBLE (>=2 clicks)
 */
#define DEBOUNCE_MS         (30U)
#define MULTI_CLICK_MS      (300U)

typedef struct {
    uint8_t  stable;            /* Debounced level: 1 = pressed */
    uint8_t  candidate;         /* Level currently being debounced */
    uint32_t candidate_since;   /* When candidate first appeared */
    uint8_t  click_count;       /* Completed clicks in the current window */
    uint32_t window_start;      /* Timestamp of the last completed click */
    uint8_t  window_open;       /* A multi-click window is in progress */
    ButtonEvent pending_event;
} ButtonState;

static ButtonState s_buttons[BTN_COUNT];

/* BLS on C07A V1.1: externally pulled down, driven high when pressed. */
static uint8_t read_raw(ButtonId id)
{
    (void)id;
    return HW_GPIO_READ(HW_BLS_KEY_PORT, HW_BLS_KEY_PIN) ? 1U : 0U;
}

void Button_Init(void)
{
    uint8_t i;

    for (i = 0U; i < BTN_COUNT; ++i) {
        ButtonState *b = &s_buttons[i];
        b->stable = read_raw((ButtonId)i);
        b->candidate = b->stable;
        b->candidate_since = 0U;
        b->click_count = 0U;
        b->window_start = 0U;
        b->window_open = 0U;
        b->pending_event = BTN_EVENT_NONE;
    }
}
void Button_Update(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < BTN_COUNT; ++i) {
        ButtonState *b = &s_buttons[i];
        const uint8_t raw = read_raw((ButtonId)i);

        /* --- Debounce by elapsed time --- */
        if (raw != b->candidate) {
            b->candidate = raw;
            b->candidate_since = now_ms;
        } else if (raw != b->stable &&
                   (uint32_t)(now_ms - b->candidate_since) >= DEBOUNCE_MS) {
            const uint8_t was_pressed = b->stable;

            b->stable = raw;

            /* Release completes a click. */
            if (was_pressed != 0U && raw == 0U) {
                if (b->click_count < 255U) {
                    b->click_count++;
                }
                b->window_start = now_ms;
                b->window_open = 1U;
            }
        }

        /* --- Resolve the multi-click window --- */
        if (b->window_open != 0U && b->stable == 0U &&
            (uint32_t)(now_ms - b->window_start) >= MULTI_CLICK_MS) {
            b->pending_event = (b->click_count >= 2U) ?
                               BTN_EVENT_DOUBLE_CLICK :
                               BTN_EVENT_SINGLE_CLICK;
            b->click_count = 0U;
            b->window_open = 0U;
        }
    }
}

ButtonEvent Button_GetEvent(ButtonId id)
{
    ButtonEvent ev;

    if (id >= BTN_COUNT) {
        return BTN_EVENT_NONE;
    }
    ev = s_buttons[id].pending_event;
    s_buttons[id].pending_event = BTN_EVENT_NONE;
    return ev;
}
