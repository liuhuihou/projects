#include "button_input.h"

#include "board_hardware.h"

#define BUTTON_DEBOUNCE_MS (30U)

typedef struct {
    uint8_t stable_pressed;
    uint8_t candidate_pressed;
    uint32_t candidate_since_ms;
} ButtonState;

static ButtonState s_bls;

static uint8_t read_bls_pressed(void)
{
    /* PA18 has an external pull-down and is high when BLS is pressed. */
    return HW_GPIO_READ(HW_BLS_KEY_PORT, HW_BLS_KEY_PIN) ? 1U : 0U;
}

static void button_state_init(ButtonState *state, uint8_t pressed,
                              uint32_t now_ms)
{
    state->stable_pressed = pressed;
    state->candidate_pressed = pressed;
    state->candidate_since_ms = now_ms;
}

static uint8_t button_state_update(ButtonState *state, uint8_t pressed,
                                   uint32_t now_ms, uint8_t release_event,
                                   uint8_t press_event)
{
    uint8_t event = BUTTON_EVENT_NONE;

    if (pressed != state->candidate_pressed) {
        state->candidate_pressed = pressed;
        state->candidate_since_ms = now_ms;
    }

    if ((pressed != state->stable_pressed) &&
        ((uint32_t)(now_ms - state->candidate_since_ms) >=
         BUTTON_DEBOUNCE_MS)) {
        state->stable_pressed = pressed;
        if (pressed != 0U) {
            event = press_event;
        } else {
            event = release_event;
        }
    }

    return event;
}

void ButtonInput_Init(uint32_t now_ms)
{
    /* Initialize from the live level so a held key cannot create a phantom event. */
    button_state_init(&s_bls, read_bls_pressed(), now_ms);
}

uint8_t ButtonInput_Poll(uint32_t now_ms)
{
    uint8_t event = BUTTON_EVENT_NONE;

    event |= button_state_update(&s_bls, read_bls_pressed(), now_ms,
                                 BUTTON_EVENT_BLS_CLICK, BUTTON_EVENT_NONE);
    return event;
}
