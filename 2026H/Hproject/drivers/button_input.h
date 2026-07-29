#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include <stdint.h>

/* Button event types */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK
} ButtonEvent;

/* Button IDs
 * Only BLS is available as GPIO input.
 * RESET is hardware reset pin, not usable as button. */
typedef enum {
    BTN_BLS = 0,
    BTN_COUNT
} ButtonId;

void Button_Init(void);

/* Call every 10ms from control tick or main loop */
void Button_Update(uint32_t now_ms);

/* Poll for pending event (clears after read) */
ButtonEvent Button_GetEvent(ButtonId id);

#endif
