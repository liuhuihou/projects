#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include <stdint.h>

typedef enum {
    BUTTON_EVENT_NONE = 0U,
    BUTTON_EVENT_BLS_CLICK = (1U << 0)
} ButtonEvent;

void ButtonInput_Init(uint32_t now_ms);
uint8_t ButtonInput_Poll(uint32_t now_ms);

#endif
