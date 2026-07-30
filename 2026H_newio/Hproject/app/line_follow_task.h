#ifndef LINE_FOLLOW_TASK_H
#define LINE_FOLLOW_TASK_H

#include <stdint.h>

typedef enum {
    LFMODE_NONE = 0,
    LFMODE_FULL_LAP,   /* A -> (full circle) -> A stop */
    LFMODE_A_TO_B      /* A -> B stop */
} LineFollowMode;

void LineFollow_Init(void);
void LineFollow_Start(LineFollowMode mode);
void LineFollow_Update(uint32_t now_ms);
uint8_t LineFollow_IsComplete(void);

/* Get distance traveled in cm */
float LineFollow_GetDistanceCm(void);

#endif
