#ifndef LINE_FOLLOW_TASK_H
#define LINE_FOLLOW_TASK_H

#include <stdint.h>

typedef enum {
    LFMODE_NONE = 0,
    LFMODE_FULL_LAP,              /* Q2: odometry-arm + A-line stop */
    LFMODE_Q4_DISTANCE_STOP,      /* Q4: stop at configured A->B distance */
    LFMODE_Q5_Q6_DISTANCE_STOP    /* Q5/Q6: stop at 110% lap distance */
} LineFollowMode;

void LineFollow_Init(void);
void LineFollow_SetStopRamp(uint32_t duration_ms);
void LineFollow_Start(LineFollowMode mode);
void LineFollow_RequestStop(uint32_t now_ms);
void LineFollow_Update(uint32_t now_ms);
uint8_t LineFollow_IsComplete(void);

#endif
