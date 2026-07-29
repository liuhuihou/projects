#ifndef LINE_FOLLOW_TASK_H
#define LINE_FOLLOW_TASK_H

#include <stdint.h>

typedef enum {
    LINE_FOLLOW_Q2_ONE_LAP = 0U,
    LINE_FOLLOW_Q4_WITH_BALANCE,
    LINE_FOLLOW_Q5_RETURN_TO_A,
    LINE_FOLLOW_Q6_WITH_TARGET_BALANCE
} LineFollowProfile;

void LineFollowTask_Init(void);
void LineFollowTask_Start(LineFollowProfile profile);
void LineFollowTask_Stop(void);
void LineFollowTask_Tick(uint32_t now_ms);
uint8_t LineFollowTask_IsActive(void);

#endif
