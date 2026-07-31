#ifndef BALANCE_TASK_H
#define BALANCE_TASK_H

#include <stdint.h>

typedef enum {
    BTASK_NONE = 0,
    BTASK_STATIC_MOVE,    /* Q3: move ball O -> -5 -> +5, static */
    BTASK_HOLD_CENTER,    /* Q4/Q5: hold ball at center O while moving */
    BTASK_HOLD_POSITION   /* Q6: hold ball at arbitrary position */
} BalanceTaskMode;

void BalanceTask_Init(void);
void BalanceTask_Start(BalanceTaskMode mode);
void BalanceTask_Update(uint32_t now_ms);
uint8_t BalanceTask_IsComplete(void);

/* For Q6: set the target position before starting */
void BalanceTask_SetPosition(int16_t pos_mm);

#endif
