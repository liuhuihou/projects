#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    BALANCE_TARGET_CENTER = 0U,
    BALANCE_TARGET_PLUS_FIVE_CM,
    BALANCE_TARGET_MINUS_FIVE_CM,
    BALANCE_TARGET_SPECIFIED
} BalanceTarget;

typedef enum {
    BALANCE_STATE_IDLE = 0U,
    BALANCE_STATE_RUNNING
} BalanceState;

void BalanceController_Init(void);
void BalanceController_Start(BalanceTarget target);
void BalanceController_Stop(void);
void BalanceController_Tick(uint32_t now_ms);
BalanceState BalanceController_GetState(void);

#endif
