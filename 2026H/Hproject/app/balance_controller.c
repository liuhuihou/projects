#include "balance_controller.h"

static BalanceState s_state;
static BalanceTarget s_target;

void BalanceController_Init(void)
{
    s_state = BALANCE_STATE_IDLE;
    s_target = BALANCE_TARGET_CENTER;
}

void BalanceController_Start(BalanceTarget target)
{
    s_target = target;
    s_state = BALANCE_STATE_RUNNING;
}

void BalanceController_Stop(void)
{
    s_state = BALANCE_STATE_IDLE;
}

void BalanceController_Tick(uint32_t now_ms)
{
    (void)now_ms;
    (void)s_target;
    /* MPU6050 filtering and actuator control will be added here later. */
}

BalanceState BalanceController_GetState(void)
{
    return s_state;
}
