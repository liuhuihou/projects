#include "balance_task.h"
#include "balance_controller.h"

/*
 * Balance task implementation.
 * Q3 specific: O -> +50mm -> -50mm, total <=5s, accuracy <=10mm
 * Q4/Q5: hold at center (target=0)
 * Q6: hold at designated position
 */

typedef enum {
    BT_IDLE = 0,
    BT_PHASE1,      /* Q3: moving to +5cm */
    BT_PHASE2,      /* Q3: moving to -5cm */
    BT_PHASE3,      /* Q3: stabilizing at -5cm */
    BT_HOLDING,     /* Q4/Q5/Q6: holding position */
    BT_COMPLETE
} BtState;

static BalanceTaskMode s_mode;
static BtState s_bt_state;
static uint32_t s_phase_start_ms;
static int16_t s_designated_pos;

void BalanceTask_Init(void)
{
    s_mode = BTASK_NONE;
    s_bt_state = BT_IDLE;
    s_phase_start_ms = 0;
    s_designated_pos = 0;
}

void BalanceTask_SetPosition(int16_t pos_mm)
{
    s_designated_pos = pos_mm;
}

void BalanceTask_Start(BalanceTaskMode mode)
{
    s_mode = mode;
    s_phase_start_ms = 0;

    Balance_Enable();

    switch (mode) {
        case BTASK_STATIC_MOVE:
            /* Start by moving to +50mm (5cm) from center */
            Balance_SetTarget(50);
            s_bt_state = BT_PHASE1;
            break;

        case BTASK_HOLD_CENTER:
            Balance_SetTarget(0);
            s_bt_state = BT_HOLDING;
            break;

        case BTASK_HOLD_POSITION:
            Balance_SetTarget(s_designated_pos);
            s_bt_state = BT_HOLDING;
            break;

        default:
            s_bt_state = BT_IDLE;
            break;
    }
}
uint8_t BalanceTask_IsComplete(void)
{
    return (s_bt_state == BT_COMPLETE) ? 1U : 0U;
}

void BalanceTask_Update(uint32_t now_ms)
{
    int16_t error;

    if (s_bt_state == BT_IDLE || s_bt_state == BT_COMPLETE) return;

    /* Run balance PID tick */
    Balance_Tick(now_ms);

    if (s_phase_start_ms == 0) s_phase_start_ms = now_ms;

    error = Balance_GetError();

    switch (s_mode) {
        case BTASK_STATIC_MOVE:
            /* Q3 state machine: O -> +5cm -> -5cm (stabilize) */
            switch (s_bt_state) {
                case BT_PHASE1:
                    /* Wait until ball reaches +50mm (within 10mm) */
                    if (error < 10 && error > -10) {
                        /* Reached +5cm, now go to -5cm */
                        Balance_SetTarget(-50);
                        s_bt_state = BT_PHASE2;
                        s_phase_start_ms = now_ms;
                    }
                    /* Timeout safety: 3s per phase */
                    if ((now_ms - s_phase_start_ms) > 3000) {
                        Balance_SetTarget(-50);
                        s_bt_state = BT_PHASE2;
                        s_phase_start_ms = now_ms;
                    }
                    break;

                case BT_PHASE2:
                    /* Wait until ball reaches -50mm */
                    if (error < 10 && error > -10) {
                        s_bt_state = BT_PHASE3;
                        s_phase_start_ms = now_ms;
                    }
                    if ((now_ms - s_phase_start_ms) > 3000) {
                        s_bt_state = BT_PHASE3;
                        s_phase_start_ms = now_ms;
                    }
                    break;

                case BT_PHASE3:
                    /* Stabilize at -5cm for 500ms then complete */
                    if ((now_ms - s_phase_start_ms) > 500) {
                        s_bt_state = BT_COMPLETE;
                        Balance_Disable();
                    }
                    break;

                default:
                    break;
            }
            break;

        case BTASK_HOLD_CENTER:
        case BTASK_HOLD_POSITION:
            /* Continuously holding - never self-completes */
            /* Completion is triggered externally by line_follow finishing */
            break;

        default:
            break;
    }
}
