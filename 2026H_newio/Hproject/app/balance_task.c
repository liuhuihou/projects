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

#define Q3_POSITION_TOLERANCE_MM       (10)
#define Q3_STOP_SPEED_MAX_MM_S         (30)
#define Q3_FORWARD_DEADLINE_MS         (2000U)
#define Q3_FINAL_SETTLE_MS             (500U)

static BalanceTaskMode s_mode;
static BtState s_bt_state;
static uint32_t s_phase_start_ms;
static uint32_t s_settle_start_ms;
static int16_t s_designated_pos;

static uint8_t within_abs_limit(int16_t value, int16_t limit)
{
    return (value >= -limit && value <= limit) ? 1U : 0U;
}

void BalanceTask_Init(void)
{
    s_mode = BTASK_NONE;
    s_bt_state = BT_IDLE;
    s_phase_start_ms = 0;
    s_settle_start_ms = 0;
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
    s_settle_start_ms = 0;

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
    int16_t velocity;
    uint8_t tracking;

    if (s_bt_state == BT_IDLE) return;

    /* Run balance PID tick */
    Balance_Tick(now_ms);

    /* Once Q3 has met its final-position criterion, keep running the visual
     * loop so the ball remains near -5 cm instead of freezing the last motor
     * command. */
    if (s_bt_state == BT_COMPLETE) return;

    /* Balance_GetError() only means anything while the PID is actually running
     * on a usable camera position. Reading it unconditionally was a real bug:
     * with the camera silent the error stays at its initial 0, every phase sees
     * |error| < 10 on its first tick, and Q3 walks PHASE1 -> PHASE2 -> PHASE3 ->
     * COMPLETE without the ball having moved at all. */
    tracking = Balance_IsTracking();
    error = Balance_GetError();
    velocity = Balance_GetBallVelocity();

    switch (s_mode) {
        case BTASK_STATIC_MOVE:
            /* Q3 state machine: O -> +5cm -> -5cm (stabilize) */
            switch (s_bt_state) {
                case BT_PHASE1:
                    /* Reverse only after the ball is near +50 mm and has
                     * slowed down. The deadline preserves enough of the 5 s
                     * budget to reach the final -50 mm point. */
                    if (!tracking) break;
                    if (s_phase_start_ms == 0U) s_phase_start_ms = now_ms;
                    if (within_abs_limit(error, Q3_POSITION_TOLERANCE_MM) &&
                        within_abs_limit(velocity, Q3_STOP_SPEED_MAX_MM_S)) {
                        Balance_SetTarget(-50);
                        s_bt_state = BT_PHASE2;
                        s_phase_start_ms = now_ms;
                    } else if ((now_ms - s_phase_start_ms) >=
                               Q3_FORWARD_DEADLINE_MS) {
                        Balance_SetTarget(-50);
                        s_bt_state = BT_PHASE2;
                        s_phase_start_ms = now_ms;
                    }
                    break;

                case BT_PHASE2:
                    if (tracking &&
                        within_abs_limit(error, Q3_POSITION_TOLERANCE_MM) &&
                        within_abs_limit(velocity, Q3_STOP_SPEED_MAX_MM_S)) {
                        s_bt_state = BT_PHASE3;
                        s_settle_start_ms = now_ms;
                    }
                    break;

                case BT_PHASE3:
                    /* Completion requires a continuous stable window. If the
                     * ball leaves it, return to the acquisition phase instead
                     * of reporting DONE on elapsed time alone. */
                    if (!tracking ||
                        !within_abs_limit(error, Q3_POSITION_TOLERANCE_MM) ||
                        !within_abs_limit(velocity, Q3_STOP_SPEED_MAX_MM_S)) {
                        s_bt_state = BT_PHASE2;
                        s_settle_start_ms = 0U;
                    } else if ((now_ms - s_settle_start_ms) >=
                               Q3_FINAL_SETTLE_MS) {
                        s_bt_state = BT_COMPLETE;
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
