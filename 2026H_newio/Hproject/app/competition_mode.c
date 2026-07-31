#include "competition_mode.h"
#include "app_config.h"
#include "button_input.h"
#include "vehicle_controller.h"
#include "balance_controller.h"
#include "line_follow_task.h"
#include "balance_task.h"
#include "oled_driver.h"
#include "control_config.h"

static CompetitionQuestion s_mode;
static CompetitionState s_state;
static uint32_t s_start_ms;
static uint32_t s_elapsed_ms;

static const char *mode_names[COMP_MODE_COUNT] = {
    "Q2:LAP 20S",
    "Q3:BALL +-5",
    "Q4:AB+BALL",
    "Q5:LAP+BALL",
    "Q6:LAP+POS"
};

static float mode_speed_cm_s(CompetitionQuestion q)
{
    switch (q) {
        case COMP_Q2: return APP_SPEED_Q2_CM_S;
        case COMP_Q3: return 0.0f; /* Static */
        case COMP_Q4: return APP_SPEED_Q4_CM_S;
        case COMP_Q5: return APP_SPEED_Q5_CM_S;
        case COMP_Q6: return APP_SPEED_Q5_CM_S;
        default: return 0.0f;
    }
}

void Competition_Init(void)
{
    s_mode = COMP_Q2;
    s_state = STATE_IDLE;
    s_start_ms = 0;
    s_elapsed_ms = 0;
}

CompetitionQuestion Competition_GetMode(void) { return s_mode; }
CompetitionState Competition_GetState(void) { return s_state; }
uint32_t Competition_GetElapsedMs(void) { return s_elapsed_ms; }
static void start_task(uint32_t now_ms)
{
    float speed_rpm;

    s_start_ms = now_ms;
    s_elapsed_ms = 0;
    s_state = STATE_RUNNING;

    switch (s_mode) {
        case COMP_Q2:
            /* Pure line follow, fast lap */
            speed_rpm = mode_speed_cm_s(COMP_Q2) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_FULL_LAP);
            break;

        case COMP_Q3:
            /* Static ball control */
            Control_SetMode(CTRL_STOP);
            BalanceTask_Start(BTASK_STATIC_MOVE);
            break;

        case COMP_Q4:
            /* Line A->B with balance */
            speed_rpm = mode_speed_cm_s(COMP_Q4) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_A_TO_B);
            BalanceTask_Start(BTASK_HOLD_CENTER);
            break;

        case COMP_Q5:
            /* Full lap + ball center */
            speed_rpm = mode_speed_cm_s(COMP_Q5) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_FULL_LAP);
            BalanceTask_Start(BTASK_HOLD_CENTER);
            break;

        case COMP_Q6:
            /* Full lap + ball at designated position */
            speed_rpm = mode_speed_cm_s(COMP_Q6) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_FULL_LAP);
            BalanceTask_Start(BTASK_HOLD_POSITION);
            break;

        default:
            break;
    }
}

static void stop_task(void)
{
    Control_SetMode(CTRL_STOP);
    Balance_Disable();
    s_state = STATE_DONE;
}
void Competition_ForceStop(void)
{
    stop_task();
}

void Competition_Update(uint32_t now_ms)
{
    /* Button_GetEvent reports at most one event per press sequence and
     * single/double are mutually exclusive, so no input lockout is needed. */
    const ButtonEvent ev = Button_GetEvent(BTN_BLS);

    switch (s_state) {
        case STATE_IDLE:
            if (ev == BTN_EVENT_SINGLE_CLICK) {
                start_task(now_ms);
            } else if (ev == BTN_EVENT_DOUBLE_CLICK) {
                s_mode = (CompetitionQuestion)((s_mode + 1) % COMP_MODE_COUNT);
            }
            break;

        case STATE_RUNNING:
            s_elapsed_ms = now_ms - s_start_ms;

            LineFollow_Update(now_ms);
            BalanceTask_Update(now_ms);

            if (LineFollow_IsComplete()) {
                stop_task();
            } else if (s_mode == COMP_Q3 && BalanceTask_IsComplete()) {
                stop_task();
            } else if (ev == BTN_EVENT_SINGLE_CLICK) {
                /* Manual stop */
                stop_task();
            }
            break;

        case STATE_DONE:
            /* Clear the result and return to idle */
            if (ev == BTN_EVENT_SINGLE_CLICK) {
                s_state = STATE_IDLE;
            }
            break;
    }
}
