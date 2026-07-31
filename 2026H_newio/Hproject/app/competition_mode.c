#include "competition_mode.h"
#include "app_config.h"
#include "button_input.h"
#include "vehicle_controller.h"
#include "balance_controller.h"
#include "line_follow_task.h"
#include "balance_task.h"
#include "oled_driver.h"
#include "control_config.h"
#include "stepper_driver.h"
#include "stepper_feedback.h"

#define Q3_LEVEL_SPEED_STEPS_S      (67)
#define Q3_LEVEL_TIMEOUT_MS         (10000U)

static CompetitionQuestion s_mode;
static CompetitionState s_state;
static uint32_t s_start_ms;
static uint32_t s_elapsed_ms;
static uint8_t s_q3_level_complete;

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
    s_q3_level_complete = 0U;
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

    Control_ResetDistance();

    switch (s_mode) {
        case COMP_Q2:
            /* Pure line follow, fast lap */
            speed_rpm = mode_speed_cm_s(COMP_Q2) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q2_Q4);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_FULL_LAP);
            break;

        case COMP_Q3:
            /* The mechanism is manually placed at its lowest point before
             * starting Q3. Define that point as AB=0, then move upward until
             * the measured horizontal position AB=330 is reached. */
            Control_SetMode(CTRL_STOP);
            Balance_Disable();
            StepperFeedback_ClearTravelLimits();
            StepperFeedback_ResetCounts();
            Stepper_Enable();
            Stepper_ResetPosition();
            s_q3_level_complete = 0U;
            Stepper_SetSpeed(Q3_LEVEL_SPEED_STEPS_S);
            break;

        case COMP_Q4:
            /* Line A->B with balance; odometry stop at 1.6 m */
            speed_rpm = mode_speed_cm_s(COMP_Q4) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q2_Q4);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_Q4_DISTANCE_STOP);
            BalanceTask_Start(BTASK_HOLD_CENTER);
            break;

        case COMP_Q5:
            /* Full lap + ball center; odometry stop at 110% lap */
            speed_rpm = mode_speed_cm_s(COMP_Q5) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q5_Q6);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_Q5_Q6_DISTANCE_STOP);
            BalanceTask_Start(BTASK_HOLD_CENTER);
            break;

        case COMP_Q6:
            /* Full lap + ball at designated position; 110% odometry stop */
            speed_rpm = mode_speed_cm_s(COMP_Q6) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q5_Q6);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_Q5_Q6_DISTANCE_STOP);
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

            if (s_mode == COMP_Q3) {
                StepperFeedbackSnapshot feedback;
                StepperFeedback_GetSnapshot(&feedback);

                if (s_q3_level_complete == 0U) {
                    if (feedback.quadrature_count >= BALANCE_LEVEL_AB_COUNT) {
                        Stepper_SetSpeed(0);
                        Stepper_Enable();
                        StepperFeedback_SetTravelLimits(BALANCE_TRAVEL_AB_MIN,
                                                        BALANCE_TRAVEL_AB_MAX);
                        BalanceTask_Start(BTASK_STATIC_MOVE);
                        /* Leveling is a setup stage. Start the Q3 <=5 s task
                         * clock only after AB has reached horizontal. */
                        s_start_ms = now_ms;
                        s_elapsed_ms = 0U;
                        s_q3_level_complete = 1U;
                    } else if (s_elapsed_ms >= Q3_LEVEL_TIMEOUT_MS) {
                        Stepper_SetSpeed(0);
                        Stepper_Disable();
                        s_state = STATE_DONE;
                    } else {
                        Stepper_SetSpeed(Q3_LEVEL_SPEED_STEPS_S);
                    }
                } else {
                    BalanceTask_Update(now_ms);
                    if (BalanceTask_IsComplete()) {
                        s_state = STATE_DONE;
                    }
                }
            } else {
                LineFollow_Update(now_ms);
                BalanceTask_Update(now_ms);
            }

            if (s_mode != COMP_Q3 && LineFollow_IsComplete()) {
                stop_task();
            } else if (ev == BTN_EVENT_SINGLE_CLICK) {
                /* Manual stop */
                stop_task();
            }
            break;

        case STATE_DONE:
            if (s_mode == COMP_Q3 && s_q3_level_complete != 0U) {
                BalanceTask_Update(now_ms);
            }
            /* Clear the result and return to idle. */
            if (ev == BTN_EVENT_SINGLE_CLICK) {
                if (s_mode == COMP_Q3) Balance_Disable();
                s_state = STATE_IDLE;
            }
            break;
    }
}
