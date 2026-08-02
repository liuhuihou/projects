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
#include "board_hardware.h"
#include "camera_uart.h"

static CompetitionQuestion s_mode;
static CompetitionState s_state;
static uint32_t s_start_ms;
static uint32_t s_elapsed_ms;
static uint32_t s_level_start_ms;
static uint32_t s_level_stable_since_ms;
static uint8_t s_level_stable;
static uint8_t s_q6_ball_hold_active;
static int32_t s_q6_position_sum_mm;
static uint8_t s_q6_position_sample_count;
static uint8_t s_q6_last_sample_seq;
static uint8_t s_q6_have_sample_seq;

#if APP_Q6_TARGET_SAMPLE_FRAMES == 0U || APP_Q6_TARGET_SAMPLE_FRAMES > 255U
#error "APP_Q6_TARGET_SAMPLE_FRAMES must be in the range 1..255"
#endif

static void reset_q6_ball_capture(void)
{
    s_q6_ball_hold_active = 0U;
    s_q6_position_sum_mm = 0;
    s_q6_position_sample_count = 0U;
    s_q6_last_sample_seq = 0U;
    s_q6_have_sample_seq = 0U;
}

/* The C07A core-board LED and the S28A baseboard LED share PB9 and are both
 * active LOW.  Their red/blue light can reflect from the steel ball, so keep
 * both lamps off in every mode that uses camera ball feedback.  Q2 retains the
 * original illuminated status indication. */
static void update_status_led_for_mode(CompetitionQuestion mode)
{
    if (mode == COMP_Q2) {
        HW_GPIO_LOW(HW_STATUS_LED_PORT, HW_STATUS_LED_PIN);
    } else {
        HW_GPIO_HIGH(HW_STATUS_LED_PORT, HW_STATUS_LED_PIN);
    }
}

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

static uint8_t mode_requires_initial_level(CompetitionQuestion q)
{
    return (q == COMP_Q3 || q == COMP_Q4 ||
            q == COMP_Q5 || q == COMP_Q6) ? 1U : 0U;
}

void Competition_Init(void)
{
    s_mode = COMP_Q2;
    s_state = STATE_IDLE;
    s_start_ms = 0;
    s_elapsed_ms = 0;
    s_level_start_ms = 0U;
    s_level_stable_since_ms = 0U;
    s_level_stable = 0U;
    reset_q6_ball_capture();
    update_status_led_for_mode(s_mode);
}

CompetitionQuestion Competition_GetMode(void) { return s_mode; }
CompetitionState Competition_GetState(void) { return s_state; }
uint32_t Competition_GetElapsedMs(void) { return s_elapsed_ms; }
uint8_t Competition_IsQ6BallHoldActive(void)
{
    return (s_mode == COMP_Q6 && s_q6_ball_hold_active != 0U) ? 1U : 0U;
}

static void start_initial_level(uint32_t now_ms)
{
    if (mode_requires_initial_level(s_mode) == 0U) return;

    /* AB was initialised to zero at the repeatable power-on pose.  Keep the
     * vehicle and camera outer loop inactive while a small, fixed step rate
     * moves the mechanism to the calibrated horizontal count. */
    Control_SetMode(CTRL_STOP);
    Balance_Disable();
    StepperFeedback_SetTravelLimits(APP_LEVEL_START_AB_MIN,
                                    BALANCE_TRAVEL_AB_MAX);
    Stepper_ResetPosition();
    Stepper_Enable();
    Stepper_SetSpeed(0);

    s_level_start_ms = now_ms;
    s_level_stable_since_ms = 0U;
    s_level_stable = 0U;
    reset_q6_ball_capture();
    s_elapsed_ms = 0U;
    s_state = STATE_LEVELING;
}

static void cancel_initial_level(void)
{
    Stepper_SetSpeed(0);
    Stepper_Disable();
    StepperFeedback_ClearTravelLimits();
    s_level_stable = 0U;
    reset_q6_ball_capture();
    s_state = STATE_IDLE;
}

static void update_initial_level(uint32_t now_ms)
{
    StepperFeedbackSnapshot feedback;
    int32_t error;
    int32_t speed_cmd;

    if ((uint32_t)(now_ms - s_level_start_ms) >= APP_LEVEL_TIMEOUT_MS) {
        cancel_initial_level();
        return;
    }

    StepperFeedback_GetSnapshot(&feedback);
    error = BALANCE_LEVEL_AB_COUNT - feedback.quadrature_count;

    if (error >= -APP_LEVEL_TOLERANCE_AB &&
        error <= APP_LEVEL_TOLERANCE_AB) {
        Stepper_SetSpeed(0);
        if (s_level_stable == 0U) {
            s_level_stable = 1U;
            s_level_stable_since_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_level_stable_since_ms) >=
                   APP_LEVEL_STABLE_MS) {
            /* Remove the final tolerance error and make 270 the exact datum
             * used by the normal visual balance loop.  Leave EN active so the
             * tube stays locked while waiting for the start click. */
            StepperFeedback_ResetCountsAt(BALANCE_LEVEL_AB_COUNT);
            StepperFeedback_SetTravelLimits(BALANCE_TRAVEL_AB_MIN,
                                            BALANCE_TRAVEL_AB_MAX);
            Stepper_ResetPosition();
            Stepper_Enable();
            s_state = STATE_READY;
        }
        return;
    }

    s_level_stable = 0U;
    speed_cmd = (error > 0) ? APP_LEVEL_SPEED_STEPS_S :
                              -APP_LEVEL_SPEED_STEPS_S;
    speed_cmd *= BALANCE_MOTOR_COMMAND_SIGN;
    if (StepperFeedback_IsTravelCommandAllowed(speed_cmd) == 0U) {
        speed_cmd = 0;
    }
    Stepper_SetSpeed(speed_cmd);
}

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
            Control_SetCurveExitSyncEnabled(0U);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetStartRamp(0U);
            LineFollow_SetStopRamp(0U);
            Control_SetMode(CTRL_LINE);
            LineFollow_Start(LFMODE_FULL_LAP);
            break;

        case COMP_Q3:
            /* Initial levelling has already placed and locked the tube at the
             * calibrated horizontal count.  Reassert that datum, then enter
             * the -50 mm -> +50 mm visual-control sequence. */
            Control_SetMode(CTRL_STOP);
            Balance_Disable();
            StepperFeedback_ClearTravelLimits();
            StepperFeedback_ResetCountsAt(BALANCE_LEVEL_AB_COUNT);
            StepperFeedback_SetTravelLimits(BALANCE_TRAVEL_AB_MIN,
                                            BALANCE_TRAVEL_AB_MAX);
            Stepper_ResetPosition();
            Balance_SelectPidProfile(BALANCE_PID_PROFILE_Q3);
            BalanceTask_Start(BTASK_STATIC_MOVE);
            break;

        case COMP_Q4:
            /* Q4 line run with balance; final stopped distance is 1.80 m. */
            speed_rpm = mode_speed_cm_s(COMP_Q4) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q2_Q4);
            Control_SetCurveExitSyncEnabled(0U);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetStartRamp(APP_START_RAMP_Q4_MS);
            LineFollow_SetStopRamp(APP_STOP_RAMP_Q4_MS);
            LineFollow_Start(LFMODE_Q4_DISTANCE_STOP);
            Balance_SelectPidProfile(BALANCE_PID_PROFILE_Q4);
            BalanceTask_Start(BTASK_HOLD_CENTER);
            Control_SetMode(CTRL_LINE);
            break;

        case COMP_Q5:
            /* Full lap + ball center; odometry stop at 110% lap */
            speed_rpm = mode_speed_cm_s(COMP_Q5) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q5_Q6);
            Control_SetCurveExitSyncEnabled(1U);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetStartRamp(APP_START_RAMP_Q5_MS);
            LineFollow_SetStopRamp(APP_STOP_RAMP_Q5_MS);
            LineFollow_Start(LFMODE_Q5_Q6_DISTANCE_STOP);
            Balance_SelectPidProfile(BALANCE_PID_PROFILE_Q5);
            BalanceTask_Start(BTASK_HOLD_CENTER);
            Control_SetMode(CTRL_LINE);
            break;

        case COMP_Q6:
            /* Full lap + ball held at the position captured after levelling;
             * 110% odometry stop. */
            speed_rpm = mode_speed_cm_s(COMP_Q6) * 60.0f / APP_WHEEL_CIRCUMFERENCE_CM;
            Control_SetLineProfile(CTRL_LINE_PROFILE_Q5_Q6);
            Control_SetCurveExitSyncEnabled(1U);
            Control_SetBaseSpeed(speed_rpm);
            Control_SetStartRamp(APP_START_RAMP_Q6_MS);
            LineFollow_SetStopRamp(APP_STOP_RAMP_Q6_MS);
            LineFollow_Start(LFMODE_Q5_Q6_DISTANCE_STOP);
            Balance_SelectPidProfile(BALANCE_PID_PROFILE_Q6);
            BalanceTask_Start(BTASK_HOLD_POSITION);
            Control_SetMode(CTRL_LINE);
            break;

        default:
            break;
    }
}

static void stop_task(void)
{
    Control_SetMode(CTRL_STOP);
    Balance_Disable();
    reset_q6_ball_capture();
    s_state = STATE_DONE;
}

static void finish_vehicle_task_with_balance(void)
{
    /* The wheel controller has already completed its ramp and short brake.
     * Keep BalanceTask and the stepper driver enabled in DONE so the ball is
     * still actively corrected after the chassis has stopped. */
    Control_SetMode(CTRL_STOP);
    s_state = STATE_DONE;
}

/* Q6 is armed in two stages.  The tube remains horizontal and the vehicle
 * remains stopped until the camera supplies enough real ball detections. The
 * average of those first frames is the "original position" for this run. */
static void update_q6_ready(uint32_t now_ms, ButtonEvent ev)
{
    const CameraData *cam = Camera_GetData();
    const uint8_t usable =
        Camera_IsBallUsable(now_ms, BALANCE_DATA_TIMEOUT_MS);
    const uint8_t was_active = s_q6_ball_hold_active;

    if (usable == 0U) {
        reset_q6_ball_capture();
        if (Balance_IsEnabled() != 0U) {
            /* A lost ball must not leave the last tilted command latched. */
            Balance_Tick(now_ms);
        }
        return;
    }

    if (s_q6_ball_hold_active != 0U) {
        BalanceTask_Update(now_ms);
    } else if ((cam->flags & CAMERA_FLAG_DETECTED) == 0U) {
        /* Predicted frames are useful after PID starts, but the reference
         * position must be based only on real ball detections. */
        s_q6_position_sum_mm = 0;
        s_q6_position_sample_count = 0U;
        s_q6_have_sample_seq = 0U;
    } else if (s_q6_have_sample_seq == 0U ||
               cam->seq != s_q6_last_sample_seq) {
        int32_t average_mm;

        s_q6_last_sample_seq = cam->seq;
        s_q6_have_sample_seq = 1U;
        s_q6_position_sum_mm += (int32_t)cam->ball_pos_mm;
        ++s_q6_position_sample_count;

        if (s_q6_position_sample_count >= APP_Q6_TARGET_SAMPLE_FRAMES) {
            const int32_t half_count =
                (int32_t)s_q6_position_sample_count / 2;
            if (s_q6_position_sum_mm >= 0) {
                average_mm = (s_q6_position_sum_mm + half_count) /
                             (int32_t)s_q6_position_sample_count;
            } else {
                average_mm = (s_q6_position_sum_mm - half_count) /
                             (int32_t)s_q6_position_sample_count;
            }

            Balance_SelectPidProfile(BALANCE_PID_PROFILE_Q6);
            BalanceTask_SetPosition((int16_t)average_mm);
            BalanceTask_Start(BTASK_HOLD_POSITION);
            s_q6_ball_hold_active = 1U;
        }
    }

    /* Do not treat an event from the sampling cycle as the start command. The
     * operator must press once after the averaged target is armed. */
    if (was_active != 0U && ev == BTN_EVENT_SINGLE_CLICK) {
        start_task(now_ms);
    }
}
void Competition_ForceStop(void)
{
    Control_SetMode(CTRL_STOP);
    if ((s_mode == COMP_Q4 || s_mode == COMP_Q5 || s_mode == COMP_Q6) &&
        Balance_IsEnabled() != 0U) {
        finish_vehicle_task_with_balance();
    } else {
        stop_task();
    }
}

void Competition_Update(uint32_t now_ms)
{
    /* Button_GetEvent reports at most one event per press sequence and
     * single/double are mutually exclusive, so no input lockout is needed. */
    const ButtonEvent ev = Button_GetEvent(BTN_BLS);

    switch (s_state) {
        case STATE_IDLE:
            if (ev == BTN_EVENT_DOUBLE_CLICK) {
                s_mode = (CompetitionQuestion)((s_mode + 1) % COMP_MODE_COUNT);
                reset_q6_ball_capture();
                update_status_led_for_mode(s_mode);
            } else if (ev == BTN_EVENT_LONG_PRESS &&
                       mode_requires_initial_level(s_mode) != 0U) {
                start_initial_level(now_ms);
            } else if (ev == BTN_EVENT_SINGLE_CLICK &&
                       mode_requires_initial_level(s_mode) == 0U) {
                start_task(now_ms);
            }
            break;

        case STATE_LEVELING:
            if (ev == BTN_EVENT_SINGLE_CLICK) {
                /* A short click is an explicit levelling abort. */
                cancel_initial_level();
            } else {
                update_initial_level(now_ms);
            }
            break;

        case STATE_READY:
            if (s_mode == COMP_Q6) {
                update_q6_ready(now_ms, ev);
            } else if (ev == BTN_EVENT_SINGLE_CLICK) {
                start_task(now_ms);
            }
            break;

        case STATE_RUNNING:
            s_elapsed_ms = now_ms - s_start_ms;

            if (s_mode == COMP_Q3) {
                BalanceTask_Update(now_ms);
                if (BalanceTask_IsComplete()) {
                    s_state = STATE_DONE;
                }
            } else {
                LineFollow_Update(now_ms);
                BalanceTask_Update(now_ms);
            }

            if (s_mode != COMP_Q3 && LineFollow_IsComplete()) {
                if (s_mode == COMP_Q4 || s_mode == COMP_Q5 ||
                    s_mode == COMP_Q6) {
                    finish_vehicle_task_with_balance();
                } else {
                    stop_task();
                }
            } else if (ev == BTN_EVENT_SINGLE_CLICK) {
                /* Q4-Q6 manual stops use the same controlled deceleration as
                 * their automatic endpoint. Q2/Q3 retain an immediate stop. */
                if (s_mode == COMP_Q4 || s_mode == COMP_Q5 ||
                    s_mode == COMP_Q6) {
                    LineFollow_RequestStop(now_ms);
                } else {
                    stop_task();
                }
            }
            break;

        case STATE_DONE:
            if ((s_mode == COMP_Q3 || s_mode == COMP_Q4 ||
                 s_mode == COMP_Q5 || s_mode == COMP_Q6) &&
                Balance_IsEnabled() != 0U) {
                BalanceTask_Update(now_ms);
            }
            /* Clear the result and return to idle. */
            if (ev == BTN_EVENT_SINGLE_CLICK) {
                if (s_mode == COMP_Q3 || s_mode == COMP_Q4 ||
                    s_mode == COMP_Q5 || s_mode == COMP_Q6) {
                    Balance_Disable();
                }
                reset_q6_ball_capture();
                s_state = STATE_IDLE;
            }
            break;
    }
}
