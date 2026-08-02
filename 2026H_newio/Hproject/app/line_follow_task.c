#include "line_follow_task.h"
#include "app_config.h"
#include "vehicle_controller.h"
#include "line_sensor.h"
#include "encoder_driver.h"

typedef enum {
    LF_IDLE = 0,
    LF_STARTING,       /* Initial phase, ignore stop line */
    LF_CRUISING,       /* Normal line following */
    LF_DECELERATING,   /* Linear wheel-target ramp to zero */
    LF_BRAKING,        /* Final short brake and rebound settling */
    LF_COMPLETE
} LfState;

static LineFollowMode s_mode;
static LfState s_lf_state;
static uint8_t s_stop_vote_history;
static uint32_t s_last_update_ms;
static uint32_t s_brake_start_ms;
static uint32_t s_last_line_sample_sequence;
static uint32_t s_stop_ramp_duration_ms;

#if (LINE_SENSOR_COUNT != 8U)
#error "Stop-line masks are defined for the current eight-channel sensor"
#endif

static uint8_t is_stop_line_pattern(uint8_t state)
{
    state &= LINE_SENSOR_ALL_MASK;

    if ((state & APP_STOP_PATTERN_3456_MASK) ==
        APP_STOP_PATTERN_3456_MASK) {
        return 1U;
    }
    if ((state & APP_STOP_PATTERN_2345_MASK) ==
        APP_STOP_PATTERN_2345_MASK) {
        return 1U;
    }
    if ((state & APP_STOP_PATTERN_4567_MASK) ==
        APP_STOP_PATTERN_4567_MASK) {
        return 1U;
    }
    return 0U;
}

static uint8_t count_stop_votes(uint8_t history)
{
    uint8_t count = 0U;

    history &= APP_STOP_VOTE_HISTORY_MASK;
    while (history != 0U) {
        count = (uint8_t)(count + (history & 1U));
        history >>= 1U;
    }
    return count;
}

static float get_average_wheel_distance_cm(void)
{
    float left_cm = Encoder_CountsToDistanceCm(
        Control_GetLeftDistance(), APP_WHEEL_CIRCUMFERENCE_CM);
    float right_cm = Encoder_CountsToDistanceCm(
        Control_GetRightDistance(), APP_WHEEL_CIRCUMFERENCE_CM);

    /* Encoder direction depends on motor installation. Distance travelled is
     * unsigned; averaging the two wheels estimates the chassis-centre path
     * and avoids making the shorter inside wheel block detector arming. */
    if (left_cm < 0.0f) left_cm = -left_cm;
    if (right_cm < 0.0f) right_cm = -right_cm;
    return (left_cm + right_cm) * 0.5f;
}

static float get_odometry_stop_distance_cm(LineFollowMode mode)
{
    if (mode == LFMODE_Q4_DISTANCE_STOP) {
        return APP_STOP_Q4_DISTANCE_CM;
    }
    if (mode == LFMODE_Q5_Q6_DISTANCE_STOP) {
        return APP_STOP_Q5_Q6_DISTANCE_CM;
    }
    return 0.0f;
}

static float get_stop_ramp_distance_cm(void)
{
    const float speed_cm_s = Control_GetBaseSpeedRpm() *
                             APP_WHEEL_CIRCUMFERENCE_CM / 60.0f;
    /* A linear speed ramp covers half the distance of constant-speed travel
     * over the same interval. Trigger this far early so the final stopped
     * position remains at the configured odometry target. */
    return speed_cm_s * (float)s_stop_ramp_duration_ms / 2000.0f;
}

static void reset_stop_line_detector(void)
{
    s_stop_vote_history = 0U;
}

static void start_braking(uint32_t now_ms)
{
    if (s_stop_ramp_duration_ms != 0U) {
        Control_StartStopRamp(s_stop_ramp_duration_ms);
        s_lf_state = LF_DECELERATING;
    } else {
        /* Q2 retains its proven immediate short-brake behaviour. */
        Control_SetMode(CTRL_STOP);
        s_brake_start_ms = now_ms;
        s_lf_state = LF_BRAKING;
    }
}

void LineFollow_Init(void)
{
    s_mode = LFMODE_NONE;
    s_lf_state = LF_IDLE;
    reset_stop_line_detector();
    s_last_update_ms = 0U;
    s_brake_start_ms = 0U;
    s_last_line_sample_sequence = 0U;
    s_stop_ramp_duration_ms = 0U;
}


void LineFollow_SetStopRamp(uint32_t duration_ms)
{
    s_stop_ramp_duration_ms = duration_ms;
}

void LineFollow_Start(LineFollowMode mode)
{
    s_mode = mode;
    s_lf_state = LF_STARTING;
    reset_stop_line_detector();
    s_last_update_ms = 0U;
    s_brake_start_ms = 0U;
    s_last_line_sample_sequence = LineSensor_GetSampleSequence();

    if (mode != LFMODE_NONE) {
        /* Every automatic-stop run needs its own zero. Counts collected while
         * idle or by a previous question must not affect this run. */
        Control_ResetDistance();
    }
}

uint8_t LineFollow_IsComplete(void)
{
    return (s_lf_state == LF_COMPLETE) ? 1U : 0U;
}


void LineFollow_RequestStop(uint32_t now_ms)
{
    if (s_lf_state == LF_STARTING || s_lf_state == LF_CRUISING) {
        start_braking(now_ms);
    }
}

void LineFollow_Update(uint32_t now_ms)
{
    if (s_lf_state == LF_IDLE || s_lf_state == LF_COMPLETE) return;

    /* app_main may wake on encoder and UART interrupts many times during the
     * same 10 ms control tick. Process the task only once per time-base tick
     * so confirmation counters represent real, independent sensor samples. */
    if (now_ms == s_last_update_ms) return;
    s_last_update_ms = now_ms;

    switch (s_lf_state) {
        case LF_STARTING:
            if (s_mode == LFMODE_FULL_LAP) {
                /* Odometry only arms stop-line recognition; reaching 80%
                 * alone never brakes the car. Use '>' to match the required
                 * travelled-distance threshold exactly. */
                if (get_average_wheel_distance_cm() >
                    APP_STOP_ODOMETRY_ARM_CM) {
                    reset_stop_line_detector();
                    s_last_line_sample_sequence =
                        LineSensor_GetSampleSequence();
                    s_lf_state = LF_CRUISING;
                }
            } else {
                /* Q4/Q5/Q6 start measuring from zero immediately. Their
                 * configured distance is the stop trigger, rather than an
                 * arming gate for the A-line detector. */
                s_lf_state = LF_CRUISING;
            }
            break;

        case LF_CRUISING:
            if (s_mode == LFMODE_FULL_LAP) {
                const uint32_t sample_sequence =
                    LineSensor_GetSampleSequence();

                /* Poll() holds the last good byte across a few I2C failures.
                 * Only a changed sequence number is an independent sample;
                 * otherwise one stale wide pattern could satisfy all three
                 * confirmation ticks by itself. */
                if (sample_sequence == s_last_line_sample_sequence) {
                    break;
                }
                s_last_line_sample_sequence = sample_sequence;

                s_stop_vote_history = (uint8_t)(
                    ((uint8_t)(s_stop_vote_history << 1U) |
                     is_stop_line_pattern(LineSensor_ReadRaw())) &
                    APP_STOP_VOTE_HISTORY_MASK);

                if (count_stop_votes(s_stop_vote_history) >=
                    APP_STOP_VOTE_REQUIRED) {
                    start_braking(now_ms);
                }
            } else {
                const float stop_distance_cm =
                    get_odometry_stop_distance_cm(s_mode);
                float trigger_distance_cm = stop_distance_cm -
                                            get_stop_ramp_distance_cm();

                if (trigger_distance_cm < 0.0f) {
                    trigger_distance_cm = 0.0f;
                }

                if (stop_distance_cm > 0.0f &&
                    get_average_wheel_distance_cm() >= trigger_distance_cm) {
                    start_braking(now_ms);
                }
            }
            break;

        case LF_DECELERATING:
            if (Control_GetStopRampScale() <= 0.0f) {
                Control_SetMode(CTRL_STOP);
                s_brake_start_ms = now_ms;
                s_lf_state = LF_BRAKING;
            }
            break;

        case LF_BRAKING:
            /* Keep both bridges in short-brake long enough for wheel motion
             * and chassis rebound to settle before reporting completion. */
            if ((uint32_t)(now_ms - s_brake_start_ms) >=
                APP_STOP_BRAKE_HOLD_MS) {
                s_lf_state = LF_COMPLETE;
            }
            break;

        default:
            break;
    }
}
