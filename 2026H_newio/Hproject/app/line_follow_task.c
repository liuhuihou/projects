#include "line_follow_task.h"
#include "app_config.h"
#include "vehicle_controller.h"
#include "line_sensor.h"

typedef enum {
    LF_IDLE = 0,
    LF_STARTING,       /* Initial phase, ignore stop line */
    LF_CRUISING,       /* Normal line following */
    LF_STOPPING,       /* Deceleration before stop */
    LF_COMPLETE
} LfState;

static LineFollowMode s_mode;
static LfState s_lf_state;
static float s_target_distance_cm;
static uint8_t s_stop_line_seen;
static uint8_t s_stop_line_ticks;
static uint8_t s_stop_window_mask;
static uint8_t s_stop_window_ticks;
static uint8_t s_stop_window_has_wide_sample;
static uint16_t s_start_ignore_ticks;
static uint32_t s_last_update_ms;
static uint32_t s_brake_start_ms;

static uint8_t count_active_sensors(uint8_t state)
{
    uint8_t count = 0U;
    /* Derived from LINE_SENSOR_COUNT rather than a literal 0x3F, so the two
     * extra channels on an 8-channel board are actually counted instead of
     * being masked away - the stop-bar detector reads this count directly. */
    uint8_t bits = (uint8_t)(state & LINE_SENSOR_ALL_MASK);

    while (bits != 0U) {
        count = (uint8_t)(count + (bits & 1U));
        bits >>= 1U;
    }
    return count;
}

static void reset_stop_line_detector(void)
{
    s_stop_line_ticks = 0U;
    s_stop_window_mask = 0U;
    s_stop_window_ticks = 0U;
    s_stop_window_has_wide_sample = 0U;
}

static void start_braking(uint32_t now_ms)
{
    /* Control_SetMode applies the explicit short brake immediately, in the
     * same update that recognizes the stop line. */
    Control_SetMode(CTRL_STOP);
    s_brake_start_ms = now_ms;
    s_lf_state = LF_STOPPING;
}

void LineFollow_Init(void)
{
    s_mode = LFMODE_NONE;
    s_lf_state = LF_IDLE;
    s_target_distance_cm = 0.0f;
    s_stop_line_seen = 0;
    reset_stop_line_detector();
    s_start_ignore_ticks = 0U;
    s_last_update_ms = 0U;
    s_brake_start_ms = 0U;
}

void LineFollow_Start(LineFollowMode mode)
{
    s_mode = mode;
    s_lf_state = LF_STARTING;
    s_stop_line_seen = 0;
    reset_stop_line_detector();
    s_start_ignore_ticks = 0U;
    s_last_update_ms = 0U;
    s_brake_start_ms = 0U;
    Control_ResetDistance();

    if (mode == LFMODE_FULL_LAP) {
        s_target_distance_cm = APP_TRACK_LENGTH_CM;
    } else if (mode == LFMODE_A_TO_B) {
        s_target_distance_cm = APP_TRACK_AB_LENGTH_CM;
    } else {
        s_target_distance_cm = 0.0f;
    }
}

float LineFollow_GetDistanceCm(void)
{
    int32_t left = Control_GetLeftDistance();
    int32_t right = Control_GetRightDistance();
    float avg = (float)(left + right) / 2.0f;
    return avg * APP_CM_PER_COUNT;
}

uint8_t LineFollow_IsComplete(void)
{
    return (s_lf_state == LF_COMPLETE) ? 1U : 0U;
}
void LineFollow_Update(uint32_t now_ms)
{
    uint8_t line_state;

    if (s_lf_state == LF_IDLE || s_lf_state == LF_COMPLETE) return;

    /* app_main may wake on encoder and UART interrupts many times during the
     * same 10 ms control tick. Process the task only once per time-base tick
     * so confirmation counters represent real, independent sensor samples. */
    if (now_ms == s_last_update_ms) return;
    s_last_update_ms = now_ms;

    /* Stop-bar timing is determined by its 1.8 +/- 0.2 cm line width:
     * only about 46..57 ms at the Q2 speed. Use raw inputs here so the
     * three-tick steering filter does not consume most of that interval.
     * Vehicle steering still uses the filtered LineSensor_Read() value. */
    line_state = LineSensor_ReadRaw();

    switch (s_lf_state) {
        case LF_STARTING:
            if (s_mode == LFMODE_FULL_LAP) {
                /* Q2/Q5/Q6 do not depend on wheel odometry for parking.
                 * Ignore the starting bar for one second, then arm the
                 * position-independent stop-line detector. */
                if (s_start_ignore_ticks < APP_STOP_IGNORE_TICKS) {
                    ++s_start_ignore_ticks;
                }
                if (s_start_ignore_ticks >= APP_STOP_IGNORE_TICKS) {
                    s_lf_state = LF_CRUISING;
                }
            } else if (LineFollow_GetDistanceCm() >
                       APP_STOP_IGNORE_DISTANCE) {
                /* A-to-B mode still uses distance measurement. */
                s_lf_state = LF_CRUISING;
            }
            break;

        case LF_CRUISING:
            if (s_mode == LFMODE_FULL_LAP) {
                /* Detect a wide stop bar independently of lateral position.
                 * A direct hit keeps any three channels active for two
                 * samples. A skewed hit may move across the array, so also
                 * accumulate channel coverage over a 60 ms window. */
                const uint8_t active = count_active_sensors(line_state);
                uint8_t stop_line_detected = 0U;

                if (active >= APP_STOP_LINE_MIN_INSTANT_CHANNELS) {
                    if (s_stop_line_ticks < APP_STOP_LINE_CONFIRM_TICKS) {
                        ++s_stop_line_ticks;
                    }
                    s_stop_window_has_wide_sample = 1U;
                } else {
                    s_stop_line_ticks = 0U;
                }

                if (s_stop_window_ticks == 0U) {
                    if (active >= 2U) {
                        s_stop_window_mask = line_state;
                        s_stop_window_ticks = 1U;
                        s_stop_window_has_wide_sample =
                            (active >= APP_STOP_LINE_MIN_INSTANT_CHANNELS) ?
                            1U : 0U;
                    }
                } else if (active == 0U) {
                    reset_stop_line_detector();
                } else {
                    s_stop_window_mask |= line_state;
                    ++s_stop_window_ticks;
                }

                if (s_stop_line_ticks >= APP_STOP_LINE_CONFIRM_TICKS) {
                    stop_line_detected = 1U;
                } else if (s_stop_window_has_wide_sample != 0U &&
                           count_active_sensors(s_stop_window_mask) >=
                               APP_STOP_LINE_WINDOW_CHANNELS) {
                    stop_line_detected = 1U;
                }

                if (stop_line_detected != 0U) {
                    s_stop_line_seen = 1U;
                    start_braking(now_ms);
                } else if (s_stop_window_ticks >=
                           APP_STOP_LINE_WINDOW_TICKS) {
                    reset_stop_line_detector();
                }
            } else if (s_mode == LFMODE_A_TO_B) {
                /* For A->B, stop based on distance */
                if (LineFollow_GetDistanceCm() >=
                    s_target_distance_cm * 0.95f) {
                    start_braking(now_ms);
                }
            }
            break;

        case LF_STOPPING:
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
