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

void LineFollow_Init(void)
{
    s_mode = LFMODE_NONE;
    s_lf_state = LF_IDLE;
    s_target_distance_cm = 0.0f;
    s_stop_line_seen = 0;
}

void LineFollow_Start(LineFollowMode mode)
{
    s_mode = mode;
    s_lf_state = LF_STARTING;
    s_stop_line_seen = 0;
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
    float distance_cm;
    uint8_t line_state;

    (void)now_ms;

    if (s_lf_state == LF_IDLE || s_lf_state == LF_COMPLETE) return;

    distance_cm = LineFollow_GetDistanceCm();
    line_state = LineSensor_Read();

    switch (s_lf_state) {
        case LF_STARTING:
            /* Ignore stop line detection until we've traveled enough */
            if (distance_cm > APP_STOP_IGNORE_DISTANCE) {
                s_lf_state = LF_CRUISING;
            }
            break;

        case LF_CRUISING:
            if (s_mode == LFMODE_FULL_LAP) {
                /* Detect stop line (all sensors black) near expected distance */
                if (distance_cm > (s_target_distance_cm * 0.8f)) {
                    if (line_state == APP_STOP_LINE_PATTERN) {
                        s_stop_line_seen = 1;
                    }
                    /* Once we've seen stop line and passed it, stop */
                    if (s_stop_line_seen && line_state != APP_STOP_LINE_PATTERN) {
                        s_lf_state = LF_STOPPING;
                    }
                }
                /* Also stop if distance exceeded (safety) */
                if (distance_cm > s_target_distance_cm * 1.1f) {
                    s_lf_state = LF_STOPPING;
                }
            } else if (s_mode == LFMODE_A_TO_B) {
                /* For A->B, stop based on distance */
                if (distance_cm >= s_target_distance_cm * 0.95f) {
                    s_lf_state = LF_STOPPING;
                }
            }
            break;

        case LF_STOPPING:
            /* Immediate stop - let the competition_mode handle deceleration later */
            Control_SetMode(CTRL_STOP);
            s_lf_state = LF_COMPLETE;
            break;

        default:
            break;
    }
}
