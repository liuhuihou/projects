#include "line_follow_task.h"

#include "app_config.h"
#include "vehicle_controller.h"

static uint8_t s_active;
static LineFollowProfile s_profile;

void LineFollowTask_Init(void)
{
    s_active = 0U;
    s_profile = LINE_FOLLOW_Q2_ONE_LAP;
}

void LineFollowTask_Start(LineFollowProfile profile)
{
    s_profile = profile;
    s_active = 1U;

    /* The existing vehicle controller remains the line-follow implementation. */
    Control_SetBaseSpeed(APP_START_SPEED_CM_S * 60.0f /
                         APP_WHEEL_CIRCUMFERENCE_CM);
    Control_SetMode(CTRL_LINE);
}

void LineFollowTask_Stop(void)
{
    s_active = 0U;
    Control_SetMode(CTRL_STOP);
}

void LineFollowTask_Tick(uint32_t now_ms)
{
    (void)now_ms;
    (void)s_profile;
    /* Lap detection and stop conditions are deliberately deferred. */
}

uint8_t LineFollowTask_IsActive(void)
{
    return s_active;
}
