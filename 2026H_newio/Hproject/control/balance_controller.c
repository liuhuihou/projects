#include "balance_controller.h"
#include "control_config.h"
#include "camera_uart.h"
#include "stepper_driver.h"

static int16_t s_target_mm;
static int16_t s_error;
static float s_integral;
static float s_prev_error;
static uint8_t s_enabled;

void Balance_Init(void)
{
    s_target_mm = 0;
    s_error = 0;
    s_integral = 0.0f;
    s_prev_error = 0.0f;
    s_enabled = 0;
}

void Balance_SetTarget(int16_t target_mm)
{
    s_target_mm = target_mm;
}

int16_t Balance_GetTarget(void) { return s_target_mm; }
int16_t Balance_GetError(void) { return s_error; }
uint8_t Balance_IsEnabled(void) { return s_enabled; }

void Balance_Enable(void)
{
    s_enabled = 1;
    s_integral = 0.0f;
    s_prev_error = 0.0f;
    Stepper_Enable();
}

void Balance_Disable(void)
{
    s_enabled = 0;
    Stepper_SetSpeed(0);
    Stepper_Disable();
}

void Balance_Tick(uint32_t now_ms)
{
    const CameraData *cam;
    float error_f, derivative, output;
    int32_t speed_cmd;

    if (!s_enabled) return;

    cam = Camera_GetData();
    if (!Camera_IsDataFresh(now_ms, 100U)) {
        /* No valid camera data - hold position */
        Stepper_SetSpeed(0);
        return;
    }
    /* PID calculation */
    s_error = s_target_mm - cam->ball_pos_mm;
    error_f = (float)s_error;

    /* Integral with anti-windup */
    s_integral += error_f;
    if (s_integral > BALANCE_INTEGRAL_LIMIT) s_integral = BALANCE_INTEGRAL_LIMIT;
    if (s_integral < -BALANCE_INTEGRAL_LIMIT) s_integral = -BALANCE_INTEGRAL_LIMIT;

    /* Derivative */
    derivative = error_f - s_prev_error;
    s_prev_error = error_f;

    /* PID output -> stepper speed */
    output = BALANCE_KP * error_f
           + BALANCE_KI * s_integral
           + BALANCE_KD * derivative;

    /* Clamp output */
    if (output > (float)BALANCE_OUTPUT_LIMIT) output = (float)BALANCE_OUTPUT_LIMIT;
    if (output < -(float)BALANCE_OUTPUT_LIMIT) output = -(float)BALANCE_OUTPUT_LIMIT;

    speed_cmd = (int32_t)output;
    Stepper_SetSpeed(speed_cmd);
}
