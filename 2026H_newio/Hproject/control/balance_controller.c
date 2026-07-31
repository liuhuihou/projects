#include "balance_controller.h"
#include "control_config.h"
#include "camera_uart.h"
#include "stepper_driver.h"

static int16_t s_target_mm;
static int16_t s_error;
static float s_integral;
static uint8_t s_enabled;
static uint8_t s_tracking;
static int16_t s_output;

#if !BALANCE_USE_CAMERA_VELOCITY
/* Derivative state, only needed when differencing position here. The camera
 * velocity path is stateless - vel is already a rate.
 *
 * s_have_prev guards the first tick after enabling: differencing against
 * s_prev_error = 0 treats a ball sitting at one end as a full-scale rate and
 * kicks the stepper before the P term takes over.
 *
 * SEQ and the frame timestamp are kept so the difference is taken per frame at
 * the true frame interval, not once per tick. */
static uint8_t s_have_prev;
static float s_prev_error;
static float s_prev_derivative;
static uint8_t s_prev_seq;
static uint32_t s_prev_frame_ms;

static void derivative_reset(void)
{
    s_have_prev = 0;
    s_prev_error = 0.0f;
    s_prev_derivative = 0.0f;
    s_prev_seq = 0;
    s_prev_frame_ms = 0;
}
#else
#define derivative_reset() ((void)0)
#endif

void Balance_Init(void)
{
    s_target_mm = 0;
    s_error = 0;
    s_integral = 0.0f;
    s_enabled = 0;
    s_tracking = 0;
    s_output = 0;
    derivative_reset();
}

void Balance_SetTarget(int16_t target_mm)
{
    s_target_mm = target_mm;
}

int16_t Balance_GetTarget(void) { return s_target_mm; }
int16_t Balance_GetError(void) { return s_error; }
uint8_t Balance_IsEnabled(void) { return s_enabled; }
uint8_t Balance_IsTracking(void) { return s_tracking; }
int16_t Balance_GetOutput(void) { return s_output; }

void Balance_Enable(void)
{
    s_enabled = 1;
    s_integral = 0.0f;
    derivative_reset();
    Stepper_Enable();
}

void Balance_Disable(void)
{
    s_enabled = 0;
    s_tracking = 0;
    s_output = 0;
    Stepper_SetSpeed(0);
    Stepper_Disable();
}

void Balance_Tick(uint32_t now_ms)
{
    const CameraData *cam;
    float error_f, derivative, output;
    int32_t speed_cmd;

    if (!s_enabled) return;

    /* Gate on the position being usable, not just on the link being alive.
     * Camera_IsDataFresh() is true whenever frames keep arriving, and they
     * arrive even with the ball out of view or the ruler unlocked - driving the
     * PID from those would chase a position that is not in millimetres. */
    cam = Camera_GetData();
    if (!Camera_IsBallUsable(now_ms, BALANCE_DATA_TIMEOUT_MS)) {
        s_tracking = 0;
        s_output = 0;
        /* Drop the integral rather than freezing it. Whatever charge it holds
         * was accumulated against a position we can no longer see, and the ball
         * is free-rolling while we are blind, so on reacquisition it would be
         * unwinding a correction for a state that no longer exists. */
        s_integral = 0.0f;
        derivative_reset();
        Stepper_SetSpeed(0);
        return;
    }

    s_error = (int16_t)(s_target_mm - cam->ball_pos_mm);
    error_f = (float)s_error;

    /* Integral with anti-windup. Accumulating the raw error once per tick makes
     * the effective gain KI/BALANCE_PERIOD_MS; that is how the current tuning
     * was arrived at, so it is left alone. */
    s_integral += error_f;
    if (s_integral > BALANCE_INTEGRAL_LIMIT) s_integral = BALANCE_INTEGRAL_LIMIT;
    if (s_integral < -BALANCE_INTEGRAL_LIMIT) s_integral = -BALANCE_INTEGRAL_LIMIT;

#if BALANCE_USE_CAMERA_VELOCITY
    /* error = target - pos, so d(error)/dt = -velocity. Scale from mm/s into
     * "mm per tick" so KD keeps the same meaning it had when the derivative was
     * a per-tick difference. */
    derivative = -((float)cam->ball_vel_mm_s)
                 * ((float)BALANCE_PERIOD_MS / 1000.0f);
#else
    /* Fallback: difference position, but only when the frame actually changed.
     * Recomputing every tick gave a real delta on ticks that followed a new
     * frame and exactly zero on the rest, so KD saw a square wave rather than a
     * rate. Normalising by the true frame interval keeps the units the same as
     * the velocity branch above. */
    if (!s_have_prev) {
        derivative = 0.0f;
        s_have_prev = 1;
        s_prev_error = error_f;
        s_prev_seq = cam->seq;
        s_prev_frame_ms = cam->last_update_ms;
    } else if (cam->seq != s_prev_seq) {
        uint32_t dt_ms = (uint32_t)(cam->last_update_ms - s_prev_frame_ms);
        if (dt_ms == 0U) {
            derivative = 0.0f;
        } else {
            derivative = (error_f - s_prev_error)
                         * ((float)BALANCE_PERIOD_MS / (float)dt_ms);
        }
        s_prev_error = error_f;
        s_prev_seq = cam->seq;
        s_prev_frame_ms = cam->last_update_ms;
    } else {
        /* Same frame as last tick. Reuse the rate computed when the frame
         * arrived rather than reporting zero - the ball did not stop moving
         * just because no new measurement landed in this 20 ms window. */
        derivative = s_prev_derivative;
    }
    s_prev_derivative = derivative;
#endif

    output = BALANCE_KP * error_f
           + BALANCE_KI * s_integral
           + BALANCE_KD * derivative;

    if (output > (float)BALANCE_OUTPUT_LIMIT) output = (float)BALANCE_OUTPUT_LIMIT;
    if (output < -(float)BALANCE_OUTPUT_LIMIT) output = -(float)BALANCE_OUTPUT_LIMIT;

    speed_cmd = (int32_t)output;
    s_output = (int16_t)speed_cmd;
    s_tracking = 1;
    Stepper_SetSpeed(speed_cmd);
}
