#include "balance_controller.h"
#include "control_config.h"
#include "camera_uart.h"
#include "stepper_driver.h"
#include "stepper_feedback.h"

static int16_t s_target_mm;
static int16_t s_error;
static float s_integral;
static uint8_t s_enabled;
static uint8_t s_tracking;
static int16_t s_output;
static int16_t s_ball_velocity_mm_s;
static int32_t s_target_ab;
static uint32_t s_last_tick_ms;
static uint8_t s_tick_started;

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
    s_ball_velocity_mm_s = 0;
    s_target_ab = BALANCE_LEVEL_AB_COUNT;
    s_last_tick_ms = 0U;
    s_tick_started = 0U;
    derivative_reset();
}

void Balance_SetTarget(int16_t target_mm)
{
    if (target_mm != s_target_mm) {
        s_integral = 0.0f;
        derivative_reset();
    }
    s_target_mm = target_mm;
}

int16_t Balance_GetTarget(void) { return s_target_mm; }
int16_t Balance_GetError(void) { return s_error; }
uint8_t Balance_IsEnabled(void) { return s_enabled; }
uint8_t Balance_IsTracking(void) { return s_tracking; }
int16_t Balance_GetOutput(void) { return s_output; }
int16_t Balance_GetBallVelocity(void) { return s_ball_velocity_mm_s; }
int32_t Balance_GetTargetAb(void) { return s_target_ab; }

void Balance_Enable(void)
{
    s_enabled = 1;
    s_integral = 0.0f;
    s_tracking = 0U;
    s_output = 0;
    s_ball_velocity_mm_s = 0;
    s_target_ab = BALANCE_LEVEL_AB_COUNT;
    s_last_tick_ms = 0U;
    s_tick_started = 0U;
    derivative_reset();
    Stepper_Enable();
}

void Balance_Disable(void)
{
    s_enabled = 0;
    s_tracking = 0;
    s_output = 0;
    s_ball_velocity_mm_s = 0;
    s_target_ab = BALANCE_LEVEL_AB_COUNT;
    s_tick_started = 0U;
    Stepper_SetSpeed(0);
    Stepper_Disable();
}

void Balance_Tick(uint32_t now_ms)
{
    const CameraData *cam;
    StepperFeedbackSnapshot feedback;
    float error_f;
    float ball_velocity;
    float outer_output;
    float beam_error;
    float output;
    int32_t speed_cmd;

    if (!s_enabled) return;

    /* Competition_Update() runs from the foreground loop and can be awakened
     * by encoder and UART interrupts much faster than the intended control
     * rate. Keep both loops on a deterministic 50 Hz cadence. */
    if (s_tick_started != 0U &&
        (uint32_t)(now_ms - s_last_tick_ms) < BALANCE_PERIOD_MS) {
        return;
    }
    s_last_tick_ms = now_ms;
    s_tick_started = 1U;

    cam = Camera_GetData();
    if (!Camera_IsBallUsable(now_ms, BALANCE_DATA_TIMEOUT_MS)) {
        s_tracking = 0;
        s_ball_velocity_mm_s = 0;
        s_integral = 0.0f;
        derivative_reset();
        /* With no trustworthy ball position, horizontal is the safest beam
         * command. The AB inner loop remains active so a stale tilted command
         * is not held indefinitely. */
        s_target_ab = BALANCE_LEVEL_AB_COUNT;
    } else {
        {
            int32_t error_wide = (int32_t)s_target_mm -
                                 (int32_t)cam->ball_pos_mm;
            if (error_wide > 32767) error_wide = 32767;
            if (error_wide < -32768) error_wide = -32768;
            s_error = (int16_t)error_wide;
        }
        error_f = (float)s_error;

        s_integral += error_f * ((float)BALANCE_PERIOD_MS / 1000.0f);
        if (s_integral > BALANCE_POSITION_INTEGRAL_LIMIT) {
            s_integral = BALANCE_POSITION_INTEGRAL_LIMIT;
        }
        if (s_integral < -BALANCE_POSITION_INTEGRAL_LIMIT) {
            s_integral = -BALANCE_POSITION_INTEGRAL_LIMIT;
        }

#if BALANCE_USE_CAMERA_VELOCITY
        ball_velocity = (float)cam->ball_vel_mm_s;
#else
        /* Fallback derives velocity only when a new frame arrives, using its
         * real interval. s_prev_derivative stores d(error)/dt, hence the final
         * minus sign converts it to ball velocity. */
        if (!s_have_prev) {
            ball_velocity = 0.0f;
            s_have_prev = 1;
            s_prev_error = error_f;
            s_prev_seq = cam->seq;
            s_prev_frame_ms = cam->last_update_ms;
        } else if (cam->seq != s_prev_seq) {
            uint32_t dt_ms = (uint32_t)(cam->last_update_ms - s_prev_frame_ms);
            if (dt_ms == 0U) {
                s_prev_derivative = 0.0f;
            } else {
                s_prev_derivative = (error_f - s_prev_error)
                                  * (1000.0f / (float)dt_ms);
            }
            s_prev_error = error_f;
            s_prev_seq = cam->seq;
            s_prev_frame_ms = cam->last_update_ms;
            ball_velocity = -s_prev_derivative;
        } else {
            ball_velocity = -s_prev_derivative;
        }
#endif

        if (ball_velocity > 32767.0f) ball_velocity = 32767.0f;
        if (ball_velocity < -32768.0f) ball_velocity = -32768.0f;
        s_ball_velocity_mm_s = (int16_t)ball_velocity;

        /* The standard outer PID output has the sign of acceleration along the
         * tube. Positive AB raises the positive/front end and therefore causes
         * negative acceleration, so negate the outer result before adding it
         * to the horizontal AB count. */
        outer_output = -(BALANCE_POSITION_KP * error_f
                       + BALANCE_POSITION_KI * s_integral
                       - BALANCE_VELOCITY_KD * ball_velocity);
        if (outer_output > BALANCE_TILT_LIMIT_AB) {
            outer_output = BALANCE_TILT_LIMIT_AB;
        }
        if (outer_output < -BALANCE_TILT_LIMIT_AB) {
            outer_output = -BALANCE_TILT_LIMIT_AB;
        }
        s_target_ab = BALANCE_LEVEL_AB_COUNT + (int32_t)outer_output;
        if (s_target_ab < BALANCE_TRAVEL_AB_MIN) {
            s_target_ab = BALANCE_TRAVEL_AB_MIN;
        }
        if (s_target_ab > BALANCE_TRAVEL_AB_MAX) {
            s_target_ab = BALANCE_TRAVEL_AB_MAX;
        }
        s_tracking = 1;
    }

    StepperFeedback_GetSnapshot(&feedback);
    beam_error = (float)(s_target_ab - feedback.quadrature_count);
    output = BALANCE_ANGLE_KP * beam_error;
    if (output > (float)BALANCE_OUTPUT_LIMIT) output = (float)BALANCE_OUTPUT_LIMIT;
    if (output < -(float)BALANCE_OUTPUT_LIMIT) output = -(float)BALANCE_OUTPUT_LIMIT;

    speed_cmd = (int32_t)(output * (float)BALANCE_MOTOR_COMMAND_SIGN);
    if (StepperFeedback_IsTravelCommandAllowed(speed_cmd) == 0U) {
        speed_cmd = 0;
    }
    s_output = (int16_t)speed_cmd;
    Stepper_SetSpeed(speed_cmd);
}
