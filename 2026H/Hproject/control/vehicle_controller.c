#include "vehicle_controller.h"
#include "control_config.h"
#include "encoder_driver.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "board_hardware.h"

static volatile ControlMode s_mode;
static volatile float s_base_rpm;
static volatile float s_left_integral;
static volatile float s_right_integral;
static volatile int32_t s_left_distance;
static volatile int32_t s_right_distance;
static volatile float s_left_rpm;
static volatile float s_right_rpm;
static volatile int32_t s_left_speed_counts;
static volatile int32_t s_right_speed_counts;
static volatile uint8_t s_speed_sample_ticks;
static volatile float s_left_target_rpm;
static volatile float s_right_target_rpm;
static volatile int32_t s_left_duty;
static volatile int32_t s_right_duty;
static volatile int s_line_error;
static volatile uint8_t s_line_valid;
static volatile uint16_t s_line_lost_ticks;
/* Base speed after curve scaling; s_base_rpm stays the commanded value. */
static volatile float s_curve_base_rpm;
static volatile float s_previous_line_error;
static volatile int s_previous_heading_error;
static volatile float s_last_line_correction;
static volatile uint32_t s_control_ticks;

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float slew_float(float current, float requested, float step)
{
    const float delta = requested - current;
    if (delta > step) return current + step;
    if (delta < -step) return current - step;
    return requested;
}

static int32_t speed_pi(float target_rpm, float measured_rpm,
                         float feed_forward, volatile float *integral,
                         uint8_t integrate)
{
    const float error = target_rpm - measured_rpm;
    float candidate_integral = *integral;
    float duty;
    if (integrate != 0U) {
        candidate_integral = clamp_float(*integral + error,
                                          -SPEED_INTEGRAL_LIMIT,
                                          SPEED_INTEGRAL_LIMIT);
    }

    duty = feed_forward * target_rpm + SPEED_KP * error
         + SPEED_KI * candidate_integral;

    if (integrate != 0U) {
        if (!((duty >= (float)MOTOR_PWM_LIMIT && error > 0.0f) ||
              (duty <= 0.0f && error < 0.0f))) {
            *integral = candidate_integral;
        }
        duty = feed_forward * target_rpm + SPEED_KP * error
             + SPEED_KI * (*integral);
    }
    duty = clamp_float(duty, 0.0f, (float)MOTOR_PWM_LIMIT);
    return (int32_t)duty;
}

void Control_Init(void)
{
    s_mode = CTRL_STOP;
    s_base_rpm = 0.0f;
    s_left_integral = 0.0f;
    s_right_integral = 0.0f;
    s_left_distance = 0;
    s_right_distance = 0;
    s_left_rpm = 0.0f;
    s_right_rpm = 0.0f;
    s_left_speed_counts = 0;
    s_right_speed_counts = 0;
    s_speed_sample_ticks = 0U;
    s_left_target_rpm = 0.0f;
    s_right_target_rpm = 0.0f;
    s_left_duty = 0;
    s_right_duty = 0;
    s_line_error = 0;
    s_previous_line_error = 0;
    s_previous_heading_error = 0;
    s_last_line_correction = 0.0f;
    s_control_ticks = 0U;
    s_line_lost_ticks = 0U;
    s_curve_base_rpm = 0.0f;
    Motor_Stop();
}

void Control_SetMode(ControlMode mode)
{
    if (mode != s_mode) {
        s_left_integral = 0.0f;
        s_right_integral = 0.0f;
        s_previous_line_error = 0;
        s_previous_heading_error = 0;
        s_last_line_correction = 0.0f;
    }
    s_mode = mode;
    if (mode == CTRL_STOP) Motor_Brake();
}
ControlMode Control_GetMode(void) { return s_mode; }
void Control_SetBaseSpeed(float rpm) { s_base_rpm = (rpm > 0.0f) ? rpm : 0.0f; }
float Control_GetLeftRpm(void) { return s_left_rpm; }
float Control_GetRightRpm(void) { return s_right_rpm; }
float Control_GetBaseSpeedRpm(void) { return s_base_rpm; }
float Control_GetLeftTargetRpm(void) { return s_left_target_rpm; }
float Control_GetRightTargetRpm(void) { return s_right_target_rpm; }
int32_t Control_GetLeftDuty(void) { return s_left_duty; }
int32_t Control_GetRightDuty(void) { return s_right_duty; }
int Control_GetLineError(void) { return s_line_error; }
uint8_t Control_GetLineState(void) { return LineSensor_Read(); }
uint32_t Control_GetTickCount(void) { return s_control_ticks; }
int32_t Control_GetLeftDistance(void) { return s_left_distance; }
int32_t Control_GetRightDistance(void) { return s_right_distance; }
void Control_ResetDistance(void)
{
    __disable_irq();
    s_left_distance = 0;
    s_right_distance = 0;
    __enable_irq();
}

void Control_Tick(void)
{
    int32_t encoder_a, encoder_b;
    int32_t left_count, right_count;
    float target_left, target_right;
    float correction, line_error;
    /* Called only from the TIMER_0 ISR. A same-priority interrupt cannot
     * pre-empt itself on Cortex-M0+, so no re-entry guard is needed - and a
     * guard that early-returns would stall s_control_ticks, which is the
     * time base the whole application reads through app_time_ms(). */
    ++s_control_ticks;

    LineSensor_Update();
    Encoder_TakeCounts(&encoder_a, &encoder_b);

    /* Physical mapping: J2/PWM B = left wheel (encoder B)
     * J1/PWM A = right wheel (encoder A, negated) */
    left_count = encoder_b;
    right_count = -encoder_a;
    s_left_distance += left_count;
    s_right_distance += right_count;
    s_left_speed_counts += left_count;
    s_right_speed_counts += right_count;

    if (++s_speed_sample_ticks >= SPEED_SAMPLE_TICKS) {
        s_left_rpm = Encoder_CountsToRpm(s_left_speed_counts, SPEED_SAMPLE_MS);
        s_right_rpm = Encoder_CountsToRpm(s_right_speed_counts, SPEED_SAMPLE_MS);
        if (s_left_rpm < 0.0f) s_left_rpm = -s_left_rpm;
        if (s_right_rpm < 0.0f) s_right_rpm = -s_right_rpm;
        s_left_speed_counts = 0;
        s_right_speed_counts = 0;
        s_speed_sample_ticks = 0U;
    }
    /* Keep line error updated even when stopped, so it can be observed
     * on the display while positioning the car. */
    s_line_valid = LineSensor_GetSteeringError(&line_error);
    if (s_line_valid != 0U) {
        s_line_error = (line_error >= 0.0f) ?
                       (int)(line_error + 0.5f) :
                       (int)(line_error - 0.5f);
    } else {
        line_error = 0.0f;
        s_line_error = 0;
    }

    if (s_mode == CTRL_STOP) {
        s_left_integral = 0.0f;
        s_right_integral = 0.0f;
        s_left_target_rpm = 0.0f;
        s_right_target_rpm = 0.0f;
        s_left_duty = 0;
        s_right_duty = 0;
        s_left_speed_counts = 0;
        s_right_speed_counts = 0;
        s_speed_sample_ticks = 0U;
        Motor_Brake();
        return;
    }

    if (s_mode == CTRL_STRAIGHT) {
        const int heading_error = (int)(s_left_distance - s_right_distance);
        correction = STRAIGHT_KP * (float)heading_error
                   + STRAIGHT_KD * (float)(heading_error - s_previous_heading_error);
        s_previous_heading_error = heading_error;
        target_left = s_base_rpm - correction;
        target_right = s_base_rpm + correction;
    } else { /* CTRL_LINE - line_error already computed above */
        if (s_line_valid != 0U) {
            correction = LINE_KP * line_error
                       + LINE_KD * (line_error - s_previous_line_error);
            s_previous_line_error = line_error;
            correction = clamp_float(correction,
                                     -LINE_CORRECTION_LIMIT,
                                     LINE_CORRECTION_LIMIT);
            correction = slew_float(s_last_line_correction, correction,
                                    LINE_CORRECTION_STEP);
            s_last_line_correction = correction;
            s_line_lost_ticks = 0U;
        } else {
            /* Line lost. Keep steering the way we were so the line can be
             * re-acquired, but decay the correction: holding a full-scale
             * turn indefinitely drives the car off the track when the line
             * is lost on a curve exit rather than mid-curve. */
            if (s_line_lost_ticks < 0xFFFFU) {
                ++s_line_lost_ticks;
            }
            if (s_line_lost_ticks <= LINE_LOST_HOLD_TICKS) {
                correction = s_last_line_correction;
            } else {
                s_last_line_correction *= LINE_LOST_DECAY;
                correction = s_last_line_correction;
            }
        }
        /* Curve speed scaling. Turn radius for a given wheel-speed
         * differential grows with forward speed, so a correction that holds
         * the R=0.5 m arc from a slow entry is not enough after a full
         * straight of acceleration. Scale the base speed down in proportion
         * to how hard we are steering, so entry speed no longer decides
         * whether the car makes the corner. */
        {
            const float steer_mag = (correction >= 0.0f) ? correction : -correction;
            float scale = 1.0f - CURVE_SLOWDOWN_GAIN
                                 * (steer_mag / LINE_CORRECTION_LIMIT);

            if (scale < CURVE_SPEED_MIN_SCALE) {
                scale = CURVE_SPEED_MIN_SCALE;
            }
            s_curve_base_rpm = s_base_rpm * scale;
        }
        target_left = s_curve_base_rpm + correction;
        target_right = s_curve_base_rpm - correction;
    }

    target_left = (target_left > 0.0f) ? target_left : 0.0f;
    target_right = (target_right > 0.0f) ? target_right : 0.0f;
    s_left_target_rpm = target_left;
    s_right_target_rpm = target_right;

    /* Suspend integration while steering hard. In a curve the wheels are
     * commanded to different speeds for geometric reasons and the outer wheel
     * simply cannot keep up, so the error is not a steady-state offset for the
     * integrator to correct - integrating it just winds both integrators to
     * opposite extremes, and the car keeps cutting inward after the curve
     * until they unwind. Only integrate when running near-straight. */
    {
        const float steer = (s_last_line_correction >= 0.0f) ?
                            s_last_line_correction : -s_last_line_correction;
        const uint8_t straight = (steer < SPEED_INTEGRATE_STEER_MAX) ? 1U : 0U;
        const uint8_t integrate = ((s_speed_sample_ticks == 0U) && straight) ?
                                  1U : 0U;

        /* While steering hard, actively bleed the integrators toward zero so
         * no charge is left to fight the straight that follows the curve. */
        if (straight == 0U) {
            s_left_integral *= SPEED_INTEGRAL_BLEED;
            s_right_integral *= SPEED_INTEGRAL_BLEED;
        }

        s_left_duty = speed_pi(target_left, s_left_rpm, SPEED_FF_LEFT,
                               &s_left_integral, integrate);
        s_right_duty = speed_pi(target_right, s_right_rpm, SPEED_FF_RIGHT,
                                &s_right_integral, integrate);
    }
    Motor_SetDuty(s_left_duty, s_right_duty);
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        Control_Tick();
    }
}
