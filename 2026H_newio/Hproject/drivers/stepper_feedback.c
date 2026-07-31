#include "stepper_feedback.h"
#include "board_hardware.h"
#include "stepper_driver.h"

static volatile int32_t s_quadrature_count;
static volatile int32_t s_index_count;
static volatile uint32_t s_invalid_transition_count;
static volatile uint32_t s_pwm_period_ticks;
static volatile uint32_t s_pwm_high_ticks;
static volatile uint16_t s_pwm_angle_tenths;
static volatile uint8_t s_pwm_valid;
static volatile uint8_t s_previous_ab;
static volatile int32_t s_travel_minimum;
static volatile int32_t s_travel_maximum;
static volatile uint8_t s_travel_limits_enabled;

static uint8_t read_ab(void)
{
    const uint8_t a = HW_STEPPER_ENC_READ_A() ? 1U : 0U;
    const uint8_t b = HW_STEPPER_ENC_READ_B() ? 1U : 0U;
    return (uint8_t)((a << 1U) | b);
}

void StepperFeedback_Init(void)
{
    s_quadrature_count = 0;
    s_index_count = 0;
    s_invalid_transition_count = 0U;
    s_pwm_period_ticks = 0U;
    s_pwm_high_ticks = 0U;
    s_pwm_angle_tenths = 0U;
    s_pwm_valid = 0U;
    s_previous_ab = read_ab();
    s_travel_limits_enabled = 0U;

    DL_GPIO_clearInterruptStatus(HW_STEPPER_ENC_PORT,
        HW_STEPPER_ENC_A_PIN | HW_STEPPER_ENC_B_PIN |
        HW_STEPPER_ENC_Z_PIN);
    HW_STEPPER_POS_CAP_RELOAD();
    NVIC_ClearPendingIRQ(HW_STEPPER_POS_CAP_IRQN);
    NVIC_EnableIRQ(HW_STEPPER_POS_CAP_IRQN);
    HW_STEPPER_POS_CAP_START();
}

void StepperFeedback_SetTravelLimits(int32_t minimum, int32_t maximum)
{
    const uint32_t primask = __get_PRIMASK();
    if (minimum >= maximum) return;
    __disable_irq();
    s_travel_minimum = minimum;
    s_travel_maximum = maximum;
    s_travel_limits_enabled = 1U;
    if (primask == 0U) __enable_irq();
}

void StepperFeedback_ClearTravelLimits(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_travel_limits_enabled = 0U;
    if (primask == 0U) __enable_irq();
}

uint8_t StepperFeedback_IsTravelCommandAllowed(int32_t steps_per_sec)
{
    int32_t count;
    int32_t minimum;
    int32_t maximum;
    uint8_t enabled;
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    count = s_quadrature_count;
    minimum = s_travel_minimum;
    maximum = s_travel_maximum;
    enabled = s_travel_limits_enabled;
    if (primask == 0U) __enable_irq();

    if (enabled == 0U || steps_per_sec == 0) return 1U;
    if (steps_per_sec > 0 && count >= maximum) return 0U;
    if (steps_per_sec < 0 && count <= minimum) return 0U;
    return 1U;
}

void StepperFeedback_ResetCounts(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_quadrature_count = 0;
    s_index_count = 0;
    s_invalid_transition_count = 0U;
    s_previous_ab = read_ab();
    DL_GPIO_clearInterruptStatus(HW_STEPPER_ENC_PORT,
        HW_STEPPER_ENC_A_PIN | HW_STEPPER_ENC_B_PIN |
        HW_STEPPER_ENC_Z_PIN);
    if (primask == 0U) __enable_irq();
}

void StepperFeedback_GetSnapshot(StepperFeedbackSnapshot *snapshot)
{
    const uint32_t primask = __get_PRIMASK();
    if (snapshot == 0) return;
    __disable_irq();
    snapshot->quadrature_count = s_quadrature_count;
    snapshot->index_count = s_index_count;
    snapshot->invalid_transition_count = s_invalid_transition_count;
    snapshot->pwm_period_ticks = s_pwm_period_ticks;
    snapshot->pwm_high_ticks = s_pwm_high_ticks;
    snapshot->pwm_angle_tenths = s_pwm_angle_tenths;
    snapshot->pwm_valid = s_pwm_valid;
    if (primask == 0U) __enable_irq();
    snapshot->phase_a = HW_STEPPER_ENC_READ_A() ? 1U : 0U;
    snapshot->phase_b = HW_STEPPER_ENC_READ_B() ? 1U : 0U;
    snapshot->index_level = HW_STEPPER_ENC_READ_Z() ? 1U : 0U;
}

void StepperFeedback_HandleGpioInterrupt(uint32_t status)
{
    static const int8_t transition_delta[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    const uint32_t ab_mask = HW_STEPPER_ENC_A_PIN | HW_STEPPER_ENC_B_PIN;

    if ((status & ab_mask) != 0U) {
        const uint8_t current_ab = read_ab();
        const uint8_t transition =
            (uint8_t)((s_previous_ab << 2U) | current_ab);
        const int8_t delta = transition_delta[transition];

        if (delta != 0) {
            s_quadrature_count += delta;
            if (s_travel_limits_enabled != 0U &&
                ((delta > 0 && s_quadrature_count >= s_travel_maximum) ||
                 (delta < 0 && s_quadrature_count <= s_travel_minimum))) {
                Stepper_SetSpeed(0);
            }
        } else if (current_ab != s_previous_ab) {
            ++s_invalid_transition_count;
        }
        s_previous_ab = current_ab;
    }

    if ((status & HW_STEPPER_ENC_Z_PIN) != 0U) {
        ++s_index_count;
    }
}

void STEPPER_POS_CAP_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(HW_STEPPER_POS_CAP_TIMER)) {
        case DL_TIMER_IIDX_CC0_DN: {
            const uint32_t period = HW_STEPPER_POS_CAP_PERIOD_TICKS();
            const uint32_t high = HW_STEPPER_POS_CAP_HIGH_TICKS();
            if (period != 0U && high <= period) {
                s_pwm_period_ticks = period;
                s_pwm_high_ticks = high;
                s_pwm_angle_tenths =
                    (uint16_t)((high * 3600U + period / 2U) / period);
                s_pwm_valid = 1U;
            } else {
                s_pwm_valid = 0U;
            }
            HW_STEPPER_POS_CAP_RELOAD();
            break;
        }
        case DL_TIMER_IIDX_ZERO:
            s_pwm_valid = 0U;
            break;
        default:
            break;
    }
}
