#ifndef STEPPER_FEEDBACK_H
#define STEPPER_FEEDBACK_H

#include <stdint.h>

typedef struct {
    int32_t quadrature_count;
    int32_t index_count;
    uint32_t invalid_transition_count;
    uint32_t pwm_period_ticks;
    uint32_t pwm_high_ticks;
    uint16_t pwm_angle_tenths;
    uint8_t pwm_valid;
    uint8_t phase_a;
    uint8_t phase_b;
    uint8_t index_level;
} StepperFeedbackSnapshot;

void StepperFeedback_Init(void);
void StepperFeedback_ResetCounts(void);
void StepperFeedback_GetSnapshot(StepperFeedbackSnapshot *snapshot);
void StepperFeedback_HandleGpioInterrupt(uint32_t status);

/* Absolute A/B-count travel window. The GPIO edge ISR immediately stops an
 * outward STEP command at a boundary; inward motion remains available. */
void StepperFeedback_SetTravelLimits(int32_t minimum, int32_t maximum);
void StepperFeedback_ClearTravelLimits(void);
uint8_t StepperFeedback_IsTravelCommandAllowed(int32_t steps_per_sec);

#endif
