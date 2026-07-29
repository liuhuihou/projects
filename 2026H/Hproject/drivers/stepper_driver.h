#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>

/*
 * Stepper motor driver interface (Step/Dir).
 * Used to control the balance beam angle via A4988/DRV8825/TMC2209.
 *
 * The stepper controls the angle of the PPR pipe (balance beam).
 * Positive steps tilt the beam one way, negative the other.
 */

void Stepper_Init(void);

/* Enable/disable the motor driver */
void Stepper_Enable(void);
void Stepper_Disable(void);

/* Set target speed in steps/sec (signed: direction) */
void Stepper_SetSpeed(int32_t steps_per_sec);

/* Get current position in steps from home */
int32_t Stepper_GetPosition(void);

/* Reset position counter */
void Stepper_ResetPosition(void);

/* Call from timer ISR to generate step pulses */
void Stepper_Tick(void);

#endif
