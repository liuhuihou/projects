#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>

/*
 * Stepper driver for the balance beam (D36A / ATD5984, Step/Dir/Enable).
 *
 * The pulse train is hardware PWM on ST1 (PB16, TIMG7_CCP1): speed is set by
 * rewriting the timer period, so the CPU is not involved in pulse generation
 * and the train stays even regardless of interrupt load. The previous
 * software version toggled a GPIO from a Stepper_Tick() that nothing ever
 * called, so the motor never moved at all.
 *
 * Positive speed drives DIR1 high, negative drives it low. Which way the
 * beam physically tilts depends on the motor wiring; swap the coil pair or
 * flip the sign at the call site if it is inverted.
 */

/* Mechanical constants, matching the D36A DIP switch setting (MS1..MS3 for
 * 1/16 step) and a 1.8 deg motor. Change MICROSTEPS if the switches move. */
#define STEPPER_FULL_STEPS_PER_REV  (200U)
#define STEPPER_MICROSTEPS          (16U)
#define STEPPER_PULSES_PER_REV      (STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEPS)

/* Pulse rate limits in steps/sec. The ceiling is the D36A maximum. The floor
 * is set by the 16-bit timer at a 1 MHz tick (1e6/65536 = 15.3 Hz); anything
 * slower is treated as stopped, which at 3200 pulses/rev is 0.3 rev/min and
 * indistinguishable from held for a balance beam. */
#define STEPPER_MAX_FREQ_HZ         (5000U)
#define STEPPER_MIN_FREQ_HZ         (16U)

void Stepper_Init(void);

/* Enable/disable the driver outputs. EN1 is active HIGH on the D36A: enabled
 * means the coils are energised and the beam is held. */
void Stepper_Enable(void);
void Stepper_Disable(void);

/* Set pulse rate in steps/sec; sign selects direction, 0 stops the train.
 * Magnitudes above STEPPER_MAX_FREQ_HZ are clamped, magnitudes below
 * STEPPER_MIN_FREQ_HZ stop it. */
void Stepper_SetSpeed(int32_t steps_per_sec);

/* Signed pulse count since the last reset, maintained by the PWM period
 * interrupt. Divide by STEPPER_PULSES_PER_REV for revolutions. */
int32_t Stepper_GetPosition(void);
void Stepper_ResetPosition(void);

#endif
