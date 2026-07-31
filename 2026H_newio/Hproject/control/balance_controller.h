#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

/*
 * Balance controller for the steel ball on the PPR pipe.
 * Uses K230 camera position feedback + stepper motor output.
 *
 * Control loop: PID on ball position error -> stepper speed command.
 *
 * The loop is gated on Camera_IsBallUsable(), not on link freshness alone: a
 * frame arrives every camera iteration whether or not the ball is visible, so
 * fresh data does not imply a usable position. When the position is not usable
 * the stepper is stopped, the integral is cleared, and Balance_IsTracking()
 * goes false. Callers that act on Balance_GetError() - the Q3 phase machine
 * especially - must check Balance_IsTracking() first, or they will read a stale
 * error and treat "no data" as "target reached".
 */

void Balance_Init(void);

/* Set target ball position in mm relative to center O (-120 to +120) */
void Balance_SetTarget(int16_t target_mm);

/* Get current target */
int16_t Balance_GetTarget(void);

/* Call periodically (every BALANCE_PERIOD_MS) */
void Balance_Tick(uint32_t now_ms);

/* Enable/disable balance control */
void Balance_Enable(void);
void Balance_Disable(void);

/* Get current ball position error for display.
 * Only meaningful while Balance_IsTracking() is true; otherwise it is the last
 * error computed before the camera went away. */
int16_t Balance_GetError(void);

/* Is balance active? */
uint8_t Balance_IsEnabled(void);

/*
 * 1 when the last tick ran the PID on a usable camera position. False both when
 * balance is disabled and when it is enabled but the camera has nothing usable.
 */
uint8_t Balance_IsTracking(void);

/* Last stepper speed command in steps/sec, for display and tuning. */
int16_t Balance_GetOutput(void);

#endif
