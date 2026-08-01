#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    BALANCE_PID_PROFILE_Q3 = 0,
    BALANCE_PID_PROFILE_Q4,
    BALANCE_PID_PROFILE_Q5,
    BALANCE_PID_PROFILE_Q6
} BalancePidProfile;

typedef enum {
    BALANCE_Q3_DIRECTION_FORWARD = 0, /* O -> -5 cm */
    BALANCE_Q3_DIRECTION_REVERSE      /* -5 cm -> +5 cm */
} BalanceQ3Direction;

/*
 * Balance controller for the steel ball on the PPR pipe.
 * Uses K230 camera position feedback + stepper motor output.
 *
 * Cascaded control:
 *   ball position/velocity -> target beam AB count -> stepper speed command.
 *
 * The loop is gated on Camera_IsBallUsable(), not on link freshness alone: a
 * frame arrives every camera iteration whether or not the ball is visible, so
 * fresh data does not imply a usable position. When the position is not usable
 * the target beam angle returns to horizontal, the outer-loop state is cleared,
 * and Balance_IsTracking() goes false. Callers that act on Balance_GetError()
 * must check Balance_IsTracking() first.
 */

void Balance_Init(void);

/* Select the independent outer-loop gains for the active question. Changing
 * profile clears integral/derivative history so one question cannot carry
 * controller state into another. */
void Balance_SelectPidProfile(BalancePidProfile profile);
BalancePidProfile Balance_GetPidProfile(void);

/* Select the complete Q3 PID group for the active movement direction. */
void Balance_SelectQ3Direction(BalanceQ3Direction direction);
BalanceQ3Direction Balance_GetQ3Direction(void);

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

/* Velocity used by the outer loop, in mm/s. */
int16_t Balance_GetBallVelocity(void);

/* Current beam-angle request expressed as an absolute AB count. */
int32_t Balance_GetTargetAb(void);

#endif
