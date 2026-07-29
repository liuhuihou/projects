#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

/*
 * Balance controller for the steel ball on the PPR pipe.
 * Uses K230 camera position feedback + stepper motor output.
 *
 * Control loop: PID on ball position error -> stepper speed command.
 * Optional: MPU6050 acceleration feedforward for curve compensation.
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

/* Get current ball position error for display */
int16_t Balance_GetError(void);

/* Is balance active? */
uint8_t Balance_IsEnabled(void);

#endif
