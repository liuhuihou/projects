#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>
#include "board_hardware.h"

/*
 * Hiwonder I2C line sensor.
 *
 * Split into a bus half and a logic half because the transfer must not run
 * in the control ISR:
 *   LineSensor_Poll()   - blocking I2C read, call from the main loop only.
 *   LineSensor_Update() - filters the cached sample, safe from the 100 Hz ISR.
 * The GPIO version could be sampled anywhere; an I2C read cannot, since a
 * stretched clock or an absent sensor would stall the control loop with the
 * motors latched at their last duty.
 *
 * Bit convention throughout: highest active bit = leftmost channel,
 * bit 0 = rightmost, 1 = black line seen. The driver normalises the sensor's
 * own bit order to this (see HW_LINE_SENSOR_BIT0_IS_LEFT).
 */

/* Channel count, 8 on the current board. Everything derived. */
#define LINE_SENSOR_COUNT       HW_LINE_SENSOR_COUNT

/* Mask with every channel black - a perpendicular bar such as the start line. */
#define LINE_SENSOR_ALL_MASK    ((uint8_t)((1U << LINE_SENSOR_COUNT) - 1U))

/* Consecutive samples a channel must agree on before the filtered state
 * follows it. */
#define LINE_SENSOR_FILTER_N    (3U)

/* Consecutive failed I2C reads tolerated before the cached state is cleared.
 * Below this the last good sample is held, which rides out a single glitched
 * transfer; at or above it the sensor is treated as line-lost, so the
 * existing hold-then-decay path in the line follower takes over rather than
 * the car steering on stale data. */
#define LINE_SENSOR_FAIL_LIMIT  (5U)

void LineSensor_Init(void);

/* Blocking I2C read into the cache. Main loop only. Returns 1 on success. */
uint8_t LineSensor_Poll(void);

/* Runs the per-channel filter over the cached sample. ISR-safe. */
void LineSensor_Update(void);

/* Latest cached sample, unfiltered. */
uint8_t LineSensor_ReadRaw(void);

/* Filtered channel bitmask. */
uint8_t LineSensor_Read(void);

/* Weighted steering error in channel units, negative = line is left of
 * centre. Returns 0 when the line is lost or the pattern is not contiguous. */
uint8_t LineSensor_GetSteeringError(float *error);

/* 0 once LINE_SENSOR_FAIL_LIMIT consecutive reads have failed - the sensor is
 * unplugged, unpowered, or the bus is stuck. */
uint8_t LineSensor_IsOnline(void);

#endif
