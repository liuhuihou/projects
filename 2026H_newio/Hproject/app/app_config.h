#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============ Vehicle Parameters ============ */
#define APP_WHEEL_DIAMETER_CM      (6.6f)
#define APP_WHEEL_CIRCUMFERENCE_CM (20.7345f)

/* ============ Speed Presets (cm/s) ============ */
#define APP_SPEED_Q2_CM_S          (35.0f)   /* Fast lap (Q2: <=20s) */
#define APP_SPEED_Q4_CM_S          (25.0f)   /* A->B with balance (Q4: <=8s) */
#define APP_SPEED_Q5_CM_S          (22.0f)   /* Full lap with balance (Q5: <=30s) */

/* ============ Stop Line Detection ============ */
/* Eight probes are numbered 1..8 from left to right after the driver
 * normalises the sensor byte. Accept the centred 3456 pattern or either
 * four-adjacent-channel offset pattern 2345/4567. Other channels are allowed,
 * but one of these three four-channel groups must be present in full. */
#define APP_STOP_PATTERN_3456_MASK      (0x3CU)
#define APP_STOP_PATTERN_2345_MASK      (0x78U)
#define APP_STOP_PATTERN_4567_MASK      (0x1EU)

/* Odometry is an arming gate, not a forced stop. The full-lap modes may
 * recognise the A stop bar only after the average wheel travel exceeds 80%
 * of the 614 cm track. The remaining 20% tolerates wheel-diameter and encoder
 * calibration error while still rejecting the curve patterns earlier on the
 * lap. Actual braking still requires the stop-line vote below. */
#define APP_TRACK_LAP_LENGTH_CM          (614.0f)
#define APP_STOP_ODOMETRY_ARM_RATIO      (0.80f)
#define APP_STOP_ODOMETRY_ARM_CM         \
    (APP_TRACK_LAP_LENGTH_CM * APP_STOP_ODOMETRY_ARM_RATIO)

/* Q4/Q5/Q6 use odometry as the actual stop trigger. Q4 stops after the
 * requested 1.6 m. Q5/Q6 deliberately run 110% of the nominal lap so the
 * vehicle has passed A before braking even with moderate under-counting. */
#define APP_STOP_Q4_DISTANCE_CM           (180.0f)
#define APP_STOP_Q5_Q6_DISTANCE_RATIO     (1.10f)
#define APP_STOP_Q5_Q6_DISTANCE_CM        \
    (APP_TRACK_LAP_LENGTH_CM * APP_STOP_Q5_Q6_DISTANCE_RATIO)

/* At 35 cm/s the 1.6..2.0 cm bar is present for about 46..57 ms. Vote over
 * five successful I2C samples and accept two hits. The odometry gate and
 * restricted spatial patterns reject early/one-sided curve detections, while
 * two votes tolerate a short valid pattern when the car crosses A at an
 * angle. */
#define APP_STOP_VOTE_HISTORY_MASK      (0x1FU) /* Last five samples */
#define APP_STOP_VOTE_REQUIRED          (2U)
#define APP_STOP_BRAKE_HOLD_MS       (150U)

#endif
