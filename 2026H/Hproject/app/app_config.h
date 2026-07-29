#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============ Vehicle Parameters ============ */
#define APP_WHEEL_DIAMETER_CM      (6.6f)
#define APP_WHEEL_CIRCUMFERENCE_CM (20.7345f)

/* ============ Speed Presets (cm/s) ============ */
#define APP_SPEED_Q2_CM_S          (35.0f)   /* Fast lap (Q2: <=20s) */
#define APP_SPEED_Q4_CM_S          (25.0f)   /* A->B with balance (Q4: <=8s) */
#define APP_SPEED_Q5_CM_S          (22.0f)   /* Full lap with balance (Q5: <=30s) */

/* ============ Track Geometry ============ */
/* Oval: 2x1.5m straights + 2x semicircles R=0.5m */
/* Total circumference: 2*1.5 + 2*PI*0.5 = 3.0 + 3.14159 = 6.14159 m */
#define APP_TRACK_LENGTH_CM        (614.0f)
#define APP_TRACK_AB_LENGTH_CM     (150.0f)

/* ============ Encoder to Distance ============ */
/* 13 lines * 2 edges * 30 gear = 780 counts/motor_rev
 * 1 wheel_rev = 20.7345 cm -> 780 counts = 20.7345 cm
 * 1 count = 0.02658 cm */
#define APP_COUNTS_PER_WHEEL_REV   (780.0f)
#define APP_CM_PER_COUNT           (APP_WHEEL_CIRCUMFERENCE_CM / APP_COUNTS_PER_WHEEL_REV)

/* ============ Stop Line Detection ============ */
/* The stop line at A is a 5cm perpendicular bar.
 * When all 6 sensors detect black simultaneously -> stop line. */
#define APP_STOP_LINE_PATTERN      (0x3FU)
#define APP_STOP_IGNORE_DISTANCE   (30.0f)  /* Ignore stop line for first 30cm */

#endif
