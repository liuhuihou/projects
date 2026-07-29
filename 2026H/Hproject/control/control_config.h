#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* ============ Timing ============ */
#define CONTROL_PERIOD_MS       (10U)
#define SPEED_SAMPLE_TICKS      (5U)
#define SPEED_SAMPLE_MS         (CONTROL_PERIOD_MS * SPEED_SAMPLE_TICKS)

/* ============ Speed PI Loop ============ */
#define SPEED_KP                (10.0f)
#define SPEED_KI                (0.35f)
#define SPEED_FF_LEFT           (65.0f)
#define SPEED_FF_RIGHT          (58.0f)
#define SPEED_INTEGRAL_LIMIT    (1500.0f)

/* ============ Line Following PD ============ */
#define LINE_KP                 (3.00f)
#define LINE_KD                 (1.50f)
#define LINE_CORRECTION_LIMIT   (10.0f)
#define LINE_CORRECTION_STEP    (3.0f)

/* ============ Straight Heading Correction ============ */
#define STRAIGHT_KP             (0.20f)
#define STRAIGHT_KD             (0.70f)

/* ============ Balance PID (Stepper) ============ */
#define BALANCE_KP              (5.0f)
#define BALANCE_KI              (0.1f)
#define BALANCE_KD              (2.0f)
#define BALANCE_OUTPUT_LIMIT    (3000)   /* max steps/sec */
#define BALANCE_INTEGRAL_LIMIT  (500.0f)
#define BALANCE_PERIOD_MS       (20U)    /* 50Hz update rate */

#endif
