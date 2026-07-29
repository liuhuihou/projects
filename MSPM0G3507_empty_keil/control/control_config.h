#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#define CONTROL_PERIOD_MS       (10U)
#define SPEED_SAMPLE_TICKS      (5U)
#define SPEED_SAMPLE_MS         (CONTROL_PERIOD_MS * SPEED_SAMPLE_TICKS)

#define SPEED_KP                (10.0f)
#define SPEED_KI                (0.35f)
#define SPEED_FF_LEFT           (65.0f)
#define SPEED_FF_RIGHT          (58.0f)
#define SPEED_INTEGRAL_LIMIT    (1500.0f)

#define LINE_KP                 (3.00f)
#define LINE_KD                 (1.50f)
#define LINE_CORRECTION_LIMIT   (10.0f)
#define LINE_CORRECTION_STEP    (3.0f)
#define STRAIGHT_KP             (0.20f)
#define STRAIGHT_KD             (0.70f)

#endif
