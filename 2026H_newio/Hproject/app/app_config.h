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
/* 13 lines * 2 counts/line * 30 gear = 780 counts per wheel revolution.
 * Must stay in sync with encoder_driver.c. */
#define APP_COUNTS_PER_WHEEL_REV   (780.0f)
#define APP_CM_PER_COUNT           (APP_WHEEL_CIRCUMFERENCE_CM / APP_COUNTS_PER_WHEEL_REV)

/* ============ Stop Line Detection ============ */
/* The car can cross the 5 cm stop bar with lateral offset or yaw, so detection
 * must not depend on a fixed set of channels. A normal 1.8 cm guide line
 * covers one or two of the eight channels, but on a diagonal or at a sharp
 * corner it reaches three - which is why three used to false-trigger. The
 * stop bar must now light four channels at once (a direct hit on the ~12 mm
 * probe pitch), or sweep across six within a short window (a skewed hit).
 *
 * Both counts are absolute channel counts, not fractions, so they had to be
 * re-tuned when the array went from six to eight probes. */
#define APP_STOP_LINE_MIN_INSTANT_CHANNELS (4U)
#define APP_STOP_IGNORE_DISTANCE   (30.0f)  /* Ignore stop line for first 30cm */
#define APP_STOP_IGNORE_TICKS      (100U)   /* Full lap: ignore first 1 second */
#define APP_STOP_LINE_CONFIRM_TICKS (2U)    /* Two consecutive 10ms samples */
#define APP_STOP_LINE_WINDOW_TICKS  (6U)    /* 60ms temporal coverage window */
/* Must stay strictly above APP_STOP_LINE_MIN_INSTANT_CHANNELS. If the two are
 * equal, one single wide sample both arms the window and fills the mask, so
 * this path fires on that frame alone and short-circuits the two-sample
 * confirmation above - the looser of the two branches wins. */
#define APP_STOP_LINE_WINDOW_CHANNELS (6U)
#define APP_STOP_BRAKE_HOLD_MS       (150U)

#endif
