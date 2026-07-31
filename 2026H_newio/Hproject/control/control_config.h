#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* ============ Timing ============ */
#define CONTROL_PERIOD_MS       (10U)
#define SPEED_SAMPLE_TICKS      (2U)
#define SPEED_SAMPLE_MS         (CONTROL_PERIOD_MS * SPEED_SAMPLE_TICKS)

/* ============ Speed PI Loop ============ */
#define SPEED_KP                (10.0f)
#define SPEED_KI                (0.9f)
/* Feed-forward is the dominant term: duty = FF * target_rpm + PI.
 * The reference project's 65/58 saturated duty at a 101 RPM target, so the
 * PI loop lost authority and each wheel ran to its own open-loop limit.
 * Scaled down to leave PI headroom. Both wheels use the same coefficient -
 * the PI loop absorbs any motor-to-motor difference. */
#define SPEED_FF_LEFT           (32.0f)
#define SPEED_FF_RIGHT          (32.0f)
/* Integral windup limit. During a curve the outer wheel under-runs and the
 * inner wheel over-runs, so their integrators charge to opposite extremes.
 * With KI=1.0 a limit of 1500 lets each contribute +-1500 duty, which takes
 * several hundred ms to unwind after the curve - the car keeps cutting inward
 * on the following straight. Capping it well below the steady-state need
 * keeps the loop accurate without the long recovery. */
#define SPEED_INTEGRAL_LIMIT    (400.0f)
/* Per-cycle decay applied to both integrators while steering hard, so the
 * curve leaves no accumulated charge to unwind on the following straight. */
#define SPEED_INTEGRAL_BLEED      (0.90f)

/* ============ Line Following Profiles ============ */
/* Q2/Q4 keep the currently proven line-following response. Values that
 * produce a wheel-speed difference are expressed in RPM, so applying them
 * unchanged at Q5/Q6's lower speed makes the relative steering much stronger
 * and can drive the inside wheel close to stall. The Q5/Q6 values are scaled
 * to about 22/35 of the Q2 values, preserving approximately the same wheel
 * speed ratio at a given sensor error.
 *
 * Curve slowdown gain/minimum scale and line-lost behaviour are also separate
 * even though their initial values match, so later tuning one question group
 * cannot change the other group. */

/* Q2 + Q4 profile */
#define LINE_Q2_Q4_KP                       (10.00f)
#define LINE_Q2_Q4_KD                       (6.00f)
#define LINE_Q2_Q4_CORRECTION_LIMIT         (25.0f)
#define LINE_Q2_Q4_CORRECTION_STEP          (20.0f)
#define LINE_Q2_Q4_LOST_HOLD_TICKS          (2U)
#define LINE_Q2_Q4_LOST_DECAY               (0.70f)
#define LINE_Q2_Q4_CURVE_SLOWDOWN_GAIN      (0.30f)
#define LINE_Q2_Q4_CURVE_SPEED_MIN_SCALE    (0.70f)
#define LINE_Q2_Q4_RIGHT_DECEL_STEP_RPM     (15.0f)
#define LINE_Q2_Q4_INTEGRATE_STEER_MAX      (8.0f)

/* Q5 + Q6 low-speed, ball-control profile */
#define LINE_Q5_Q6_KP                       (6.30f)
#define LINE_Q5_Q6_KD                       (3.80f)
#define LINE_Q5_Q6_CORRECTION_LIMIT         (16.0f)
#define LINE_Q5_Q6_CORRECTION_STEP          (12.0f)
#define LINE_Q5_Q6_LOST_HOLD_TICKS          (2U)
#define LINE_Q5_Q6_LOST_DECAY               (0.70f)
#define LINE_Q5_Q6_CURVE_SLOWDOWN_GAIN      (0.30f)
#define LINE_Q5_Q6_CURVE_SPEED_MIN_SCALE    (0.70f)
#define LINE_Q5_Q6_RIGHT_DECEL_STEP_RPM     (10.0f)
#define LINE_Q5_Q6_INTEGRATE_STEER_MAX      (5.0f)

/* ============ Straight Heading Correction ============ */
#define STRAIGHT_KP             (0.20f)
#define STRAIGHT_KD             (0.70f)

/* ============ Ball/Beam Cascade Control ============ */
/* AB count at the measured horizontal position and the permanent mechanical
 * travel window. Positive STEP increases AB and raises the tube's positive
 * (front) end. */
#define BALANCE_LEVEL_AB_COUNT          (270)
#define BALANCE_TRAVEL_AB_MIN           (50)
#define BALANCE_TRAVEL_AB_MAX           (1050)

/* Outer loop: ball position/velocity -> requested beam offset in AB counts.
 * Each competition question has its own gains.  They intentionally start with
 * the previously tuned common values so selecting a profile does not change
 * behaviour; from now on each question can be tuned independently. */
#define BALANCE_Q3_POSITION_KP          (0.70f)
#define BALANCE_Q3_POSITION_KI          (0.10f)
#define BALANCE_Q3_VELOCITY_KD          (0.30f)

#define BALANCE_Q4_POSITION_KP          (0.70f)
#define BALANCE_Q4_POSITION_KI          (0.10f)
#define BALANCE_Q4_VELOCITY_KD          (0.30f)

#define BALANCE_Q5_POSITION_KP          (0.70f)
#define BALANCE_Q5_POSITION_KI          (0.10f)
#define BALANCE_Q5_VELOCITY_KD          (0.30f)

#define BALANCE_Q6_POSITION_KP          (0.70f)
#define BALANCE_Q6_POSITION_KI          (0.10f)
#define BALANCE_Q6_VELOCITY_KD          (0.30f)

/* These limits and the AB angle loop describe the common mechanism, so they
 * remain shared by all four outer-loop profiles. */
#define BALANCE_POSITION_INTEGRAL_LIMIT (500.0f)
#define BALANCE_TILT_LIMIT_AB           (250.0f)

/* Inner loop: requested AB count -> signed STEP frequency. */
#define BALANCE_ANGLE_KP                (8.0f)
#define BALANCE_OUTPUT_LIMIT            (1500)   /* max steps/sec */
#define BALANCE_MOTOR_COMMAND_SIGN      (+1)
#define BALANCE_PERIOD_MS       (20U)    /* 50Hz update rate */
/* A camera frame older than this means the link is down, not that the ball is
 * merely out of view - the K230 sends a frame every iteration either way. Three
 * missed frames at 27 fps is ~110 ms, so 150 ms tolerates a hiccup without
 * letting the PID drive on genuinely dead data. */
#define BALANCE_DATA_TIMEOUT_MS (150U)
/* Take the D term from the velocity the K230 already filtered, instead of
 * differencing position here. Two reasons: the balance tick runs at 50 Hz while
 * frames arrive at ~27 Hz, so a per-tick difference is a real delta on some
 * ticks and exactly zero on the others - a square wave through KD; and the
 * K230's own filter works on frame timestamps, which are the correct dt.
 * Set to 0 to fall back to differencing, which now only recomputes on a new
 * frame and normalises by the actual frame interval. */
#ifndef BALANCE_USE_CAMERA_VELOCITY
#define BALANCE_USE_CAMERA_VELOCITY (1)
#endif

#endif
