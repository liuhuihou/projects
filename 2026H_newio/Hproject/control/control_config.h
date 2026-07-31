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
/* Integration is suspended while |steering correction| exceeds this, i.e.
 * during curves, where the speed error is geometric rather than a
 * steady-state offset. Below it the car is running near-straight and the
 * integrator does its normal job of trimming out residual error. */
#define SPEED_INTEGRATE_STEER_MAX (8.0f)
/* Per-cycle decay applied to both integrators while steering hard, so the
 * curve leaves no accumulated charge to unwind on the following straight. */
#define SPEED_INTEGRAL_BLEED      (0.90f)

/* ============ Line Following PD ============ */
#define LINE_KP                 (10.00f)
#define LINE_KD                 (6.00f)
#define LINE_CORRECTION_LIMIT   (25.0f)
#define LINE_CORRECTION_STEP    (20.0f)
/* Line-lost behaviour. The last steering correction is held for this many
 * 10 ms cycles so a brief dropout mid-curve does not straighten the car,
 * then decayed each cycle so a sustained dropout on a curve exit does not
 * keep the car turning at full scale until it leaves the track. */
#define LINE_LOST_HOLD_TICKS    (2U)
#define LINE_LOST_DECAY         (0.70f)

/* ============ Curve Slowdown ============ */
/* Base speed is scaled down in proportion to steering effort:
 *   scale = 1 - GAIN * (|correction| / LINE_CORRECTION_LIMIT)
 * At full deflection with GAIN=0.25 the car runs at 75% of the straight-line
 * speed. This decouples cornering from entry speed: without it the first
 * curve (entered slowly after the start) is made and the second (entered at
 * full speed off a 1.5 m straight) is not, because turn radius for a given
 * wheel-speed differential grows with forward speed. */
#define CURVE_SLOWDOWN_GAIN     (0.30f)
#define CURVE_SPEED_MIN_SCALE   (0.70f)
/* Maximum right-wheel target RPM decrease per 10 ms control period. */
#define RIGHT_TARGET_DECEL_STEP_RPM (15.0f)

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
