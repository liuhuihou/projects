# Six-channel line control

## Runtime sequence

`TIMG0` is configured for a 10 ms period. `TIMER_0_INST_IRQHandler()` calls
`Control_Tick()` on every period event. Steering targets update every 10 ms;
the encoder speed loop uses a 50 ms count window to avoid large RPM jumps at
the 5 cm/s target speed. PWM is still recalculated every 10 ms so a line
offset changes the left/right drive promptly.

Each tick performs these operations:

1. Read the six RYZD channels and apply a three-sample integrate-and-hold
   filter.
2. Convert the filtered state to a weighted line error.
3. Atomically take both quadrature encoder deltas and accumulate them for the
   50 ms speed window.
4. Calculate the left/right target RPM from line PD or straight-line heading
   correction.
5. Run one PI loop per wheel and write the two PWM duties.

## Sensor state and error

Software channels are ordered from the physical left to the physical right:

```text
IR1  IR2  IR3  IR4  IR5  IR6
CH2  CH3  CH4  CH5  CH6  CH7
bit5 bit4 bit3 bit2 bit1 bit0
```

The sensor state is displayed and processed at the electrical level: `1` means
black line and `0` means white background. For example, `011000` means IR2
and IR3 are over black. Its position error is `(-3 + -1) / 2 = -2`, so the
controller turns left by reducing the left-wheel target and increasing the
right-wheel target.

The line weights are `-5, -3, -1, +1, +3, +5`. They represent the six evenly
spaced middle channels CH2 through CH7 on the eight-channel RYZD board. The
weighted average is kept as a floating-point value so adjacent sensor
combinations retain their intermediate positions. Positive error means the
line is on the right side of the sensor bar. The controller therefore increases
left-wheel target speed and decreases right-wheel target speed for positive
error, which turns the car right.

The current RYZD electrical level is used directly; no software inversion is
applied. Black turns the sensor LED off and produces `1`; white turns the LED
on and produces `0`. The OLED displays the raw GPIO level, while the control
loop uses the corresponding three-sample filtered state.

`LineSensor_GetSteeringError()` defines the response for all 64 six-bit input
states. The 20 proper, contiguous black groups produce these errors:

| Black width | State and error |
| --- | --- |
| 1 | `100000=-5`, `010000=-3`, `001000=-1`, `000100=+1`, `000010=+3`, `000001=+5` |
| 2 | `110000=-4`, `011000=-2`, `001100=0`, `000110=+2`, `000011=+4` |
| 3 | `111000=-3`, `011100=-1`, `001110=+1`, `000111=+3` |
| 4 | `111100=-2`, `011110=0`, `001111=+2` |
| 5 | `111110=-1`, `011111=+1` |

The remaining 44 states are neutral straight-ahead states: all white
`000000`, all black `111111`, and all 42 non-contiguous patterns such as
`100111`. Entering a neutral state clears the previous PD error and correction;
there is deliberately no lost-line search, last-direction hold, stop, or
reverse behavior.

Trackable states use the weighted PD correction with `LINE_KP=1.80` and
`LINE_KD=0.50`, limited to 10 RPM with a maximum change of 3 RPM per 10 ms
tick. The outer two sensors reach the correction limit sooner because their
physical distance from the center is larger.

## Reference-project comparison

The RYZD seven-channel example uses fixed wheel-speed branches. Its "simple
PID" example is a proportional nine-position table made from five single
channels and four adjacent-channel pairs; it does not execute integral or
derivative control. This project keeps that useful adjacent-pair interpolation
but generalizes it to every contiguous six-channel state with a weighted
centroid. Steering then uses PD, while the two encoders run independent PI
wheel-speed loops.

## Speed loop

The encoder convention follows the reference project:

```text
left_count  = encoder B delta (J2)
right_count = -encoder A delta (J1)
```

The default encoder model is 13 lines, two counted edges, and a 30:1 gearbox:
`780 counts per wheel revolution`.

The per-wheel controller uses the latest 50 ms speed measurement and writes
PWM every 10 ms. Its integral term advances once per 50 ms speed window:

```text
error     = target_rpm - measured_rpm
integral += error
duty      = feed_forward * target_rpm + 10 * error + 0.35 * integral
```

The integral is limited to `+/-1500` and is held when the PWM is saturated in
the same direction as the speed error. PWM duty is limited to `0..7800` for an
8000-count PWM period. Initial feed-forward values are 65 for the left wheel
and 58 for the right wheel.

## Startup and bench verification

1. Power up with the motor wheels lifted. The controller remains in
   `CTRL_STOP` until the C07A BLS key is pressed and released.
2. The first line-follow run uses a base target of about 14.5 RPM (5 cm/s).
   Verify that both
   wheels rotate forward and that both reported RPM values are positive.
3. If a wheel rotates backward, first swap that motor's two output wires or
   correct the corresponding `IN1/IN2` forward combination in
   `drivers/motor_driver.c`.
4. If a forward wheel reports negative RPM, reverse only its encoder sign in
   `control/vehicle_controller.c` and repeat the lifted-wheel test.
5. Place the sensor bar over black and white areas. Confirm the filtered state
   changes after about two 10 ms ticks and confirm left-to-right error signs.
6. Tune base speed first, then `LINE_KP`, then `LINE_KD`. Increase speed only
   after the two wheel RPM loops track the same target on a straight test.

The main application can select the modes through `Control_SetMode()`:

```c
Control_SetBaseSpeed(14.5f);
Control_SetMode(CTRL_LINE);
Control_SetMode(CTRL_STRAIGHT);
Control_SetMode(CTRL_STOP);
```

## Wheel mapping and target speed

The physical motor connectors are defined as `J1 = right wheel` and
`J2 = left wheel`. In the firmware this maps to `PB2/TIMA1 CCP0 + encoder A`
for J1 and `PB3/TIMA1 CCP1 + encoder B` for J2. The initial line-follow target
is 5 cm/s. With the 6.6 cm wheel diameter in the reference data this is about
14.5 RPM. The OLED shows each measured left/right wheel speed and its separate
target in cm/s, the six filtered IR bits, and the line error.
