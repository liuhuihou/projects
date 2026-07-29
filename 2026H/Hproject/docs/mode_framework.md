# Mode framework

## Runtime flow

```text
power on
  -> hardware initialization
  -> display Q2 and wait
  -> BLS single click pending
       -> no second click in 350 ms: start selected mode
       -> second click in 350 ms: select next mode and keep waiting
  -> run selected task wrappers
```

The application polls buttons in the foreground while the existing 10 ms
`TIMG0` interrupt continues to run the wheel and line controller. The task
wrappers therefore coordinate the controller and future stop conditions without
duplicating its control loop.

## Hardware boundary

`SW2/RESET` is connected to `NRST` and remains a hardware reset. It is not a
GPIO event and no software reset alias is added. `SW3/BLS` is `PA18` with a
pull-down and is debounced in `app/button_input.c`.

`PB8` is retained in the generated board configuration as the S28A expansion
key input. It is not used for RESET or BLS mode selection.

## Task boundary

`LineFollowTask` calls the existing `Control_SetMode(CTRL_LINE)` and owns the
future lap and finish conditions. `BalanceController` currently owns only
target/state/lifecycle placeholders. MPU6050 acquisition, ball-position input,
and actuator output are intentionally deferred.
