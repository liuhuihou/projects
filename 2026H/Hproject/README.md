# H Balance Ball Control System

MSPM0G3507 firmware framework for the H-problem vehicle. The hardware layer
and line-following controller are reused from `MSPM0G3507_empty_keil`.

## Project layout

```text
app/       application entry point, buttons, mode dispatcher, task wrappers
board/     stable hardware aliases; wiring remains unchanged
config/    SysConfig source of truth
generated/ SysConfig-generated files; do not edit manually
control/   reused line-following and wheel-speed controller
drivers/   motor, encoder, line sensor, OLED, and UART drivers
communication/ Bluetooth service
keil/      Keil uVision project
docs/      wiring and framework notes
```

## Button operation

- `RESET/SW2`: hardware reset through `NRST`; the firmware does not sample it.
- `BLS/SW3`: `PA18`, active high.
- One BLS click starts the currently selected question after the double-click
  decision window expires.
- Two BLS clicks within 350 ms select the next question and do not start it.

## Mode mapping

| Mode | Current framework behavior |
| --- | --- |
| Q2 | line-follow task wrapper |
| Q3 | balance task stub only |
| Q4 | line-follow wrapper plus balance stub |
| Q5 | line-follow task wrapper |
| Q6 | line-follow wrapper plus target balance stub |

The line-follow controller remains in `control/vehicle_controller.c`; lap
detection and stop conditions are task-level work. The balance module exposes
its lifecycle interface but intentionally contains no MPU6050 or actuator
algorithm yet.

## Build

Open `keil/s28a_c07a_line_follower.uvprojx` in Keil uVision and build. The
pre-build step runs `tools/generate_syscfg.bat`. SysConfig must report success
before compiling.
