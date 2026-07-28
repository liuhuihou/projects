# S28A/C07A Line Follower

MSPM0G3507 firmware for the S28A baseboard and C07A core board.

This directory contains one application project: `s28a_c07a_line_follower`.

## Project Layout

```text
app/       Application entry point and user-facing speed configuration
board/     Stable board-level pin and peripheral aliases
config/    SysConfig source of truth
control/   Line-following and per-wheel speed control
drivers/   Encoder, motor, RYZD sensor, and OLED drivers
generated/ SysConfig-generated C/H files; do not edit manually
docs/      Wiring and control documentation
keil/      Keil uVision project definition
build/     Compiler output and HEX files
tools/     Repeatable local build helpers
```

## Common Changes

- Vehicle speed: `app/app_config.h`, `APP_START_SPEED_CM_S`
- Speed-loop gains: `control/control_config.h`
- Line-sensor polarity and pin aliases: `board/board_hardware.h`
- Pin mux and peripheral assignment: `config/board.syscfg`

## Build

Open `keil/s28a_c07a_line_follower.uvprojx` in Keil uVision and build/rebuild.
The pre-build step runs `tools/generate_syscfg.bat`; the resulting HEX is in
`build/keil/Objects/s28a_c07a_line_follower.hex`.

The last verified Keil build used ARMCLANG V6.21 and completed with 0 errors
and 0 warnings.

See `docs/hardware_wiring.md` for S28A/C07A wiring and
`docs/line_following.md` for the six-channel control algorithm.
