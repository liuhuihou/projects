# Bluetooth UART integration

## Hardware

The Bluetooth module uses MSPM0 UART1 at 9600 baud, 8 data bits, no parity,
and one stop bit:

```text
Bluetooth TXD -> PB7 / UART1_RX
Bluetooth RXD -> PB6 / UART1_TX
Bluetooth GND -> S28A/C07A GND
Bluetooth VCC -> voltage specified by the module
```

The Type-C debug serial port uses UART0 on PA10/PA11 at 115200 baud. SysConfig
owns both UART pin mux and baud-rate settings.

## Module boundaries

- `drivers/bluetooth_uart.c` owns only UART1 byte transmission and reception.
- `drivers/debug_uart.c` owns only UART0 debug transmission.
- `communication/bluetooth_service.c` owns boot, heartbeat, and forwarding
  policy.
- `app/app_main.c` initializes and periodically services Bluetooth. It does not
  parse Bluetooth data or alter motor and line-follow control from Bluetooth.

No files, startup code, generated configuration, or SDK copies from the
standalone `projects/bluetooth test` project are included in this project.

## Runtime behavior

After initialization, UART1 sends:

```text
CAR_MASTER:BOOT
```

It then sends the following message once per second, including while the car
is waiting for the BLS mode-selection/start key:

```text
CAR_MASTER:HELLO
```

Every byte received from Bluetooth is forwarded unchanged to the Type-C debug
UART. The debug UART also reports each transmitted heartbeat. This preserves
the observable behavior of the standalone Bluetooth test without coupling it
to vehicle control.
