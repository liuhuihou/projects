# C07A Bluetooth UART test

Target: TI MSPM0G3507 on the WHEELTEC C07A core board.

## Wiring

- Bluetooth TXD -> PB7 / UART1_RX
- Bluetooth RXD -> PB6 / UART1_TX
- Bluetooth GND -> board GND
- Bluetooth VCC -> 3.3 V, according to the C07A reference wiring

## Serial settings

- Bluetooth UART1: 9600 baud, 8 data bits, no parity, 1 stop bit
- Type-C debug UART0: 115200 baud, 8 data bits, no parity, 1 stop bit

After reset, the Bluetooth link sends `CAR_MASTER:BOOT`. It then sends
`CAR_MASTER:HELLO` once per second and toggles the PB9 user LED. Bytes received
from Bluetooth are forwarded unchanged to the Type-C debug serial port.

Open `demo1.uvprojx` in Keil 5, build, flash, and open the board's Type-C COM
port at 115200 baud. Open the receiving Bluetooth COM port at 9600 baud.
