from machine import FPIOA, UART


class Mspm0UartLink:
    """Line-oriented UART2 link for the ALIENTEK K230D BOX J8 port."""

    def __init__(self, baudrate=115200, max_line_length=95):
        self._max_line_length = max_line_length
        self._rx_line = bytearray()
        self._line_queue = []
        self._discarding_line = False

        fpioa = FPIOA()
        fpioa.set_function(44, FPIOA.UART2_TXD)
        fpioa.set_function(45, FPIOA.UART2_RXD)
        self._uart = UART(
            UART.UART2,
            baudrate=baudrate,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )

    def send_line(self, message):
        if isinstance(message, str):
            message = message.encode()
        self._uart.write(message + b"\r\n")

    def poll_line(self):
        """Return one decoded line when available, otherwise return None."""
        if self._line_queue:
            return self._line_queue.pop(0)

        data = self._uart.read(128)
        if data is None:
            return None

        for value in data:
            if self._discarding_line:
                if value == 10:
                    self._discarding_line = False
                continue

            if value == 10:
                if not self._rx_line:
                    continue
                line = bytes(self._rx_line).decode()
                self._rx_line = bytearray()
                if len(self._line_queue) >= 4:
                    self._line_queue.pop(0)
                self._line_queue.append(line)
                continue

            if value == 13:
                continue

            if len(self._rx_line) >= self._max_line_length:
                self._rx_line = bytearray()
                self._discarding_line = True
                continue

            self._rx_line.append(value)

        if self._line_queue:
            return self._line_queue.pop(0)
        return None
