#include "debug_uart.h"
#include "board_hardware.h"

void DebugUart_WriteByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(HW_USB_UART, data);
}

void DebugUart_WriteString(const char *text)
{
    if (text == 0) return;
    while (*text != '\0') {
        DebugUart_WriteByte((uint8_t)*text);
        ++text;
    }
}
