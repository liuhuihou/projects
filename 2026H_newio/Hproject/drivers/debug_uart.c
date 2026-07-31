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

void DebugUart_WriteUint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < (uint8_t)sizeof(digits));

    while (count > 0U) {
        DebugUart_WriteByte((uint8_t)digits[--count]);
    }
}

void DebugUart_WriteInt32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        DebugUart_WriteByte((uint8_t)'-');
        /* Avoid signed overflow for INT32_MIN. */
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    DebugUart_WriteUint32(magnitude);
}
