#include "bluetooth_uart.h"

#include "board_hardware.h"

void BluetoothUart_Init(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(HW_BLUETOOTH_UART)) {
        (void)DL_UART_Main_receiveData(HW_BLUETOOTH_UART);
    }
}

uint8_t BluetoothUart_TryRead(uint8_t *data)
{
    if (data == 0 || DL_UART_Main_isRXFIFOEmpty(HW_BLUETOOTH_UART)) {
        return 0U;
    }

    *data = DL_UART_Main_receiveData(HW_BLUETOOTH_UART);
    return 1U;
}

void BluetoothUart_WriteByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(HW_BLUETOOTH_UART, data);
}

void BluetoothUart_WriteString(const char *text)
{
    if (text == 0) {
        return;
    }

    while (*text != '\0') {
        BluetoothUart_WriteByte((uint8_t)*text);
        ++text;
    }
}
