#include "bluetooth_service.h"

#include "bluetooth_uart.h"
#include "debug_uart.h"

#define BLUETOOTH_HELLO_INTERVAL_MS (1000U)

static uint32_t s_last_hello_ms;
static uint8_t s_initialized;

void BluetoothService_Init(uint32_t now_ms)
{
    BluetoothUart_Init();
    s_last_hello_ms = now_ms;
    s_initialized = 1U;

    DebugUart_WriteString(
        "\r\nC07A Bluetooth UART integrated.\r\n"
        "UART1: PB6(TX), PB7(RX), 9600-8-N-1.\r\n");
    BluetoothUart_WriteString("CAR_MASTER:BOOT\r\n");
}

void BluetoothService_Process(uint32_t now_ms)
{
    uint8_t data;

    if (s_initialized == 0U) {
        return;
    }

    /* Preserve the reference project's byte-for-byte Bluetooth-to-USB bridge. */
    while (BluetoothUart_TryRead(&data) != 0U) {
        DebugUart_WriteByte(data);
    }

    if ((uint32_t)(now_ms - s_last_hello_ms) >=
        BLUETOOTH_HELLO_INTERVAL_MS) {
        s_last_hello_ms = now_ms;
        BluetoothUart_WriteString("CAR_MASTER:HELLO\r\n");
        DebugUart_WriteString("[BT TX] CAR_MASTER:HELLO\r\n");
    }
}
