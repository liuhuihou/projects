#include <stdint.h>
#include "ti_msp_dl_config.h"

static volatile uint32_t g_milliseconds;

/*
 * MSPM0 flash programming is performed in 64-bit units. Keep this explicit
 * word in the image so the final Keil load region is a multiple of 8 bytes.
 */
__attribute__((used, section(".flash_padding")))
const uint32_t g_flash_padding = 0xFFFFFFFFU;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static void uart_putc(UART_Regs *uart, uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(uart, data);
}

static void uart_write(UART_Regs *uart, const char *text)
{
    while (*text != '\0') {
        uart_putc(uart, (uint8_t) *text);
        text++;
    }
}

int main(void)
{
    uint32_t lastTransmit = 0U;

    SYSCFG_DL_init();
    __enable_irq();

    uart_write(DEBUG_UART_INST,
        "\r\nC07A Bluetooth UART test started.\r\n"
        "UART1: PB6(TX), PB7(RX), 9600-8-N-1.\r\n");
    uart_write(BT_UART_INST, "CAR_MASTER:BOOT\r\n");

    while (1) {
        /* Forward every Bluetooth byte to the board Type-C debug UART. */
        while (!DL_UART_Main_isRXFIFOEmpty(BT_UART_INST)) {
            uart_putc(DEBUG_UART_INST,
                DL_UART_Main_receiveData(BT_UART_INST));
        }

        /* Send a visible link-test message once per second. */
        if ((uint32_t) (g_milliseconds - lastTransmit) >= 1000U) {
            lastTransmit = g_milliseconds;
            uart_write(BT_UART_INST, "CAR_MASTER:HELLO\r\n");
            uart_write(DEBUG_UART_INST, "[BT TX] CAR_MASTER:HELLO\r\n");
            DL_GPIO_togglePins(STATUS_LED_PORT, STATUS_LED_PIN);
        }
    }
}
