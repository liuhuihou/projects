/*
 * Host stubs for the peripherals the code under test writes to. The transmit
 * path records bytes instead of shifting them out a UART; the stepper records
 * the last speed command so the balance harness can report it.
 */

#include "ti_msp_dl_config.h"
#include "stepper_driver.h"

uint8_t  stub_tx_buf[256];
uint32_t stub_tx_len;

void DL_UART_Main_transmitDataBlocking(void *uart, uint8_t byte)
{
    (void)uart;
    if (stub_tx_len < sizeof(stub_tx_buf)) {
        stub_tx_buf[stub_tx_len++] = byte;
    }
}

/* Stepper state, observable by the harness. */
int32_t stub_stepper_speed;
int     stub_stepper_enabled;

void Stepper_Init(void)    { stub_stepper_speed = 0; stub_stepper_enabled = 0; }
void Stepper_Enable(void)  { stub_stepper_enabled = 1; }
void Stepper_Disable(void) { stub_stepper_enabled = 0; }

void Stepper_SetSpeed(int32_t steps_per_sec)
{
    stub_stepper_speed = steps_per_sec;
}

int32_t Stepper_GetPosition(void)   { return 0; }
void    Stepper_ResetPosition(void) { }
