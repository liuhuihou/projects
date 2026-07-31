#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

/*
 * Host stand-in for the SysConfig-generated header, so the protocol code can be
 * compiled and tested with plain gcc. Only the handful of symbols the driver
 * under test actually touches are declared.
 */

#include <stdint.h>

#define K230_INST       ((void *)0)
#define K230_BAUD_RATE  (115200U)

/* Transmit capture: the harness reads these back to check the bytes the driver
 * put on the wire against what the K230 decoder expects. */
extern uint8_t  stub_tx_buf[256];
extern uint32_t stub_tx_len;

void DL_UART_Main_transmitDataBlocking(void *uart, uint8_t byte);

#endif
