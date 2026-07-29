#ifndef BLUETOOTH_UART_H
#define BLUETOOTH_UART_H

#include <stdint.h>

void BluetoothUart_Init(void);
uint8_t BluetoothUart_TryRead(uint8_t *data);
void BluetoothUart_WriteByte(uint8_t data);
void BluetoothUart_WriteString(const char *text);

#endif
