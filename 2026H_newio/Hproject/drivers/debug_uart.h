#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

void DebugUart_WriteByte(uint8_t data);
void DebugUart_WriteString(const char *text);
void DebugUart_WriteUint32(uint32_t value);
void DebugUart_WriteInt32(int32_t value);

#endif
