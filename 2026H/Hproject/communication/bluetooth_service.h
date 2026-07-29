#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#include <stdint.h>

void BluetoothService_Init(uint32_t now_ms);
void BluetoothService_Process(uint32_t now_ms);

#endif
