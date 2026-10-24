#ifndef APP_BLUETOOTH_TEST_H
#define APP_BLUETOOTH_TEST_H

#include <stdint.h>

void App_BluetoothTest_Init(void);
int BluetoothTest_SendData(const uint8_t *data, uint16_t length);

#endif