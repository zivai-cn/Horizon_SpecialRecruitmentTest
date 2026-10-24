#include "usart.h"

// USART1 发送封装：应用层统一使用该接口
void USART1_Transmit(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms);
}
