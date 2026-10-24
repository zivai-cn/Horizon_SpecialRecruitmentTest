#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;

void USART1_Transmit(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // __USART_H__
