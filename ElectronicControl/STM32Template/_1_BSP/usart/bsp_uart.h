#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include "main.h"

#define USART_MAX_INST 3   //最大串口实例数

//串口工作模式
typedef enum
{
    UART_MODE_POLLING = 0,
    UART_MODE_IT,
    UART_MODE_DMA,
} USART_MODE;

typedef struct USARTInstance USARTInstance;

//回调函数指针，参数为触发回调的实例
typedef void (*uart_callback_t)(USARTInstance *instance);

//串口实例
struct USARTInstance
{
    UART_HandleTypeDef *huart;          //HAL串口句柄

    USART_MODE          rx_mode;        //接收模式
    USART_MODE          tx_mode;        //发送模式

    uint8_t            *rx_buf;         //接收缓冲（模块提供）
    uint16_t            rx_size;        //缓冲大小
    volatile uint16_t   rx_len;         //本次收到的数据长度

    uart_callback_t     on_rx;          //收完一帧回调
    uart_callback_t     on_tx_done;     //发送完成回调
    void               *owner;          //指向所属模块

    volatile uint8_t    tx_busy;        //发送忙标志
};

//注册配置
typedef struct
{
    UART_HandleTypeDef *huart;
    USART_MODE          rx_mode;
    USART_MODE          tx_mode;
    uint8_t            *rx_buf;
    uint16_t            rx_size;
    uart_callback_t     on_rx;
    uart_callback_t     on_tx_done;
    void               *owner;
} USART_Init_Config_S;

//注册实例函数
USARTInstance *USARTRegister(const USART_Init_Config_S *cfg);

//发送一帧；轮询模式下完成回调在函数返回前执行
int USARTSend(USARTInstance *ins, const uint8_t *buf, uint16_t len);

//切换收发模式；发送忙时禁止修改，接收模式切换后自动重启接收
int USARTSetMode(USARTInstance *ins, USART_MODE rx_mode,
                 USART_MODE tx_mode);

#endif