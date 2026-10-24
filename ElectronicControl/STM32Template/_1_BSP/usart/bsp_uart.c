#include "bsp_uart.h"
#include <string.h>

//实例池
static USARTInstance  s_pool[USART_MAX_INST];
static uint8_t        usart_idx = 0;

//按huart查找对应的实例
static USARTInstance *usart_find_instance(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < usart_idx; i++)
        if (s_pool[i].huart == huart)
            return &s_pool[i];
    return NULL;
}

//接收模式是否合法
static int usart_rx_mode_is_valid(USART_MODE mode)
{
    return mode == UART_MODE_IT || mode == UART_MODE_DMA;
}

//发送模式是否合法
static int usart_tx_mode_is_valid(USART_MODE mode)
{
    return mode == UART_MODE_POLLING ||
           mode == UART_MODE_IT ||
           mode == UART_MODE_DMA;
}

//发送完成：设置发送标志位为闲并通知模块
static void usart_tx_complete(USARTInstance *ins)
{
    ins->tx_busy = 0;
    if (ins->on_tx_done != NULL)
        ins->on_tx_done(ins);
}

//启动发送并设置忙标志
static int usart_start_tx(USARTInstance *ins, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef result;

    ins->tx_busy = 1;//设置发送标志位为忙

    switch (ins->tx_mode)//根据发送模式设置发送函数
    {
        case UART_MODE_POLLING:
            result = HAL_UART_Transmit(ins->huart, (uint8_t *)buf, len, 100);
            if (result == HAL_OK)
            {
                usart_tx_complete(ins);//轮询发送完成，直接设置空闲并回调
            }
            break;
        case UART_MODE_IT:
            result = HAL_UART_Transmit_IT(ins->huart, (uint8_t *)buf, len);
            break;
        case UART_MODE_DMA:
            result = HAL_UART_Transmit_DMA(ins->huart, (uint8_t *)buf, len);
            break;
        default:
            result = -1;
            break;
    }

    if (result != HAL_OK)//发送失败
    {
        ins->tx_busy = 0;
        return -1;
    }
    return 0;
}

//启动接收
static int usart_start_rx(USARTInstance *ins)
{
    HAL_StatusTypeDef result;

    switch (ins->rx_mode)//根据接收模式设置接收函数
    {
        case UART_MODE_DMA:
            if (ins->huart->hdmarx == NULL)
                return -1;
            result = HAL_UARTEx_ReceiveToIdle_DMA(ins->huart, ins->rx_buf,ins->rx_size);
            if (result == HAL_OK)
                __HAL_DMA_DISABLE_IT(ins->huart->hdmarx, DMA_IT_HT);
            break;
        case UART_MODE_IT:
            result = HAL_UARTEx_ReceiveToIdle_IT(ins->huart, ins->rx_buf,ins->rx_size);
            break;
        default:
            return -1;
    }

    return result == HAL_OK ? 0 : -1;
}


//注册串口实例，成功返回实例指针，失败返回 NULL
USARTInstance *USARTRegister(const USART_Init_Config_S *cfg)
{
    if (cfg == NULL || cfg->huart == NULL || cfg->rx_buf == NULL || cfg->rx_size == 0 || usart_idx >= USART_MAX_INST || usart_find_instance(cfg->huart) != NULL)
        return NULL;
    if (!usart_rx_mode_is_valid(cfg->rx_mode) || !usart_tx_mode_is_valid(cfg->tx_mode))
        return NULL;

    USARTInstance *ins = &s_pool[usart_idx++];
    memset(ins, 0, sizeof(*ins));

    ins->huart      = cfg->huart;
    ins->rx_mode    = cfg->rx_mode;
    ins->tx_mode    = cfg->tx_mode;
    ins->rx_buf     = cfg->rx_buf;
    ins->rx_size    = cfg->rx_size;
    ins->on_rx      = cfg->on_rx;
    ins->on_tx_done = cfg->on_tx_done;
    ins->owner      = cfg->owner;

    if (usart_start_rx(ins))
    {
        usart_idx--;
        return NULL;
    }
    return ins;
}

//发送一帧，忙则返回 -1，由上层（如 FreeRTOS 队列）负责排队
int USARTSend(USARTInstance *ins, const uint8_t *buf, uint16_t len)
{
    if (ins == NULL || buf == NULL || len == 0 || ins->tx_busy)
        return -1;
    return usart_start_tx(ins, buf, len);
}

//切换收发模式：发送必须空闲，接收模式切换后自动重新启动接收
int USARTSetMode(USARTInstance *ins, USART_MODE rx_mode, USART_MODE tx_mode)
{
    if (ins == NULL || ins->tx_busy ||
        !usart_rx_mode_is_valid(rx_mode) ||
        !usart_tx_mode_is_valid(tx_mode))
        return -1;

    if (rx_mode != ins->rx_mode)
    {
        USART_MODE old_rx_mode = ins->rx_mode;

        if (HAL_UART_AbortReceive(ins->huart) != HAL_OK)
            return -1;

        ins->rx_mode = rx_mode;
        if (usart_start_rx(ins) != 0)
        {
            //新配置启动失败，恢复原接收模式
            ins->rx_mode = old_rx_mode;
            (void)usart_start_rx(ins);
            return -1;
        }
    }

    ins->tx_mode = tx_mode;
    return 0;
}

//HAL 接收完成回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    USARTInstance *ins = usart_find_instance(huart);
    if (ins == NULL) return;

    ins->rx_len = Size;
    if (ins->on_rx != NULL)
        ins->on_rx(ins);            //通知模块

    usart_start_rx(ins);
}

//HAL 发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    USARTInstance *ins = usart_find_instance(huart);
    if (ins == NULL) return;

    usart_tx_complete(ins);         //通知模块发送完成
}