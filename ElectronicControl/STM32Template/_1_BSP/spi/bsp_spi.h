#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>
#include "main.h"

#define SPI_MAX_INST        4       //最大从机实例数
#define SPI_TIMEOUT_MS      100     //阻塞模式默认超时(ms)

//SPI收发工作模式
typedef enum
{
    SPI_BLOCK_MODE = 0,             //阻塞模式
    SPI_IT_MODE,                    //中断模式
    SPI_DMA_MODE,                   //DMA模式
} SPI_TXRX_MODE_e;

typedef struct SPIInstance SPIInstance;//先定义，再使用

//传输完成回调，参数为触发回调的实例
typedef void (*spi_callback_t)(SPIInstance *instance);

//SPI实例
struct SPIInstance
{
    SPI_HandleTypeDef *hspi;        //HAL SPI句柄
    GPIO_TypeDef      *cs_port;     //片选端口
    uint16_t           cs_pin;      //片选引脚，低电平有效

    SPI_TXRX_MODE_e    mode;        //收发模式

    uint8_t           *rx_buf;      //本次接收缓冲区(模块提供)
    uint16_t           rx_size;     //本次接收长度

    spi_callback_t     on_rx;       //接收完成回调
    spi_callback_t     on_tx_done;  //发送完成回调
    void              *owner;       //指向所属模块

    volatile uint8_t   busy;        //总线忙标志
    volatile uint8_t   op;          //当前操作类型，仅BSP内部使用
};

//注册配置
typedef struct
{
    SPI_HandleTypeDef *hspi;        //HAL SPI句柄
    GPIO_TypeDef      *cs_port;     //片选端口
    uint16_t           cs_pin;      //片选引脚，低电平有效

    SPI_TXRX_MODE_e    mode;        //收发模式  

    spi_callback_t     on_rx;       //接收完成回调
    spi_callback_t     on_tx_done;  //发送完成回调
    void              *owner;       //指向所属模块
} SPI_Init_Config_S;

//注册实例，成功返回实例指针，失败返回NULL
SPIInstance *SPIRegister(const SPI_Init_Config_S *cfg);

//发送一帧；阻塞模式下完成回调在函数返回前执行
int SPITransmit(SPIInstance *ins, const uint8_t *buf, uint16_t len);

//接收一帧；阻塞模式下完成回调在函数返回前执行
int SPIReceive(SPIInstance *ins, uint8_t *buf, uint16_t len);

//全双工收发；完成后调用on_rx
int SPITransmitAndReceive(SPIInstance *ins, uint8_t *rx_buf,
                          const uint8_t *tx_buf, uint16_t len);

//切换收发模式，仅空闲时可用
int SPISetMode(SPIInstance *ins, SPI_TXRX_MODE_e mode);

#endif
