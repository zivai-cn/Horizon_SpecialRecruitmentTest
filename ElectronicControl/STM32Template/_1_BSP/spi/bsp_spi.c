#include "bsp_spi.h"
#include <string.h>

//当前操作类型
#define SPI_OP_NONE 0//无操作
#define SPI_OP_TX   1//发送操作
#define SPI_OP_RX   2//接收操作
#define SPI_OP_TXRX 3//发送接收操作

#define SPI_BSY_TIMEOUT 100000     //等待BSY清零的循环次数上限

//实例池
static SPIInstance s_pool[SPI_MAX_INST];
static uint8_t      spi_idx = 0;

//查找正在占用该SPI总线的实例
static SPIInstance *spi_find_who_got(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < spi_idx; i++)
    {
        if (s_pool[i].hspi == hspi && s_pool[i].busy)
        return &s_pool[i];
    }

    return NULL;//未找到占用总线的实例
}

//按总线和片选查找实例，用于注册查重
static SPIInstance *spi_find_is_exist(const SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    for (uint8_t i = 0; i < spi_idx; i++)
    {
        if (s_pool[i].hspi == hspi && s_pool[i].cs_port == cs_port && s_pool[i].cs_pin == cs_pin)//查找匹配的实例
        return &s_pool[i];
    }   

    return NULL;
}

//发送完成时，最后一个字节可能还在移位，等BSY清零后再释放片选
static void spi_wait_busy(SPI_HandleTypeDef *hspi)
{
    uint32_t timeout = SPI_BSY_TIMEOUT;//等待BSY清零的循环次数上限
    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) != RESET && --timeout != 0U)
    {
    }
}

//占用总线并拉低片选
static int spi_begin(SPIInstance *ins, uint8_t op)
{
    if (ins->busy || spi_find_who_got(ins->hspi) != NULL)
        return -1;

    ins->op   = op;
    ins->busy = 1;
    HAL_GPIO_WritePin(ins->cs_port, ins->cs_pin, GPIO_PIN_RESET);
    return 0;
}

//拉高片选并清除忙标志
static void spi_end(SPIInstance *ins)
{
    HAL_GPIO_WritePin(ins->cs_port, ins->cs_pin, GPIO_PIN_SET);
    ins->busy = 0;
    ins->op   = SPI_OP_NONE;
}

//传输完成处理：从HAL句柄定位实例，释放片选并通知模块
static void spi_complete(SPI_HandleTypeDef *hspi)
{
    SPIInstance *ins = spi_find_who_got(hspi);//查找当前占用总线的实例
    if (ins == NULL)
        return;

    uint8_t op = ins->op;//获取当前操作类型

    if (op == SPI_OP_TX)//发送完成
    {
        spi_wait_busy(hspi);         //仅发送时BSY可能仍为1
    }

    spi_end(ins);//释放片选

    switch (op)//根据操作类型通知模块完成
    {
        case SPI_OP_TX:
            if (ins->on_tx_done != NULL)
                ins->on_tx_done(ins);
            break;

        case SPI_OP_RX:
        case SPI_OP_TXRX:
            if (ins->on_rx != NULL)
                ins->on_rx(ins);
            break;

        case SPI_OP_NONE:
        default:
            break;
    }
}

//注册实例，成功返回实例指针，失败返回NULL
SPIInstance *SPIRegister(const SPI_Init_Config_S *cfg)
{
    //检查参数
    if (cfg == NULL)
        return NULL;
    if (cfg->hspi == NULL)
        return NULL;
    if (cfg->cs_port == NULL)
        return NULL;
    if (cfg->cs_pin == 0)
        return NULL;
    if (cfg->mode != SPI_BLOCK_MODE && cfg->mode != SPI_IT_MODE && cfg->mode != SPI_DMA_MODE)
        return NULL;
    if (spi_idx >= SPI_MAX_INST || spi_find_is_exist(cfg->hspi, cfg->cs_port, cfg->cs_pin) != NULL)
        return NULL;
    //初始化实例
    SPIInstance *ins = &s_pool[spi_idx++];
    memset(ins, 0, sizeof(*ins));//清空实例内存
    //初始化实例成员
    ins->hspi       = cfg->hspi;
    ins->cs_port    = cfg->cs_port;
    ins->cs_pin     = cfg->cs_pin;
    ins->mode       = cfg->mode;
    ins->on_rx      = cfg->on_rx;
    ins->on_tx_done = cfg->on_tx_done;
    ins->owner      = cfg->owner;
    //初始化片选引脚为高电平
    HAL_GPIO_WritePin(ins->cs_port, ins->cs_pin, GPIO_PIN_SET);
    return ins;
}

//发送一帧
int SPITransmit(SPIInstance *ins, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef result;
    //检查参数
    if (ins == NULL || buf == NULL || len == 0 ||
        spi_begin(ins, SPI_OP_TX))
        return -1;
    //根据模式选择发送函数
    switch (ins->mode)
    {
        case SPI_BLOCK_MODE:
            result = HAL_SPI_Transmit(ins->hspi, (uint8_t *)buf, len,
                                      SPI_TIMEOUT_MS);
            if (result == HAL_OK)
            {
                spi_wait_busy(ins->hspi);//等待最后一个字节移出
                spi_end(ins);//释放片选
                if (ins->on_tx_done != NULL)
                    ins->on_tx_done(ins);
            }
            break;
        case SPI_IT_MODE:
            result = HAL_SPI_Transmit_IT(ins->hspi, (uint8_t *)buf, len);
            break;
        case SPI_DMA_MODE:
            result = HAL_SPI_Transmit_DMA(ins->hspi, (uint8_t *)buf, len);
            break;
        default:
            result = HAL_ERROR;
            break;
    }
    //检查发送结果,如果失败则释放总线
    if (result != HAL_OK)
    {
        spi_end(ins);
        return -1;
    }

    return 0;
}

//接收一帧
int SPIReceive(SPIInstance *ins, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef result;

    if (ins == NULL || buf == NULL || len == 0 || spi_begin(ins, SPI_OP_RX))
        return -1;

    ins->rx_buf  = buf;
    ins->rx_size = len;

    switch (ins->mode)
    {
        case SPI_BLOCK_MODE:
            result = HAL_SPI_Receive(ins->hspi, buf, len, SPI_TIMEOUT_MS);
            if (result == HAL_OK)
            {
                spi_end(ins);//释放片选
                if (ins->on_rx != NULL)
                    ins->on_rx(ins);
            }
            break;
        case SPI_IT_MODE:
            result = HAL_SPI_Receive_IT(ins->hspi, buf, len);
            break;
        case SPI_DMA_MODE:
            result = HAL_SPI_Receive_DMA(ins->hspi, buf, len);
            break;
        default:
            result = HAL_ERROR;
            break;
    }

    if (result != HAL_OK)
    {
        spi_end(ins);
        return -1;
    }

    return 0;
}

//全双工收发
int SPITransmitAndReceive(SPIInstance *ins, uint8_t *rx_buf, const uint8_t *tx_buf, uint16_t len)
{
    HAL_StatusTypeDef result;

    if (ins == NULL || rx_buf == NULL || tx_buf == NULL || len == 0 ||
        spi_begin(ins, SPI_OP_TXRX))
        return -1;

    ins->rx_buf  = rx_buf;
    ins->rx_size = len;

    switch (ins->mode)
    {
        case SPI_BLOCK_MODE:
            result = HAL_SPI_TransmitReceive(ins->hspi, (uint8_t *)tx_buf,
                                             rx_buf, len, SPI_TIMEOUT_MS);
            if (result == HAL_OK)
            {
                spi_wait_busy(ins->hspi);//等待最后一个字节移出
                spi_end(ins);//释放片选
                if (ins->on_rx != NULL)
                    ins->on_rx(ins);
            }
            break;
        case SPI_IT_MODE:
            result = HAL_SPI_TransmitReceive_IT(ins->hspi, (uint8_t *)tx_buf,
                                                rx_buf, len);
            break;
        case SPI_DMA_MODE:
            result = HAL_SPI_TransmitReceive_DMA(ins->hspi, (uint8_t *)tx_buf,
                                                 rx_buf, len);
            break;
        default:
            result = HAL_ERROR;
            break;
    }

    if (result != HAL_OK)
    {
        spi_end(ins);
        return -1;
    }

    return 0;
}

//切换收发模式，仅空闲时可用
int SPISetMode(SPIInstance *ins, SPI_TXRX_MODE_e mode)
{
    if (ins == NULL || ins->busy || (mode != SPI_BLOCK_MODE && mode != SPI_IT_MODE && mode != SPI_DMA_MODE))
        return -1;

    ins->mode = mode;
    return 0;
}

//HAL发送完成回调
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_complete(hspi);
}

//HAL接收完成回调
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_complete(hspi);
}

//HAL全双工完成回调
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_complete(hspi);
}

//HAL错误回调：同样释放片选和忙标志，避免总线死锁
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    spi_complete(hspi);
}
