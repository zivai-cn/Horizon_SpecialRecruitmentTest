#include "w25q64.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>



// 写入 24 位地址，W25Q64 使用大端格式
static void w25q64_set_address(uint8_t *buffer, uint32_t address)
{
    buffer[1] = (uint8_t)(address >> 16);
    buffer[2] = (uint8_t)(address >> 8);
    buffer[3] = (uint8_t)address;
}

// 读取状态寄存器 1，bit0 为 WIP 忙标志，bit1 为 WEL 写使能标志
static int w25q64_read_status(W25Q64Data *data, uint8_t *status)
{
    uint8_t tx[2] = {W25Q64_CMD_READ_STATUS, 0xFFU};
    uint8_t rx[2];

    // 命令和返回状态必须在同一次 CS 事务中完成
    if (SPITransmitAndReceive(data->instance, rx, tx, sizeof(tx)) != 0)
        return -1;

    *status = rx[1];
    return 0;
}

// 发送写使能命令，并确认 WEL 已经置位
static int w25q64_write_enable(W25Q64Data *data)
{
    uint8_t command = W25Q64_CMD_WRITE_ENABLE;
    uint8_t status;

    if (SPITransmit(data->instance, &command, 1) != 0)
        return -1;
    if (w25q64_read_status(data, &status) != 0)
        return -1;

    return ((status & 0x02U) != 0U) ? 0 : -1;
}

// 轮询 WIP，直到写入或擦除完成；超时返回 -1
static int w25q64_wait_idle(W25Q64Data *data, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t  status;

    for (;;)
    {
        if (w25q64_read_status(data, &status) != 0)
            return -1;

        // WIP=0 表示芯片空闲
        if ((status & 0x01U) == 0U)
            return 0;

        if ((HAL_GetTick() - start) >= timeout_ms)
            return -1;

        // 避免忙等，让出 CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 从当前页内地址开始编程一页，调用前必须保证目标区域已擦除且不跨页
static int w25q64_page_program(W25Q64Data *data, uint32_t address,
                               const uint8_t *buffer, uint16_t length)
{
    // 等待上一次操作完成，避免写使能命令被忽略
    if (w25q64_wait_idle(data, W25Q64_TIMEOUT_MS) != 0)
        return -1;
    if (w25q64_write_enable(data) != 0)
        return -1;

    // 页编程命令、24 位地址和数据必须在同一次 CS 事务中发送
    data->tx_buf[0] = W25Q64_CMD_PAGE_PROGRAM;
    w25q64_set_address(data->tx_buf, address);
    // 复制数据到发送缓冲区
    memcpy(&data->tx_buf[4], buffer, length);

    if (SPITransmit(data->instance, data->tx_buf,
                    (uint16_t)(length + 4U)) != 0)
        return -1;

    return w25q64_wait_idle(data, W25Q64_TIMEOUT_MS);
}

// 初始化模块句柄、注册 SPI 并检查芯片 ID
W25Q64Data *W25Q64Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                       uint16_t cs_pin)
{
    if (hspi == NULL || cs_port == NULL || cs_pin == 0)
        return NULL;

    W25Q64Data *data = pvPortMalloc(sizeof(*data));
    if (data == NULL)
        return NULL;
    memset(data, 0, sizeof(*data));

    SPI_Init_Config_S config = {
        .hspi       = hspi,
        .cs_port    = cs_port,
        .cs_pin     = cs_pin,
        .mode       = SPI_BLOCK_MODE,
        .on_rx      = NULL,
        .on_tx_done = NULL,
        .owner      = data
    };

    // W25Q64 使用阻塞式 SPI，简单且便于保证命令事务连续
    data->instance = SPIRegister(&config);
    if (data->instance == NULL)
    {
        vPortFree(data);
        return NULL;
    }

    // 读取 ID 确认总线上确实连接了 W25Q64
    uint32_t id;
    if (W25Q64ReadID(data, &id) != 0 || id != W25Q64_JEDEC_ID)
    {
        vPortFree(data);
        return NULL;
    }

    data->id = id;
    return data;
}

// 读取 JEDEC ID，返回值为 厂商ID、存储类型和容量信息的组合
int W25Q64ReadID(W25Q64Data *data, uint32_t *id)
{
    // 0x9F 后发送 3 个空字节，接收芯片返回的 3 字节 ID
    uint8_t tx[4] = {W25Q64_CMD_JEDEC_ID, 0xFFU, 0xFFU, 0xFFU};
    uint8_t rx[4];

    if (data == NULL || id == NULL)
        return -1;
    if (SPITransmitAndReceive(data->instance, rx, tx, sizeof(tx)) != 0)
        return -1;

    *id = ((uint32_t)rx[1] << 16) |
          ((uint32_t)rx[2] << 8)  |
           (uint32_t)rx[3];
    return 0;
}

// 按分块方式读取任意长度数据
int W25Q64Read(W25Q64Data *data, uint32_t address, uint8_t *buffer,
               uint32_t length)
{
    if (data == NULL || buffer == NULL || length == 0 ||
        address >= W25Q64_CAPACITY ||
        length > W25Q64_CAPACITY - address)
        return -1;

    while (length > 0)
    {
        // 限制单次传输长度，避免占用过大的临时缓冲区
        uint16_t chunk = (length > W25Q64_READ_CHUNK)
                             ? W25Q64_READ_CHUNK
                             : (uint16_t)length;

        // 0x03 + 24 位地址 + 空字节必须在同一次 CS 事务中完成
        data->tx_buf[0] = W25Q64_CMD_READ_DATA;
        w25q64_set_address(data->tx_buf, address);
        memset(&data->tx_buf[4], 0xFF, chunk);

        if (SPITransmitAndReceive(data->instance, data->rx_buf, data->tx_buf,
                                  (uint16_t)(chunk + 4U)) != 0)
            return -1;

        // 前 4 个接收字节是命令和地址阶段的回显，数据从 rx_buf[4] 开始
        memcpy(buffer, &data->rx_buf[4], chunk);
        buffer  += chunk;
        address += chunk;
        length  -= chunk;
    }
    return 0;
}

// 按页边界拆分并写入数据，不负责擦除
int W25Q64Write(W25Q64Data *data, uint32_t address, const uint8_t *buffer,
                uint32_t length)
{
    if (data == NULL || buffer == NULL || length == 0 ||
        address >= W25Q64_CAPACITY ||
        length > W25Q64_CAPACITY - address)
        return -1;

    while (length > 0)
    {
        // 计算当前页剩余空间，保证本次写入不跨 256 字节页边界
        uint16_t page_remaining = (uint16_t)(W25Q64_PAGE_SIZE -
                                             (address & (W25Q64_PAGE_SIZE - 1U)));
        uint16_t chunk = (length > page_remaining)
                             ? page_remaining
                             : (uint16_t)length;

        if (w25q64_page_program(data, address, buffer, chunk) != 0)
            return -1;

        buffer  += chunk;
        address += chunk;
        length  -= chunk;
    }
    return 0;
}

// 擦除 address 所在的 4KB 扇区
int W25Q64SectorErase(W25Q64Data *data, uint32_t address)
{
    if (data == NULL || address >= W25Q64_CAPACITY)
        return -1;

    // 擦除操作以扇区为单位，地址向下对齐到扇区起始位置
    address &= ~(W25Q64_SECTOR_SIZE - 1U);

    // 等待其他操作结束，并重新发送写使能
    if (w25q64_wait_idle(data, W25Q64_TIMEOUT_MS) != 0)
        return -1;
    if (w25q64_write_enable(data) != 0)
        return -1;

    // 擦除命令和 24 位地址必须在同一次 CS 事务中发送
    data->tx_buf[0] = W25Q64_CMD_SECTOR_ERASE;
    w25q64_set_address(data->tx_buf, address);

    if (SPITransmit(data->instance, data->tx_buf, 4) != 0)
        return -1;

    return w25q64_wait_idle(data, W25Q64_TIMEOUT_MS);
}