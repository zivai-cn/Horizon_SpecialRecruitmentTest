#ifndef W25Q64_H
#define W25Q64_H

#include <stdint.h>
#include "bsp_spi.h"

#define W25Q64_PAGE_SIZE        256U        // 页大小：256 字节
#define W25Q64_SECTOR_SIZE      4096U       // 扇区大小：4 KB
#define W25Q64_CAPACITY         0x800000U   // 总容量：8 MB
#define W25Q64_READ_CHUNK       64U         // 单次读取分块大小

// W25Q64 命令
#define W25Q64_CMD_WRITE_ENABLE   0x06U     // 写使能
#define W25Q64_CMD_READ_STATUS    0x05U     // 读状态寄存器 1
#define W25Q64_CMD_READ_DATA      0x03U     // 读数据
#define W25Q64_CMD_PAGE_PROGRAM   0x02U     // 页编程
#define W25Q64_CMD_SECTOR_ERASE   0x20U     // 4KB 扇区擦除
#define W25Q64_CMD_JEDEC_ID       0x9FU     // 读取 JEDEC ID

// 模块参数
#define W25Q64_JEDEC_ID           0xEF4017U // W25Q64 的 JEDEC ID
#define W25Q64_TIMEOUT_MS         5000U     // 写入和擦除的等待超时


typedef struct W25Q64Data W25Q64Data;

struct W25Q64Data
{
    SPIInstance *instance;                  // SPI BSP 实例
    uint32_t     id;                        // 芯片 JEDEC ID

    uint8_t      tx_buf[W25Q64_PAGE_SIZE + 4U]; // 命令、地址和页数据共用发送缓冲区
    uint8_t      rx_buf[W25Q64_READ_CHUNK + 4U]; // 分块读取时的接收缓冲区
};

// 注册 SPI 实例并检查 JEDEC ID，成功返回模块句柄，失败返回 NULL
W25Q64Data *W25Q64Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

// 读取 JEDEC ID，W25Q64 正常返回 0xEF4017
int W25Q64ReadID(W25Q64Data *data, uint32_t *id);
// 从指定地址读取数据，支持任意长度
int W25Q64Read(W25Q64Data *data, uint32_t address, uint8_t *buffer, uint32_t length);
// 写入数据，自动按 256 字节页拆分；调用前目标区域必须已擦除
int W25Q64Write(W25Q64Data *data, uint32_t address, const uint8_t *buffer, uint32_t length);
// 擦除 address 所在的一个 4KB 扇区
int W25Q64SectorErase(W25Q64Data *data, uint32_t address);

#endif