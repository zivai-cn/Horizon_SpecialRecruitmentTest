#ifndef BLUE_TOOTH_H
#define BLUE_TOOTH_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "bsp_uart.h"

// 单帧最大数据长度
#define BLUETOOTH_DATASIZE    128
// 帧头(1) + 长度(1) + 数据 + 帧尾(1)
#define BLUETOOTH_BUFFERSIZE  (BLUETOOTH_DATASIZE + 3)

// 数据帧头尾
#define FRAME_HEAD  0x54
#define FRAME_END   0x55

// 发送队列深度
#define BLUETOOTH_TX_QUEUE_LEN 4

// 一帧完整数据（队列元素）
typedef struct
{
    uint8_t length;                         // 实际发送长度
    uint8_t data[BLUETOOTH_BUFFERSIZE];     // 组好的帧
} BluetoothFrame;

// 蓝牙数据结构体
typedef struct
{
    USARTInstance      *instance;
    SemaphoreHandle_t   data_ready_semaphore;
    QueueHandle_t       tx_queue;
    BluetoothFrame      tx_frame;           // 正在发送的帧，HAL 异步引用，必须常驻
    volatile uint8_t    new_data_flag;
    uint8_t             receive_data[BLUETOOTH_DATASIZE];
    uint8_t             receive_length;
} BluetoothData;

BluetoothData *BluetoothInit(UART_HandleTypeDef *huart);

// 组帧后放入发送队列，0 表示已入队，-1 表示参数错误或队列已满
int BluetoothSendData(BluetoothData *data, const uint8_t *payload,
                      uint8_t payload_length);

#endif
