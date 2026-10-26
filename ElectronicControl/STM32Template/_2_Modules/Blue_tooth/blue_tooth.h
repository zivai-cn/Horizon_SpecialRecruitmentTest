#ifndef BLUE_TOOTH_H
#define BLUE_TOOTH_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "bsp_uart.h"

// 单帧最大数据长度
#define BLUETOOTH_DATASIZE    128
// 帧头(1) 帧尾(1)
#define BLUETOOTH_BUFFERSIZE  (BLUETOOTH_DATASIZE + 2)
// 发送队列深度
#define BLUETOOTH_TX_QUEUE_LEN 4

// 数据帧头尾
#define FRAME_HEAD  0x54
#define FRAME_END   0x55



// 一帧完整数据（队列元素）
typedef struct
{
    uint8_t length;                         // 实际发送长度
    uint8_t data[BLUETOOTH_BUFFERSIZE];     // 组好的帧
} BluetoothFrame;

// 蓝牙数据结构体
typedef struct
{
    USARTInstance      *instance;             // 串口实例指针
    SemaphoreHandle_t   data_ready_semaphore; // 数据就绪二进制信号量
    QueueHandle_t       tx_queue;             // 发送队列句柄
    BluetoothFrame      tx_frame;             // 正在发送的帧
    volatile uint8_t    new_data_flag;        // 新数据标志位
    BluetoothFrame      rx_frame;               // 接收帧
} Bluetooth;

Bluetooth *BluetoothInit(UART_HandleTypeDef *huart);

// 组帧后放入发送队列，0 表示已入队，-1 表示参数错误或队列已满
int BluetoothSendData(Bluetooth *bluetooth, const uint8_t *data, uint8_t data_length);

#endif