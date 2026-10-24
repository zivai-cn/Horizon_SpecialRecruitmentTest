#include "blue_tooth.h"
#include "task.h"
#include <string.h>

// 蓝牙接收缓冲区
static uint8_t bluetooth_rx_buffer[BLUETOOTH_BUFFERSIZE];

// 从队列取一帧并启动发送（任务上下文，调用前需保证 UART 中断被屏蔽）
static void Bluetooth_PopAndSend(BluetoothData *data)
{
    if (xQueueReceive(data->tx_queue, &data->tx_frame, 0) == pdPASS)
        USARTSend(data->instance, data->tx_frame.data, data->tx_frame.length);
}

// 任务上下文：空闲时启动队列中的下一帧
static void Bluetooth_KickTx(BluetoothData *data)
{
    taskENTER_CRITICAL();
    if (!data->instance->tx_busy)
        Bluetooth_PopAndSend(data);
    taskEXIT_CRITICAL();
}

// 发送完成回调（中断上下文）：继续发送下一帧
static void Bluetooth_TxDoneCallback(USARTInstance *instance)
{
    BluetoothData *data = (BluetoothData *)instance->owner;

    if (xQueueReceiveFromISR(data->tx_queue, &data->tx_frame, NULL) == pdPASS)
        USARTSend(data->instance, data->tx_frame.data, data->tx_frame.length);
}

// 蓝牙接收回调函数
static void Bluetooth_RxCallback(USARTInstance *instance)
{
    BluetoothData *data = (BluetoothData *)instance->owner;
    uint8_t       *buffer = instance->rx_buf;
    uint16_t       rx_len = instance->rx_len;

    if (rx_len < 3)                                     //帧头+长度+帧尾
        return;
    if (buffer[0] != FRAME_HEAD)                        //检查帧头
        return;

    uint8_t payload_length = buffer[1];
    if (payload_length > BLUETOOTH_DATASIZE)            //长度非法
        return;
    if (rx_len < (uint16_t)payload_length + 3)          //数据不完整
        return;
    if (buffer[payload_length + 2] != FRAME_END)        //检查帧尾
        return;

    for (uint16_t i = 0; i < payload_length; i++)
        data->receive_data[i] = buffer[i + 2];

    data->receive_length = payload_length;
    data->new_data_flag = 1;

    BaseType_t higher_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(data->data_ready_semaphore,
                          &higher_task_woken);
    portYIELD_FROM_ISR(higher_task_woken);
}

BluetoothData *BluetoothInit(UART_HandleTypeDef *huart)
{
    BluetoothData *data = pvPortMalloc(sizeof(BluetoothData));
    if (data == NULL)
        return NULL;
    memset(data, 0, sizeof(*data));

    data->data_ready_semaphore = xSemaphoreCreateBinary();
    data->tx_queue = xQueueCreate(BLUETOOTH_TX_QUEUE_LEN, sizeof(BluetoothFrame));
    if (data->data_ready_semaphore == NULL || data->tx_queue == NULL)
    {
        if (data->tx_queue != NULL)             vQueueDelete(data->tx_queue);
        if (data->data_ready_semaphore != NULL) vSemaphoreDelete(data->data_ready_semaphore);
        vPortFree(data);
        return NULL;
    }

    USART_Init_Config_S config = {
        .huart      = huart,
        .rx_mode    = UART_MODE_IT,
        .tx_mode    = UART_MODE_IT,
        .rx_buf     = bluetooth_rx_buffer,
        .rx_size    = sizeof(bluetooth_rx_buffer),
        .on_rx      = Bluetooth_RxCallback,
        .on_tx_done = Bluetooth_TxDoneCallback,
        .owner      = data
    };

    data->instance = USARTRegister(&config);
    if (data->instance == NULL)
    {
        vQueueDelete(data->tx_queue);
        vSemaphoreDelete(data->data_ready_semaphore);
        vPortFree(data);
        return NULL;
    }

    return data;
}

int BluetoothSendData(BluetoothData *data, const uint8_t *payload,
                      uint8_t payload_length)
{
    if (data == NULL || payload == NULL ||
        payload_length > BLUETOOTH_DATASIZE)
        return -1;

    BluetoothFrame frame;
    frame.data[0] = FRAME_HEAD;
    frame.data[1] = payload_length;

    memcpy(&frame.data[2], payload, payload_length);
    frame.data[payload_length + 2] = FRAME_END;
    frame.length = (uint8_t)(payload_length + 3);

    if (xQueueSend(data->tx_queue, &frame, 0) != pdPASS)
        return -1;                 //队列满，丢弃本帧

    Bluetooth_KickTx(data);
    return 0;
}
