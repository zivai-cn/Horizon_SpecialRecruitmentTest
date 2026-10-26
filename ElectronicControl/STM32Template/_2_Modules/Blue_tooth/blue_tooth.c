#include "blue_tooth.h"
#include "task.h"
#include <string.h>

// 蓝牙接收缓冲区
static uint8_t bluetooth_rx_buffer[BLUETOOTH_BUFFERSIZE];

// 发送完成回调（中断上下文）：继续发送下一帧
static void Bluetooth_TxDoneCallback(USARTInstance *instance)
{
    Bluetooth *bluetooth = (Bluetooth *)instance->owner;

    if (xQueueReceiveFromISR(bluetooth->tx_queue, &bluetooth->tx_frame, NULL) == pdPASS)//从队列取一帧发送
    {
        USARTSend(bluetooth->instance, bluetooth->tx_frame.data, bluetooth->tx_frame.length);
    }
}

// 蓝牙接收回调函数
static void Bluetooth_RxCallback(USARTInstance *instance)
{
    Bluetooth *bluetooth = (Bluetooth *)instance->owner;
    uint8_t       *buffer = instance->rx_buf;             // 接收缓冲区指针
    uint16_t       rx_len = instance->rx_len;               // 接收数据长度

    if (buffer[0] == FRAME_HEAD && buffer[BLUETOOTH_BUFFERSIZE - 1] == FRAME_END)                        //检查帧头
    {
        bluetooth->rx_frame.length = (uint8_t)rx_len;
        memcpy(bluetooth->rx_frame.data, buffer, rx_len);

        bluetooth->new_data_flag = 1;

        BaseType_t higher_task_woken = pdFALSE;//是否需要切换任务
        xSemaphoreGiveFromISR(bluetooth->data_ready_semaphore, &higher_task_woken);//释放信号量
        portYIELD_FROM_ISR(higher_task_woken);//切换任务
    }

}

// 初始化蓝牙数据结构体
Bluetooth *BluetoothInit(UART_HandleTypeDef *huart)
{
    Bluetooth *bluetooth = pvPortMalloc(sizeof(Bluetooth));
    memset(bluetooth, 0, sizeof(*bluetooth));

    bluetooth->data_ready_semaphore = xSemaphoreCreateBinary();  //数据就绪二进制信号量
    bluetooth->tx_queue = xQueueCreate(BLUETOOTH_TX_QUEUE_LEN, sizeof(BluetoothFrame));//发送队列
    
    if (bluetooth->data_ready_semaphore == NULL || bluetooth->tx_queue == NULL)
    {
        if (bluetooth->tx_queue != NULL) 
        {
            vQueueDelete(bluetooth->tx_queue);//删除发送队列
        }            
        if (bluetooth->data_ready_semaphore != NULL)
        {
            vSemaphoreDelete(bluetooth->data_ready_semaphore);//删除信号量
        }
        vPortFree(bluetooth);//释放内存
        return ;
    }

    USART_Init_Config_S config = {
        .huart      = huart,                            //串口实例指针
        .rx_mode    = UART_MODE_IT,                     //接收模式
        .tx_mode    = UART_MODE_IT,                     //发送模式
        .rx_buf     = bluetooth_rx_buffer,              //接收缓冲区指针
        .rx_size    = sizeof(bluetooth_rx_buffer),      //接收缓冲区大小
        .on_rx      = Bluetooth_RxCallback,             //接收回调函数
        .on_tx_done = Bluetooth_TxDoneCallback,         //发送完成回调函数
        .owner      = bluetooth                              //所有者指针，指向蓝牙数据结构体
    };
    bluetooth->instance = USARTRegister(&config);//注册串口实例
    
    if (bluetooth->instance == NULL)//注册串口实例失败
    {
        vQueueDelete(bluetooth->tx_queue);//删除发送队列
        vSemaphoreDelete(bluetooth->data_ready_semaphore);//删除信号量
        vPortFree(bluetooth);//释放内存
        return NULL;
    }

    return (Bluetooth *)&bluetooth;//返回蓝牙数据结构体指针
}

// 发送蓝牙数据
int BluetoothSendData(Bluetooth *bluetooth, const uint8_t *data, uint8_t data_length)
{

    BluetoothFrame frame;   //发送帧
    frame.data[0] = FRAME_HEAD;
    frame.data[BLUETOOTH_BUFFERSIZE - 1] = FRAME_END;

    memcpy(&frame.data[2], data, data_length);//复制数据到发送帧
    

    if (xQueueSend(bluetooth->tx_queue, &frame, 0) != pdPASS)
        return -1;

    // 若串口空闲，立即从队列取一帧启动发送
    taskENTER_CRITICAL();//进入临界区
    if (bluetooth->instance->tx_busy == 0)
    {
        if (xQueueReceive(bluetooth->tx_queue, &bluetooth->tx_frame, 0) == pdPASS)//从队列取一帧发送
        {
            USARTSend(bluetooth->instance, bluetooth->tx_frame.data, bluetooth->tx_frame.length);
        }
    }
    taskEXIT_CRITICAL();//退出临界区

    return 0;
}