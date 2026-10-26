#include "app_bluetooth_test.h"
#include "blue_tooth.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define BLUETOOTH_TEST_TASK_STACK      512
#define BLUETOOTH_TEST_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)

static Bluetooth *bluetooth;

// 静态任务控制块与栈
static StaticTask_t bluetooth_test_tcb;//静态任务控制块
static StackType_t  bluetooth_test_stack[BLUETOOTH_TEST_TASK_STACK];

// 收到一帧后，将 payload 原样回传
static void BluetoothTest_Task(void *argument)
{

    for (;;)
    {
        if (xSemaphoreTake(bluetooth->data_ready_semaphore,
                           portMAX_DELAY) == pdPASS)
        {
            BluetoothSendData(bluetooth, bluetooth->rx_frame.data + 1,
                              bluetooth->rx_frame.length - 2);
        }
    }
}

// 供其他 App 通过蓝牙发送测试数据，长度不能超过单帧 payload 上限
int BluetoothTest_SendData(const uint8_t *data, uint16_t length)
{
    if (length > BLUETOOTH_DATASIZE)
        return -1;

    return BluetoothSendData(bluetooth, data, (uint8_t)length);
}

void App_BluetoothTest_Init(void)
{
    bluetooth = BluetoothInit(&huart1);
    if (bluetooth == NULL)
        return;

    xTaskCreateStatic(BluetoothTest_Task, "bluetooth_test",
                      BLUETOOTH_TEST_TASK_STACK, NULL,
                      BLUETOOTH_TEST_TASK_PRIORITY,
                      bluetooth_test_stack, &bluetooth_test_tcb);
}