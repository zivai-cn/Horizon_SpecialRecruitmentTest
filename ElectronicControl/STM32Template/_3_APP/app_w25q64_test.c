#include "app_w25q64_test.h"
#include "app_bluetooth_test.h"
#include "w25q64.h"
#include "spi.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define W25Q64_TEST_TASK_STACK       512
#define W25Q64_TEST_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)

#define W25Q64_TEST_CS_PORT          GPIOA
#define W25Q64_TEST_CS_PIN           GPIO_PIN_4

#define W25Q64_TEST_LED_PORT         GPIOC
#define W25Q64_TEST_LED_PIN          GPIO_PIN_13

#define W25Q64_TEST_ADDRESS          (W25Q64_CAPACITY - W25Q64_SECTOR_SIZE)

// 写入 W25Q64 的固定测试内容
static const uint8_t w25q64_test_data[] =
    "W25Q64 fixed data: ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n";
#define W25Q64_TEST_LENGTH           (sizeof(w25q64_test_data) - 1U)

static W25Q64Data *w25q64;

// 静态任务控制块与栈，不占 FreeRTOS 堆
static StaticTask_t w25q64_test_tcb;
static StackType_t  w25q64_test_stack[W25Q64_TEST_TASK_STACK];

// 测试失败时快速闪烁 LED
static void W25Q64Test_Fail(void)
{
    for (;;)
    {
        HAL_GPIO_TogglePin(W25Q64_TEST_LED_PORT, W25Q64_TEST_LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 初始化片选引脚，默认保持高电平
static void W25Q64Test_CsInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(W25Q64_TEST_CS_PORT, W25Q64_TEST_CS_PIN,
                      GPIO_PIN_SET);

    GPIO_InitStruct.Pin = W25Q64_TEST_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W25Q64_TEST_CS_PORT, &GPIO_InitStruct);
}

// 写入固定内容，读回校验后通过蓝牙发送一次
static void W25Q64Test_Task(void *argument)
{
    (void)argument;

    uint8_t rx_data[W25Q64_TEST_LENGTH];

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(200));
    // 初始化 W25Q64 实例
    w25q64 = W25Q64Init(&hspi1, W25Q64_TEST_CS_PORT, W25Q64_TEST_CS_PIN);
    if (w25q64 == NULL)
        W25Q64Test_Fail();
    // 擦除测试地址
    if (W25Q64SectorErase(w25q64, W25Q64_TEST_ADDRESS) != 0)
        W25Q64Test_Fail();
    if (W25Q64Write(w25q64, W25Q64_TEST_ADDRESS, w25q64_test_data, W25Q64_TEST_LENGTH) != 0)
        W25Q64Test_Fail();
    if (W25Q64Read(w25q64, W25Q64_TEST_ADDRESS, rx_data, W25Q64_TEST_LENGTH) != 0)
        W25Q64Test_Fail();
    if (memcmp(w25q64_test_data, rx_data, W25Q64_TEST_LENGTH) != 0)
        W25Q64Test_Fail();

    if (BluetoothTest_SendData(rx_data, W25Q64_TEST_LENGTH) != 0)
        W25Q64Test_Fail();

    // PC13 低电平点亮，表示测试成功
    HAL_GPIO_WritePin(W25Q64_TEST_LED_PORT, W25Q64_TEST_LED_PIN,
                      GPIO_PIN_RESET);

    for (;;)
        vTaskDelay(portMAX_DELAY);
}

void App_W25Q64Test_Init(void)
{
    W25Q64Test_CsInit();//初始化片选引脚
    // 创建任务
    xTaskCreateStatic(W25Q64Test_Task, "w25q64_test", W25Q64_TEST_TASK_STACK, NULL, W25Q64_TEST_TASK_PRIORITY, w25q64_test_stack, &w25q64_test_tcb);
}