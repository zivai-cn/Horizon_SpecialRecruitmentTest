#include "app_led_test.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_TEST_TASK_STACK      128
#define LED_TEST_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define LED_BLINK_PERIOD_MS      500

// 蓝板 PC13 LED，低电平点亮
#define LED_GPIO_PORT            GPIOC
#define LED_GPIO_PIN             GPIO_PIN_13

// 静态任务控制块与栈，不占 FreeRTOS 堆
static StaticTask_t led_test_tcb;
static StackType_t  led_test_stack[LED_TEST_TASK_STACK];

static void LedTest_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_PERIOD_MS));
    }
}

void App_LedTest_Init(void)
{
    // PC13 的 GPIO 初始化由 CubeMX 生成的 MX_GPIO_Init() 完成
    xTaskCreateStatic(LedTest_Task, "led_test",
                      LED_TEST_TASK_STACK, NULL,
                      LED_TEST_TASK_PRIORITY,
                      led_test_stack, &led_test_tcb);
}
