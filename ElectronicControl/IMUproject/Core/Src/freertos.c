/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "semphr.h"
#include <stdio.h>

#include "usart.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 角度文本：整数部分 + 三位小数 + 符号
typedef struct
{
  long        ip;
  long        fp;
  const char *sign;
} AngleText;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_START_DELAY_MS           300
#define IMU_RETRY_DELAY_MS           500
#define IMU_SAMPLE_PERIOD_MS         1
#define COMM_PERIOD_MS               1
#define STATUS_REPORT_PERIOD_MS      500
#define UART_TIMEOUT_MS              10
#define IMU_MAX_CONSECUTIVE_ERRORS   10

#define IMU_TASK_STACK_WORDS         384
#define COMM_TASK_STACK_WORDS        256
#define LED_TASK_STACK_WORDS         128

#define IMU_TASK_PRIORITY            (tskIDLE_PRIORITY + 24)
#define COMM_TASK_PRIORITY           (tskIDLE_PRIORITY + 2)
#define LED_TASK_PRIORITY            (tskIDLE_PRIORITY + 1)

// IMU 状态码，与串口错误帧 ERR,<status> 对应
#define IMU_STATUS_BOOT              0
#define IMU_STATUS_INIT              1
#define IMU_STATUS_CALIBRATE         2
#define IMU_STATUS_UPDATE_ERROR      3
#define IMU_STATUS_READY             4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static SemaphoreHandle_t imuDataMutex;
static TaskHandle_t imuTaskHandle;
static TaskHandle_t commTaskHandle;
static TaskHandle_t ledTaskHandle;
static MPU6050_t *volatile g_mpu6050 = NULL;

volatile uint8_t g_imu_ready = 0;
volatile uint8_t g_imu_status = IMU_STATUS_BOOT;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void IMUTask(void *argument);
static void CommTask(void *argument);
static void LEDTask(void *argument);
static void angle_to_text(float value, AngleText *out);
static void comm_transmit(char *buffer, size_t buffer_size, int length);
/* USER CODE END FunctionPrototypes */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void angle_to_text(float value, AngleText *out)
{
  static const float round_half = 0.5;
  long md;

  // 转换为毫度，四舍五入并记录符号。
  md = (long)(value * 1000 + ((value >= 0) ? round_half : -round_half));
  out->sign = (md < 0) ? "-" : "";
  if (md < 0)
  {
    md = -md;
  }
  out->ip = md / 1000;
  out->fp = md % 1000;
}

static void comm_transmit(char *buffer, size_t buffer_size, int length)
{
  size_t tx_length;

  if ((buffer == NULL) || (buffer_size == 0) || (length <= 0))
  {
    return;
  }

  tx_length = (size_t)length;
  if (tx_length >= buffer_size)
  {
    tx_length = buffer_size - 1;
  }

  USART1_Transmit((const uint8_t *)buffer,
                  (uint16_t)tx_length,
                  UART_TIMEOUT_MS);
}
/* USER CODE END 0 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 创建应用任务并删除 CubeMX 默认启动任务。
// 由 main.c 中的 StartDefaultTask() 调用。
void App_Run(void)
{
  imuDataMutex = xSemaphoreCreateMutex();
  configASSERT(imuDataMutex != NULL);

  configASSERT(xTaskCreate(IMUTask, "IMU", IMU_TASK_STACK_WORDS, NULL,
                           IMU_TASK_PRIORITY, &imuTaskHandle) == pdPASS);
  configASSERT(xTaskCreate(CommTask, "Comm", COMM_TASK_STACK_WORDS, NULL,
                           COMM_TASK_PRIORITY, &commTaskHandle) == pdPASS);
  configASSERT(xTaskCreate(LEDTask, "LED", LED_TASK_STACK_WORDS, NULL,
                           LED_TASK_PRIORITY, &ledTaskHandle) == pdPASS);

  vTaskDelete(NULL);
}

// MPU6050 初始化、校准、采集与姿态解算任务。
// argument: 未使用。
static void IMUTask(void *argument)
{
  MPU6050_t mpu6050;
  TickType_t last_wake;
  uint16_t update_error_count;
  uint8_t update_ok;

  (void)argument;

  // 传感器实例的生命周期与 IMUTask 相同，供 CommTask 只读访问。
  g_mpu6050 = &mpu6050;
  vTaskDelay(pdMS_TO_TICKS(IMU_START_DELAY_MS));

  for (;;)
  {
    g_imu_ready = 0;
    g_imu_status = IMU_STATUS_INIT;

    if (MPU6050_Init(&mpu6050) == 0)
    {
      vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_DELAY_MS));
      continue;
    }

    g_imu_status = IMU_STATUS_CALIBRATE;
    if (MPU6050_Calibrate(&mpu6050) == 0)
    {
      vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_DELAY_MS));
      continue;
    }

    g_imu_status = IMU_STATUS_READY;
    g_imu_ready = 1;
    update_error_count = 0;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
      // 严格 1 kHz 采样，与 MPU6050 输出周期对齐。
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));

      (void)xSemaphoreTake(imuDataMutex, portMAX_DELAY);
      update_ok = MPU6050_Update(&mpu6050);
      (void)xSemaphoreGive(imuDataMutex);

      if (update_ok == 0)
      {
        update_error_count++;
        if (update_error_count >= IMU_MAX_CONSECUTIVE_ERRORS)
        {
          g_imu_status = IMU_STATUS_UPDATE_ERROR;
          break;
        }
      }
      else
      {
        update_error_count = 0;
      }

      // 任务被长时间阻塞后重新对齐采样基准，避免连续补采样。
      if ((xTaskGetTickCount() - last_wake) > pdMS_TO_TICKS(2 * IMU_SAMPLE_PERIOD_MS))
      {
        last_wake = xTaskGetTickCount();
      }
    }

    // 连续读取失败后停止输出，重新初始化并校准传感器。
    g_imu_ready = 0;
    vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_DELAY_MS));
  }
}

// 串口通信任务，每 1 ms 输出一帧欧拉角。
// argument: 未使用。
static void CommTask(void *argument)
{
  float euler[3];
  AngleText text[3];
  char tx_buff[64];
  TickType_t last_wake;
  TickType_t last_status_tick = 0;
  int length;

  (void)argument;

  last_wake = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMM_PERIOD_MS));

    if ((g_imu_ready == 0) || (g_mpu6050 == NULL))
    {
      TickType_t now = xTaskGetTickCount();

      // IMU 未就绪时每 500 ms 发送一次错误状态，便于上位机定位问题。
      if ((now - last_status_tick) >= pdMS_TO_TICKS(STATUS_REPORT_PERIOD_MS))
      {
        length = snprintf(tx_buff, sizeof(tx_buff),
                          "ERR,%u\n", (unsigned)g_imu_status);
        comm_transmit(tx_buff, sizeof(tx_buff), length);
        last_status_tick = now;
      }
      continue;
    }

    last_status_tick = xTaskGetTickCount();

    (void)xSemaphoreTake(imuDataMutex, portMAX_DELAY);
    MPU6050_GetEuler(g_mpu6050, euler);
    (void)xSemaphoreGive(imuDataMutex);

    angle_to_text(euler[0], &text[0]);
    angle_to_text(euler[1], &text[1]);
    angle_to_text(euler[2], &text[2]);

    length = snprintf(tx_buff, sizeof(tx_buff),
                      "%s%ld.%03ld,%s%ld.%03ld,%s%ld.%03ld\n",
                      text[0].sign, text[0].ip, text[0].fp,
                      text[1].sign, text[1].ip, text[1].fp,
                      text[2].sign, text[2].ip, text[2].fp);
    comm_transmit(tx_buff, sizeof(tx_buff), length);

    // 串口阻塞导致周期异常时重新对齐发送基准。
    if ((xTaskGetTickCount() - last_wake) > pdMS_TO_TICKS(2 * COMM_PERIOD_MS))
    {
      last_wake = xTaskGetTickCount();
    }
  }
}

// LED 状态指示任务。
// argument: 未使用。
static void LEDTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    vTaskDelay(pdMS_TO_TICKS((g_imu_ready != 0) ? 500 : 100));
  }
}
/* USER CODE END Application */
