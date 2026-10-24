# BSP_UART开发文档

<p align='right'>zivai_cn@163.com</p>
<p align='right'>zivai_cn@outlook.com</p>

## 使用说明

1. 目前的代码文件强依赖于stm32hal库，在bsp_uart.c文件中，已经引用了main.h文件，如果需要使用uart功能，请确保已经初始化了hal库的uart模块。

2. 文件说明
在bsp_uart.h文件中，包含所有的对外接口函数和结构体定义，以及相关的宏定义。
在bsp_uart.c文件中，实现了所有的对外接口函数，以及相关的内部函数。

3. 收发完全独立，互不干扰。接收永远开着，发送具备忙标志位。BSP 层不内置发送队列，对于需要排队发送的场景，请在 Modules 层级通过 FreeRTOS 队列等机制自行实现。

4. 没有提供接收函数，接收通过 `on_rx` 回调实现。数据到达后自动通知模块，模块直接从 `ins->rx_buf` 读取。

5. 返回 `int` 的接口统一使用 `0` 表示成功、`-1` 表示失败；返回实例指针的接口失败时返回 `NULL`。

### 使用注意事项
1. 在`USARTInstance`结构体中，只包含`*rx_buf`接收缓冲区的指针，需要在 Modules 层级文件中自行创建接收缓冲区，并在注册实例时传递接收缓冲区的指针。

2. 收发模式在注册时分别通过 `cfg.rx_mode` 和 `cfg.tx_mode` 指定，可在发送空闲时通过 `USARTSetMode` 修改。接收模式仅支持 IT 和 DMA。

## 变量说明

`USART_MODE`：串口收发工作模式枚举，包括 POLLING、IT、DMA 三种。接收和发送各独立配置。

`USARTInstance`：串口实例结构体，包含 HAL 句柄、收发模式、缓冲区指针、回调函数指针、忙标志等。

`USART_Init_Config_S`：注册配置结构体，包含 huart、rx_mode、tx_mode、rx_buf、rx_size、on_rx、on_tx_done、owner。

## 函数说明

### `USARTRegister`
注册实例函数
在 Modules 层级文件中调用 `USARTRegister(&cfg)`，传入配置：
- `huart`：HAL 串口句柄
- `rx_mode`：接收模式（DMA 或 IT）
- `tx_mode`：发送模式（POLLING / IT / DMA）
- `rx_buf`：模块自行分配的接收缓冲区
- `rx_size`：接收缓冲区大小
- `on_rx`：收完一帧的回调函数指针
- `on_tx_done`：发送完成的回调函数指针
- `owner`：指向模块自身的指针（方便回调中反查）
注册成功后立刻调用 `usart_start_rx` 启动接收监听，返回实例指针，失败返回 NULL。

### `USARTSend`
发送一帧数据
在 Modules 层级文件中调用 `USARTSend(ins, buf, len)`，传入：
- `ins`：通过 `USARTRegister` 获取的实例指针
- `buf`：待发送数据的指针
- `len`：待发送数据的长度
发送模式可在发送空闲时通过 `USARTSetMode` 修改。参数错误、发送忙或 HAL 启动失败时返回 `-1`，成功启动或完成时返回 `0`。BSP 层不排队，上层自行处理重试或排队。

### `USARTSetMode`
切换实例的接收和发送模式。

```c
int USARTSetMode(USARTInstance *ins, USART_MODE rx_mode,
                 USART_MODE tx_mode);
```

- `rx_mode`：接收模式，仅支持 `UART_MODE_IT` 和 `UART_MODE_DMA`
- `tx_mode`：发送模式，支持 `UART_MODE_POLLING`、`UART_MODE_IT` 和 `UART_MODE_DMA`

仅在实例发送空闲时允许切换，并且应在任务上下文调用，不要从 UART 中断回调中调用。如果接收模式发生变化，函数会先停止当前接收，再使用新模式重新启动接收；重新启动失败时会尝试恢复原接收模式并返回 `-1`。模式切换成功返回 `0`。

## 底层流动逻辑

**接收**

1. 通过 `USARTRegister` 注册实例，传入接收相关参数。
2. 实例注册成功后，调用 `usart_start_rx` 启动监听。`usart_start_rx` 根据 `rx_mode` 选择 DMA 或中断模式，DMA 模式下会关闭半传输中断（仅保留空闲中断）。
3. 数据帧到达，`HAL_UARTEx_RxEventCallback` 被触发：保存 `rx_len`，调用 `on_rx(ins)` 通知模块，并重启接收。

**发送**

1. Modules 层级调用 `USARTSend(ins, buf, len)` 发送数据。
2. 根据当前 `tx_mode`：
   - **POLLING 模式**：调用 `HAL_UART_Transmit` 阻塞等待发送完成，完成后清 busy 并调 `on_tx_done`；该回调在 `USARTSend` 返回前执行。
   - **IT 模式**：调用 `HAL_UART_Transmit_IT` 立即返回，发送完成后由 `HAL_UART_TxCpltCallback` 清 busy 并调 `on_tx_done`。
   - **DMA 模式**：调用 `HAL_UART_Transmit_DMA` 立即返回，DMA 传输完成后由 `HAL_UART_TxCpltCallback` 清 busy 并调 `on_tx_done`。
3. 如果当前忙（`tx_busy == 1`）或 HAL 启动失败，`USARTSend` 返回 `-1`，不排队。上层需自行处理重试或通过 FreeRTOS 队列串行化。

**模式切换**

1. 在任务上下文中调用 `USARTSetMode(ins, rx_mode, tx_mode)`，先校验模式取值和发送忙状态。
2. 如果仅发送模式变化，直接更新 `tx_mode`。
3. 如果接收模式变化，先调用 `HAL_UART_AbortReceive` 停止当前接收，再更新 `rx_mode` 并调用 `usart_start_rx` 重新启动。
4. 新接收模式启动失败时恢复原 `rx_mode`，尝试重新启动原接收配置，并返回 `-1`。