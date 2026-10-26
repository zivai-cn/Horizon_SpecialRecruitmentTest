# Module_BlueTooth 开发文档

## 数据流

**发送**

`App → BluetoothSendData → 组帧 → 入队 → (串口空闲则)取帧 → USARTSend → UART IT`

帧格式：`0x54 | 数据长度 | 数据 | 0x55`

- `BluetoothSendData` 负责组帧、放入发送队列，若串口空闲则立即启动发送。
- `Bluetooth_TxDoneCallback` 在发送完成后取出下一帧继续发送。

**接收**

`UART IT → HAL_UARTEx_RxEventCallback → bsp_uart → Bluetooth_RxCallback`

收到数据后依次检查：

- 帧头为 `0x54`
- 帧尾为 `0x55`

校验通过后：

- 将完整帧拷贝到 `rx_frame.data`，长度写入 `rx_frame.length`
- 设置 `new_data_flag = 1`
- 释放 `data_ready_semaphore`

BSP 会在接收回调返回后重新启动 UART IT 接收。

## 接口说明

### `BluetoothInit`

注册 UART 实例并创建发送队列和接收信号量，成功返回 `Bluetooth` 结构指针，失败返回 `NULL`。

### `BluetoothSendData`

```c
int BluetoothSendData(Bluetooth *bluetooth, const uint8_t *data, uint8_t data_length);
```

组帧后放入发送队列，若串口空闲则立即启动发送。成功返回 `0`，参数错误或队列已满返回 `-1`。