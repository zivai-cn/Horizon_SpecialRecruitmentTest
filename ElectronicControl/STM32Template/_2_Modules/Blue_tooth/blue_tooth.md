# Module_BlueTooth 开发文档

## 数据流

**发送**

`App → BluetoothSendData → 组帧 → tx_queue → Bluetooth_KickTx → USARTSend → UART IT`

帧格式：`0x54 | 数据长度 | 数据 | 0x55`

- `BluetoothSendData` 负责组帧并放入发送队列。
- 队列有数据且 UART 空闲时，`Bluetooth_KickTx` 启动发送。
- `Bluetooth_TxDoneCallback` 在发送完成后取出下一帧继续发送。

**接收**

`UART IT → HAL_UARTEx_RxEventCallback → bsp_uart → Bluetooth_RxCallback`

收到数据后依次检查：

- 帧头为 `0x54`
- 数据长度不超过 `BLUETOOTH_DATASIZE`
- 接收长度包含完整的帧头、长度、数据和帧尾
- 帧尾为 `0x55`

校验通过后：

- 拷贝数据到 `receive_data`，长度写入 `receive_length`
- 设置 `new_data_flag = 1`
- 释放 `data_ready_semaphore`

BSP 会在接收回调返回后重新启动 UART IT 接收。

## 接口说明

### `BluetoothInit`

注册 UART 实例并创建发送队列和接收信号量，成功返回蓝牙数据结构指针，失败返回 `NULL`。

### `BluetoothSendData`

组帧后放入发送队列。成功入队返回 `0`，参数错误或队列已满返回 `-1`。