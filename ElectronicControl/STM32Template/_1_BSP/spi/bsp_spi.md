# BSP_SPI开发文档

<p align='right'>zivai_cn@163.com</p>
<p align='right'>zivai_cn@outlook.com</p>

## 使用说明

1. 目前的代码文件强依赖于 STM32 HAL 库。`bsp_spi.c` 通过 `bsp_spi.h` 引用了 `main.h`，使用 SPI 功能前请确保已经完成对应 `SPI_HandleTypeDef` 的初始化，并初始化片选 GPIO。

2. 文件说明
在 `bsp_spi.h` 文件中，包含所有对外接口函数、结构体、枚举和宏定义。
在 `bsp_spi.c` 文件中，实现所有对外接口函数、总线仲裁逻辑以及 HAL 回调处理。

3. SPI 总线支持阻塞、中断和 DMA 三种工作模式。BSP 层不内置传输队列；同一 SPI 总线在同一时刻只允许一个实例操作，不同 SPI 总线可以独立工作。对于需要排队发送的场景，请在 Modules 层级通过 FreeRTOS 队列等机制自行实现。

4. 片选由 BSP 层统一管理，低电平有效。每次传输开始前拉低片选，传输完成或出错后拉高片选，避免模块层遗漏释放片选。

5. 返回 `int` 的接口统一使用 `0` 表示成功、`-1` 表示失败；返回实例指针的接口失败时返回 `NULL`。

### 使用注意事项

1. `cs_port` 和 `cs_pin` 必须在注册前完成 GPIO 初始化，并将片选配置为推挽输出。注册成功后 BSP 会将片选置为高电平。

2. `SPI_Init_Config_S` 中不包含收发缓冲区。调用 `SPIReceive` 或 `SPITransmitAndReceive` 时需要由 Modules 层级传入有效缓冲区，BSP 只保存本次操作的缓冲区指针和长度。

3. 在 IT/DMA 模式下，发送缓冲区和接收缓冲区必须在对应回调触发前保持有效，不能在函数返回后立即释放或复用。

4. IT/DMA 模式要求对应 SPI 的 HAL 中断或 DMA 已经正确配置并使能，否则传输可能无法完成。

5. 同一 `hspi` 可以注册多个不同片选的从机实例，但总线忙时其他实例的传输请求会直接返回 `-1`，BSP 不会等待或重试。

6. `SPISetMode` 只能在实例空闲时切换模式，传输过程中不允许修改。

## 变量说明

`SPI_TXRX_MODE_e`：SPI 收发工作模式枚举，包括 `SPI_BLOCK_MODE`（阻塞）、`SPI_IT_MODE`（中断）和 `SPI_DMA_MODE`（DMA）三种。

`SPIInstance`：SPI 实例结构体，包含 HAL 句柄、片选引脚、工作模式、本次接收缓冲区、回调函数指针、所属模块指针以及总线和操作状态标志。

`SPI_Init_Config_S`：注册配置结构体，包含 `hspi`、`cs_port`、`cs_pin`、`mode`、`on_rx`、`on_tx_done` 和 `owner`。

`SPI_MAX_INST`：BSP 支持的最大 SPI 实例数，默认为 4，实例使用静态实例池管理。

`SPI_TIMEOUT_MS`：阻塞模式下 HAL 传输接口使用的默认超时时间，默认为 100 ms。

## 函数说明

### `SPIRegister`

注册实例函数。在 Modules 层级文件中调用 `SPIRegister(&cfg)`，传入配置：

- `hspi`：HAL SPI 句柄
- `cs_port`：片选 GPIO 端口
- `cs_pin`：片选 GPIO 引脚
- `mode`：收发模式（`SPI_BLOCK_MODE` / `SPI_IT_MODE` / `SPI_DMA_MODE`）
- `on_rx`：接收完成或全双工传输完成后的回调函数指针
- `on_tx_done`：仅发送完成后的回调函数指针
- `owner`：指向模块自身的指针，方便在回调中反查

注册时会检查参数、实例池容量以及 `hspi + cs_port + cs_pin` 是否重复。注册成功后将片选置高并返回实例指针，失败返回 `NULL`。

### `SPITransmit`

发送一帧数据。在 Modules 层级文件中调用 `SPITransmit(ins, buf, len)`，传入：

- `ins`：通过 `SPIRegister` 获取的实例指针
- `buf`：待发送数据的指针
- `len`：待发送数据的长度，不能为 0

函数会先占用对应 SPI 总线并拉低片选，然后按照实例的 `mode` 调用对应的 HAL 发送接口。

- **阻塞模式**：等待发送完成并确保 SPI 的 BSY 标志清零后释放片选，随后调用 `on_tx_done`；回调在函数返回前执行。
- **IT/DMA 模式**：HAL 启动成功后立即返回，传输完成时由 `HAL_SPI_TxCpltCallback` 释放片选并调用 `on_tx_done`。

参数错误、总线忙、实例忙或 HAL 启动失败时返回 `-1`；成功启动或完成时返回 `0`。BSP 层不排队，上层自行处理重试或排队。

### `SPIReceive`

接收一帧数据。在 Modules 层级文件中调用 `SPIReceive(ins, buf, len)`，传入：

- `ins`：通过 `SPIRegister` 获取的实例指针
- `buf`：接收缓冲区指针
- `len`：本次接收长度，不能为 0

函数会先占用对应 SPI 总线并拉低片选，然后按照实例的 `mode` 调用对应的 HAL 接收接口。

- **阻塞模式**：等待接收完成后释放片选并调用 `on_rx`；回调在函数返回前执行。
- **IT/DMA 模式**：HAL 启动成功后立即返回，接收完成时由 `HAL_SPI_RxCpltCallback` 释放片选并调用 `on_rx`。

参数错误、总线忙、实例忙或 HAL 启动失败时返回 `-1`；成功启动或完成时返回 `0`。

### `SPITransmitAndReceive`

全双工收发一帧数据。在 Modules 层级文件中调用 `SPITransmitAndReceive(ins, rx_buf, tx_buf, len)`，传入：

- `ins`：通过 `SPIRegister` 获取的实例指针
- `rx_buf`：接收缓冲区指针
- `tx_buf`：发送缓冲区指针
- `len`：本次收发长度，不能为 0

该接口同时发送 `tx_buf` 中的数据并接收数据到 `rx_buf`。无论阻塞、中断还是 DMA 模式，全双工传输完成后统一调用 `on_rx`，不会调用 `on_tx_done`。阻塞模式下回调在函数返回前执行，IT/DMA 模式下回调由 `HAL_SPI_TxRxCpltCallback` 触发。

参数错误、总线忙、实例忙或 HAL 启动失败时返回 `-1`；成功启动或完成时返回 `0`。

### `SPISetMode`

切换实例的收发模式。调用 `SPISetMode(ins, mode)`，仅在实例空闲且模式参数合法时生效。切换成功返回 `0`，参数错误或实例忙时返回 `-1`。

## 底层流动逻辑

**总线占用与片选**

1. 调用任一收发接口时，`spi_begin` 先检查当前实例是否忙，以及同一 `hspi` 是否已有其他实例正在使用。
2. 总线空闲时设置当前实例的 `busy` 和操作类型 `op`，随后拉低片选。
3. 传输完成、HAL 启动失败或 SPI 错误发生时，统一拉高片选并清除 `busy`，防止总线死锁。

**发送**

1. Modules 层级调用 `SPITransmit(ins, buf, len)`。
2. 根据当前 `mode` 选择 HAL 阻塞发送、中断发送或 DMA 发送。
3. 阻塞模式在 HAL 发送完成后等待 BSY 清零，再释放片选并调用 `on_tx_done`。
4. IT/DMA 模式由 `HAL_SPI_TxCpltCallback` 进入统一完成处理，等待 BSY 清零后释放片选并调用 `on_tx_done`。
5. 如果当前忙或 HAL 启动失败，接口返回 `-1`，不排队。

**接收**

1. Modules 层级调用 `SPIReceive(ins, buf, len)`，BSP 保存本次接收缓冲区指针和长度。
2. 根据实例的 `mode` 选择 HAL 阻塞接收、中断接收或 DMA 接收。
3. 阻塞模式在 HAL 接收完成后释放片选并调用 `on_rx`。
4. IT/DMA 模式由 `HAL_SPI_RxCpltCallback` 释放片选并调用 `on_rx`。

**全双工收发**

1. Modules 层级调用 `SPITransmitAndReceive(ins, rx_buf, tx_buf, len)`。
2. BSP 根据实例的 `mode` 选择对应的 HAL 全双工接口，并在操作期间保持片选有效。
3. 阻塞模式在传输完成后释放片选并调用 `on_rx`。
4. IT/DMA 模式由 `HAL_SPI_TxRxCpltCallback` 释放片选并调用 `on_rx`。

**错误处理**

HAL 触发 `HAL_SPI_ErrorCallback` 时，BSP 会查找当前占用该总线的实例，释放片选、清除忙标志，并根据当前操作类型调用 `on_tx_done` 或 `on_rx`。当前接口不会向回调传递传输成功或失败状态，模块层如有需要应结合业务协议自行判断数据有效性。