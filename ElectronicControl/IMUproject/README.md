# stlintest2 - HorizonProject 移植版

本工程将 `HorizonProject` 的 MPU6050 姿态解算、FreeRTOS 任务和 1 kHz 串口协议移植到 STM32F103C8T6。

## 硬件配置

- MCU：STM32F103C8T6，LQFP48，64 KB Flash / 20 KB RAM
- 系统时钟：HSE 8 MHz，PLL x9，72 MHz
- 调试接口：SWD，PA13/PA14
- LED：PC13（板载 LED）
- MPU6050：I2C1
  - PB6：I2C1_SCL
  - PB7：I2C1_SDA
  - 400 kHz
- 上位机通信：USART1
  - PA9：USART1_TX
  - PA10：USART1_RX
  - 921600，8N1

## 运行逻辑

- `LEDTask`：IMU 未就绪时 100 ms 快闪，IMU 正常时 500 ms 慢闪。
- `IMUTask`：初始化 MPU6050，约 1 kHz 读取数据，完成欧拉角姿态融合（陀螺积分 + 加速度计互补校正）和零偏估计。
- `CommTask`：每 1 ms 通过 USART1 发送一行 `roll,pitch,yaw` ASCII 文本。
- 数据格式：`<roll>,<pitch>,<yaw>\n`，每个角度固定 3 位小数，例如 `-12.345,3.100,179.999\n`。

## 构建与烧录

在 VSCode 中使用 STM32CubeIDE 扩展构建，或使用命令行：

```powershell
cmake -S . -B build/Debug -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake `
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --parallel
```

然后运行 `ST-LINK Flash` 任务，或手动执行：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg `
  -c "transport select swd" -c "reset_config none" `
  -c "program ./build/Debug/stlintest2.elf verify reset exit"
```

## 注意事项

- MPU6050 模块必须和开发板共地；模块无上拉电阻时，PB6/PB7 需要外部上拉。
- 如果 LED 一直快闪、`g_imu_ready` 为 0，先检查 MPU6050 的 SDA/SCL、供电和共地。
- CubeMX 负责 I2C1、USART1 和 FreeRTOS 的底层初始化；串口接口适配位于 `Core/Src/usart.c`，重新生成后请确认它仍在 CMake 源文件列表中。
