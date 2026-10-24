# STM32 MPU6050 姿态解算开发文档

## 1. 文档范围

本文说明 STM32F103C8T6 + MPU6050 六轴姿态系统的主要实现，重点包括：

- MPU6050 初始化、数据读取与物理量换算；
- 上电零偏校准与静止零漂跟踪；
- Roll/Pitch 互补滤波和 Yaw 陀螺积分；
- 输出 EMA 平滑；
- 相关函数职责、调用顺序与关键参数。

系统使用 USART1 以 1 ms 周期输出欧拉角，FreeRTOS 负责任务调度。

## 2. 数据流

```text
MPU6050
  -> MPU6050_ReadData()
  -> 原始值换算 Ax/Ay/Az、Gx/Gy/Gz
  -> MPU6050_is_still()
  -> mpu6050_update_bias()
  -> MPU6050_AttitudeFusion()
  -> MPU6050_GetEuler() + 输出 EMA
  -> USART1: roll,pitch,yaw
```

任务分工：

| 任务 | 主要职责 |
|------|----------|
| `IMUTask` | 初始化、校准、1 kHz 数据更新和姿态解算 |
| `CommTask` | 1 ms 周期读取欧拉角、转文本并发送 |
| `LEDTask` | 指示初始化、校准或运行状态 |

## 3. 硬件配置与初始化

| 项目 | 配置 |
|------|------|
| 接口 | I2C1，PB6=SCL，PB7=SDA，400 kHz |
| 从机地址 | `0x68`，HAL 8 位地址 `0xD0` |
| 采样率 | 1 kHz，`SMPLRT_DIV=0` |
| 数字低通 | `CONFIG=0x06`，约 5 Hz |
| 陀螺仪量程 | ±250 °/s，131 LSB/(°/s) |
| 加速度计量程 | ±2 g，16384 LSB/g |
| I2C 超时 | 20 ms |

`MPU6050_Init()` 执行以下步骤：

1. 读取 `WHO_AM_I`，排除 `0x00`、`0xFF` 等无效响应；
2. 写 `PWR_MGMT_1=0x01` 解除休眠并选择陀螺仪 PLL；
3. 延时 50 ms；
4. 配置采样率、DLPF、陀螺仪量程和加速度计量程。

任意 I2C 读写失败均返回 0。

## 4. 数据结构与驱动状态

```c
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    float roll;
    float pitch;
    float yaw;

    uint8_t still;
} MPU6050_t;
```

- `Ax/Ay/Az`：加速度，单位 g；
- `Gx/Gy/Gz`：角速度，单位 °/s；
- `roll/pitch/yaw`：姿态角，单位 °；
- `still`：静止标志。

驱动内部还维护以下静态状态：

| 状态 | 用途 |
|------|------|
| `gyro_bias[3]` | 三轴陀螺仪零偏 |
| `prev_tick` | 上一次姿态积分时刻 |
| `attitude_initialized` | 姿态是否已完成首次初始化 |
| `euler_filt[3]` | 输出 EMA 状态 |
| `euler_filt_init` | EMA 是否已初始化 |

注意：这些状态位于驱动内部，因此当前实现按单 MPU6050 实例设计。

## 5. 姿态解算方案

### 5.1 加速度计观测角

重力方向用于计算 Roll 和 Pitch：

```text
accel_roll  = atan2(Ay, Az) × 180/π
accel_pitch = atan2(-Ax, sqrt(Ay² + Az²)) × 180/π
```

Yaw 无法由重力方向获得，因此不使用加速度计校正。

### 5.2 陀螺仪积分预测

先扣除静态零偏，再对角速度积分：

```text
ωx = Gx - bias_x
ωy = Gy - bias_y
ωz = Gz - bias_z

roll  = roll  + ωx × dt
pitch = pitch + ωy × dt
yaw   = yaw   + ωz × dt
```

`dt` 由 FreeRTOS tick 计算，单位为秒：

```text
dt = (now_tick - prev_tick) × 0.001
```

若 `dt <= 0` 或 `dt > 0.2 s`，则回退为 0.001 s，避免异常时间差导致角度突跳。

### 5.3 Roll/Pitch 互补滤波

陀螺仪积分动态响应好，但存在零漂；加速度计长期稳定，但易受振动和线加速度影响。因此采用互补滤波：

```text
roll_error  = wrap180(accel_roll  - roll)
pitch_error = wrap180(accel_pitch - pitch)

roll  = roll  + (1 - α) × roll_error
pitch = pitch + (1 - α) × pitch_error
```

当前参数：

| 参数 | 值 | 作用 |
|------|----|------|
| `ATTITUDE_FUSION_ALPHA` | 0.98 | 陀螺积分权重 98%，加速度计校正权重 2% |
| `ACCEL_TRUST_MIN_G` | 0.75 | 加速度模长有效下限 |
| `ACCEL_TRUST_MAX_G` | 1.25 | 加速度模长有效上限 |

每 1 ms 只做 2% 的加速度计校正，对应时间常数约：

```text
τ ≈ 0.98 / 0.02 × 0.001 ≈ 49 ms
```

当加速度模长不在 0.75 g ~ 1.25 g 内时，本轮只进行陀螺积分，不执行加速度计校正，避免把运动加速度误认为重力方向。

首次进入姿态解算时，若加速度有效，则直接用加速度计观测角初始化 Roll/Pitch；否则初始化为 0。Yaw 初始化为 0。

### 5.4 Yaw 处理

Yaw 使用纯陀螺积分，并增加静止死区：

```text
if |ωz| < YAW_DEADBAND_DPS:
    ωz = 0
```

当前 `YAW_DEADBAND_DPS=0.3`，用于抑制静止时残余零偏和噪声被持续积分。

Yaw 不做 ±180° 归一化，因此会随积分持续增大或减小；Roll 在每次更新后归一化到 `[-180°, 180°]`。

### 5.5 方案特点与限制

- 直接维护三个欧拉角，运算量小、输出直观；
- Roll/Pitch 由加速度计长期校正，Yaw 仅依赖陀螺仪；
- 无磁力计，Yaw 仍会缓慢漂移；
- 直接欧拉角方案在 Pitch 接近 ±90° 时存在耦合和奇点，不适合全姿态连续解算；
- 若需要更高动态性能或全姿态输出，可改用四元数 Mahony/Madgwick 等方案。

## 6. 滤波与抗漂移方案

系统采用五层处理：

| 层级 | 方案 | 作用 |
|------|------|------|
| 1 | MPU6050 内部 DLPF，约 5 Hz | 抑制高频振动和传感器噪声 |
| 2 | 上电静态零偏校准 | 估计三轴陀螺仪初始零偏 |
| 3 | 运行时静止零偏跟踪 | 跟踪温漂和缓慢变化的零偏 |
| 4 | 陀螺积分 + 加速度计互补滤波 | 兼顾动态响应与长期姿态稳定 |
| 5 | 输出 EMA | 平滑串口输出，不改变融合状态 |

### 6.1 上电静态零偏校准

`MPU6050_Calibrate()` 采集 500 个样本，并筛选有效数据：

| 参数 | 值 | 含义 |
|------|----|------|
| `CAL_SAMPLES` | 500 | 总采样数，约 0.5 s |
| `CAL_MIN_SAMPLES` | 300 | 最少有效样本数 |
| `CAL_ACCEL_TOL_G` | 0.20 | 加速度模长允许偏差 |
| `CAL_GYRO_MEAN_MAX_DPS` | 10.0 | 陀螺仪均值上限 |
| `CAL_GYRO_STD_MAX_DPS` | 2.0 | 陀螺仪标准差上限 |

满足静止判断条件 `abs(|a|-1g) <= 0.20g` 的样本才参与统计。随后计算三轴角速度均值和标准差：

- 有效样本少于 300：失败；
- 任一轴均值超过 10 °/s：失败；
- 任一轴标准差超过 2 °/s：失败；
- 全部通过后，将均值写入 `gyro_bias[3]` 并重置姿态状态。

校准期间必须保持传感器静止。

### 6.2 静止检测与运行时零偏跟踪

`MPU6050_is_still()` 同时检查：

```text
abs(|a| - 1g) <= 0.08g
abs(Gx - bias_x) <= 1.0°/s
abs(Gy - bias_y) <= 1.0°/s
abs(Gz - bias_z) <= 1.0°/s
```

仅当 `still=1` 时，才跟踪零偏：

```text
bias = bias + 0.0005 × (G - bias)
```

`ZERO_RATE_GAIN=0.0005`，在 1 kHz 更新下时间常数约为 2 s，因此不会快速跟随真实运动。

### 6.3 输出 EMA

`MPU6050_GetEuler()` 在输出前执行一级 EMA：

```text
filtered = 0.15 × angle + 0.85 × filtered
```

1 kHz 调用时时间常数约 6 ms。EMA 只作用于对外输出，不影响内部互补滤波。

## 7. 相关函数

| 函数 | 作用 | 返回值 |
|------|------|--------|
| `MPU6050_ReadData()` | 连续读取 14 字节并解析六轴原始值 | 1 成功，0 失败 |
| `MPU6050_Init()` | 设备检测、唤醒和寄存器配置 | 1 成功，0 失败 |
| `MPU6050_Calibrate()` | 上电静态零偏校准 | 1 成功，0 失败 |
| `MPU6050_Update()` | 单次完整更新：读取、换算、静止判断、零偏更新和姿态融合 | 1 成功，0 失败 |
| `MPU6050_AttitudeFusion()` | 陀螺积分和 Roll/Pitch 互补校正 | 无 |
| `MPU6050_GetEuler()` | 获取经 EMA 平滑的三轴欧拉角 | 无 |
| `MPU6050_GetGyroBias()` | 获取当前三轴零偏，单位 °/s | 无 |

内部辅助函数：

| 函数 | 作用 |
|------|------|
| `attitude_fusion_reset()` | 清零姿态、时间戳和 EMA 状态 |
| `angle_wrap_180()` | 将角度归一化到 `[-180°, 180°]` |
| `MPU6050_is_still()` | 根据加速度模长和角速度判断静止 |
| `mpu6050_update_bias()` | 静止时缓慢更新陀螺仪零偏 |

推荐调用顺序：

```text
MPU6050_Init()
MPU6050_Calibrate()

while (1)
{
    MPU6050_Update()
    MPU6050_GetEuler()
}
```

`MPU6050_GetEuler()` 应保持固定调用周期，否则 EMA 时间常数会随调用频率变化。

## 8. 任务与串口输出

- `IMUTask` 每 1 ms 调用一次 `MPU6050_Update()`；
- `CommTask` 每 1 ms 获取互斥锁并调用 `MPU6050_GetEuler()`；
- 正常帧格式：`roll,pitch,yaw\n`，每个角度固定 3 位小数；
- 未就绪时每 500 ms 发送一次 `ERR,<status>\n`。

| 状态码 | 含义 |
|--------|------|
| 0 | 任务启动中，尚未进入初始化 |
| 1 | 初始化失败 |
| 2 | 校准失败 |
| 3 | 连续 10 次运行更新失败 |
| 4 | IMU 已就绪 |
