#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>

// I2C 从机地址
#define MPU6050_ADDR                (0x68 << 1)

// 寄存器地址
#define MPU_REG_SMPLRT_DIV          0x19   // 采样率分频
#define MPU_REG_CONFIG              0x1A   // DLPF 配置
#define MPU_REG_GYRO_CONFIG         0x1B   // 陀螺仪量程
#define MPU_REG_ACCEL_CONFIG        0x1C   // 加速度计量程
#define MPU_REG_ACCEL_XOUT_H        0x3B   // 加速度计 X 高字节（从此起连续 14 字节）
#define MPU_REG_PWR_MGMT_1          0x6B   // 电源管理 1
#define MPU_REG_WHO_AM_I            0x75   // 设备身份寄存器

#define IMU_DATA_LEN                14     // MPU6050 连续输出数据长度

#define MPU_VAL_PWR_MGMT_1_WAKE     0x01   // 解除休眠，时钟源选 X 轴陀螺 PLL
#define MPU_VAL_SMPLRT_DIV_1KHZ     0x00   // 1kHz 采样率
#define MPU_VAL_CONFIG_DLPF_5HZ     0x06   // DLPF=5Hz（陀螺仪+加速度计）
#define MPU_VAL_GYRO_CONFIG_250DPS  0x00   // 陀螺仪 ±250°/s
#define MPU_VAL_ACCEL_CONFIG_2G     0x00   // 加速度计 ±2g
#define MPU_WHO_AM_I_VALUE          0x68   // MPU6050 默认设备 ID

// 传感器数据结构
typedef struct
{
    int16_t Accel_X_RAW;    // 加速度计 X 原始值
    int16_t Accel_Y_RAW;    // 加速度计 Y 原始值
    int16_t Accel_Z_RAW;    // 加速度计 Z 原始值
    int16_t Gyro_X_RAW;     // 陀螺仪 X 原始值
    int16_t Gyro_Y_RAW;     // 陀螺仪 Y 原始值
    int16_t Gyro_Z_RAW;     // 陀螺仪 Z 原始值

    float Ax;             // 加速度计 X (g)
    float Ay;             // 加速度计 Y (g)
    float Az;             // 加速度计 Z (g)
    float Gx;             // 陀螺仪 X (°/s)
    float Gy;             // 陀螺仪 Y (°/s)
    float Gz;             // 陀螺仪 Z (°/s)

    float roll;             // 横滚角 (°)
    float pitch;            // 纵滚角 (°)
    float yaw;              // 偏航角 (°)

    uint8_t still;            // 是否静止

} MPU6050_t;


// API
uint8_t MPU6050_ReadData(MPU6050_t *mpu6050);

uint8_t MPU6050_Init(MPU6050_t *mpu6050);
uint8_t MPU6050_Calibrate(MPU6050_t *mpu6050);

void    MPU6050_AttitudeFusion(MPU6050_t *mpu6050);
uint8_t MPU6050_Update(MPU6050_t *mpu6050);

void    MPU6050_GetEuler(const MPU6050_t *mpu6050, float *pEuler);
void    MPU6050_GetGyroBias(const MPU6050_t *mpu6050, float *pBias);

#endif