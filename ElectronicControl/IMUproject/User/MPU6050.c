#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "MPU6050.h"
#include <math.h>
#include <stddef.h>

// I2C1 句柄
extern I2C_HandleTypeDef hi2c1;

// 内部常量
#define ACCEL_SCALE             16384.0f // 加速度计 ±2g 对应 16384 LSB/g
#define GYRO_SCALE              131.0f   // 陀螺仪 ±250°/s 对应 131 LSB/(°/s)
#define I2C_TIMEOUT             20      // I2C 超时时间
#define RAD_TO_DEG              57.29578f

#define ATTITUDE_FUSION_ALPHA   0.98f
#define ACCEL_TRUST_MIN_G       0.75f
#define ACCEL_TRUST_MAX_G       1.25f
#define EULER_EMA_ALPHA         0.15f // 欧拉角滤波系数
#define CAL_SAMPLES             500
#define CAL_MIN_SAMPLES         ((CAL_SAMPLES * 6) / 10)
#define CAL_GYRO_MEAN_MAX_DPS   10.0f
#define CAL_GYRO_STD_MAX_DPS    2.0f
#define STILL_GYRO_DPS          1.0f
#define STILL_ACCEL_TOL_G       0.08f
#define CAL_ACCEL_TOL_G         0.20f
#define ZERO_RATE_GAIN          0.0005f
#define YAW_DEADBAND_DPS        0.3f

// 运算状态。传感器数据全部由调用者传入的 MPU6050_t 实例保存。
static uint32_t prev_tick = 0;
static uint8_t attitude_initialized = 0;

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

static float euler_filt[3] = {0.0f, 0.0f, 0.0f};
static uint8_t euler_filt_init = 0;

// 姿态融合重置
static void attitude_fusion_reset(MPU6050_t *mpu6050)
{

    mpu6050->roll  = 0.0f;
    mpu6050->pitch = 0.0f;
    mpu6050->yaw   = 0.0f;
    mpu6050->still = 0;

    prev_tick = 0;
    attitude_initialized = 0;

    euler_filt[0] = 0.0f;
    euler_filt[1] = 0.0f;
    euler_filt[2] = 0.0f;
    euler_filt_init = 0;
}

// 角度归一化：-180° ~ +180°
static float angle_wrap_180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

// 使用传入实例中的加速度和陀螺仪数据判断是否静止
static uint8_t MPU6050_is_still(const MPU6050_t *mpu6050)
{
    float accel_norm_g;//加速度模长

    accel_norm_g = sqrtf(mpu6050->Ax * mpu6050->Ax +
                         mpu6050->Ay * mpu6050->Ay +
                         mpu6050->Az * mpu6050->Az);

    if (fabsf(accel_norm_g - 1.0f) > STILL_ACCEL_TOL_G)
    {
        return 0;
    }

    if ((fabsf(mpu6050->Gx - gyro_bias[0]) > STILL_GYRO_DPS) ||
        (fabsf(mpu6050->Gy - gyro_bias[1]) > STILL_GYRO_DPS) ||
        (fabsf(mpu6050->Gz - gyro_bias[2]) > STILL_GYRO_DPS))
    {
        return 0;
    }

    return 1;
}

// 读取 MPU6050 原始数据并解析到传入实例中
uint8_t MPU6050_ReadData(MPU6050_t *mpu6050)
{
    uint8_t raw[IMU_DATA_LEN];

    if (mpu6050 == NULL)
    {
        return 0;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, raw, IMU_DATA_LEN,
                         I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    mpu6050->Accel_X_RAW = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    mpu6050->Accel_Y_RAW = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    mpu6050->Accel_Z_RAW = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    mpu6050->Gyro_X_RAW  = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    mpu6050->Gyro_Y_RAW  = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    mpu6050->Gyro_Z_RAW  = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

    // 六轴数据全为 0 通常表示总线读取异常或设备未正常工作。
    if ((mpu6050->Accel_X_RAW == 0) && (mpu6050->Accel_Y_RAW == 0) &&
        (mpu6050->Accel_Z_RAW == 0) && (mpu6050->Gyro_X_RAW == 0) &&
        (mpu6050->Gyro_Y_RAW == 0) && (mpu6050->Gyro_Z_RAW == 0))
    {
        return 0;
    }

    return 1;
}

// MPU6050 初始化：唤醒 、配置采样率、DLPF 和量程
uint8_t MPU6050_Init(MPU6050_t *mpu6050)
{
    uint8_t value;
    uint8_t who_am_i = 0;

    if (mpu6050 == NULL)
    {
        return 0;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1,
                         I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    // 兼容部分 MPU6500/MPU9250 系列模块，排除无效总线数据。
    if ((who_am_i == 0x00) || (who_am_i == 0xFF))
    {
        return 0;
    }

    value = MPU_VAL_PWR_MGMT_1_WAKE;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &value, 1,
                          I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    value = MPU_VAL_SMPLRT_DIV_1KHZ;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_SMPLRT_DIV,
                          I2C_MEMADD_SIZE_8BIT, &value, 1,
                          I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    value = MPU_VAL_CONFIG_DLPF_5HZ;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &value, 1,
                          I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    value = MPU_VAL_GYRO_CONFIG_250DPS;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_GYRO_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &value, 1,
                          I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    value = MPU_VAL_ACCEL_CONFIG_2G;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_ACCEL_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &value, 1,
                          I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    return 1;
}
// 上电自校准：采集静止陀螺样本求零偏
uint8_t MPU6050_Calibrate(MPU6050_t *mpu6050)
{
    float gyro_rate_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_rate_sq_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t valid_samples = 0;//有效样本数
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float accel_norm_g;
    float gyro_mean[3];//陀螺仪均值
    float gyro_variance;//陀螺仪方差
    uint8_t axis;

    if (mpu6050 == NULL)
    {
        return 0;
    }

    for (uint16_t i = 0; i < CAL_SAMPLES; i++)
    {
        if (MPU6050_ReadData(mpu6050) != 0)
        {
            ax = (float)mpu6050->Accel_X_RAW / ACCEL_SCALE;
            ay = (float)mpu6050->Accel_Y_RAW / ACCEL_SCALE;
            az = (float)mpu6050->Accel_Z_RAW / ACCEL_SCALE;
            gx = (float)mpu6050->Gyro_X_RAW / GYRO_SCALE;
            gy = (float)mpu6050->Gyro_Y_RAW / GYRO_SCALE;
            gz = (float)mpu6050->Gyro_Z_RAW / GYRO_SCALE;

            accel_norm_g = sqrtf(ax * ax + ay * ay + az * az);

            if (fabsf(accel_norm_g - 1.0f) <= CAL_ACCEL_TOL_G)
            {
                gyro_rate_sum[0] += gx;
                gyro_rate_sum[1] += gy;
                gyro_rate_sum[2] += gz;
                gyro_rate_sq_sum[0] += gx * gx;
                gyro_rate_sq_sum[1] += gy * gy;
                gyro_rate_sq_sum[2] += gz * gz;
                valid_samples++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (valid_samples < CAL_MIN_SAMPLES)
    {
        return 0;
    }

    for (axis = 0; axis < 3; axis++)
    {
        gyro_mean[axis] =
            gyro_rate_sum[axis] / (float)valid_samples;

        if (fabsf(gyro_mean[axis]) > CAL_GYRO_MEAN_MAX_DPS)
        {
            return 0;
        }

        gyro_variance =
            (gyro_rate_sq_sum[axis] / (float)valid_samples) -
            (gyro_mean[axis] * gyro_mean[axis]);

        if (gyro_variance < 0.0f)
        {
            gyro_variance = 0.0f;
        }

        if (sqrtf(gyro_variance) > CAL_GYRO_STD_MAX_DPS)
        {
            return 0;
        }
    }

    gyro_bias[0] = gyro_mean[0];
    gyro_bias[1] = gyro_mean[1];
    gyro_bias[2] = gyro_mean[2];

    attitude_fusion_reset(mpu6050);
    return 1;
}
// 仅在静止时缓慢更新陀螺仪零偏
static void mpu6050_update_bias(const MPU6050_t *mpu6050)
{
    if (mpu6050->still == 0)
    {
        return;
    }

    gyro_bias[0] += ZERO_RATE_GAIN * (mpu6050->Gx - gyro_bias[0]);
    gyro_bias[1] += ZERO_RATE_GAIN * (mpu6050->Gy - gyro_bias[1]);
    gyro_bias[2] += ZERO_RATE_GAIN * (mpu6050->Gz - gyro_bias[2]);
}

// 姿态融合：陀螺仪积分 + 加速度计修正 roll/pitch
void MPU6050_AttitudeFusion(MPU6050_t *mpu6050)
{
    float accel_roll;
    float accel_pitch;
    float accel_norm_g;
    float gyro_rate_x;
    float gyro_rate_y;
    float gyro_rate_z;
    float dt;
    uint32_t now;
    uint8_t accel_valid;

    if (mpu6050 == NULL)
    {
        return;
    }

    gyro_rate_x = mpu6050->Gx - gyro_bias[0];
    gyro_rate_y = mpu6050->Gy - gyro_bias[1];
    gyro_rate_z = mpu6050->Gz - gyro_bias[2];

    if (fabsf(gyro_rate_z) < YAW_DEADBAND_DPS)
    {
        gyro_rate_z = 0.0f;
    }

    accel_roll = atan2f(mpu6050->Ay, mpu6050->Az) * RAD_TO_DEG;
    accel_pitch = atan2f(-mpu6050->Ax,
                         sqrtf(mpu6050->Ay * mpu6050->Ay +
                               mpu6050->Az * mpu6050->Az)) * RAD_TO_DEG;

    accel_norm_g = sqrtf(mpu6050->Ax * mpu6050->Ax +
                         mpu6050->Ay * mpu6050->Ay +
                         mpu6050->Az * mpu6050->Az);
    accel_valid = ((accel_norm_g >= ACCEL_TRUST_MIN_G) &&
                   (accel_norm_g <= ACCEL_TRUST_MAX_G)) ? 1 : 0;

    now = xTaskGetTickCount();

    if (attitude_initialized == 0)
    {
        if (accel_valid != 0)
        {
            mpu6050->roll  = accel_roll;
            mpu6050->pitch = accel_pitch;
        }
        else
        {
            mpu6050->roll  = 0.0f;
            mpu6050->pitch = 0.0f;
        }

        mpu6050->yaw = 0.0f;
        prev_tick = now;
        attitude_initialized = 1;
        return;
    }

    dt = (float)(now - prev_tick) * 0.001f;
    if ((dt <= 0.0f) || (dt > 0.2f))
    {
        dt = 0.001f;
    }
    prev_tick = now;

    mpu6050->roll  += gyro_rate_x * dt;
    mpu6050->pitch += gyro_rate_y * dt;
    mpu6050->yaw   += gyro_rate_z * dt;

    if (accel_valid != 0)
    {
        mpu6050->roll += (1.0f - ATTITUDE_FUSION_ALPHA) *
                         angle_wrap_180(accel_roll - mpu6050->roll);
        mpu6050->pitch += (1.0f - ATTITUDE_FUSION_ALPHA) *
                          angle_wrap_180(accel_pitch - mpu6050->pitch);
    }

    mpu6050->roll = angle_wrap_180(mpu6050->roll);
}

// 读取、换算、静止判断、零偏更新和姿态融合
uint8_t MPU6050_Update(MPU6050_t *mpu6050)
{
    if (mpu6050 == NULL)
    {
        return 0;
    }


    if (MPU6050_ReadData(mpu6050) == 0)
    {
        return 0;
    }

    mpu6050->Ax = (float)mpu6050->Accel_X_RAW / ACCEL_SCALE;
    mpu6050->Ay = (float)mpu6050->Accel_Y_RAW / ACCEL_SCALE;
    mpu6050->Az = (float)mpu6050->Accel_Z_RAW / ACCEL_SCALE;

    mpu6050->Gx = (float)mpu6050->Gyro_X_RAW / GYRO_SCALE;
    mpu6050->Gy = (float)mpu6050->Gyro_Y_RAW / GYRO_SCALE;
    mpu6050->Gz = (float)mpu6050->Gyro_Z_RAW / GYRO_SCALE;

    mpu6050->still = MPU6050_is_still(mpu6050);
    mpu6050_update_bias(mpu6050);
    MPU6050_AttitudeFusion(mpu6050);

    return 1;
}

// 获取滤波后的欧拉角
void MPU6050_GetEuler(const MPU6050_t *mpu6050, float *pEuler)
{
    if ((mpu6050 == NULL) || (pEuler == NULL))
    {
        return;
    }

    float angles[3];
    float alpha;//滤波系数

    angles[0] = mpu6050->roll;
    angles[1] = mpu6050->pitch;
    angles[2] = mpu6050->yaw;

    alpha = (euler_filt_init == 0) ? 1.0f : EULER_EMA_ALPHA;

    for (uint8_t i = 0; i < 3; i++)
    {
        euler_filt[i] = alpha * angles[i] + (1.0f - alpha) * euler_filt[i];
        pEuler[i] = euler_filt[i];
    }

    euler_filt_init = 1;
}

// 获取陀螺仪零偏，单位：°/s
void MPU6050_GetGyroBias(const MPU6050_t *mpu6050, float *pBias)
{
    if ((mpu6050 == NULL) || (pBias == NULL))
    {
        return;
    }
    pBias[0] = gyro_bias[0];
    pBias[1] = gyro_bias[1];
    pBias[2] = gyro_bias[2];
}