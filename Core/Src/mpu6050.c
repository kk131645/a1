/*
 * MPU6050 底层驱动
 *
 * 本文件只负责“和芯片通信以及单位换算”，不负责姿态角计算：
 * - I2C1：SCL=PB6，SDA=PB7；
 * - 读取原始加速度、温度和陀螺仪寄存器；
 * - 将原始整数换算成 g、°/s 和 ℃；
 * - 保存并扣除上电校准得到的陀螺仪零偏。
 *
 * 更高一层的四元数融合、欧拉角计算位于 ins.c。
 */

#include "mpu6050.h"

#include "i2c.h"

/* 这里只列出本工程实际使用的 MPU6050 寄存器地址。 */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_ADDR    (0x68 << 1)  // AD0 接地，7 位地址 0x68
#define MPU6050_TIMEOUT 100          // I2C 超时时间，单位 ms

// 满量程对应的灵敏度
#define MPU6050_ACCEL_SEN 16384.0f  // ±2g：      LSB/g
#define MPU6050_GYRO_SEN  131.0f    // ±250°/s：  LSB/(°/s)

static float gyro_bias[3];  // X/Y/Z 三轴陀螺仪零偏，单位 °/s

/**
 * @brief 通过 I2C 向 MPU6050 的一个 8 位寄存器写入数据。
 * @param reg 寄存器地址。
 * @param val 要写入的 8 位值。
 * @return 1 表示 HAL 返回 HAL_OK，0 表示通信超时或总线错误。
 */
static uint8_t write_reg(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, MPU6050_TIMEOUT) == HAL_OK;
}

/**
 * @brief 从指定寄存器开始连续读取若干字节。
 * @param reg 起始寄存器地址；MPU6050 会自动递增内部地址。
 * @param buf 接收缓冲区，必须至少能容纳 len 个字节。
 * @param len 需要读取的字节数。
 * @return 1 表示读取成功，0 表示通信失败。
 */
static uint8_t read_reg(uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, MPU6050_TIMEOUT) == HAL_OK;
}

/**
 * @brief 复位并配置 MPU6050。
 *
 * 配置结果：采样率 100 Hz、数字低通约 44 Hz、加速度量程 +/-2 g、
 * 陀螺仪量程 +/-250 °/s。AD0 接地时器件 7 位地址为 0x68。
 *
 * @note 当前实现没有读取 WHO_AM_I，也没有向上返回初始化失败状态。因此若接线
 *       错误，程序仍会继续运行，但读取结果通常保持为 0；调试时应优先检查 I2C。
 */
void bsp_mpu6050_init(void) {
    HAL_Delay(50);

    write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);   // 复位
    HAL_Delay(100);
    write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);   // 唤醒，时钟源选择 X 轴陀螺 PLL
    HAL_Delay(10);

    write_reg(MPU6050_REG_CONFIG, 0x03);        // 数字低通滤波 44Hz
    write_reg(MPU6050_REG_SMPLRT_DIV, 0x09);    // 采样率 = 1kHz / (1 + 9) = 100Hz
    write_reg(MPU6050_REG_GYRO_CONFIG, 0x00);   // 陀螺仪 ±250°/s
    write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00);  // 加速度计 ±2g
}

/**
 * @brief 连续读取 MPU6050 的 14 字节测量数据，并换算为物理量。
 *
 * 数据排列为 AX、AY、AZ、温度、GX、GY、GZ，每项都是高字节在前的有符号
 * 16 位数。陀螺仪结果会减去 bsp_mpu6050_calibrate() 得到的零偏。
 *
 * @return 一份按值返回的传感器数据：加速度单位 g、角速度单位 °/s、温度单位 ℃。
 * @note I2C 读取失败时返回全 0 结构体，调用方目前无法区分“静止”与“读取失败”。
 */
bsp_mpu6050_data_t bsp_mpu6050_read(void) {
    bsp_mpu6050_data_t ret = {0};
    uint8_t buf[14];

    // 读取：加速度(6) + 温度(2) + 角速度(6)
    if (read_reg(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t temp = (int16_t)((buf[6] << 8) | buf[7]);
        int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

        ret.accel[0] = (float)ax / MPU6050_ACCEL_SEN;
        ret.accel[1] = (float)ay / MPU6050_ACCEL_SEN;
        ret.accel[2] = (float)az / MPU6050_ACCEL_SEN;

        ret.gyro[0] = (float)gx / MPU6050_GYRO_SEN - gyro_bias[0];
        ret.gyro[1] = (float)gy / MPU6050_GYRO_SEN - gyro_bias[1];
        ret.gyro[2] = (float)gz / MPU6050_GYRO_SEN - gyro_bias[2];

        ret.temp = (float)temp / 340.0f + 36.53f;
    }

    return ret;
}

/**
 * @brief 静止采样若干次，求三轴陀螺仪平均值作为零偏。
 * @param samples 校准采样次数；传入 0 时直接返回。当前 ins_init() 传入 500。
 *
 * 校准期间必须让 MPU6050 保持静止，否则真实转动会被误认为零偏，导致姿态角
 * 持续漂移。每次采样后延时 1 ms，500 次校准至少需要约 0.5 秒。
 *
 * @note 当前代码即使某次 I2C 读取失败，最终仍除以 samples；通信不稳定时会影响
 *       校准结果，后续可改为按“成功读取次数”求平均。
 */
void bsp_mpu6050_calibrate(uint16_t samples) {
    if (samples == 0) {
        return;
    }

    uint8_t buf[6];
    float sum[3] = {0};

    for (uint16_t i = 0; i < samples; i++) {
        if (read_reg(MPU6050_REG_GYRO_XOUT_H, buf, sizeof(buf))) {
            sum[0] += (float)(int16_t)((buf[0] << 8) | buf[1]) / MPU6050_GYRO_SEN;
            sum[1] += (float)(int16_t)((buf[2] << 8) | buf[3]) / MPU6050_GYRO_SEN;
            sum[2] += (float)(int16_t)((buf[4] << 8) | buf[5]) / MPU6050_GYRO_SEN;
        }
        HAL_Delay(1);
    }

    gyro_bias[0] = sum[0] / (float)samples;
    gyro_bias[1] = sum[1] / (float)samples;
    gyro_bias[2] = sum[2] / (float)samples;
}
