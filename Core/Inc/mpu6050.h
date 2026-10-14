//
// MPU6050 底层驱动（I2C1：SCL=PB6，SDA=PB7）
//

#pragma once

#include <stdint.h>

/** MPU6050 单次采样结果，所有字段均已从原始 ADC 计数换算为物理量。 */
typedef struct {
    float accel[3];  /**< 加速度，单位 g */
    float gyro[3];   /**< 角速度，单位 °/s（已去除零偏） */
    float temp;      /**< 温度，单位 ℃ */
} bsp_mpu6050_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 MPU6050，配置为100 Hz采样、±2 g、±250 °/s和约44 Hz低通。
 * @note 当前接口不返回错误码；可通过后续数据是否变化判断I2C通信是否正常。
 */
void bsp_mpu6050_init(void);

/**
 * @brief 读取一次加速度、角速度和温度，并完成单位换算及陀螺零偏扣除。
 * @return 换算后的加速度、角速度与温度；I2C失败时当前实现返回全0。
 */
bsp_mpu6050_data_t bsp_mpu6050_read(void);

/**
 * @brief 计算三轴陀螺仪零偏，调用期间传感器必须保持静止。
 * @param samples 采样次数；次数越多平均效果越好，但启动等待时间越长。
 */
void bsp_mpu6050_calibrate(uint16_t samples);

#ifdef __cplusplus
}
#endif
