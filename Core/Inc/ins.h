//
// INS 姿态解算（基于 MPU6050，Mahony 互补滤波）
//

#pragma once

#include <stdint.h>

#include "mpu6050.h"

/** 姿态解算模块对外提供的完整状态快照。 */
typedef struct {
    float q[4];              /**< 姿态四元数 [w, x, y, z] */
    float roll, pitch, yaw;  /**< 姿态角，单位 ° */
    float yaw_total_angle;   /**< 累计偏航角，单位 °，可超过 ±180 */
    float gyro[3];           /**< 角速度，单位 °/s */
    float gyro_accel[3];     /**< 角加速度，单位 °/s² */
    uint32_t timestamp;      /**< 更新时间戳，单位 ms */
    bsp_mpu6050_data_t raw;  /**< 本次读取的底层数据 */
} ins_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化MPU6050及姿态状态，并完成陀螺仪零偏校准。
 * @note 调用期间必须保持传感器静止；通常只在main()启动阶段调用一次。
 */
void ins_init(void);

/**
 * @brief 读取一次传感器并执行一次姿态融合，需要在主循环中周期性调用。
 */
void ins_update(void);

/**
 * @brief 获取最新的完整解算结果。
 * @return 指向模块内部静态数据的指针，不需要也不能由调用者释放。
 */
ins_data_t *ins_data(void);

/**
 * @brief 获取角速度数组。
 * @return X/Y/Z三轴角速度只读数组，单位°/s，长度为3。
 */
const float *ins_get_gyro(void);

/**
 * @brief 获取角加速度数组。
 * @return X/Y/Z三轴角加速度只读数组，单位°/s²，长度为3。
 */
const float *ins_get_gyro_accel(void);

#ifdef __cplusplus
}
#endif
