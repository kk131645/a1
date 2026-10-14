/*
 * INS 姿态解算模块（简化 Mahony 互补滤波）
 *
 * 数据流：MPU6050原始采样 -> 去零偏的角速度/加速度 -> 四元数积分
 *       -> 加速度计重力方向校正 -> roll/pitch/yaw。
 *
 * 陀螺仪短时间响应快但会漂移；加速度计长期能给出重力方向但容易受振动影响。
 * Mahony滤波把两者结合起来。本工程没有磁力计，因此yaw只能依靠陀螺仪积分，
 * 长时间运行产生一定漂移属于正常现象。
 */

#include "ins.h"

#include <math.h>
#include <string.h>

#include "main.h"

// 滤波参数
#define INS_KP           0.8f           // 加速度计比例增益（越大收敛越快、越受振动影响）
#define INS_GYRO_ACC_LPF 0.3f           // 角加速度低通系数（0~1）
#define RAD2DEG          57.2957795f    // 弧度转角度
#define DEG2RAD          0.0174532925f  // 角度转弧度

/* 模块内部只保存一份最新解算结果，通过 ins_data() 返回其地址。 */
static ins_data_t _data;

static float _gyro_prev[3];     // 上一次的角速度，用于求角加速度
static uint32_t _last_tick;     // 上一次更新的时间戳
static uint8_t _inited;         // 是否已初始化
static uint8_t _first;          // 是否为第一次更新
static int16_t _yaw_round_count;// 偏航角累计圈数
static float _yaw_last;         // 上一次的偏航角，用于判断跳变

/**
 * @brief 初始化传感器和姿态解算状态。
 *
 * 函数先配置 MPU6050，再采样 500 次计算陀螺仪零偏，然后把姿态四元数设置为
 * 单位四元数 [1,0,0,0]，表示程序启动时暂按“无旋转”作为初始姿态。
 *
 * @note 校准阶段必须保持模块静止；该函数会阻塞约0.5秒以上，只应在启动时调用。
 */
void ins_init(void) {
    bsp_mpu6050_init();
    bsp_mpu6050_calibrate(500);  // 上电静止校准陀螺零偏，至少约0.5秒

    memset(&_data, 0, sizeof(_data));
    _data.q[0] = 1.0f;            // 初始姿态为单位四元数

    memset(_gyro_prev, 0, sizeof(_gyro_prev));
    _yaw_round_count = 0;
    _yaw_last = 0.0f;
    _first = 1;
    _last_tick = HAL_GetTick();
    _inited = 1;
}

/**
 * @brief 读取一次 MPU6050，并更新角速度、角加速度、四元数和欧拉角。
 *
 * 主要步骤：
 * 1. 读取底层传感器数据并由系统毫秒计数计算实际 dt；
 * 2. 对角速度做差分得到角加速度，再用一阶低通滤波；
 * 3. 将加速度向量归一化，把它视为测得的重力方向；
 * 4. 用“测量重力”和“四元数估计重力”的叉积修正陀螺仪；
 * 5. 积分并归一化四元数，最后转换为 roll/pitch/yaw；
 * 6. 检测yaw跨越+/-180度的跳变，生成可连续累计的yaw_total_angle。
 *
 * @note 本函数在 main() 主循环中反复调用，不应在定时器中断里执行I2C操作。
 * @note 未调用 ins_init() 时函数直接返回，不会访问未初始化的传感器。
 */
void ins_update(void) {
    if (!_inited) {
        return;
    }

    // 读取底层数据并更新时间戳
    bsp_mpu6050_data_t raw = bsp_mpu6050_read();
    _data.raw = raw;
    _data.timestamp = HAL_GetTick();

    // 计算积分周期 dt
    float dt = (float)(_data.timestamp - _last_tick) / 1000.0f;
    _last_tick = _data.timestamp;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.001f;
    }

    // 角速度直接输出（°/s）
    for (uint8_t i = 0; i < 3; i++) {
        _data.gyro[i] = raw.gyro[i];
    }

    // 角加速度 = 角速度差分，一阶低通抑制噪声（°/s²）
    if (_first) {
        for (uint8_t i = 0; i < 3; i++) {
            _gyro_prev[i] = raw.gyro[i];
            _data.gyro_accel[i] = 0.0f;
        }
    } else {
        for (uint8_t i = 0; i < 3; i++) {
            float da = (raw.gyro[i] - _gyro_prev[i]) / dt;
            _data.gyro_accel[i] += INS_GYRO_ACC_LPF * (da - _data.gyro_accel[i]);
            _gyro_prev[i] = raw.gyro[i];
        }
    }

    // 加速度计归一化，得到重力方向
    float ax = raw.accel[0], ay = raw.accel[1], az = raw.accel[2];
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm > 0.0001f) {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
    }

    float q0 = _data.q[0], q1 = _data.q[1], q2 = _data.q[2], q3 = _data.q[3];

    // 由四元数估计重力方向（半矢量）
    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    // 测量重力与估计重力的叉积，作为姿态误差
    float halfex = ay * halfvz - az * halfvy;
    float halfey = az * halfvx - ax * halfvz;
    float halfez = ax * halfvy - ay * halfvx;

    // 比例反馈修正角速度（四元数积分需要 rad/s）
    float gx = raw.gyro[0] * DEG2RAD + 2.0f * INS_KP * halfex;
    float gy = raw.gyro[1] * DEG2RAD + 2.0f * INS_KP * halfey;
    float gz = raw.gyro[2] * DEG2RAD + 2.0f * INS_KP * halfez;

    // 一阶龙格库塔法积分四元数
    float half_t = 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_t;
    q1 += ( qa * gx + qc * gz - q3 * gy) * half_t;
    q2 += ( qa * gy - qb * gz + q3 * gx) * half_t;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_t;

    // 四元数归一化
    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_norm > 0.0001f) {
        q0 /= q_norm;
        q1 /= q_norm;
        q2 /= q_norm;
        q3 /= q_norm;
    }
    _data.q[0] = q0;
    _data.q[1] = q1;
    _data.q[2] = q2;
    _data.q[3] = q3;

    // 四元数转欧拉角（输出为 °）
    _data.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    float sp = 2.0f * (q0 * q2 - q3 * q1);
    if (sp > 1.0f) {
        sp = 1.0f;
    } else if (sp < -1.0f) {
        sp = -1.0f;
    }
    _data.pitch = asinf(sp) * RAD2DEG;
    _data.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;

    // 累计偏航角，处理 ±180° 跳变
    if (_first) {
        _yaw_last = _data.yaw;
        _first = 0;
    }
    float d_yaw = _data.yaw - _yaw_last;
    if (d_yaw < -180.0f) {
        _yaw_round_count++;
    } else if (d_yaw > 180.0f) {
        _yaw_round_count--;
    }
    _yaw_last = _data.yaw;
    _data.yaw_total_angle = _data.yaw + (float)_yaw_round_count * 360.0f;
}

/**
 * @brief 获取模块内部最新姿态数据的地址。
 * @return 指向静态 ins_data_t 对象的指针，程序运行期间始终有效。
 * @note 调用者应读取该对象，不要长期修改其中字段；下一次ins_update()会覆盖数据。
 */
ins_data_t *ins_data(void) {
    return &_data;
}

/**
 * @brief 获取最新三轴角速度数组。
 * @return 长度为3的只读数组，依次为X/Y/Z轴，单位°/s。
 */
const float *ins_get_gyro(void) {
    return _data.gyro;
}

/**
 * @brief 获取最新三轴角加速度数组。
 * @return 长度为3的只读数组，依次为X/Y/Z轴，单位°/s²。
 */
const float *ins_get_gyro_accel(void) {
    return _data.gyro_accel;
}
