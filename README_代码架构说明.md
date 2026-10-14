# banlance5 新工程代码架构说明

## 1. 工程要完成什么

这个工程使用 STM32F103、TB6612、带 AB 相霍尔编码器的 TT 电机和 MPU6050，
实现两种可通过 VOFA 切换的控制方式：

1. 速度模式：VOFA 发送 `I10:<目标RPM>`，编码器反馈实际转速，速度 PID 输出 PWM。
2. 角度模式：VOFA 发送 `I9:<目标角度>`，MPU6050 反馈 pitch；角度 PID 先计算目标
   RPM，再由编码器速度 PID 输出 PWM。这是“角度外环 + 速度内环”的串级 PID。

## 2. 数据如何流动

```text
速度模式：
VOFA I10 -> 目标RPM ------------------------------┐
                                                  v
编码器A/B -> TIM3计数 -> 实际RPM -> 速度PID -> TB6612 PWM -> 电机

角度模式：
VOFA I9 -> 目标角度 -> 角度PID -> 目标RPM --------┐
                         ^                        v
MPU6050 -> I2C -> 姿态解算 -> pitch -> 速度PID -> TB6612 PWM -> 电机
                                  ^
编码器A/B -> TIM3计数 -> 实际RPM --┘
```

角度模式并不是绕过编码器直接控制电机。角度 PID 只产生“应该转多快”，最终 PWM
仍由速度 PID 产生，因此两个模式都依赖编码器。

## 3. 主要文件和职责

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 系统启动、VOFA通信、模式切换、电机驱动、角度PID和速度PID |
| `Core/Src/mpu6050.c` | MPU6050寄存器读写、原始数据换算、陀螺仪零偏校准 |
| `Core/Src/ins.c` | 四元数姿态融合，输出roll、pitch、yaw、角速度和角加速度 |
| `Core/Src/tim.c` | TIM2 PWM、TIM3编码器、TIM4控制周期的硬件配置 |
| `Core/Src/usart.c` | USART1 115200-8-N-1、RX中断和TX DMA资源配置 |
| `Core/Src/i2c.c` | I2C1 100 kHz和PB6/PB7配置 |
| `Core/Src/gpio.c` | TB6612方向/待机引脚及PC13指示灯配置 |
| `Core/Src/dma.c` | USART1_TX所需DMA1通道4时钟和中断配置 |
| `Core/Src/stm32f1xx_it.c` | 硬件中断入口，把事件转交给HAL回调 |
| `Core/Src/stm32f1xx_hal_msp.c` | HAL全局底层初始化，释放PB3并保留SWD调试口 |
| `Core/Src/syscalls.c`、`sysmem.c` | C标准库在裸机环境中的系统调用和堆内存适配 |
| `Core/Src/system_stm32f1xx.c` | ST提供的芯片系统时钟基础支持代码 |

`Drivers`、启动汇编和链接脚本属于ST/CubeMX基础设施，不是本工程的控制算法。

## 4. 上电后的执行顺序

1. `HAL_Init()`：初始化HAL和1 ms SysTick。
2. `SystemClock_Config()`：配置72 MHz系统时钟。
3. `MX_*_Init()`：配置GPIO、DMA、定时器、串口和I2C。
4. PWM先置0，再拉高TB6612的STBY。
5. 启动TIM2 PWM、TIM3编码器、TIM4的10 ms中断和USART1接收中断。
6. `ins_init()`初始化MPU6050并进行静止零偏校准。
7. 进入`while(1)`：不断调用`ins_update()`，并每20 ms向VOFA发送一次数据。
8. 与主循环并行，TIM4每10 ms触发一次速度闭环；角度外环每两次TIM4中断运行一次。

## 5. main.c中的关键函数

### `angle_pid_update(angle, angle_rate)`

角度外环。计算`目标角度 - 实际角度`，使用P、I以及角速度阻尼项得到目标RPM。
输出限制为`±150 RPM`，不会直接写PWM。

### `motor_set(speed)`

TB6612最终输出函数。正负号控制AIN1/AIN2方向，绝对值写入TIM2_CH1比较寄存器，
范围限制为`0～999`。传入0时方向引脚和PWM都清零。

### `str_prefix_to_float_strict()`

把`I9:10.5`或`I10:-60`中冒号后面的部分转换为浮点数。它只负责取值，命令类型
由串口接收回调继续判断。

### `HAL_UART_RxCpltCallback()`

USART1每收到一个字节调用一次。收到换行后：

- `I9`更新目标角度并切换到角度模式；
- `I10`更新目标RPM并切换到速度模式；
- 设置复位请求，让控制中断安全地清空PID历史状态。

### `HAL_TIM_PeriodElapsedCallback()`

整个闭环控制最重要的函数，每10 ms执行：

1. 读取并清零TIM3编码器计数；
2. 换算实际RPM并低通滤波；
3. 根据模式确定速度目标；
4. 运行速度PID及积分限幅；
5. 把PID输出交给`motor_set()`。

## 6. MPU6050和姿态模块函数

| 函数 | 作用 |
|---|---|
| `write_reg()` | 通过I2C写一个MPU6050寄存器 |
| `read_reg()` | 从指定寄存器开始连续读取若干字节 |
| `bsp_mpu6050_init()` | 复位并配置100 Hz、±2 g、±250 °/s、44 Hz低通 |
| `bsp_mpu6050_read()` | 读取14字节并换算成g、°/s、℃ |
| `bsp_mpu6050_calibrate()` | 静止采样求陀螺仪三轴零偏 |
| `ins_init()` | 初始化传感器和四元数状态 |
| `ins_update()` | 执行一次姿态融合和欧拉角计算 |
| `ins_data()` | 返回最新完整姿态结果 |
| `ins_get_gyro()` | 返回三轴角速度数组 |
| `ins_get_gyro_accel()` | 返回三轴角加速度数组 |

没有磁力计约束时，yaw长时间漂移是正常现象。当前角度控制使用pitch和Y轴角速度。

## 7. 定时器的分工

| 定时器 | 用途 | 关键参数 |
|---|---|---|
| TIM2_CH1 | TB6612 PWMA | PA0，1 kHz，比较值0～999 |
| TIM3_CH1/CH2 | AB相编码器 | PA6/PA7，TI12四倍频，16位计数 |
| TIM4 | 控制定时中断 | 100 Hz，即10 ms一次 |

## 8. VOFA通道表

| 通道 | 数据 |
|---|---|
| I0～I2 | X/Y/Z角速度，°/s |
| I3～I5 | X/Y/Z角加速度，°/s² |
| I6、I7、I8 | roll、pitch、yaw，度 |
| I9 | 目标角度，度 |
| I10 | 速度模式目标转速，RPM |
| I11 | 编码器实际转速，RPM |
| I12 | 速度PID输出，约等于PWM比较值 |
| I13 | 最近10 ms的编码器增量，不是累计计数 |
| I14 | 模式：0=速度，1=角度 |
| I15 | 当前速度内环最终目标，RPM |
| I16 | 角度外环输出的目标转速，RPM |

VOFA使用FireWater，串口参数115200-8-N-1。发送命令时必须带换行符`\n`。

## 9. 当前必须注意的问题

1. `MOTOR_PPR=11`、`REDUCE_RATIO=30`与这次的13线、1:48/1:90 TT电机不匹配，
   因此当前I11转速数值不准确。1:48应改为13和48；1:90应改为13和90。
2. 编码器手册规定供电4.5～5 V；PA6/PA7不是可直接承受5 V的模拟复用引脚，
   使用5 V编码器时应增加分压或电平转换。
3. MPU6050驱动没有WHO_AM_I检查，也没有把I2C错误上传给主程序；断线时数据可能全0。
4. VOFA周期上传使用阻塞式串口发送，数据量继续增加时应改用DMA发送。
5. `cmake/stm32cubemx/CMakeLists.txt`仍保存旧电脑`C:/Users/91103/...`的HAL路径，
   `build/Debug`缓存也指向`D:/cubemx1/banlance5`；换电脑后需要重新配置本机工具链和HAL路径。
6. OLED显示和按键设置目标角度目前尚未实现。

