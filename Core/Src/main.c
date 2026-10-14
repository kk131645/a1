/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "ins.h"
#include "cmsis_gcc.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * VOFA 命令接收缓冲区。
 * 串口中断每次只接收 1 个字节，rx_idx 指向下一个待写入的位置；收到 '\n'
 * 后把整行当成一条命令解析，例如 "I10:60\n"。
 */
uint8_t rx_buf[16] = {0};
uint8_t rx_idx = 0;

/* 旧版 4 字节接收方案保留的变量，目前控制流程没有使用。 */
uint8_t uart_rx_buf[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_SPEED 999

/** 控制器当前工作模式。两种模式最终都通过同一个速度 PID 驱动电机。 */
typedef enum
{
  CONTROL_MODE_SPEED = 0, /**< 速度模式：VOFA 的 I10 直接指定目标 RPM。 */
  CONTROL_MODE_ANGLE = 1  /**< 角度模式：VOFA 的 I9 指定目标角度。 */
} control_mode_t;

/* -------------------- 控制模式与角度外环状态 -------------------- */
volatile control_mode_t control_mode = CONTROL_MODE_SPEED;
volatile float target_angle = 0.0f;       /**< 用户设定的目标角度，单位：度。 */
volatile float control_angle = 0.0f;      /**< MPU6050 当前俯仰角 pitch，单位：度。 */
volatile float control_angle_rate = 0.0f; /**< Y 轴角速度，单位：度/秒。 */
volatile float angle_target_rpm = 0.0f;   /**< 角度外环计算出的目标转速。 */
volatile float speed_setpoint = 0.0f;     /**< 送入速度内环的最终目标转速。 */
volatile uint8_t control_reset_requested = 1; /**< 切换模式/目标后请求清空 PID 历史状态。 */

#define ANGLE_KP 4.0f             /**< 角度环比例系数：决定对角度误差的响应强度。 */
#define ANGLE_KI 0.4f             /**< 角度环积分系数：消除长期静差。 */
#define ANGLE_KD 0.3f             /**< 角度环微分系数：利用角速度抑制超调。 */
#define ANGLE_LOOP_DT 0.020f       /**< 角度环周期 20 ms，即 50 Hz。 */
#define ANGLE_INTEGRAL_LIMIT 40.0f /**< 角度积分限幅，防止积分饱和。 */
#define ANGLE_SPEED_LIMIT 150.0f   /**< 角度环允许给出的最大目标转速，单位 RPM。 */
#define ANGLE_TARGET_LIMIT 45.0f   /**< VOFA 可设置的目标角度范围为 +/-45 度。 */
#define ANGLE_SAFETY_LIMIT 75.0f   /**< 实际角度超过该值时立即停止电机。 */
#define ANGLE_CONTROL_DIRECTION 1.0f // 若角度误差越来越大，改为-1.0f

float angle_err = 0.0f;
float angle_err_sum = 0.0f;

/* -------------------- 编码器测速参数与状态 -------------------- */
volatile int16_t pulse_delta; /**< 最近 10 ms 内 TIM3 的有符号增量计数。 */
volatile float speed_rpm_raw = 0.0f;
volatile float speed_rpm = 0.0f;
#define MOTOR_PPR 11.0f      // 电机轴单通道每圈脉冲数；当前值必须按实际编码器修改
#define REDUCE_RATIO 30.0f   // 减速箱减速比；本项目所用 1:48/1:90 电机不可继续使用 30
#define ENCODER_MULTIPLIER 4.0f // TIM3使用TI12模式，对AB相进行4倍频计数
#define ENCODER_DIRECTION 1.0f  // 若正转时测速为负，改为-1.0f
#define SPEED_LOOP_DT 0.010f    // TIM4控制周期：10 ms
#define SPEED_FILTER_ALPHA 0.30f

/* -------------------- 速度 PID 参数与状态 -------------------- */
volatile float target_rpm = 0.0f;
#define SPEED_TARGET_LIMIT 200.0f
float sp_kp = 1.8f;
float sp_ki = 0.25f;
float sp_kd = 0.03f;
float sp_err = 0;
float sp_err_last = 0;
float sp_err_sum = 0;
volatile float speed_pid_out = 0;
#define PWM_LIMIT 999.0f
#define INTEGRAL_LIMIT 2000.0f





/**
 * @brief 计算角度外环 PID，并把角度误差换算为速度内环的目标转速。
 *
 * 该函数不直接驱动电机。它只完成“角度 -> 目标转速”的转换，返回值随后交给
 * 速度 PID。这样形成串级控制：角度外环决定电机应该转多快，速度内环再决定
 * TB6612 应输出多大的 PWM。
 *
 * @param angle 当前姿态角，单位为度；本工程传入 MPU6050 解算得到的 pitch。
 * @param angle_rate 当前角速度，单位为度/秒；直接用作微分反馈，避免对误差再次差分。
 * @return 速度内环目标值，单位为 RPM，并限制在 +/-ANGLE_SPEED_LIMIT 范围内。
 *
 * @note 由 50 Hz 角度环调用，因此积分项使用 ANGLE_LOOP_DT=0.020 s。
 */
float angle_pid_update(float angle, float angle_rate)
{
  angle_err = target_angle - angle;
  angle_err_sum += angle_err * ANGLE_LOOP_DT;

  if(angle_err_sum > ANGLE_INTEGRAL_LIMIT) angle_err_sum = ANGLE_INTEGRAL_LIMIT;
  if(angle_err_sum < -ANGLE_INTEGRAL_LIMIT) angle_err_sum = -ANGLE_INTEGRAL_LIMIT;

  float output = ANGLE_CONTROL_DIRECTION *
                 (ANGLE_KP * angle_err + ANGLE_KI * angle_err_sum - ANGLE_KD * angle_rate);

  if(output > ANGLE_SPEED_LIMIT) output = ANGLE_SPEED_LIMIT;
  if(output < -ANGLE_SPEED_LIMIT) output = -ANGLE_SPEED_LIMIT;
  return output;
}

/**
 * @brief 根据有符号控制量设置 TB6612 的方向引脚和 PWM 占空比。
 *
 * speed 的正负决定方向，绝对值决定 TIM2_CH1 的比较值：
 * - speed > 0：AIN1=1、AIN2=0，电机正转；
 * - speed < 0：AIN1=0、AIN2=1，电机反转；
 * - speed = 0：AIN1=0、AIN2=0、PWM=0，电机停止。
 *
 * @param speed 有符号 PWM 控制量，允许范围为 -999～999。
 *              超出范围时会先限幅，避免写入非法的定时器比较值。
 * @note 本函数只控制 A 通道；TB6612 的 STBY 已在 main() 启动阶段拉高。
 */
void motor_set(int16_t speed)
{
  //限幅，防止PWM溢出
  if(speed > MAX_SPEED)
  {
    speed = MAX_SPEED;
  }
  if(speed < -MAX_SPEED)
  {
    speed = -MAX_SPEED;
  }
  if(speed > 0)
  {
    //电机正向
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
  }
  else if(speed < 0)
  {
    //电机反向
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -speed);
  }
  else
  {
    //停止
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  }
}


/* USER CODE END 0 */

/**
  * @brief 程序入口：初始化硬件、启动控制中断，并在主循环完成姿态更新和VOFA上传。
  *
  * 执行顺序：
  * 1. HAL_Init() 初始化 HAL、SysTick 和中断优先级基础环境；
  * 2. SystemClock_Config() 把系统主频配置为 72 MHz；
  * 3. 初始化 GPIO、DMA、定时器、串口、I2C；
  * 4. 启动 PWM、编码器计数、10 ms 定时中断和串口单字节接收；
  * 5. 初始化 MPU6050 并校准陀螺仪；
  * 6. 进入无限循环，持续更新姿态并以 50 Hz 向 VOFA 发送监控数据。
  *
  * 电机 PID 不在 while(1) 中运行，而是在 TIM4 的周期中断回调中运行，
  * 因此主循环偶尔变慢不会直接改变 PID 的固定采样周期。
  *
  * @retval int 嵌入式程序不会正常退出，理论上不会执行到返回位置。
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //启动编码器
  //HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  //HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  //启动1ms定时中断跑PID
  //HAL_TIM_Base_Start_IT(&htim4);
  //开启串口中断接收VOFA浮点数
  //HAL_UART_Receive_IT(&huart1, uart_rx_buf,4);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* 上电先把PWM设为0，再使能TB6612，防止初始化过程中电机意外动作。 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);          // TIM2_CH1：输出电机PWM
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   // TIM3_CH1/CH2：AB相正交解码
  HAL_TIM_Base_Start_IT(&htim4);                    // TIM4：每10ms触发一次速度环
  HAL_UART_Receive_IT(&huart1, &rx_buf[rx_idx], 1); // USART1：逐字节接收VOFA命令



  ins_init(); // 初始化MPU6050并进行静止零偏校准，此阶段不要移动传感器
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 发送缓冲区
  char buff[256];
  uint32_t vofa_last_tick = 0;
  // 获取陀螺仪数据的指针
  ins_data_t *ins = ins_data();
  while (1)
  {// 更新 MPU6050 数据（陀螺仪 + 加速度计 + 姿态解算）
    ins_update();

    // I2C读取留在主循环；控制中断只使用最近一次姿态结果
    control_angle = ins->pitch;
    control_angle_rate = ins->gyro[1];

    /*
     * 速度模式：I10直接给定speed_setpoint。
     * 角度模式：I9经角度PID生成speed_setpoint。
     * 两种模式最终都只允许速度PID写PWM。
     */

    /*
     * VOFA使用FireWater：50 Hz发送，避免阻塞串口过度占用CPU。
     * 逗号分隔字段会按顺序显示为：
     * I0~I2   = X/Y/Z角速度；        I3~I5   = X/Y/Z角加速度；
     * I6~I8   = roll/pitch/yaw；      I9      = 目标角度；
     * I10     = 速度模式目标RPM；     I11     = 编码器实际RPM；
     * I12     = 速度PID输出PWM；      I13     = 最近10ms编码器增量；
     * I14     = 控制模式(0速度/1角度)；I15     = 速度内环最终目标；
     * I16     = 角度外环给出的目标RPM。
     */
    if ((HAL_GetTick() - vofa_last_tick) >= 20U)
    {
      vofa_last_tick = HAL_GetTick();
      sprintf(buff, "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.0f,%.3f,%.3f\n",
              ins->gyro[0], ins->gyro[1], ins->gyro[2],
              ins->gyro_accel[0], ins->gyro_accel[1], ins->gyro_accel[2],
              ins->roll, ins->pitch, ins->yaw, target_angle,
              target_rpm, speed_rpm, speed_pid_out, (int)pulse_delta,
              (float)control_mode, speed_setpoint, angle_target_rpm);
      HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 100);
    }

    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief 配置 STM32F103 系统时钟为 72 MHz。
  *
  * 时钟路径为：外部高速晶振 HSE 8 MHz -> PLL 倍频 9 -> SYSCLK 72 MHz。
  * AHB 保持 72 MHz，APB1 分频为 36 MHz，APB2 保持 72 MHz。由于 APB1
  * 分频不为 1，挂在 APB1 上的 TIM2/TIM3/TIM4 定时器时钟会自动倍频到
  * 72 MHz。USART1 位于 APB2，I2C1 位于 APB1。
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 把标准库 printf 的单字符输出重定向到 USART1。
 * @param ch 待发送的字符。
 * @param f 标准库传入的文件流指针，本实现不使用该参数。
 * @return 成功发送后原样返回 ch，符合 fputc 的接口约定。
 * @note 该实现为阻塞发送，适合调试信息，不适合在高频中断中调用。
 */
int fputc(int ch,FILE *f)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,10);
  return ch;
}

/**
 * @brief 解析冒号后面的浮点数，例如把 "I10:-35.5" 解析为 -35.5。
 * @param str 以 '\0' 结尾的命令字符串。
 * @param out_val 输出参数，解析成功时写入浮点值；失败时通常写入 0。
 * @return 0 表示成功，-1 表示参数为空、没有冒号或冒号后没有有效数字。
 * @note 本函数只负责取数，不判断 I9/I10 命令类型；命令类型由串口回调判断。
 */
int str_prefix_to_float_strict(const char *str, float *out_val)
{
  if (str == NULL || out_val == NULL)
    return -1;

  char *p_colon = strchr(str, ':');
  if (p_colon == NULL)
  {
    *out_val = 0;
    return -1;
  }
  const char *num_start = p_colon + 1;
  char *endptr;

  double tmp = strtod(num_start, &endptr);

  // endptr == num_start 代表没有解析到数字
  if (endptr == num_start)
  {
    *out_val = 0;
    return -1;
  }
  *out_val = (float)tmp;
  return 0;
}

/**
 * @brief HAL 串口接收完成回调，负责拼接并处理 VOFA 下发的一行命令。
 *
 * USART1 每收到一个字节都会进入一次本函数：普通字符写入 rx_buf；回车 '\r'
 * 被忽略；换行 '\n' 表示一条命令结束。当前支持：
 * - I9:<角度>  ：切换为角度模式，范围限制为 +/-45 度；
 * - I10:<转速>：切换为速度模式，范围限制为 +/-200 RPM。
 *
 * 新目标到来后只设置 control_reset_requested，真正的 PID 状态清零在 TIM4
 * 中断中完成，避免主循环和中断同时修改同一组控制状态。
 *
 * @param huart 触发回调的串口句柄；本函数只处理 USART1。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  if(huart->Instance == USART1)
  {
    uint8_t received_char = rx_buf[rx_idx];

    if(received_char == '\n')
    {
      float received_value;

      /* 用字符串结束符覆盖换行符 */
      rx_buf[rx_idx] = '\0';

      if(str_prefix_to_float_strict(
            (const char *)rx_buf,
            &received_value) == 0)
      {
        /*
         * 判断命令前缀：
         * i9  修改目标角度
         * i10 修改目标转速
         */
        if(strstr((const char *)rx_buf, "I9:") != NULL)
        {
          if(received_value > ANGLE_TARGET_LIMIT) received_value = ANGLE_TARGET_LIMIT;
          if(received_value < -ANGLE_TARGET_LIMIT) received_value = -ANGLE_TARGET_LIMIT;
          target_angle = received_value;
          control_mode = CONTROL_MODE_ANGLE;
          control_reset_requested = 1;
        }
        else if(strstr((const char *)rx_buf, "I10:") != NULL)
        {
          if(received_value > SPEED_TARGET_LIMIT) received_value = SPEED_TARGET_LIMIT;
          if(received_value < -SPEED_TARGET_LIMIT) received_value = -SPEED_TARGET_LIMIT;
          target_rpm = received_value;
          control_mode = CONTROL_MODE_SPEED;
          control_reset_requested = 1;
        }
      }

      /* 清空缓冲区，准备接收下一条命令 */
      memset(rx_buf, 0, sizeof(rx_buf));
      rx_idx = 0;
    }
    else if(received_char == '\r')
    {
      /*
       * 忽略回车。
       * rx_idx 不增加，下一字节会覆盖当前位置。
       */
    }
    else
    {
      rx_idx++;

      /* 防止缓冲区越界 */
      if(rx_idx >= sizeof(rx_buf) - 1)
      {
        memset(rx_buf, 0, sizeof(rx_buf));
        rx_idx = 0;
      }
    }

    /* 始终一次只接收一个字节 */
    HAL_UART_Receive_IT(&huart1, &rx_buf[rx_idx], 1);
  }
}

/**
 * @brief TIM4 的 10 ms 周期回调，是整个电机闭环控制的实时核心。
 *
 * 每次进入后的处理顺序：
 * 1. 读取 TIM3 编码器增量并清零计数器；
 * 2. 把 10 ms 脉冲数换算成输出轴 RPM，并进行一阶低通滤波；
 * 3. 若刚切换模式/目标，清空两个 PID 的历史状态并安全停车一次；
 * 4. 角度模式每 20 ms 计算一次角度外环，得到目标 RPM；速度模式直接采用 I10；
 * 5. 计算速度 PID，执行积分限幅和抗积分饱和；
 * 6. 将 PID 输出限幅后交给 motor_set() 更新方向和 PWM。
 *
 * @param htim 触发回调的定时器句柄；只有 TIM4 会执行控制代码。
 * @note 中断中不能加入 printf、HAL_Delay 或耗时 I2C 读取，否则会破坏固定控制周期。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t angle_loop_divider = 0;

  if(htim->Instance == TIM4)
  {
    //读取脉冲，立刻清零计数器
    pulse_delta = (int16_t) __HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3,0);

    // 根据10 ms采样窗口和AB相4倍频计数计算转速，并低通滤波
    speed_rpm_raw = ENCODER_DIRECTION * (float)pulse_delta * 60.0f /
                    (MOTOR_PPR * ENCODER_MULTIPLIER * REDUCE_RATIO * SPEED_LOOP_DT);
    speed_rpm += SPEED_FILTER_ALPHA * (speed_rpm_raw - speed_rpm);

    if (control_reset_requested)
    {
      sp_err = 0.0f;
      sp_err_last = 0.0f;
      sp_err_sum = 0.0f;
      angle_err = 0.0f;
      angle_err_sum = 0.0f;
      angle_target_rpm = 0.0f;
      speed_setpoint = 0.0f;
      speed_pid_out = 0.0f;
      angle_loop_divider = 0;
      control_reset_requested = 0;
      motor_set(0);
      return;
    }

    if (control_mode == CONTROL_MODE_ANGLE)
    {
      // 姿态超出安全范围时立即停止，避免机构继续失控
      if (fabsf(control_angle) > ANGLE_SAFETY_LIMIT)
      {
        angle_target_rpm = 0.0f;
        speed_setpoint = 0.0f;
        speed_pid_out = 0.0f;
        sp_err_sum = 0.0f;
        motor_set(0);
        return;
      }

      // 速度内环100 Hz，角度外环每两次运行一次，即50 Hz
      angle_loop_divider++;
      if (angle_loop_divider >= 2U)
      {
        angle_loop_divider = 0;
        angle_target_rpm = angle_pid_update(control_angle, control_angle_rate);
      }
      speed_setpoint = angle_target_rpm;
    }
    else
    {
      angle_loop_divider = 0;
      angle_target_rpm = 0.0f;
      speed_setpoint = target_rpm;
    }

    // 速度模式下0 RPM作为明确停车命令；角度模式下仍保留速度环以抵抗扰动
    if (control_mode == CONTROL_MODE_SPEED && fabsf(speed_setpoint) < 0.01f)
    {
      sp_err_sum = 0.0f;
      sp_err_last = 0.0f;
      speed_pid_out = 0.0f;
      motor_set(0);
      return;
    }

    // ----- PID计算 -----
    sp_err = speed_setpoint - speed_rpm;

    // 先计算候选积分；若输出已沿误差方向饱和，则停止继续积分
    float sp_err_sum_candidate = sp_err_sum + sp_err;
    if(sp_err_sum_candidate > INTEGRAL_LIMIT) sp_err_sum_candidate = INTEGRAL_LIMIT;
    if(sp_err_sum_candidate < -INTEGRAL_LIMIT) sp_err_sum_candidate = -INTEGRAL_LIMIT;

    float sp_p = sp_kp * sp_err;
    float sp_d = sp_kd * (sp_err - sp_err_last);
    float output_candidate = sp_p + sp_ki * sp_err_sum_candidate + sp_d;
    if (!((output_candidate > PWM_LIMIT && sp_err > 0.0f) ||
          (output_candidate < -PWM_LIMIT && sp_err < 0.0f)))
    {
      sp_err_sum = sp_err_sum_candidate;
    }

    speed_pid_out = sp_p + sp_ki * sp_err_sum + sp_d;
    sp_err_last = sp_err;

    //输出限幅
    if(speed_pid_out > PWM_LIMIT) speed_pid_out = PWM_LIMIT;
    if(speed_pid_out < -PWM_LIMIT) speed_pid_out = -PWM_LIMIT;

    motor_set((int16_t)speed_pid_out);
  }
}


/* USER CODE END 4 */

/**
  * @brief 致命错误处理函数。
  *
  * 外设或时钟初始化失败时会进入这里。函数关闭全局中断并永久停留，便于调试器
  * 定位故障；产品化时可在这里增加关闭电机、点亮故障灯或触发看门狗复位。
  *
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief HAL参数断言失败时的调试回调。
  * @param file 触发断言的源文件名。
  * @param line 触发断言的源代码行号。
  * @note 只有定义USE_FULL_ASSERT时才会编译；当前函数预留给断点或日志输出。
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
