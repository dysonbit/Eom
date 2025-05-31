/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 定义编码器和车轮参数
#define ENCODER_CPR_QUADRATURE    122880.0f  // 1024线编码器 * 30减速比 * 4倍频
#define WHEEL_DIAMETER_M          0.0667f    // 车轮直径 (米)
#define M_PI                      3.1415926535f // 圆周率

// 假设左右轮间距 (米) 用于计算角速度
// 你需要测量你的小车两个轮子中心之间的距离 (米)
#define WHEEL_TRACK_M             0.173f

// 定义速度环PID的输出限制 (即平衡环目标角度的限制)
// 防止小车倾斜角度过大导致失控或摔倒
#define SPEED_PID_OUTPUT_MAX_DEG  5.0f      // 速度环输出的最大目标倾斜角度 (度)
#define INTEGRAL_ERROR_MAX_ABS    0.1f       // 积分项累积的最大绝对值，防止积分饱和

// 定义电机最小启动速度百分比
#define MIN_MOTOR_SPEED_PERCENTAGE 5 // 电机能有效启动的最小速度百分比，需要根据实际调试确定

// 浮点数比较阈值，用于判断是否接近0
#define FLOAT_ZERO_THRESHOLD 0.001f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//float g_pitch_angle = 0.0f;         // 当前 Pitch 角 (度)
//float g_accel_angle = 0.0f;         // 加速度计计算出的 Pitch 角 (度)
//float g_gyro_y_dps = 0.0f;          // Y轴角速度 (度/秒) - 假设 Y 轴对应 Pitch 方向旋转
//float g_target_pitch_angle = 0.0f;  // 目标平衡角 (度), 通常是 0
//
//// PID 参数 (这些值需要根据实际调试来确定!)
//// 平衡环 (PD 控制器)
//float g_balance_kp = 20.0f;  // 比例系数
//float g_balance_kd = 0.5f;   // 微分系数

// 互补滤波参数
// COMPLEMENTARY_FILTER_KP 越大，陀螺仪权重越高，响应快但易受漂移
// COMPLEMENTARY_FILTER_KP 越小，加速度计权重越高，稳定但易受振动干扰
//#define COMPLEMENTARY_FILTER_KP 0.98f // 陀螺仪权重 (建议 0.95 - 0.998)
//#define CONTROL_LOOP_PERIOD_S (0.01f) // 控制周期，与 TIM5 中断频率对应 (100Hz -> 0.01s)

//// 电机输出值 (这里存储最终的 PWM 值或者速度百分比)
//int16_t g_motor_output_left = 0;
//int16_t g_motor_output_right = 0;

// Raw sensor data from MPU6500
volatile int16_t g_accel_x_raw, g_accel_y_raw, g_accel_z_raw;
volatile int16_t g_gyro_x_raw, g_gyro_y_raw, g_gyro_z_raw;

// Sensor Bias values (calculated during initialization calibration)
volatile int16_t g_gyro_x_bias_raw = 0;
volatile int16_t g_gyro_y_bias_raw = 0;
volatile int16_t g_gyro_z_bias_raw = 0;
volatile float g_accel_pitch_bias_deg = 0.0f; // Bias for accelerometer calculated pitch angle

// Processed sensor data and estimated angle
volatile float g_pitch_angle = 0.0f;         // 当前 Pitch 角 (度) - 融合后
volatile float g_accel_angle = 0.0f;         // 加速度计计算出的 Pitch 角 (度)
volatile float g_gyro_x_dps = 0.0f;          // X轴角速度 (度/秒) - 假设 X 轴对应 Pitch
volatile float g_gyro_y_dps = 0.0f;          // Y轴角速度 (度/秒) - 假设 Y 轴对应 Roll
volatile float g_gyro_z_dps = 0.0f;          // Z轴角速度 (度/秒) - 假设 Z 轴对应 Yaw

volatile float g_target_pitch_angle = 0.0f;  // 目标平衡角 (度), 通常是 0

// PID 参数 (这些值需要根据实际调试来确定!)
// 平衡环 (PD 控制器)
volatile float g_balance_kp = 5.0;  // 比例系数
volatile float g_balance_kd = 0.1f;   // 微分系数
 volatile float g_balance_ki = 10.0f; // 积分系数 (暂不使用)

// 互补滤波参数
const float COMPLEMENTARY_FILTER_KP = 0.98f; // 陀螺仪权重 (建议 0.95 - 0.998)
const float CONTROL_LOOP_PERIOD_S = (0.01f); // 控制周期，与 TIM5 中断频率对应 (100Hz -> 0.01s)

// 电机输出值 (这里存储最终的速度百分比 -100到100)
volatile int16_t g_motor_output_left = 0;
volatile int16_t g_motor_output_right = 0;

// 编码器相关变量
volatile uint16_t  g_encoder_count_left_prev = 0;   // 上次读取的左轮编码器计数值 (TIM3)
volatile uint16_t  g_encoder_count_right_prev = 0;  // 上次读取的右轮编码器计数值 (TIM4)
int32_t g_encoder_delta_left = 0;        // 左轮编码器计数差值 (每控制周期)
int32_t g_encoder_delta_right = 0;       // 右轮编码器计数差值 (每控制周期)

volatile float g_left_wheel_speed_rps = 0.0f;     // 左轮速度 (转/秒)
volatile float g_right_wheel_speed_rps = 0.0f;    // 右轮速度 (转/秒)
volatile float g_robot_linear_speed_mps = 0.0f;   // 小车直线速度 (米/秒)
volatile float g_robot_angular_speed_dps = 0.0f;  // 小车角速度 (度/秒)

// 新增：速度环PID参数和变量
volatile float g_target_linear_speed_mps = 0.0f; // 目标直线速度 (米/秒)，通常是0，即原地平衡
volatile float g_speed_kp = 3469.0f;      // 速度环比例系数 (需要调试)
volatile float g_speed_ki = 300.f;     // 速度环积分系数 (需要调试)
volatile float g_speed_kd = 0.7f;     // 速度环微分系数 (需要调试)


// 速度环PID的静态变量，用于积分和微分项
static float g_speed_integral_error = 0.0f; // 速度积分误差累积
static float g_speed_prev_linear_speed_mps = 0.0f; // 上一次的直线速度，用于计算微分

volatile bool g_main_loop_motor_override = false;

volatile float left_right = 3.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

	// -------------------------------------------------------------------------
	// 编码器数据读取与速度计算 BEGIN
	// -------------------------------------------------------------------------

	// 1. 读取当前编码器计数值
	// 注意：TIM3用于左轮，TIM4用于右轮。编码器模式下，计数器会自动增减，反映方向。
	// 读取后，立即清零计数器，以便下一个控制周期重新计数。
	uint16_t current_count_left = __HAL_TIM_GET_COUNTER(&htim3);
//	__HAL_TIM_SET_COUNTER(&htim3, 0); // 将左轮编码器计数器清零

	uint16_t current_count_right = __HAL_TIM_GET_COUNTER(&htim4);
//	__HAL_TIM_SET_COUNTER(&htim4, 0); // 将右轮编码器计数器清零

	// 2. 计算每个控制周期内的编码器计数差值 (delta)
	// 由于计数器已清零，delta就是本次读取到的值。将其转换为int32_t以便处理负值。
//	g_encoder_delta_left = - (int32_t)current_count_left; // 加了个负号因为是相反的
//	g_encoder_delta_right = (int32_t)current_count_right;
	int32_t delta_left = (int32_t)current_count_left - (int32_t)g_encoder_count_left_prev;
	int32_t delta_right = (int32_t)current_count_right - (int32_t)g_encoder_count_right_prev;

	// 处理环绕 (Overflow/Underflow)
	// 如果差值的绝对值大于计数器周期的一半 (65535 / 2 = 32767.5)
	const int32_t HALF_PERIOD = 65536 / 2; // Using 65536 for uint16_t period
	const int32_t FULL_PERIOD = 65536;

	if (delta_left > HALF_PERIOD) { // 向下溢出 (例如从很小的正数跳到很大的正数)
	    delta_left -= FULL_PERIOD;
	} else if (delta_left < -HALF_PERIOD) { // 向上溢出 (例如从很小的负数(大无符号数)跳到很小的负数)
	    delta_left += FULL_PERIOD;
	}

	if (delta_right > HALF_PERIOD) { // 向下溢出
	    delta_right -= FULL_PERIOD;
	} else if (delta_right < -HALF_PERIOD) { // 向上溢出
	    delta_right += FULL_PERIOD;
	}

	// 3. **修正编码器计数方向**
	// 根据你的测试，左轮计数方向与右轮相反
	// 假设我们希望小车前进时，左右轮计数delta都为正。
	// 右轮：前转 -> 向上计数 (正 delta) - 保持不变
	// 左轮：前转 -> 向下计数 (负 delta) - 需要反转符号
	g_encoder_delta_left = -delta_left; // 反转左轮计数差值的符号
	g_encoder_delta_right = delta_right; // 右轮保持不变


	// 4. 更新上一次的计数值
	g_encoder_count_left_prev = current_count_left;
	g_encoder_count_right_prev = current_count_right;

	// 5. 将编码器计数差值转换为轮子转速 (转/秒, RPS)
	// 转速 = (计数差值 / 每转总计数) / 控制周期时间
	g_left_wheel_speed_rps = (float)g_encoder_delta_left / ENCODER_CPR_QUADRATURE / CONTROL_LOOP_PERIOD_S;
	g_right_wheel_speed_rps = (float)g_encoder_delta_right / ENCODER_CPR_QUADRATURE / CONTROL_LOOP_PERIOD_S;

	// 6. 将轮子转速转换为小车线速度 (米/秒, MPS)
	// 线速度 = 转速 * 车轮周长
	// 车轮周长 = PI * 直径
	float wheel_circumference = WHEEL_DIAMETER_M * M_PI;
	float left_wheel_linear_speed_mps = g_left_wheel_speed_rps * (WHEEL_DIAMETER_M * M_PI);
	float right_wheel_linear_speed_mps = g_right_wheel_speed_rps * (WHEEL_DIAMETER_M * M_PI);

	// 7. 计算小车整体的线速度和角速度
	// 小车线速度 (向前/向后) 是左右轮线速度的平均值
	g_robot_linear_speed_mps = (left_wheel_linear_speed_mps + right_wheel_linear_speed_mps) / 2.0f;
//	g_robot_linear_speed_mps = -g_robot_linear_speed_mps;
	// **重要检查：验证g_robot_linear_speed_mps的符号**
	// 如果小车向前移动时，g_robot_linear_speed_mps 是负值，则需要将其反转
	// 简单测试方法：手动向前推动小车，打印g_robot_linear_speed_mps，看是正还是负。
	// 如果是负，则取消下面一行的注释：
//	 g_robot_linear_speed_mps = -g_robot_linear_speed_mps;
//	 printf("g_robot_linear_speed_mps: %d", g_robot_linear_speed_mps);


	// 小车角速度 (原地转弯) 是左右轮线速度的差值除以轮距
	float angular_speed_radps = (right_wheel_linear_speed_mps - left_wheel_linear_speed_mps) / WHEEL_TRACK_M;
	// 将弧度/秒转换为度/秒
	g_robot_angular_speed_dps = angular_speed_radps * (180.0f / M_PI);

//	printf("g_encoder_delta_right: %d\r\n", g_encoder_delta_right);
//    printf("g_encoder_delta_left: %d\r\n", g_encoder_delta_left);
//    printf("right_wheel_linear_speed_mps: %d\r\n", right_wheel_linear_speed_mps);
//    printf("left_wheel_linear_speed_mps: %d\r\n", left_wheel_linear_speed_mps);
//    printf("g_robot_linear_speed_mps: %d\r\n", g_robot_linear_speed_mps);


	// -------------------------------------------------------------------------
	// 编码器数据读取与速度计算 END
	// -------------------------------------------------------------------------


	// -------------------------------------------------------------------------
	// 姿态数据处理 (MPU6500) BEGIN
	// -------------------------------------------------------------------------

	// Read raw sensor data from MPU6500
	mpu6500_read_accel_raw(&g_accel_x_raw, &g_accel_y_raw, &g_accel_z_raw);
	mpu6500_read_gyro_raw(&g_gyro_x_raw, &g_gyro_y_raw, &g_gyro_z_raw);

	// Apply bias correction to Gyro raw data
	int16_t gyro_x_corrected_raw = g_gyro_x_raw - g_gyro_x_bias_raw;
	int16_t gyro_y_corrected_raw = g_gyro_y_raw - g_gyro_y_bias_raw;
	int16_t gyro_z_corrected_raw = g_gyro_z_raw - g_gyro_z_bias_raw;

	// Convert bias-corrected Gyro data and raw Accel data to physical units
	float accel_x_g = (float)g_accel_x_raw / ACCEL_SENSITIVITY_2G;
	float accel_y_g = (float)g_accel_y_raw / ACCEL_SENSITIVITY_2G;
	float accel_z_g = (float)g_accel_z_raw / ACCEL_SENSITIVITY_2G;

	g_gyro_x_dps = (float)gyro_x_corrected_raw / GYRO_SENSITIVITY_2000DPS; // Pitch rate
	g_gyro_y_dps = (float)gyro_y_corrected_raw / GYRO_SENSITIVITY_2000DPS; // Roll rate
	g_gyro_z_dps = (float)gyro_z_corrected_raw / GYRO_SENSITIVITY_2000DPS; // Yaw rate

	// Calculate accelerometer angle (Pitch)
	if (accel_z_g == 0.0f) accel_z_g = 0.001f;
	g_accel_angle = atan2f(accel_x_g, accel_z_g) * (180.0f / M_PI);
	g_accel_angle -= g_accel_pitch_bias_deg;

	// Sensor fusion (Complementary Filter)
	g_pitch_angle = COMPLEMENTARY_FILTER_KP * (g_pitch_angle + g_gyro_x_dps * CONTROL_LOOP_PERIOD_S) + (1.0f - COMPLEMENTARY_FILTER_KP) * g_accel_angle;

	// -------------------------------------------------------------------------
	// 姿态数据处理 (MPU6500) END
	// -------------------------------------------------------------------------


	// -------------------------------------------------------------------------
	// 速度环 PID 控制 BEGIN
	// -------------------------------------------------------------------------

	// 1. 计算速度误差
	float speed_error = g_target_linear_speed_mps - g_robot_linear_speed_mps;

	// 2. 累积积分误差并进行抗积分饱和 (Anti-Windup)
	g_speed_integral_error += speed_error * CONTROL_LOOP_PERIOD_S;
	// 限制积分项的累积，防止电机长时间饱和导致过冲
	if (g_speed_integral_error > INTEGRAL_ERROR_MAX_ABS) g_speed_integral_error = INTEGRAL_ERROR_MAX_ABS;
	if (g_speed_integral_error < -INTEGRAL_ERROR_MAX_ABS) g_speed_integral_error = -INTEGRAL_ERROR_MAX_ABS;

	// 3. 计算微分项 (使用当前速度的导数，而不是误差的导数，更稳定)
	// 速度的导数就是加速度
	float speed_derivative_term = (g_robot_linear_speed_mps - g_speed_prev_linear_speed_mps) / CONTROL_LOOP_PERIOD_S;
	g_speed_prev_linear_speed_mps = g_robot_linear_speed_mps; // 更新上一次速度

	// 4. 计算速度环 PID 输出
	// 这里的输出是期望的倾斜角度（度）
	float speed_pid_output = g_speed_kp * speed_error +
	                         g_speed_ki * g_speed_integral_error - // 积分项
	                         g_speed_kd * speed_derivative_term;   // 微分项，注意这里是负号，因为我们希望通过反向的加速度来减小速度偏差
//    printf("speed_pid_output: %.2f\r\n", speed_pid_output);

	// 5. 限制速度环PID输出，作为平衡环的目标倾斜角度
	// 确保目标角度不会过大，防止小车摔倒
	if (speed_pid_output > SPEED_PID_OUTPUT_MAX_DEG) speed_pid_output = SPEED_PID_OUTPUT_MAX_DEG;
	if (speed_pid_output < -SPEED_PID_OUTPUT_MAX_DEG) speed_pid_output = -SPEED_PID_OUTPUT_MAX_DEG;

	// 将速度环的输出作为平衡环的目标角度
	// 如果小车向前加速，speed_error为负，speed_pid_output为负，则g_target_pitch_angle为负，
	// 意味着期望小车向后倾斜以减速。
	g_target_pitch_angle = speed_pid_output;

//	printf("\nEncoder Delta Left: %.2f\r\n", g_encoder_delta_left);
//	    printf("\nEncoder Delta Right: %.2f\r\n", g_encoder_delta_right);
//	    printf("\nLeft Wheel Speed (RPS): %.2f\r\n", g_left_wheel_speed_rps);
//	    printf("\nRight Wheel Speed (RPS): %.2f\r\n", g_right_wheel_speed_rps);
//	    printf("\nRobot Linear Speed (MPS): %.2f\r\n", g_robot_linear_speed_mps);

	// -------------------------------------------------------------------------
	// 速度环 PID 控制 END
	// -------------------------------------------------------------------------


	// -------------------------------------------------------------------------
	// 平衡环 PID 控制与电机驱动 BEGIN
	// -------------------------------------------------------------------------

	// PID Control Calculation
	// PID inputs: Pitch error (current angle - target angle) and Pitch rate (g_gyro_x_dps)

	// Note: g_target_pitch_angle is now dynamically set by the speed loop
	float pitch_error = g_target_pitch_angle - g_pitch_angle; // Angle error for P and I terms

	// **关键修正：平衡环D项符号**
	// 如果小车向前倾斜 (g_gyro_x_dps > 0)，需要向后推以减小角速度，所以D项贡献应为负。
	float control_output = g_balance_kp * pitch_error - g_balance_kd * g_gyro_x_dps; // ***重要：g_balance_kd前加负号***

	// Map Control Output to Motor Speed Percentage
	int16_t motor_speed = (int16_t)control_output;


	// **处理电机最小启动阈值（死区补偿）**
	// 如果计算出的速度在0和最小启动阈值之间（非零），将其提升到最小阈值
	if (motor_speed > 0 && motor_speed < MIN_MOTOR_SPEED_PERCENTAGE) {
	    motor_speed = MIN_MOTOR_SPEED_PERCENTAGE;
	} else if (motor_speed < 0 && motor_speed > -MIN_MOTOR_SPEED_PERCENTAGE) {
	    motor_speed = -MIN_MOTOR_SPEED_PERCENTAGE;
	}
	// 如果control_output接近0，则强制motor_speed为0，防止死区补偿导致在静止时抖动
	// 注意：这里使用 fabs(control_output) < FLOAT_ZERO_THRESHOLD 进行浮点数比较
	if (fabs(control_output) < FLOAT_ZERO_THRESHOLD) {
	    motor_speed = 0;
	}


	// Apply motor speed limits (-100 to 100 percent)
	if (motor_speed > 100) motor_speed = 100;
	if (motor_speed < -100) motor_speed = -100;

	// Basic Safety Check: Stop motor if tilt angle exceeds a safe threshold
	float safety_angle_threshold = 30.0f; // Degrees (adjust as needed, e.g., 30 degrees)

	if (fabs(g_pitch_angle) > safety_angle_threshold) {
	    // Stop motors if the robot is falling
	    g_motor_output_left = 0;
	    g_motor_output_right = 0;
	    // TODO: Add other safety actions like an alarm or state change
	}
	// 将 "=" 改为 "=="
	// 同时，为了浮点数比较的健壮性，使用 fabs 和 FLOAT_ZERO_THRESHOLD
	else if (fabs(left_right - 1.0f) < FLOAT_ZERO_THRESHOLD) { // 检查是否是左转指令
	    g_motor_output_left = motor_speed;
	    g_motor_output_right = 0;
	}
	else if (fabs(left_right - 2.0f) < FLOAT_ZERO_THRESHOLD) { // 检查是否是右转指令
		g_motor_output_left = 0;
	    g_motor_output_right = motor_speed;

	}
	else {
	    // If within safe angle, apply the calculated motor speed
	    g_motor_output_left = motor_speed;
	    g_motor_output_right = motor_speed;
	}

	// Apply Motor Speed using the Motor module functions
	    Car_Move(g_motor_output_left, g_motor_output_right);


	// --- PID control calculation and Motor application END ---

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
