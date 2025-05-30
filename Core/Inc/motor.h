//
// Created by Guan on 5/11/2025.
//

#include "main.h" // 包含STM32 HAL库和CubeMX生成的引脚定义
#include "tim.h"  // 包含htim1的定义

#ifndef __MOTOR_H__
#define __MOTOR_H__

// 电机索引宏定义 (与main.c中保持一致)
#define MOTOR_A 0 // 假设电机A为左轮
#define MOTOR_B 1 // 假设电机B为右轮

// 电机方向宏定义 (与main.c中保持一致)
#define MOTOR_FORWARD  1 // 前进
#define MOTOR_BACKWARD 0 // 后退
#define MOTOR_STOP     2 // 停止 (此状态在SetSpeed中用于判断，实际停止是0速度)

// PWM周期 (ARR值, 来自CubeMX为TIM1的配置)
// (ARR + 1) 是PWM的总计数步数
#define PWM_PERIOD 839 // ARR 值 (产生 0 到 839 共 840 个计数步骤)

/*
 * 注意：以下 M_AIN1_Pin, M_AIN1_GPIO_Port 等宏定义
 * 预期是由 STM32CubeMX 在 main.h 中生成的。
 * 如果没有，您需要根据实际的 GPIO 配置在此处或 motor.c 中进行定义。
 * 例如 (如果在main.h中已经由CubeMX生成，则无需在此重复定义):
 * #define M_AIN1_Pin       GPIO_PIN_0
 * #define M_AIN1_GPIO_Port GPIOC
 * #define M_AIN2_Pin       GPIO_PIN_1
 * #define M_AIN2_GPIO_Port GPIOC
 * #define M_BIN1_Pin       GPIO_PIN_2
 * #define M_BIN1_GPIO_Port GPIOC
 * #define M_BIN2_Pin       GPIO_PIN_3
 * #define M_BIN2_GPIO_Port GPIOC
 */


// 函数原型声明

/**
  * @brief 初始化电机控制模块
  * @note  此函数应在定时器和GPIO初始化之后调用。
  *        它会启动PWM通道。
  * @param None
  * @retval None
  */
void Motor_Init(void);

/**
  * @brief 设置指定电机的速度和方向
  * @param motor_idx: 电机索引 (MOTOR_A 或 MOTOR_B)
  * @param speed_percent: 速度百分比 (-100 到 100)。
  *                       正值表示前进，负值表示后退，0表示停止。
  * @retval None
  */
void Motor_SetSpeed(uint8_t motor_idx, int16_t speed_percent);

/**
  * @brief 停止指定电机
  * @param motor_idx: 电机索引 (MOTOR_A 或 MOTOR_B)
  * @retval None
  */
void Motor_Stop(uint8_t motor_idx);

/**
  * @brief 控制小车整体移动
  * @param speed_A_percent: 电机A的速度百分比 (-100 到 100)
  * @param speed_B_percent: 电机B的速度百分比 (-100 到 100)
  * @retval None
  */
void Car_Move(int16_t speed_A_percent, int16_t speed_B_percent);

/**
  * @brief 小车前进
  * @param speed_percent: 速度百分比 (0 到 100)
  * @retval None
  */
void Car_Forward(uint8_t speed_percent);

/**
  * @brief 小车后退
  * @param speed_percent: 速度百分比 (0 到 100)
  * @retval None
  */
void Car_Backward(uint8_t speed_percent);

/**
  * @brief 小车左转 (差速原地转弯)
  * @param speed_percent: 速度百分比 (0 到 100)
  * @retval None
  */
void Car_TurnLeft(uint8_t speed_percent);

/**
  * @brief 小车右转 (差速原地转弯)
  * @param speed_percent: 速度百分比 (0 到 100)
  * @retval None
  */
void Car_TurnRight(uint8_t speed_percent);

/**
  * @brief 小车停止
  * @param None
  * @retval None
  */
void Car_Stop(void);


#endif /* __MOTOR_H__ */
