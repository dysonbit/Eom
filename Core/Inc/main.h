/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include"string.h"
#include <stdlib.h>
#include <time.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#define M_LEFT_IN1_Pin GPIO_PIN_0
#define M_LEFT_IN1_GPIO_Port GPIOC
#define M_LEFT_IN2_Pin GPIO_PIN_1
#define M_LEFT_IN2_GPIO_Port GPIOC
#define M_RIGHT_IN1_Pin GPIO_PIN_2
#define M_RIGHT_IN1_GPIO_Port GPIOC
#define M_RIGHT_IN2_Pin GPIO_PIN_3
#define M_RIGHT_IN2_GPIO_Port GPIOC
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define led_Pin GPIO_PIN_5
#define led_GPIO_Port GPIOA
#define gyro_cs_Pin GPIO_PIN_15
#define gyro_cs_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
// 全局变量的外部声明 (它们在 stm32f4xx_it.c 中定义)
extern volatile float g_pitch_angle;
extern volatile float g_accel_angle;
extern volatile float g_gyro_x_dps;
extern volatile float g_robot_linear_speed_mps;
extern volatile float g_robot_angular_speed_dps; // 即使暂时不用，也可以声明

extern volatile int16_t g_motor_output_left;
extern volatile int16_t g_motor_output_right;

extern volatile float g_target_pitch_angle;
extern volatile float g_balance_kp;
extern volatile float g_balance_kd;

extern volatile float g_target_linear_speed_mps; // 确保有这个声明
extern volatile float g_speed_kp;
extern volatile float g_speed_ki;
extern volatile float g_speed_kd;
extern volatile float g_target_linear_speed_mps;
extern volatile float g_target_pitch_angle; // 也可以暴露，但通常由速度环内部控制

extern volatile float left_right; //左右转向控制

// 定义基础前进/后退速度

#define PULSE_FORWARD_SPEED_MPS     0.03f   // 脉冲前进的目标速度 (米/秒)。这是一个示例值，需要调试。
#define PULSE_DURATION_MS           50    // 每次前进脉冲的持续时间 (毫秒)
#define BALANCE_DURATION_MS         1000   // 停下并调整平衡的持续时间 (毫秒)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
