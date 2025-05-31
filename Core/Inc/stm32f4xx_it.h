/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6500.h"
#include "motor.h"
#include "bluetooth_hc05.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern volatile int16_t g_accel_x_raw, g_accel_y_raw, g_accel_z_raw;
extern volatile int16_t g_gyro_x_raw, g_gyro_y_raw, g_gyro_z_raw;

// Sensor Bias values (calculated during initialization calibration)
extern volatile int16_t g_gyro_x_bias_raw;
extern volatile int16_t g_gyro_y_bias_raw; // Although mainly X is used for pitch, calibrate all
extern volatile int16_t g_gyro_z_bias_raw; // Although mainly X is used for pitch, calibrate all
extern volatile float g_accel_pitch_bias_deg; // Bias for accelerometer calculated pitch angle

extern volatile float g_pitch_angle;         // Current Pitch angle (degrees) - fused
extern volatile float g_accel_angle;         // Pitch angle from accelerometer (degrees)
extern volatile float g_gyro_x_dps;          // X-axis gyroscope angular rate (deg/s) - assuming X is Pitch axis
extern volatile float g_gyro_y_dps;          // Y-axis gyroscope angular rate (deg/s) - assuming Y is Roll axis
extern volatile float g_gyro_z_dps;          // Z-axis gyroscope angular rate (deg/s) - assuming Z is Yaw axis

extern volatile int16_t g_motor_output_left;
extern volatile int16_t g_motor_output_right;
extern volatile float g_target_pitch_angle;
// PID parameters - can be adjusted locally or remotely
extern volatile float g_balance_kp;  // Proportional gain
extern volatile float g_balance_kd;   // Derivative gain
extern volatile float g_balance_ki;
// extern volatile float g_balance_ki; // Integral gain (if used later)

// Complementary filter constant
extern const float COMPLEMENTARY_FILTER_KP;
extern const float CONTROL_LOOP_PERIOD_S;
//extern const float g_target_linear_speed_mps;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM5_IRQHandler(void);
void USART6_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
