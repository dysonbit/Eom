//
// Created by Guan on 5/11/2025.
//

#include "motor.h"

// 外部变量声明 (htim1 是在 tim.c 中定义的，并通过 tim.h 声明)
// extern TIM_HandleTypeDef htim1; // 如果tim.h没有包含它，就需要这行，但通常CubeMX生成的tim.h会包含

/**
  * @brief 初始化电机控制模块
  * @note  此函数应在定时器和GPIO初始化之后调用。
  *        它会启动PWM通道。
  */
void Motor_Init(void) {
    // 启动所有四个 PWM 通道
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); } // Motor A AIN1 (Speed/PWM)
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); } // Motor A AIN2 (High/PWM)
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); } // Motor B BIN1 (Speed/PWM)
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); } // Motor B BIN2 (High/PWM)

    // 初始状态停止电机
    Car_Stop();
    // printf("Motor PWM channels started.\r\n");
}

/**
  * @brief 设置指定电机的速度和方向 (Slow Decay PWM - Two PWM Inputs)
  * @param motor_idx: 电机索引 (MOTOR_A 或 MOTOR_B)
  * @param speed_percent: 速度百分比 (-100 到 100)。
  *                       正值表示前进，负值表示后退，0表示停止。
  */
void Motor_SetSpeed(uint8_t motor_idx, int16_t speed_percent) {
    uint32_t pwm_val_for_variable_pin; // 用于可变PWM信号的占空比 (0 = 最快，PWM_PERIOD = 最慢/制动)
    uint32_t pwm_val_for_fixed_pin;    // 用于固定高电平的占空比 (通常是 PWM_PERIOD，表示100%占空比)
    int16_t abs_speed_percent;

    // 限制速度百分比在 -100 到 100 之间
    if (speed_percent > 100) speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    // 获取速度百分比的绝对值用于计算 PWM 幅度
    if (speed_percent >= 0) {
        abs_speed_percent = speed_percent;
    } else {
        abs_speed_percent = -speed_percent; // abs(speed_percent)
    }

    // 根据速度百分比设置 PWM 值和固定高电平值
    if (speed_percent == 0) {
        // 停止 (滑行模式): 两个控制引脚都设为低电平 (0%占空比)
        pwm_val_for_variable_pin = 0;
        pwm_val_for_fixed_pin = 0;
    } else {
        // 对于慢衰减模式，我们需要的PWM占空比与期望速度成反比。
        // 例如：100% 速度对应 0% PWM 占空比 (0值)
        //       10% 速度对应 90% PWM 占空比 (0.9 * PWM_PERIOD 值)
        pwm_val_for_variable_pin = (uint32_t)((float)(100 - abs_speed_percent) * PWM_PERIOD / 100.0f);

        // 确保即使是非常小的非零速度百分比，也能产生实际的驱动。
        // 如果计算出的 pwm_val_for_variable_pin 等于 PWM_PERIOD
        // 而 abs_speed_percent 却大于 0，说明由于浮点精度或PWM_PERIOD值小，
        // 导致计算结果实际是100%占空比，这会导致电机停止（制动效果）。
        // 此时我们将其设置为 PWM_PERIOD - 1，以确保有微小的非零驱动。
        if (abs_speed_percent > 0 && pwm_val_for_variable_pin >= PWM_PERIOD) {
             pwm_val_for_variable_pin = PWM_PERIOD - 1; // 确保不完全制动
        }

        // 固定高电平的引脚，始终保持100%占空比
        pwm_val_for_fixed_pin = PWM_PERIOD;
    }

    // 根据电机索引和方向设置对应的 PWM 通道
    if (motor_idx == MOTOR_A) {
        // Motor A: AIN1 (PA8, TIM1_CH1), AIN2 (PA10, TIM1_CH3)
        if (speed_percent > 0) { // 前进 (Slow Decay Forward: AIN1=High, AIN2=PWM)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val_for_fixed_pin);     // AIN1 = 固定高电平
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_val_for_variable_pin);  // AIN2 = 可变PWM
        } else if (speed_percent < 0) { // 后退 (Slow Decay Reverse: AIN1=PWM, AIN2=High)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val_for_variable_pin);  // AIN1 = 可变PWM
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_val_for_fixed_pin);     // AIN2 = 固定高电平
        } else { // 停止 (speed_percent == 0) - 已在上面统一处理
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val_for_fixed_pin); // AIN1 = 0 (实际上是0，因为fixed_pwm_value和variable_pwm_value都为0)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_val_for_variable_pin); // AIN2 = 0
        }

    } else if (motor_idx == MOTOR_B) {
        // Motor B: BIN1 (PA11, TIM1_CH4), BIN2 (PA9, TIM1_CH2)
        if (speed_percent > 0) { // 前进 (Slow Decay Forward: BIN1=High, BIN2=PWM)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val_for_fixed_pin);     // BIN1 = 固定高电平
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_val_for_variable_pin);  // BIN2 = 可变PWM
        } else if (speed_percent < 0) { // 后退 (Slow Decay Reverse: BIN1=PWM, BIN2=High)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val_for_variable_pin);  // BIN1 = 可变PWM
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_val_for_fixed_pin);     // BIN2 = 固定高电平
        } else { // 停止 (speed_percent == 0) - 已在上面统一处理
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val_for_fixed_pin); // BIN1 = 0
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_val_for_variable_pin); // BIN2 = 0
        }
    }
}

/**
  * @brief 停止指定电机
  * @param motor_idx: 电机索引 (MOTOR_A 或 MOTOR_B)
  */
void Motor_Stop(uint8_t motor_idx) {
    Motor_SetSpeed(motor_idx, 0); // 设置速度为0即可停止
}

/**
  * @brief 控制小车整体移动
  * @param speed_A_percent: 电机A的速度百分比 (-100 到 100)
  * @param speed_B_percent: 电机B的速度百分比 (-100 到 100)
  */
void Car_Move(int16_t speed_A_percent, int16_t speed_B_percent) {
    Motor_SetSpeed(MOTOR_A, speed_A_percent);
    Motor_SetSpeed(MOTOR_B, speed_B_percent);
}

/**
  * @brief 小车前进
  * @param speed_percent: 速度百分比 (0 到 100)
  */
void Car_TurnLeft(uint8_t speed_percent) {
	Car_Move(-(int16_t)speed_percent, speed_percent);
}

/**
  * @brief 小车后退
  * @param speed_percent: 速度百分比 (0 到 100)
  */
void Car_TurnRight(uint8_t speed_percent) {
    // 注意速度参数是 int16_t，所以可以直接传递负值
	Car_Move(speed_percent, -(int16_t)speed_percent);
}

/**
  * @brief 小车左转 (差速原地转弯)
  * @param speed_percent: 速度百分比 (0 到 100)
  * @note  电机A后退，电机B前进
  */
void Car_Forward(uint8_t speed_percent) {
    Car_Move(speed_percent, speed_percent);
}

/**
  * @brief 小车右转 (差速原地转弯)
  * @param speed_percent: 速度百分比 (0 到 100)
  * @note  电机A前进，电机B后退
  */
void Car_Backward(uint8_t speed_percent) {
    Car_Move(-(int16_t) speed_percent, -(int16_t) speed_percent);
}

/**
  * @brief 小车停止
  */
void Car_Stop(void) {
    Car_Move(0, 0);
}


