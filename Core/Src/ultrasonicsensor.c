#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include "ultrasonicsensor.h"

void ultrasonic_get_cm(float *out)
{
    printf("🧩 ultrasonic_get_cm() RUNNING from %s\n", __FILE__);
    GPIO_PinState echo = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
    printf("🔍 ECHO current state = %d\r\n", echo);

    uint32_t echo_time = 0;
    float distance = 0.0f;

    // 发出 TRIG 脉冲
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    printf("TRIG pulse sent\n");

    // 启动定时器
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);

    // 等待 ECHO 高电平开始
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) {
        if (__HAL_TIM_GET_COUNTER(&htim3) > 60000) {
            printf("Timeout waiting for ECHO HIGH\n");
            HAL_TIM_Base_Stop(&htim3);
            *out = 0.0f;
            return;
        }
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 等待 ECHO 低电平结束
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(&htim3) > 60000) {
            printf("Timeout waiting for ECHO LOW\n");
            HAL_TIM_Base_Stop(&htim3);
            *out = 0.0f;
            return;
        }
    }

    echo_time = __HAL_TIM_GET_COUNTER(&htim3);
    HAL_TIM_Base_Stop(&htim3);

    // 🧠 安全范围检查：过滤错误脉冲
    if (echo_time < 100) {
        printf("⚠️ Echo too short (<100us), invalid reading\n");
        *out = 0.0f;
        return;
    } else if (echo_time > 58000) {
        printf("⚠️ Echo too long (>58ms), out of range\n");
        *out = 0.0f;
        return;
    }

    distance = echo_time * 0.0343f / 2.0f;
    printf("✅ Echo time: %lu us, Distance: %.2f cm\n", echo_time, distance);
    *out = distance;
}
