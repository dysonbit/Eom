//
// Created by Guan on 5/11/2025.
//

#ifndef EOM_MPU6500_2_BLUETOOTH_HC05_AT_H
#define EOM_MPU6500_2_BLUETOOTH_HC05_AT_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

// 定义EN/KEY引脚 (根据你的CubeMX配置修改)
#define HC05_EN_PORT GPIOB
#define HC05_EN_PIN  GPIO_PIN_0

// 定义AT指令模式下使用的UART句柄 (应与数据模式下相同，但波特率可能不同)
extern UART_HandleTypeDef huart1; // 假设使用USART1
#define HC05_AT_UART_HANDLE (&huart1)

// 函数声明

/**
 * @brief 使HC-05模块进入AT指令模式 (方式一：上电前拉高EN)
 * @note  此函数会先拉高EN引脚，然后提示用户给HC-05重新上电。
 *        之后需要将UART波特率设置为38400 bps。
 */
void hc05_enter_at_mode_hardware_reset(void);

/**
 * @brief 退出AT指令模式 (通过拉低EN引脚并提示重新上电)
 */
void hc05_exit_at_mode_hardware_reset(void);


/**
 * @brief 发送AT指令并等待响应
 * @param cmd 要发送的AT指令字符串 (例如 "AT\r\n", "AT+VERSION?\r\n")
 * @param response_buffer 用于存储模块响应的缓冲区
 * @param buffer_size response_buffer 的大小
 * @param timeout_ms 等待响应的超时时间 (毫秒)
 * @retval bool true 表示成功接收到响应 (不一定是"OK"), false 表示超时或错误
 */
bool hc05_send_at_command(const char* cmd, char* response_buffer, uint16_t buffer_size, uint32_t timeout_ms);

/**
 * @brief 重新配置UART波特率 (用于在AT模式和数据模式间切换)
 * @param baudrate 新的波特率
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef hc05_reconfigure_uart_baudrate(uint32_t baudrate);

#ifdef __cplusplus
}
#endif


#endif //EOM_MPU6500_2_BLUETOOTH_HC05_AT_H
