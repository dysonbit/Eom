//
// Created by Guan on 5/11/2025.
//

#include "bluetooth_hc05_at.h"
#include <stdio.h> // 用于 printf 调试

// 假设 huart1 在 main.c 中定义并由 CubeMX 初始化
// extern UART_HandleTypeDef huart1; // 如果没有在.h中extern，则在此处extern

/**
 * @brief 重新配置UART波特率
 */
HAL_StatusTypeDef hc05_reconfigure_uart_baudrate(uint32_t baudrate) {
//    HAL_UART_DeInit(HC05_AT_UART_HANDLE); // 先反初始化
//    HC05_AT_UART_HANDLE->Init.BaudRate = baudrate;
//    return HAL_UART_Init(HC05_AT_UART_HANDLE);
}

/**
 * @brief 使HC-05模块进入AT指令模式 (方式一)
 */
void hc05_enter_at_mode_hardware_reset(void) {
    printf("Preparing to enter AT mode...\r\n");
    // 1. 拉高EN/KEY引脚
    HAL_GPIO_WritePin(HC05_EN_PORT, HC05_EN_PIN, GPIO_PIN_SET);
    printf("EN/KEY pin set HIGH.\r\n");
    printf("Please POWER OFF and then POWER ON the HC-05 module NOW.\r\n");
    printf("Waiting for HC-05 to restart in AT mode (approx 5 seconds delay)...\r\n");
    HAL_Delay(5000); // 给用户时间重新上电模块，并等待模块启动

    // 2. 配置STM32的UART波特率为38400 (AT模式固定波特率)
    if (hc05_reconfigure_uart_baudrate(38400) != HAL_OK) {
        printf("Failed to reconfigure UART to 38400 bps for AT mode.\r\n");
        // 可以选择在此处进行错误处理
    } else {
        printf("UART reconfigured to 38400 bps for AT mode.\r\n");
    }
    printf("HC-05 should now be in AT command mode (38400 bps).\r\n");
    printf("Send 'AT\\r\\n' to test. Expect 'OK\\r\\n'.\r\n");
}

/**
 * @brief 退出AT指令模式
 */
void hc05_exit_at_mode_hardware_reset(void) {
    // 拉低EN/KEY引脚
    HAL_GPIO_WritePin(HC05_EN_PORT, HC05_EN_PIN, GPIO_PIN_RESET);
    printf("EN/KEY pin set LOW.\r\n");
    printf("Please POWER OFF and then POWER ON the HC-05 module to return to data mode.\r\n");
    printf("Remember to reconfigure UART to data mode baud rate (e.g., 9600 bps).\r\n");
}


/**
 * @brief 发送AT指令并等待响应
 */
bool hc05_send_at_command(const char* cmd, char* response_buffer, uint16_t buffer_size, uint32_t timeout_ms) {
    if (cmd == NULL || response_buffer == NULL || buffer_size == 0) {
        return false;
    }

    memset(response_buffer, 0, buffer_size); // 清空响应缓冲区
    printf("Sending AT command: %s", cmd); // 打印发送的指令 (AT指令通常包含\r\n)

    // 发送指令
    if (HAL_UART_Transmit(HC05_AT_UART_HANDLE, (uint8_t*)cmd, strlen(cmd), timeout_ms) != HAL_OK) {
        printf("AT command transmit failed.\r\n");
        return false;
    }

    // 等待并接收响应
    // 注意：这里的接收方式比较简单，阻塞式接收，且可能无法完整接收所有响应
    // 更健壮的方式是使用中断接收，并设置一个超时来判断响应结束
    // HAL_UART_Receive 对于未知长度的响应不是最佳选择
    // 我们可以尝试分多次小块接收，或者期望一个特定的结束符

    uint32_t start_time = HAL_GetTick();
    uint16_t received_len = 0;
    char temp_char;

    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (HAL_UART_Receive(HC05_AT_UART_HANDLE, (uint8_t*)&temp_char, 1, 10) == HAL_OK) { // 短超时尝试接收1字节
            if (received_len < buffer_size - 1) {
                response_buffer[received_len++] = temp_char;
                // 简单的结束判断：如果收到 \n 并且之前有 \r，或者缓冲区快满了
                if (temp_char == '\n' && received_len > 1 && response_buffer[received_len-2] == '\r') {
                    response_buffer[received_len] = '\0'; // 确保字符串结束
                    printf("AT Response: %s", response_buffer);
                    return true;
                }
            } else { // 缓冲区满
                response_buffer[buffer_size - 1] = '\0';
                printf("AT Response (buffer full): %s", response_buffer);
                return true; // 返回已接收的部分
            }
        }
        // 如果没有立即收到数据，继续循环直到超时
    }

    if (received_len > 0) { // 超时，但收到了一些数据
        response_buffer[received_len] = '\0';
        printf("AT Response (timeout, partial): %s", response_buffer);
        return true;
    }

    printf("AT command response timeout or no response.\r\n");
    return false;
}
