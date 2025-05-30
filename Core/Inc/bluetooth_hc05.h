// bluetooth_hc05.h

#ifndef __BLUETOOTH_HC05_H
#define __BLUETOOTH_HC05_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h" // 包含HAL库头文件 (根据你的MCU系列选择，例如 stm32f1xx_hal.h, stm32f4xx_hal.h 等)
#include <stdbool.h>       // 用于 bool 类型
#include <stdint.h>        // 用于 uint8_t 等类型
#include <string.h>        // 用于 memset, memcpy 等

// 假设你在 CubeMX 中配置 USART6 用于蓝牙，并且其句柄名为 huart6
// 如果你的句柄名称不同，请修改此处
extern UART_HandleTypeDef huart6;

#define BLUETOOTH_UART_HANDLE (&huart6) // 方便切换UART
#define BLUETOOTH_USART_IRQn USART6_IRQn // USART6 中断号

// 蓝牙接收缓冲区大小
#define BLUETOOTH_RX_BUFFER_SIZE 128 // 可以根据需要调整

// 函数声明

/**
 * @brief 初始化HC-05蓝牙模块接口 (主要是启动UART接收中断)
 * @retval HAL_StatusTypeDef HAL_OK表示成功, 其他表示失败
 */
HAL_StatusTypeDef hc05_init(void);

/**
 * @brief 检查是否有新的蓝牙数据接收完成
 * @retval bool true 表示有完整数据帧, false 表示没有
 */
bool hc05_is_data_received(void);

/**
 * @brief 获取接收到的蓝牙数据
 * @param buffer 指向用于存储数据的缓冲区的指针
 * @param buffer_len 缓冲区的最大长度
 * @retval uint16_t 实际复制到buffer中的数据长度 (不包括末尾的'\0')
 * 如果返回0，表示没有新数据或获取失败
 * @note 调用此函数后，内部的接收完成标志会被清除，数据缓冲区也会被清空，并重新启动中断接收。
 */
uint16_t hc05_get_received_data(uint8_t *buffer, uint16_t buffer_len);

/**
 * @brief 通过蓝牙发送数据
 * @param data 指向要发送数据的缓冲区的指针
 * @param len 要发送的数据长度
 * @retval HAL_StatusTypeDef HAL_OK表示成功, 其他表示失败
 */
HAL_StatusTypeDef hc05_transmit_data(const uint8_t *data, uint16_t len);

/**
 * @brief UART接收回调处理函数 (由HAL_UART_RxCpltCallback调用)
 * 此函数在 bluetooth_hc05.c 中实现。
 * @param huart 发生中断的UART句柄指针
 */
void hc05_uart_rx_callback_handler(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_HC05_H */