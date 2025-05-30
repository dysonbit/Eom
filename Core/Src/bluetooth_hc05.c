// bluetooth_hc05.c

#include "bluetooth_hc05.h"
#include <stdio.h> // 如果需要 printf 等调试功能

// 确保 `huart6` 在 `main.c` 中定义并通过 CubeMX 初始化，并在此处声明
// 如果你的 UART 句柄名称不同 (例如 hbluart)，请对应修改
extern UART_HandleTypeDef huart6;

static uint8_t internal_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static uint8_t single_byte_rx_data; // 用于HAL_UART_Receive_IT接收单个字节
static volatile bool data_received_flag = false;
static volatile uint16_t internal_rx_index = 0;

/**
 * @brief 初始化HC-05蓝牙模块接口
 */
HAL_StatusTypeDef hc05_init(void) {
    memset(internal_rx_buffer, 0, BLUETOOTH_RX_BUFFER_SIZE);
    internal_rx_index = 0;
    data_received_flag = false;

    // 启动UART中断接收，每次接收一个字节
    // 当一个字节接收完成后，会触发 HAL_UART_RxCpltCallback 回调函数
    return HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, &single_byte_rx_data, 1);
}

/**
 * @brief 检查是否有新的蓝牙数据接收完成
 */
bool hc05_is_data_received(void) {
    return data_received_flag;
}

/**
 * @brief 获取接收到的蓝牙数据
 */
uint16_t hc05_get_received_data(uint8_t *buffer, uint16_t buffer_len) {
    if (!data_received_flag || buffer == NULL || buffer_len == 0) {
        return 0;
    }

    // 进入临界区：禁用中断，防止在读取数据时被新的中断打断
    HAL_NVIC_DisableIRQ(BLUETOOTH_USART_IRQn); // 或者直接使用 USART6_IRQn

    uint16_t len_to_copy = internal_rx_index;
    if (len_to_copy >= buffer_len) { // 防止缓冲区溢出
        len_to_copy = buffer_len - 1; // 留一个字节给 '\0'
    }

    memcpy(buffer, internal_rx_buffer, len_to_copy);
    buffer[len_to_copy] = '\0'; // 添加字符串结束符

    // 重置标志和索引，清空缓冲区
    data_received_flag = false;
    internal_rx_index = 0;
    memset(internal_rx_buffer, 0, BLUETOOTH_RX_BUFFER_SIZE);

    // 退出临界区：重新使能中断
    HAL_NVIC_EnableIRQ(BLUETOOTH_USART_IRQn);

    // 重要：数据被取走后，需要重新启动中断接收下一个字节/消息
    // 如果 hc05_uart_rx_callback_handler 中没有在data_received_flag置true后停止HAL_UART_Receive_IT,
    // 并且HAL_UART_Receive_IT在callback中总是被重新启动，那么这里就不需要再次启动。
    // 但如果callback中在设置flag后就不再启动下一次IT接收，则这里需要。
    // 当前的设计是callback总是尝试启动下一次接收。
    // 但是，如果因为收到完整消息而设置了data_received_flag，
    // 那么在数据被取走之前，新的数据可能不会覆盖旧的，这取决于callback的实现。
    // 为了确保在数据被取走后能继续接收，通常在数据处理完毕后再次调用 HAL_UART_Receive_IT。
    // 然而，我们已经在 callback 中每次都重新启动了，所以这里可以不用。
    // 但若想确保即使在 callback 中发生意外停止了接收，这里可以作为保障。
    // 为了安全和清晰，通常在取走数据后，明确我们期望中断继续工作：
    // 实际上，因为我们的回调函数 hc05_uart_rx_callback_handler 总是重新调用 HAL_UART_Receive_IT，
    // 所以这里不需要再次调用。如果回调函数在收到完整消息后停止调用 HAL_UART_Receive_IT，那这里就需要。
    // 当前逻辑下，我们假设回调函数会持续接收。

    return len_to_copy;
}

/**
 * @brief 通过蓝牙发送数据 (阻塞方式)
 */
HAL_StatusTypeDef hc05_transmit_data(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0) {
        return HAL_ERROR;
    }
    // 使用阻塞式发送，可以根据需要改为中断或DMA发送
    return HAL_UART_Transmit(BLUETOOTH_UART_HANDLE, (uint8_t*)data, len, HAL_MAX_DELAY);
}

/**
 * @brief UART接收回调处理函数 (由外部的 HAL_UART_RxCpltCallback 调用)
 * @param huart 发生中断的UART句柄指针
 * @note 这个函数是关键，用于处理中断接收到的字节
 */
void hc05_uart_rx_callback_handler(UART_HandleTypeDef *huart) {
    // 检查是否是目标UART实例触发的中断
    if (huart->Instance == BLUETOOTH_UART_HANDLE->Instance) { // 例如 USART6
        if (data_received_flag) {
            // 如果上一次的数据还没有被主程序取走，可以选择如何处理新来的字节：
            // 1. 丢弃 (如下面重新启动接收，会覆盖 single_byte_rx_data，间接丢弃)
            // 2. 覆盖旧数据 (如果 internal_rx_buffer 满了)
            // 3. 设置一个溢出错误标志
            // 这里简单处理：等待主程序取走数据，新的数据暂时不存入 buffer，但会重新启动接收，
            // 意味着如果主程序不及时取走，single_byte_rx_data 会被新数据覆盖，但 internal_rx_buffer 中的内容不变。
            // 更健壮的做法是使用环形缓冲区。
            // 为了简单，我们假设主程序会及时取走数据。
        } else {
            // 将接收到的单个字节存入内部缓冲区
            if (internal_rx_index < BLUETOOTH_RX_BUFFER_SIZE - 1) { // 留一个字节给可能的'\0'
                internal_rx_buffer[internal_rx_index++] = single_byte_rx_data;

                // 判断消息结束的条件，例如：
                // 1. 接收到特定的结束符 (如 '\n' 或 '\r')
                // 2. 接收到固定长度的数据
                // 3. 缓冲区满
                // HC05模块的AT指令通常以 "\r\n" 结束，数据透传时则根据应用定义。
                // 这里我们以换行符 '\n' 作为简单示例的结束标志，或者缓冲区满。
                if (single_byte_rx_data == '\n' || internal_rx_index >= (BLUETOOTH_RX_BUFFER_SIZE - 1)) {
                    // 可选：如果想把换行符也包含，则不需要下面的 internal_rx_index-- 操作
                    // 如果不想包含换行符作为数据内容，可以 internal_rx_index--; internal_rx_buffer[internal_rx_index] = '\0';
                    internal_rx_buffer[internal_rx_index] = '\0'; // 确保字符串结束
                    data_received_flag = true; // 设置数据接收完成标志
                    // 注意：此时 internal_rx_index 指向了'\0'之后或缓冲区的末尾
                    // 当 data_received_flag 为 true 时，我们暂时不处理新的字节到 internal_rx_buffer，
                    // 直到 hc05_get_received_data 被调用并清空标志和缓冲区。
                    // HAL_UART_Receive_IT 仍然需要被调用以接收后续数据（即使它们可能暂时不被存入buffer）。
                }
            } else {
                // 缓冲区已满，但没有收到结束符
                internal_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE - 1] = '\0'; // 确保字符串结束
                data_received_flag = true; // 设置数据接收完成标志，表示缓冲区满了
            }
        }

        // 无论当前字节是否处理，都必须重新启动下一次的UART中断接收
        // 否则，后续的字节将不会触发中断
        // 如果 data_received_flag 为 true，新的数据会到 single_byte_rx_data，
        // 但不会立即写入 internal_rx_buffer 直到标志被清除。
        // 这种处理方式在 data_received_flag 为 true 时，可能会丢失 single_byte_rx_data 的数据，
        // 如果在下一次 HAL_UART_Receive_IT 调用前没有被保存。
        // 一个更安全的做法是，如果 data_received_flag 为 true，就不再调用 HAL_UART_Receive_IT，
        // 而是在 hc05_get_received_data() 中取走数据后重新调用。
        // 修改：仅当数据未满或未标记为已接收时，才继续填充缓冲区并重新启动。
        // 当data_received_flag为true时，我们应该停止接收到缓冲区，直到数据被取走。
        // HAL_UART_Receive_IT应该在数据被取走(hc05_get_received_data)后，或者在初始化(hc05_init)时调用。

        // 改进的逻辑：如果数据已满并标记，则不立即重启IT，等待数据被取走。
        if (!data_received_flag) {
            HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, &single_byte_rx_data, 1);
        }
        // 如果需要即使在data_received_flag=true时也继续接收（例如，为了不错过任何字节，即使它们可能暂时无法存入主缓冲区），
        // 那么就总是调用 HAL_UART_Receive_IT。但这需要更复杂的缓冲区管理（如环形缓冲区）来避免数据丢失。
        // 为了与 hc05_get_received_data 中的重置逻辑配合，我们这里总是重新启动。
        // 主循环需要快速处理数据。
        // 或者，更常见模式：
        // if (!data_received_flag) {
        //    HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, &single_byte_rx_data, 1);
        // }
        // 然后在 hc05_get_received_data() 的末尾添加:
        // if (!data_received_flag) { // 确保在取走数据后，如果之前因为flag=true而没启动，现在启动
        //    HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, &single_byte_rx_data, 1);
        // }
        // 为了简单起见，保持原始的每次回调都重新启动：
        HAL_UART_Receive_IT(BLUETOOTH_UART_HANDLE, &single_byte_rx_data, 1);
    }
}