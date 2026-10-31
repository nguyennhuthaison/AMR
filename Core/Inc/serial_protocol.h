#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief Khởi tạo buffer DMA Circular và bật ngắt IDLE trên cổng UART
 * @param huart Con trỏ tới cấu trúc phần cứng UART (ví dụ: &huart2)
 */
void SerialProtocol_Init(UART_HandleTypeDef *huart);

/**
 * @brief Quét và nạp dữ liệu từ buffer DMA vào máy trạng thái (FSM)
 * @note Hàm này được gọi trực tiếp bên trong USARTx_IRQHandler khi có ngắt IDLE
 */
void SerialProtocol_ProcessRxData(void);

/**
 * @brief Lấy giá trị vận tốc mục tiêu mới nhất từ frame hợp lệ
 * @param[out] right_vel Con trỏ nhận giá trị vận tốc bánh phải
 * @param[out] left_vel Con trỏ nhận giá trị vận tốc bánh trái
 * @return true nếu có frame dữ liệu mới hợp lệ, false nếu chưa có
 */
bool SerialProtocol_GetLatestCommand(float *right_vel, float *left_vel);

#endif /* SERIAL_PROTOCOL_H */