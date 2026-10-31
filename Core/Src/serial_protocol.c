#include "serial_protocol.h"
#include <stdlib.h>

#define DMA_RX_BUF_SIZE 64

typedef enum {
    STATE_WAIT_RIGHT_HEADER,  // Chờ ký tự 'r' bắt đầu frame
    STATE_READ_RIGHT_VAL,     // Đọc chuỗi vận tốc bánh phải
    STATE_CHECK_LEFT_HEADER,  // Chờ ký tự 'l' bắt đầu bánh trái
    STATE_READ_LEFT_VAL       // Đọc chuỗi vận tốc bánh trái
} RxState_t;

typedef struct {
    float right_wheel_target_vel;
    float left_wheel_target_vel;
    volatile bool new_command_available;
} RobotCommand_t;

// Biến quản lý phần cứng và DMA cục bộ
static UART_HandleTypeDef *protocol_uart = NULL;
static uint8_t dma_rx_buffer[DMA_RX_BUF_SIZE];
static uint16_t tail_idx = 0; // Con trỏ đọc của CPU

// Biến quản lý trạng thái FSM
static RxState_t current_state = STATE_WAIT_RIGHT_HEADER;
static char parse_buffer[16];
static uint8_t parse_buf_idx = 0;
static RobotCommand_t cmd_data = {0.0f, 0.0f, false};

/**
 * @brief Tách chuỗi định dạng "pXX.XX" hoặc "nXX.XX" thành số thực float
 */
static float parse_speed_string(const char *str) {
    if (str[0] == '\0') return 0.0f;
    float sign = (str[0] == 'p') ? 1.0f : -1.0f;
    float val = (float)atof(&str[1]);
    return sign * val;
}

/**
 * @brief Máy trạng thái giải mã từng ký tự
 */
static void SerialProtocol_ParseByte(char c) {
    switch (current_state) {
        case STATE_WAIT_RIGHT_HEADER:
            if (c == 'r') {
                parse_buf_idx = 0;
                current_state = STATE_READ_RIGHT_VAL;
            }
            break;

        case STATE_READ_RIGHT_VAL:
            if (c == ',') {
                parse_buffer[parse_buf_idx] = '\0';
                cmd_data.right_wheel_target_vel = parse_speed_string(parse_buffer);
                current_state = STATE_CHECK_LEFT_HEADER;
            } 
            else if (parse_buf_idx < sizeof(parse_buffer) - 1) {
                parse_buffer[parse_buf_idx++] = c;
            } 
            else {
                current_state = STATE_WAIT_RIGHT_HEADER; // Tránh tràn bộ đệm
            }
            break;

        case STATE_CHECK_LEFT_HEADER:
            if (c == 'l') {
                parse_buf_idx = 0;
                current_state = STATE_READ_LEFT_VAL;
            } 
            else {
                // Tự phục hồi trạng thái nếu bị lệch frame mà gặp lại ký tự 'r'
                current_state = (c == 'r') ? STATE_READ_RIGHT_VAL : STATE_WAIT_RIGHT_HEADER;
            }
            break;

        case STATE_READ_LEFT_VAL:
            if (c == ',') {
                parse_buffer[parse_buf_idx] = '\0';
                cmd_data.left_wheel_target_vel = parse_speed_string(parse_buffer);
                cmd_data.new_command_available = true; // Báo cờ frame hoàn tất
                current_state = STATE_WAIT_RIGHT_HEADER;
            } 
            else if (parse_buf_idx < sizeof(parse_buffer) - 1) {
                parse_buffer[parse_buf_idx++] = c;
            } 
            else {
                current_state = STATE_WAIT_RIGHT_HEADER;
            }
            break;
    }
}

void SerialProtocol_Init(UART_HandleTypeDef *huart) {
    protocol_uart = huart;
    current_state = STATE_WAIT_RIGHT_HEADER;
    parse_buf_idx = 0;
    tail_idx = 0;
    cmd_data.new_command_available = false;

    // 1. Kích hoạt DMA nhận ở chế độ Circular
    HAL_UART_Receive_DMA(protocol_uart, dma_rx_buffer, DMA_RX_BUF_SIZE);

    // 2. Bật ngắt IDLE trên phần cứng UART
    __HAL_UART_ENABLE_IT(protocol_uart, UART_IT_IDLE);
}

void SerialProtocol_ProcessRxData(void) {
    if (protocol_uart == NULL) return;

    // Tính vị trí byte mới nhất mà DMA vừa ghi vào bộ đệm
    uint16_t head_idx = DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(protocol_uart->hdmarx);

    // Quét toàn bộ byte mới đưa vào FSM giải mã
    while (tail_idx != head_idx) {
        SerialProtocol_ParseByte((char)dma_rx_buffer[tail_idx]);
        tail_idx++;
        if (tail_idx >= DMA_RX_BUF_SIZE) {
            tail_idx = 0;
        }
    }
}

bool SerialProtocol_GetLatestCommand(float *right_vel, float *left_vel) {
    if (cmd_data.new_command_available) {
        if (right_vel != NULL) *right_vel = cmd_data.right_wheel_target_vel;
        if (left_vel != NULL)  *left_vel  = cmd_data.left_wheel_target_vel;
        cmd_data.new_command_available = false; // Xóa cờ báo
        return true;
    }
    return false;
}