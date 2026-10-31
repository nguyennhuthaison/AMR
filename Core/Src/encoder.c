#include "stdint.h"
#include "encoder.h"
void Encoder_Init(Encoder_HandleTypeDef *k_enc, TIM_HandleTypeDef *htim) 
{
    k_enc->htim = htim;
    k_enc->delta_pulses = 0;
    k_enc->total_pulses = 0;
    k_enc->speed_rpm = 0.0f;
    k_enc->speed_mps = 0.0f;
    k_enc->distance_m = 0.0f;
    // Khởi động chế độ Encoder của Timer phần cứng ngay tại đây
    HAL_TIM_Encoder_Start(k_enc->htim, TIM_CHANNEL_ALL);
}

int16_t Encoder_Read_Raw(Encoder_HandleTypeDef *k_enc) 
{
    // 1. Đọc giá trị tức thời từ thanh ghi CNT của Timer được liên kết trong struct
    k_enc->delta_pulses = (int16_t)__HAL_TIM_GET_COUNTER(k_enc->htim);
    
    // 2. Reset thanh ghi CNT của Timer đó về 0
    __HAL_TIM_SET_COUNTER(k_enc->htim, 0);
    
    // 3. Cộng dồn vào biến tổng tích lũy của riêng Encoder đó
    k_enc->total_pulses += k_enc->delta_pulses;
    
    // 4. Trả về giá trị xung chu kỳ này
    return k_enc->delta_pulses;
}
int16_t Encoder_Nguoc_Read_Raw(Encoder_HandleTypeDef *k_enc)
{
    // 1. Đọc giá trị tức thời từ thanh ghi CNT của Timer được liên kết trong struct
    k_enc->delta_pulses = (int16_t)__HAL_TIM_GET_COUNTER(k_enc->htim);
    
    // 2. Reset thanh ghi CNT của Timer đó về 0
    __HAL_TIM_SET_COUNTER(k_enc->htim, 0);
    
    // 3. Cộng dồn vào biến tổng tích lũy của riêng Encoder đó
    k_enc->total_pulses -= k_enc->delta_pulses;
    
    // 4. Trả về giá trị xung chu kỳ này
    return k_enc->delta_pulses;
}
void Encoder_Update(Encoder_HandleTypeDef *k_enc)
{
    
    int16_t pulses = Encoder_Read_Raw(k_enc);
    float vong_tren_giay = ((float)pulses / ENCODER_PPR) / SAMPLE_TIME_10MS;
    float chu_vi = 2.0f * PI * WHEEL_RADIUS;
    k_enc->speed_rpm = vong_tren_giay * 60.0f;
    k_enc->speed_mps = vong_tren_giay * chu_vi;
    k_enc->distance_m = ((float)k_enc->total_pulses / ENCODER_PPR) * chu_vi;
}
