#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"
#define R_Banh_Xe 0.035f
#define ENCODER_PPR 1000.0f
#define SAMPLE_TIME_10MS 0.01f

typedef struct {
    TIM_HandleTypeDef *htim;
    volatile int16_t delta_pulses;
    volatile int32_t total_pulses;
    float speed_rpm;//van toc vong/phut
    float speed_mps;// van toc m/s
    float distance_m;// khoang duong di duoc
} Encoder_HandleTypeDef;
void Encoder_Init(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim);
int16_t Encoder_Read_Raw(Encoder_HandleTypeDef *encoder);//ham doc xung tho, reset thanh ghi CNT ve 0, va cong vao bien tong
void Encoder_Update(Encoder_HandleTypeDef *k_enc);//ham cap nhat van toc va quang duong di duoc, goi moi 10ms trong ham callback ngat timer
#endif /* __ENCODER_H */