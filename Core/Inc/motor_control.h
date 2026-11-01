#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H
#include "main.h"

// duty > 0: tien, duty < 0: lui (chieu lay theo code test: trai in1=1/in2=0, phai in1=0/in2=1)
void Motor_SetLeft(int16_t duty,TIM_HandleTypeDef *htim, uint32_t channel);

void Motor_SetRight(int16_t duty,TIM_HandleTypeDef *htim, uint32_t channel);
#endif