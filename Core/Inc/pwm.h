#ifndef PWM_H
#define PWM_H
#include "main.h"
void set_duty_cycle(uint16_t duty_cycle, TIM_HandleTypeDef *htim,int channel);
#endif /* PWM_H */