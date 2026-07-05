#include "stdint.h"
#include "pwm.h"
void set_duty_cycle(uint16_t duty_cycle, TIM_HandleTypeDef *htim,int channel) {
    if (duty_cycle > 1000) {
        duty_cycle = 1000;
    }
    switch(channel) {
        case TIM_CHANNEL_1:
            htim->Instance->CCR1 = duty_cycle;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCR2 = duty_cycle;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCR3 = duty_cycle;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCR4 = duty_cycle;
            break;
    }
}