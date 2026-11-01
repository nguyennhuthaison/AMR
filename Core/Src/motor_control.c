#include "motor_control.h"
#include "stdint.h"
#include "main.h"
// duty > 0: tien, duty < 0: lui (chieu lay theo code test: trai in1=1/in2=0, phai in1=0/in2=1)
void Motor_SetLeft(int16_t duty,TIM_HandleTypeDef *htim, uint32_t channel)
{
  if (duty >= 0)
  {
    HAL_GPIO_WritePin(L_in1_GPIO_Port, L_in1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L_in2_GPIO_Port, L_in2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(L_in1_GPIO_Port, L_in1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L_in2_GPIO_Port, L_in2_Pin, GPIO_PIN_SET);
    duty = -duty;
  }
  if (duty> 1000) {
        duty = 1000;
    }
    switch(channel) {
        case TIM_CHANNEL_1:
            htim->Instance->CCR1 = duty;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCR2 = duty;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCR3 = duty;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCR4 = duty;
            break;
    }
}

void Motor_SetRight(int16_t duty,TIM_HandleTypeDef *htim, uint32_t channel)
{
  if (duty >= 0)
  {
    HAL_GPIO_WritePin(R_in1_GPIO_Port, R_in1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(R_in2_GPIO_Port, R_in2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(R_in1_GPIO_Port, R_in1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R_in2_GPIO_Port, R_in2_Pin, GPIO_PIN_RESET);
    duty = -duty;
  }
  if (duty> 1000) {
        duty = 1000;
    }
    switch(channel) {
        case TIM_CHANNEL_1:
            htim->Instance->CCR1 = duty;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCR2 = duty;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCR3 = duty;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCR4 = duty;
            break;
    }
}