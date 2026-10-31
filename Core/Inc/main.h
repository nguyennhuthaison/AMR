/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define STBY_Pin GPIO_PIN_14
#define STBY_GPIO_Port GPIOC
#define EnRA_Pin GPIO_PIN_0
#define EnRA_GPIO_Port GPIOA
#define EnRB_Pin GPIO_PIN_1
#define EnRB_GPIO_Port GPIOA
#define L_in1_Pin GPIO_PIN_12
#define L_in1_GPIO_Port GPIOB
#define L_in2_Pin GPIO_PIN_13
#define L_in2_GPIO_Port GPIOB
#define R_in1_Pin GPIO_PIN_14
#define R_in1_GPIO_Port GPIOB
#define R_in2_Pin GPIO_PIN_15
#define R_in2_GPIO_Port GPIOB
#define EnLA_Pin GPIO_PIN_8
#define EnLA_GPIO_Port GPIOA
#define EnLB_Pin GPIO_PIN_9
#define EnLB_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define PI 3.14159265358979323846f
#define WHEEL_BASE 0.15f // Khoảng cách giữa hai bánh xe (m)
#define DT 0.01f // Thời gian lấy mẫu (s)
#define WHEEL_RADIUS 0.03f // Bán kính bánh xe (m)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
