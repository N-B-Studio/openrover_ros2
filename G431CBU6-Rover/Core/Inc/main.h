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
#include "stm32g4xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define PWM_L_Pin GPIO_PIN_0
#define PWM_L_GPIO_Port GPIOA
#define PWM_R_Pin GPIO_PIN_1
#define PWM_R_GPIO_Port GPIOA
#define DIR_L_Pin GPIO_PIN_2
#define DIR_L_GPIO_Port GPIOA
#define DIR_R_Pin GPIO_PIN_3
#define DIR_R_GPIO_Port GPIOA
#define FG_FL_Pin GPIO_PIN_4
#define FG_FL_GPIO_Port GPIOA
#define FG_FL_EXTI_IRQn EXTI4_IRQn
#define FG_RL_Pin GPIO_PIN_6
#define FG_RL_GPIO_Port GPIOA
#define FG_RL_EXTI_IRQn EXTI9_5_IRQn
#define FG_RR_Pin GPIO_PIN_0
#define FG_RR_GPIO_Port GPIOB
#define FG_RR_EXTI_IRQn EXTI0_IRQn
#define FG_FR_Pin GPIO_PIN_1
#define FG_FR_GPIO_Port GPIOB
#define FG_FR_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
