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
#define ENCODER_2_Pin GPIO_PIN_4
#define ENCODER_2_GPIO_Port GPIOA
#define ENCODER_1_Pin GPIO_PIN_6
#define ENCODER_1_GPIO_Port GPIOA
#define ENCODER_3_Pin GPIO_PIN_0
#define ENCODER_3_GPIO_Port GPIOB
#define ENCODER_4_Pin GPIO_PIN_1
#define ENCODER_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
