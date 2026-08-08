/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FG_PPR_FL  1121.1f
#define FG_PPR_RL  1123.5f
#define FG_PPR_FR  1173.4f
#define FG_PPR_RR  1174.1f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef enum
{
    WHEEL_FL = 0,
    WHEEL_RL,
    WHEEL_FR,
    WHEEL_RR,
    WHEEL_COUNT
} WheelIndex;

static volatile uint32_t fg_total[WHEEL_COUNT] = {0U};
static volatile uint32_t fg_window[WHEEL_COUNT] = {0U};

static volatile int8_t motor_direction[2] =
{
    0,  /* left */
    0   /* right */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static void Motor_ChangeDirectionSafely(
    int8_t left_direction,
    int8_t right_direction);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * 左右电机机械安装方向相反，因此“小车前进”时，
 * 两个DIR引脚通常需要输出相反电平。
 *
 * 如果实测方向不对，只改这四个定义。
 */
#define LEFT_FORWARD_LEVEL     GPIO_PIN_SET
#define LEFT_REVERSE_LEVEL     GPIO_PIN_RESET

#define RIGHT_FORWARD_LEVEL    GPIO_PIN_RESET
#define RIGHT_REVERSE_LEVEL    GPIO_PIN_SET

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(
        &huart1,
        (const uint8_t *)text,
        (uint16_t)strlen(text),
        HAL_MAX_DELAY);
}
#define PWM_PERIOD_COUNTS 6400U
#define TEST_DUTY_PERMILLE 990U   /* 90.0%，低速测试 */

static uint32_t Motor_DutyToCompare(uint16_t duty_permille)
{
    if (duty_permille > 1000U)
    {
        duty_permille = 1000U;
    }

    /*
     * 2%以下会触发保护。
     * 100%是停止/复位，20～999是合法运行范围。
     */
    if ((duty_permille < 20U) &&
        (duty_permille != 1000U))
    {
        duty_permille = 20U;
    }

    return ((uint32_t)duty_permille *
            PWM_PERIOD_COUNTS) / 1000U;
}

static void Motor_SetLeftDutyPermille(uint16_t duty_permille)
{
    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        Motor_DutyToCompare(duty_permille));
}

static void Motor_SetRightDutyPermille(uint16_t duty_permille)
{
    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_2,
        Motor_DutyToCompare(duty_permille));
}

static void Motor_SetBothDutyPermille(uint16_t duty_permille)
{
    uint32_t compare = Motor_DutyToCompare(duty_permille);

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        compare);

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_2,
        compare);
}

static void Motor_Stop(void)
{
    /*
     * 100% duty：
     * 停止电机，同时解除此前可能触发的<2%保护。
     */
    Motor_SetBothDutyPermille(1000U);
}


static void Motor_SetForward(void)
{
    HAL_GPIO_WritePin(
        DIR_L_GPIO_Port,
        DIR_L_Pin,
        LEFT_FORWARD_LEVEL);

    HAL_GPIO_WritePin(
        DIR_R_GPIO_Port,
        DIR_R_Pin,
        RIGHT_FORWARD_LEVEL);
}

static void Motor_SetReverse(void)
{
    HAL_GPIO_WritePin(
        DIR_L_GPIO_Port,
        DIR_L_Pin,
        LEFT_REVERSE_LEVEL);

    HAL_GPIO_WritePin(
        DIR_R_GPIO_Port,
        DIR_R_Pin,
        RIGHT_REVERSE_LEVEL);
}

static void Motor_SetDirection(
    int8_t left_direction,
    int8_t right_direction)
{
    /*
     * left_direction/right_direction:
     *  1 = 小车前进方向
     * -1 = 小车后退方向
     *  0 = 保持当前方向，电机由100% duty停止
     */

    if (left_direction > 0)
    {
        HAL_GPIO_WritePin(
            DIR_L_GPIO_Port,
            DIR_L_Pin,
            LEFT_FORWARD_LEVEL);
    }
    else if (left_direction < 0)
    {
        HAL_GPIO_WritePin(
            DIR_L_GPIO_Port,
            DIR_L_Pin,
            LEFT_REVERSE_LEVEL);
    }

    if (right_direction > 0)
    {
        HAL_GPIO_WritePin(
            DIR_R_GPIO_Port,
            DIR_R_Pin,
            RIGHT_FORWARD_LEVEL);
    }
    else if (right_direction < 0)
    {
        HAL_GPIO_WritePin(
            DIR_R_GPIO_Port,
            DIR_R_Pin,
            RIGHT_REVERSE_LEVEL);
    }

    motor_direction[0] = left_direction;
    motor_direction[1] = right_direction;
}

static void FG_TakeSnapshot(uint32_t pulses[WHEEL_COUNT])
{
    __disable_irq();

    for (uint32_t i = 0U; i < WHEEL_COUNT; i++)
    {
        pulses[i] = fg_window[i];
        fg_window[i] = 0U;
    }

    __enable_irq();
}

static const float fg_ppr[WHEEL_COUNT] =
{
    FG_PPR_FL,
    FG_PPR_RL,
    FG_PPR_FR,
    FG_PPR_RR
};

static void FG_PrintSnapshot(void)
{
    uint32_t pulses[WHEEL_COUNT];
    float rpm[WHEEL_COUNT];
    float left_rpm;
    float right_rpm;
    char message[192];

    /*
     * 当前每500 ms调用一次：
     *
     * RPM = pulses / PPR / 0.5s * 60
     *     = pulses * 120 / PPR
     */
    FG_TakeSnapshot(pulses);

    for (uint32_t i = 0U; i < WHEEL_COUNT; i++)
    {
        rpm[i] =
            ((float)pulses[i] * 120.0f) /
            fg_ppr[i];
    }

    /*
     * FG只有脉冲，没有方向。
     * 使用当前DIR命令给RPM添加正负号。
     */
    rpm[WHEEL_FL] *= (float)motor_direction[0];
    rpm[WHEEL_RL] *= (float)motor_direction[0];

    rpm[WHEEL_FR] *= (float)motor_direction[1];
    rpm[WHEEL_RR] *= (float)motor_direction[1];

    left_rpm =
        (rpm[WHEEL_FL] + rpm[WHEEL_RL]) * 0.5f;

    right_rpm =
        (rpm[WHEEL_FR] + rpm[WHEEL_RR]) * 0.5f;

    snprintf(
        message,
        sizeof(message),
        "[RPM] FL=%.2f RL=%.2f FR=%.2f RR=%.2f "
        "LEFT=%.2f RIGHT=%.2f\r\n",
        rpm[WHEEL_FL],
        rpm[WHEEL_RL],
        rpm[WHEEL_FR],
        rpm[WHEEL_RR],
        left_rpm,
        right_rpm);

    UART_Print(message);
}
static void FG_ResetTotals(void)
{
    __disable_irq();

    for (uint32_t i = 0U; i < WHEEL_COUNT; i++)
    {
        fg_total[i] = 0U;
        fg_window[i] = 0U;
    }

    __enable_irq();

    UART_Print("[FG] Totals reset to zero\r\n");
}

static void FG_PrintTotals(void)
{
    uint32_t totals[WHEEL_COUNT];
    char message[160];

    __disable_irq();

    for (uint32_t i = 0U; i < WHEEL_COUNT; i++)
    {
        totals[i] = fg_total[i];
    }

    __enable_irq();

    snprintf(
        message,
        sizeof(message),
        "[FG TOTAL] FL=%lu RL=%lu FR=%lu RR=%lu\r\n",
        (unsigned long)totals[WHEEL_FL],
        (unsigned long)totals[WHEEL_RL],
        (unsigned long)totals[WHEEL_FR],
        (unsigned long)totals[WHEEL_RR]);

    UART_Print(message);
}
static void Process_UART_Command(void)
{
    uint8_t command;

    /*
     * timeout=0，非阻塞接收。
     */
    if (HAL_UART_Receive(
            &huart1,
            &command,
            1U,
            0U) != HAL_OK)
    {
        return;
    }

    switch (command)
    {
        case 'F':
        case 'f':
            Motor_ChangeDirectionSafely(1, 1);
            Motor_SetBothDutyPermille(
                TEST_DUTY_PERMILLE);
            UART_Print("[MOTOR] FORWARD\r\n");
            break;

        case 'V':
        case 'v':
            Motor_ChangeDirectionSafely(-1, -1);
            Motor_SetBothDutyPermille(
                TEST_DUTY_PERMILLE);
            UART_Print("[MOTOR] REVERSE\r\n");
            break;

        case 'A':
        case 'a':
            /*
             * 原地左转：
             * 左侧后退，右侧前进
             */
            Motor_ChangeDirectionSafely(-1, 1);
            Motor_SetBothDutyPermille(
                TEST_DUTY_PERMILLE);
            UART_Print("[MOTOR] TURN LEFT\r\n");
            break;

        case 'D':
        case 'd':
            /*
             * 原地右转：
             * 左侧前进，右侧后退
             */
            Motor_ChangeDirectionSafely(1, -1);
            Motor_SetBothDutyPermille(
                TEST_DUTY_PERMILLE);
            UART_Print("[MOTOR] TURN RIGHT\r\n");
            break;

        case 'S':
        case 's':
            Motor_Stop();
            UART_Print("[MOTOR] STOP\r\n");
            FG_PrintTotals();
            break;

        case 'Z':
        case 'z':
            FG_ResetTotals();
            break;

        case 'P':
        case 'p':
            FG_PrintTotals();
            break;

        case '\r':
        case '\n':
            /*
             * 忽略串口终端发送的换行符。
             */
            break;

        default:
            UART_Print(
                "[CMD] Z=zero F=forward V=reverse "
                "A=left D=right S=stop P=print\r\n");
            break;
    }
}

static void Motor_ChangeDirectionSafely(
    int8_t left_direction,
    int8_t right_direction)
{
    Motor_Stop();
    HAL_Delay(300U);

    Motor_SetDirection(
        left_direction,
        right_direction);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  Motor_SetForward();

  /* 上电后保持100% duty至少2秒，停止并解除保护 */
  Motor_Stop();
  HAL_Delay(2000U);

  FG_ResetTotals();

  UART_Print(
      "[CMD] Z=zero F=forward V=reverse "
      "A=left D=right S=stop P=print\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_ms = 0U;
  while (1)
  {
	  Process_UART_Command();

	 if ((HAL_GetTick() - last_print_ms) >= 500U)
	 {
		 last_print_ms = HAL_GetTick();
		 FG_PrintSnapshot();
	 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_L_Pin|DIR_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_L_Pin DIR_R_Pin */
  GPIO_InitStruct.Pin = DIR_L_Pin|DIR_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FG_FL_Pin FG_RL_Pin */
  GPIO_InitStruct.Pin = FG_FL_Pin|FG_RL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FG_RR_Pin FG_FR_Pin */
  GPIO_InitStruct.Pin = FG_RR_Pin|FG_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    WheelIndex wheel;

    if (GPIO_Pin == FG_FL_Pin)
    {
        wheel = WHEEL_FL;
    }
    else if (GPIO_Pin == FG_RL_Pin)
    {
        wheel = WHEEL_RL;
    }
    else if (GPIO_Pin == FG_FR_Pin)
    {
        wheel = WHEEL_FR;
    }
    else if (GPIO_Pin == FG_RR_Pin)
    {
        wheel = WHEEL_RR;
    }
    else
    {
        return;
    }

    fg_total[wheel]++;
    fg_window[wheel]++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
