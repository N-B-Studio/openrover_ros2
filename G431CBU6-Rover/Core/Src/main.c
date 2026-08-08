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

typedef struct
{
    float rpm;
    uint16_t duty_permille;
} MotorCalibrationPoint;

typedef enum
{
    MOTOR_STATE_READY = 0,
    MOTOR_STATE_WAIT_DIRECTION
} MotorState;

static MotorState motor_state = MOTOR_STATE_READY;
static int8_t pending_direction[2] = {0, 0};
static uint16_t pending_duty[2] = {1000U, 1000U};
static uint32_t direction_change_started_ms = 0U;
static uint32_t last_motor_command_ms = 0U;
static uint8_t motor_watchdog_enabled = 0U;
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
static void Motor_SetWheelRPM(float left_rpm, float right_rpm);
static void Motor_Update(void);
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
#define PWM_PERIOD_COUNTS          6400U
#define MOTOR_STOP_DUTY_PERMILLE   1000U
#define MOTOR_MIN_RPM              6.5f
#define MOTOR_MAX_RPM              30.0f
#define MOTOR_DIRECTION_DELAY_MS   300U
#define MOTOR_COMMAND_TIMEOUT_MS   200U
#define UART_TEST_RPM              15.0f

/*
 * 架空实测标定数据。数组必须按RPM从小到大排列。
 * 970是可靠低速边界；980虽然架空可转，但落地可靠性不足，因此不使用。
 */
static const MotorCalibrationPoint left_forward_curve[] =
{
    { 5.3f, 970U }, { 6.8f, 960U }, { 8.9f, 950U },
    {14.2f, 925U }, {20.5f, 900U }, {25.9f, 875U },
    {31.1f, 850U }
};

static const MotorCalibrationPoint right_forward_curve[] =
{
    { 6.4f, 970U }, { 8.5f, 960U }, {10.5f, 950U },
    {15.9f, 925U }, {21.6f, 900U }, {26.6f, 875U },
    {31.7f, 850U }
};

static const MotorCalibrationPoint left_reverse_curve[] =
{
    { 5.7f, 970U }, { 7.8f, 960U }, {10.0f, 950U },
    {15.2f, 925U }, {21.6f, 900U }, {26.9f, 875U },
    {32.2f, 850U }
};

static const MotorCalibrationPoint right_reverse_curve[] =
{
    { 5.5f, 970U }, { 7.5f, 960U }, { 9.5f, 950U },
    {14.8f, 925U }, {20.6f, 900U }, {25.6f, 875U },
    {30.7f, 850U }
};

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

static void Motor_OutputStop(void)
{
    /*
     * 100% duty：
     * 停止电机，同时解除此前可能触发的<2%保护。
     */
    Motor_SetBothDutyPermille(MOTOR_STOP_DUTY_PERMILLE);
}

static void Motor_Stop(void)
{
    Motor_OutputStop();
    motor_state = MOTOR_STATE_READY;
    pending_direction[0] = 0;
    pending_direction[1] = 0;
    pending_duty[0] = MOTOR_STOP_DUTY_PERMILLE;
    pending_duty[1] = MOTOR_STOP_DUTY_PERMILLE;
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

    if (left_direction != 0)
    {
        motor_direction[0] = left_direction;
    }

    if (right_direction != 0)
    {
        motor_direction[1] = right_direction;
    }
}

static uint16_t Motor_InterpolateDuty(
    float target_rpm,
    const MotorCalibrationPoint *curve,
    uint32_t point_count)
{
    if (target_rpm <= curve[0].rpm)
    {
        return curve[0].duty_permille;
    }

    for (uint32_t i = 1U; i < point_count; i++)
    {
        if (target_rpm <= curve[i].rpm)
        {
            float ratio =
                (target_rpm - curve[i - 1U].rpm) /
                (curve[i].rpm - curve[i - 1U].rpm);
            float duty =
                (float)curve[i - 1U].duty_permille +
                ratio *
                ((float)curve[i].duty_permille -
                 (float)curve[i - 1U].duty_permille);

            return (uint16_t)(duty + 0.5f);
        }
    }

    return curve[point_count - 1U].duty_permille;
}

static uint16_t Motor_RPMToDuty(
    float rpm,
    uint8_t is_left)
{
    const MotorCalibrationPoint *curve;
    uint32_t point_count;
    float magnitude = (rpm < 0.0f) ? -rpm : rpm;

    if (magnitude < MOTOR_MIN_RPM)
    {
        magnitude = MOTOR_MIN_RPM;
    }
    else if (magnitude > MOTOR_MAX_RPM)
    {
        magnitude = MOTOR_MAX_RPM;
    }

    if (is_left != 0U)
    {
        curve = (rpm > 0.0f) ?
            left_forward_curve : left_reverse_curve;
        point_count = (rpm > 0.0f) ?
            (uint32_t)(sizeof(left_forward_curve) /
                       sizeof(left_forward_curve[0])) :
            (uint32_t)(sizeof(left_reverse_curve) /
                       sizeof(left_reverse_curve[0]));
    }
    else
    {
        curve = (rpm > 0.0f) ?
            right_forward_curve : right_reverse_curve;
        point_count = (rpm > 0.0f) ?
            (uint32_t)(sizeof(right_forward_curve) /
                       sizeof(right_forward_curve[0])) :
            (uint32_t)(sizeof(right_reverse_curve) /
                       sizeof(right_reverse_curve[0]));
    }

    return Motor_InterpolateDuty(
        magnitude,
        curve,
        point_count);
}

/*
 * 未来micro-ROS的/cmd_vel回调只需调用这个接口。
 * 每次调用都会刷新200 ms命令看门狗。
 */
static void Motor_SetWheelRPM(float left_rpm, float right_rpm)
{
    int8_t desired_direction[2];
    uint16_t desired_duty[2];

    last_motor_command_ms = HAL_GetTick();
    motor_watchdog_enabled = 1U;

    desired_direction[0] =
        (left_rpm > 0.0f) ? 1 :
        ((left_rpm < 0.0f) ? -1 : 0);
    desired_direction[1] =
        (right_rpm > 0.0f) ? 1 :
        ((right_rpm < 0.0f) ? -1 : 0);

    desired_duty[0] = (desired_direction[0] == 0) ?
        MOTOR_STOP_DUTY_PERMILLE :
        Motor_RPMToDuty(left_rpm, 1U);
    desired_duty[1] = (desired_direction[1] == 0) ?
        MOTOR_STOP_DUTY_PERMILLE :
        Motor_RPMToDuty(right_rpm, 0U);

    if ((desired_direction[0] == 0) &&
        (desired_direction[1] == 0))
    {
        Motor_Stop();
        return;
    }

    pending_direction[0] = desired_direction[0];
    pending_direction[1] = desired_direction[1];
    pending_duty[0] = desired_duty[0];
    pending_duty[1] = desired_duty[1];

    if (motor_state == MOTOR_STATE_WAIT_DIRECTION)
    {
        return;
    }

    if (((desired_direction[0] != 0) &&
         (desired_direction[0] != motor_direction[0])) ||
        ((desired_direction[1] != 0) &&
         (desired_direction[1] != motor_direction[1])))
    {
        Motor_ChangeDirectionSafely(
            desired_direction[0],
            desired_direction[1]);
        return;
    }

    Motor_SetDirection(
        desired_direction[0],
        desired_direction[1]);
    Motor_SetLeftDutyPermille(desired_duty[0]);
    Motor_SetRightDutyPermille(desired_duty[1]);
}

static void Motor_Update(void)
{
    uint32_t now = HAL_GetTick();

    if ((motor_watchdog_enabled != 0U) &&
        ((now - last_motor_command_ms) > MOTOR_COMMAND_TIMEOUT_MS))
    {
        Motor_Stop();
        motor_watchdog_enabled = 0U;
        UART_Print("[SAFETY] Motor command timeout -> STOP\r\n");
        return;
    }

    if ((motor_state == MOTOR_STATE_WAIT_DIRECTION) &&
        ((now - direction_change_started_ms) >=
         MOTOR_DIRECTION_DELAY_MS))
    {
        Motor_SetDirection(
            pending_direction[0],
            pending_direction[1]);
        Motor_SetLeftDutyPermille(pending_duty[0]);
        Motor_SetRightDutyPermille(pending_duty[1]);
        motor_state = MOTOR_STATE_READY;
    }
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
            Motor_SetWheelRPM(UART_TEST_RPM, UART_TEST_RPM);
            motor_watchdog_enabled = 0U;
            UART_Print("[MOTOR] FORWARD\r\n");
            break;

        case 'V':
        case 'v':
            Motor_SetWheelRPM(-UART_TEST_RPM, -UART_TEST_RPM);
            motor_watchdog_enabled = 0U;
            UART_Print("[MOTOR] REVERSE\r\n");
            break;

        case 'A':
        case 'a':
            /*
             * 原地左转：
             * 左侧后退，右侧前进
             */
            Motor_SetWheelRPM(-UART_TEST_RPM, UART_TEST_RPM);
            motor_watchdog_enabled = 0U;
            UART_Print("[MOTOR] TURN LEFT\r\n");
            break;

        case 'D':
        case 'd':
            /*
             * 原地右转：
             * 左侧前进，右侧后退
             */
            Motor_SetWheelRPM(UART_TEST_RPM, -UART_TEST_RPM);
            motor_watchdog_enabled = 0U;
            UART_Print("[MOTOR] TURN RIGHT\r\n");
            break;

        case 'S':
        case 's':
            Motor_Stop();
            motor_watchdog_enabled = 0U;
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
    pending_direction[0] = left_direction;
    pending_direction[1] = right_direction;
    Motor_OutputStop();
    direction_change_started_ms = HAL_GetTick();
    motor_state = MOTOR_STATE_WAIT_DIRECTION;
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

  Motor_SetDirection(1, 1);

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
	  Motor_Update();

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
