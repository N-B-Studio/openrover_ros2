/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Minimal bare-metal micro-ROS serial talker POC
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ENABLE_MICROROS 1

#if ENABLE_MICROROS
#include <rcl/time.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <uxr/client/transport.h>
#endif
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#if ENABLE_MICROROS

#define COMMAND_TIMEOUT_MS 500U

static rcl_allocator_t microros_allocator;
static rclc_support_t microros_support;
static rcl_node_t microros_node;
static rclc_executor_t microros_executor;

static rcl_publisher_t heartbeat_publisher;
static rcl_publisher_t status_publisher;
static rcl_subscription_t command_subscription;

static std_msgs__msg__Int32 heartbeat_message;
static std_msgs__msg__Int32 status_message;
static std_msgs__msg__Int32 command_message;

static int32_t active_command = 0;
static uint32_t last_command_ms = 0U;
static bool command_received = false;
static bool command_timed_out = false;

/* transport 和 allocator prototypes */


#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);

size_t cubemx_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error);

size_t cubemx_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout_ms,
    uint8_t *error);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);

void *microros_zero_allocate(
    size_t number_of_elements,
    size_t size_of_element,
    void *state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static void Command_Callback(const void *message_input)
{
    const std_msgs__msg__Int32 *received_message =
        (const std_msgs__msg__Int32 *)message_input;

    if (received_message == NULL)
    {
        return;
    }

    active_command = received_message->data;
    last_command_ms = HAL_GetTick();
    command_received = true;
    command_timed_out = false;

    if (active_command != 0)
    {
        /* Active-low LED: command active means LED on. */
        HAL_GPIO_WritePin(
            LED_GPIO_Port,
            LED_Pin,
            GPIO_PIN_RESET);
    }
    else
    {
        /* LED off. */
        HAL_GPIO_WritePin(
            LED_GPIO_Port,
            LED_Pin,
            GPIO_PIN_SET);
    }
}

#if ENABLE_MICROROS
static bool MicroROS_Init(void)
{
    rmw_ret_t rmw_result;
    rcl_ret_t rcl_result;

    /* USART1 is exclusively owned by the XRCE-DDS transport. */
    rmw_result = rmw_uros_set_custom_transport(
        true,
        (void *)&huart1,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    if (rmw_result != RMW_RET_OK)
    {
        return false;
    }

    microros_allocator = rcutils_get_zero_initialized_allocator();
    microros_allocator.allocate = microros_allocate;
    microros_allocator.deallocate = microros_deallocate;
    microros_allocator.reallocate = microros_reallocate;
    microros_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&microros_allocator))
    {
        return false;
    }

    rcl_result = rclc_support_init(
        &microros_support,
        0,
        NULL,
        &microros_allocator);

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    microros_node = rcl_get_zero_initialized_node();

    rcl_result = rclc_node_init_default(
        &microros_node,
        "g431_controller",
        "",
        &microros_support);

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    /*
     * Publisher 1:
     * MCU heartbeat counter.
     */
    heartbeat_publisher = rcl_get_zero_initialized_publisher();

    rcl_result = rclc_publisher_init_default(
        &heartbeat_publisher,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/g431/heartbeat");

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    /*
     * Publisher 2:
     * Current command after watchdog processing.
     */
    status_publisher = rcl_get_zero_initialized_publisher();

    rcl_result = rclc_publisher_init_default(
        &status_publisher,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/g431/status");

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    /*
     * Subscriber:
     * Command received from the ROS 2 computer.
     */
    command_subscription =
        rcl_get_zero_initialized_subscription();

    rcl_result = rclc_subscription_init_default(
        &command_subscription,
        &microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/g431/cmd");

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    /*
     * One executor handle is required for one subscription.
     */
    microros_executor = rclc_executor_get_zero_initialized_executor();

    rcl_result = rclc_executor_init(
        &microros_executor,
        &microros_support.context,
        1,
        &microros_allocator);

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    rcl_result = rclc_executor_add_subscription(
        &microros_executor,
        &command_subscription,
        &command_message,
        &Command_Callback,
        ON_NEW_DATA);

    if (rcl_result != RCL_RET_OK)
    {
        return false;
    }

    heartbeat_message.data = 0;
    status_message.data = 0;
    command_message.data = 0;

    active_command = 0;
    last_command_ms = HAL_GetTick();
    command_received = false;
    command_timed_out = false;

    return true;
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	#if ENABLE_MICROROS
	if (!MicroROS_Init())
	{
		Error_Handler();
	}
	#endif

	HAL_GPIO_WritePin(
		LED_GPIO_Port,
		LED_Pin,
		GPIO_PIN_SET);

	uint32_t last_heartbeat_ms = HAL_GetTick();
	uint32_t last_status_ms = HAL_GetTick();

	#if ENABLE_MICROROS
	rcl_ret_t publish_result = RCL_RET_OK;
	#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if ENABLE_MICROROS

    /*
     * Process incoming command first.
     * Callback may update last_command_ms.
     */
    (void)rclc_executor_spin_some(
        &microros_executor,
        RCL_MS_TO_NS(1));

    /*
     * Read time after executor processing.
     */
    uint32_t now_ms = HAL_GetTick();

    if (command_received &&
        ((now_ms - last_command_ms) >= COMMAND_TIMEOUT_MS))
    {
        active_command = 0;
        command_received = false;
        command_timed_out = true;

        HAL_GPIO_WritePin(
            LED_GPIO_Port,
            LED_Pin,
            GPIO_PIN_SET);
    }

    if ((now_ms - last_heartbeat_ms) >= 1000U)
    {
        last_heartbeat_ms = now_ms;
        heartbeat_message.data++;

        publish_result = rcl_publish(
            &heartbeat_publisher,
            &heartbeat_message,
            NULL);

        if (publish_result != RCL_RET_OK)
        {
            /* Keep watchdog running. */
        }
    }

    if ((now_ms - last_status_ms) >= 100U)
    {
        last_status_ms = now_ms;
        status_message.data = active_command;

        publish_result = rcl_publish(
            &status_publisher,
            &status_message,
            NULL);

        if (publish_result != RCL_RET_OK)
        {
            /* Keep watchdog running. */
        }
    }

    HAL_Delay(1U);

#else

    HAL_GPIO_TogglePin(
        LED_GPIO_Port,
        LED_Pin);

    HAL_Delay(1000U);

#endif
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

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                               RCC_CLOCKTYPE_SYSCLK |
                               RCC_CLOCKTYPE_PCLK1 |
                               RCC_CLOCKTYPE_PCLK2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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

  if (HAL_UARTEx_SetTxFifoThreshold(
          &huart1,
          UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetRxFifoThreshold(
          &huart1,
          UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(
      LED_GPIO_Port,
      LED_Pin,
      GPIO_PIN_SET);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  __disable_irq();

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
