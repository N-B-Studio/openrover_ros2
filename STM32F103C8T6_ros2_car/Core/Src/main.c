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

#include "jc_motor.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint32_t app_tick = 0U;

volatile uint8_t uart_rx_byte = 0U;


/*
 * ============================================================
 * MOTOR TARGET
 * ============================================================
 */

static volatile float left_target_rpm = 0.0f;
static volatile float right_target_rpm = 0.0f;


/*
 * ============================================================
 * COMMAND WATCHDOG
 * ============================================================
 */

static volatile uint32_t last_cmd_ms = 0U;

static volatile uint8_t cmd_watchdog_active = 1U;


/*
 * ============================================================
 * UART
 * ============================================================
 */

#define UART_CMD_BUFFER_SIZE    64U

static char uart_cmd_buffer[
    UART_CMD_BUFFER_SIZE
];

static volatile uint8_t uart_cmd_index = 0U;


/*
 * ============================================================
 * SYSTEM STATUS
 * ============================================================
 */

static volatile uint8_t can_started = 0U;

static volatile uint8_t uart_command_seen = 0U;


/*
 * ============================================================
 * ROCK 5C
 * ============================================================
 */

#define ROCK_TIMEOUT_MS         15000U

static char rock_ip[24] =
    "192.168.xxx.xxx";

static volatile uint32_t last_rock_ms = 0U;

static volatile uint8_t rock_seen = 0U;


/*
 * ============================================================
 * CONFIGURATION
 * ============================================================
 */

#define LEFT_FORWARD_SIGN       (+1.0f)
#define RIGHT_FORWARD_SIGN      (-1.0f)

#define MANUAL_DRIVE_RPM        50.0f
#define MANUAL_TURN_RPM         40.0f

#define COMMAND_TIMEOUT_MS      300U

#define MOTOR_TIMEOUT_MS        100U

#define TELEMETRY_PERIOD_MS     20U

#define OLED_PERIOD_MS          200U

#define ROBOT_MAX_WHEEL_RPM  50.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Robot_SetWheelRpm(
    float left_rpm,
    float right_rpm)
{
    left_target_rpm = left_rpm;
    right_target_rpm = right_rpm;
}

static void Robot_Stop(void)
{
    Robot_SetWheelRpm(
        0.0f,
        0.0f
    );
}

static void Robot_Forward(void)
{
    Robot_SetWheelRpm(
        LEFT_FORWARD_SIGN *
            MANUAL_DRIVE_RPM,

        RIGHT_FORWARD_SIGN *
            MANUAL_DRIVE_RPM
    );
}

static void Robot_Backward(void)
{
    Robot_SetWheelRpm(
        -LEFT_FORWARD_SIGN *
            MANUAL_DRIVE_RPM,

        -RIGHT_FORWARD_SIGN *
            MANUAL_DRIVE_RPM
    );
}

static void Robot_TurnLeft(void)
{
    Robot_SetWheelRpm(
        -LEFT_FORWARD_SIGN *
            MANUAL_TURN_RPM,

        RIGHT_FORWARD_SIGN *
            MANUAL_TURN_RPM
    );
}

static void Robot_TurnRight(void)
{
    Robot_SetWheelRpm(
        LEFT_FORWARD_SIGN *
            MANUAL_TURN_RPM,

        -RIGHT_FORWARD_SIGN *
            MANUAL_TURN_RPM
    );
}


/*
 * Mark a command as valid.
 *
 * This kicks the command watchdog.
 */
static void Robot_CommandReceived(void)
{
    last_cmd_ms = HAL_GetTick();

    cmd_watchdog_active = 0U;

    uart_command_seen = 1U;
}

/*
 * Formal ROS-ready UART command:
 *
 * V,<left_rpm_x100>,<right_rpm_x100>
 *
 * Examples:
 *
 * V,5000,-5000
 *   left  = +50.00 rpm
 *   right = -50.00 rpm
 *
 * V,0,0
 *   stop
 */
static void Robot_ParseVelocityCommand(
    const char *line)
{
    long left_raw;
    long right_raw;

    if (sscanf(
            line,
            "V,%ld,%ld",
            &left_raw,
            &right_raw) == 2)
    {
        /*
         * UART protocol uses rpm * 100.
         */
        float left_rpm =
            (float)left_raw / 100.0f;

        float right_rpm =
            (float)right_raw / 100.0f;

        /*
         * Safety clamp at UART boundary.
         */
        if (left_rpm > ROBOT_MAX_WHEEL_RPM)
        {
            left_rpm = ROBOT_MAX_WHEEL_RPM;
        }

        if (left_rpm < -ROBOT_MAX_WHEEL_RPM)
        {
            left_rpm = -ROBOT_MAX_WHEEL_RPM;
        }

        if (right_rpm > ROBOT_MAX_WHEEL_RPM)
        {
            right_rpm = ROBOT_MAX_WHEEL_RPM;
        }

        if (right_rpm < -ROBOT_MAX_WHEEL_RPM)
        {
            right_rpm = -ROBOT_MAX_WHEEL_RPM;
        }

        Robot_SetWheelRpm(
            left_rpm,
            right_rpm
        );

        Robot_CommandReceived();

        char message[96];

        int length = snprintf(
            message,
            sizeof(message),

            "[CMD] V L=%.2f R=%.2f rpm\r\n",

            left_rpm,
            right_rpm
        );

        if (length > 0)
        {
            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)message,
                (uint16_t)length,
                100U
            );
        }
    }
}


/*
 * Debug keyboard commands.
 *
 * These are intentionally retained for bench testing.
 */
static void Robot_HandleSingleCommand(
    uint8_t command)
{
    switch (command)
    {
        case 'w':
        case 'W':

            Robot_Forward();
            Robot_CommandReceived();
            break;


        case 's':
        case 'S':

            Robot_Backward();
            Robot_CommandReceived();
            break;


        case 'a':
        case 'A':

            Robot_TurnLeft();
            Robot_CommandReceived();
            break;


        case 'd':
        case 'D':

            Robot_TurnRight();
            Robot_CommandReceived();
            break;


        case 'x':
        case 'X':
        case ' ':

            Robot_Stop();
            Robot_CommandReceived();
            break;


        default:
            break;
    }
}

static void Robot_ParseIpCommand( const char *line)
{
    const char *ip;

    if (line == NULL)
    {
        return;
    }

    if (strncmp(
            line,
            "IP,",
            3U) != 0)
    {
        return;
    }

    ip = &line[3];


    /*
     * Reject empty or oversized strings.
     */
    size_t length =
        strlen(ip);

    if ((length == 0U) ||
        (length >= sizeof(rock_ip)))
    {
        return;
    }


    /*
     * Store IP for OLED.
     */
    strncpy(
        rock_ip,
        ip,
        sizeof(rock_ip) - 1U
    );

    rock_ip[
        sizeof(rock_ip) - 1U
    ] = '\0';


    /*
     * IP packet also acts as
     * Rock 5C heartbeat.
     */
    last_rock_ms =
        HAL_GetTick();

    rock_seen = 1U;
}



/*
 * Process one complete UART line.
 */
static void Robot_ProcessUartLine(
    const char *line)
{
    if (line == NULL)
    {
        return;
    }


    /*
     * Wheel velocity command.
     */
    if (strncmp(
            line,
            "V,",
            2U) == 0)
    {
        Robot_ParseVelocityCommand(
            line
        );

        return;
    }


    /*
     * Rock 5C IP / heartbeat.
     */
    if (strncmp(
            line,
            "IP,",
            3U) == 0)
    {
        Robot_ParseIpCommand(
            line
        );

        return;
    }
}


static void Robot_CheckWatchdog(void)
{
    uint32_t now =
        HAL_GetTick();

    if ((uint32_t)(
            now - last_cmd_ms
        ) > COMMAND_TIMEOUT_MS)
    {
        /*
         * Only stop once when watchdog trips.
         */
        if (cmd_watchdog_active == 0U)
        {
            Robot_Stop();

            cmd_watchdog_active = 1U;
        }
    }
}

static uint8_t Robot_GetStatus(void)
{
    uint8_t status = 0U;

    uint32_t now =
        HAL_GetTick();

    /*
     * bit0:
     * CAN peripheral started.
     */
    if (can_started)
    {
        status |= (1U << 0);
    }


    /*
     * bit1:
     * Left motor feedback alive.
     */
    if ((uint32_t)(
            now - motor_left.last_rx_ms
        ) <= MOTOR_TIMEOUT_MS)
    {
        status |= (1U << 1);
    }


    /*
     * bit2:
     * Right motor feedback alive.
     */
    if ((uint32_t)(
            now - motor_right.last_rx_ms
        ) <= MOTOR_TIMEOUT_MS)
    {
        status |= (1U << 2);
    }


    /*
     * bit3:
     * Command watchdog healthy.
     *
     * cmd_watchdog_active == 0
     * means command link is alive.
     */
    if (cmd_watchdog_active == 0U)
    {
        status |= (1U << 3);
    }


    /*
     * bit4:
     * At least one valid UART command
     * has been received since boot.
     */
    if (uart_command_seen)
    {
        status |= (1U << 4);
    }

    return status;
}

static void Robot_SendTelemetry(void)
{
    char buffer[160];

    uint32_t now =
        HAL_GetTick();

    /*
     * Position and velocity are transmitted
     * as scaled integers.
     *
     * position:
     * degree * 100
     *
     * velocity:
     * rpm * 100
     */
    int32_t left_pos_raw =
        (int32_t)(
            motor_left.pos_deg * 100.0f
        );

    int32_t left_vel_raw =
        (int32_t)(
            motor_left.vel_rpm * 100.0f
        );

    int32_t right_pos_raw =
        (int32_t)(
            motor_right.pos_deg * 100.0f
        );

    int32_t right_vel_raw =
        (int32_t)(
            motor_right.vel_rpm * 100.0f
        );

    uint8_t status =
        Robot_GetStatus();

    int length = snprintf(
        buffer,
        sizeof(buffer),

        "FB,%lu,%ld,%ld,%ld,%ld,%u\r\n",

        (unsigned long)now,

        (long)left_pos_raw,
        (long)left_vel_raw,

        (long)right_pos_raw,
        (long)right_vel_raw,

        (unsigned int)status
    );

    if (length <= 0)
    {
        return;
    }

    if ((size_t)length >= sizeof(buffer))
    {
        length =
            sizeof(buffer) - 1U;
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)buffer,
        (uint16_t)length,
        20U
    );
}

static uint8_t Robot_IsRockOnline(void)
{
    if (rock_seen == 0U)
    {
        return 0U;
    }


    uint32_t age =
        HAL_GetTick() -
        last_rock_ms;


    if (age >
        ROCK_TIMEOUT_MS)
    {
        return 0U;
    }


    return 1U;
}

static void Robot_UpdateOLED(void)
{
    char line[32];

    uint8_t status =
        Robot_GetStatus();

    uint8_t can_ok =
        ((status &
          (1U << 0)) != 0U);

    uint8_t left_ok =
        ((status &
          (1U << 1)) != 0U);

    uint8_t right_ok =
        ((status &
          (1U << 2)) != 0U);

    uint8_t ros_ok =
        ((status &
          (1U << 3)) != 0U);

    uint8_t rock_ok =
        Robot_IsRockOnline();


    /*
     * Convert velocity manually
     * to integer rpm for display.
     *
     * OLED is status only.
     * Full precision remains in FB telemetry.
     */
    int left_rpm =
        (int)motor_left.vel_rpm;

    int right_rpm =
        (int)motor_right.vel_rpm;


    OLED_Clear();


    /*
     * ========================================================
     * LINE 1
     * ========================================================
     */

    OLED_SetCursor(
        0U,
        0U
    );

    OLED_WriteString(
        "ROS CAR"
    );


    /*
     * ========================================================
     * LINE 2
     * ========================================================
     */

    snprintf(
        line,
        sizeof(line),
        "IP:%s",
        rock_ip
    );

    OLED_SetCursor(
        0U,
        10U
    );

    OLED_WriteString(
        line
    );


    /*
     * ========================================================
     * LINE 3
     * ========================================================
     */

    snprintf(
        line,
        sizeof(line),

        "ROCK:%s",

        rock_ok ?
            "OK" :
            "WAIT"
    );

    OLED_SetCursor(
        0U,
        20U
    );

    OLED_WriteString(
        line
    );


    /*
     * ========================================================
     * LINE 4
     * ========================================================
     */

    snprintf(
        line,
        sizeof(line),

        "CAN:%s ROS:%s",

        (can_ok &&
         left_ok &&
         right_ok) ?
            "OK" :
            "ERR",

        ros_ok ?
            "OK" :
            "WAIT"
    );

    OLED_SetCursor(
        0U,
        30U
    );

    OLED_WriteString(
        line
    );


    /*
     * ========================================================
     * LINE 5
     * ========================================================
     */

    snprintf(
        line,
        sizeof(line),

        "L:%+4d RPM",

        left_rpm
    );

    OLED_SetCursor(
        0U,
        40U
    );

    OLED_WriteString(
        line
    );


    /*
     * ========================================================
     * LINE 6
     * ========================================================
     */

    snprintf(
        line,
        sizeof(line),

        "R:%+4d RPM",

        right_rpm
    );

    OLED_SetCursor(
        0U,
        50U
    );

    OLED_WriteString(
        line
    );


    OLED_Update();
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*
   * ============================================================
   * OLED
   * ============================================================
   */

  HAL_Delay(100U);


  /*
   * First verify that an I2C device exists.
   *
   * OLED expected at 0x3C.
   */
  if (HAL_I2C_IsDeviceReady(
          &hi2c1,
          OLED_I2C_ADDR,
          3U,
          100U) == HAL_OK)
  {
      if (OLED_Init(
              &hi2c1) == HAL_OK)
      {
          OLED_Clear();

          OLED_SetCursor(
              0U,
              0U
          );

          OLED_WriteString(
              "ROS CAR"
          );

          OLED_SetCursor(
              0U,
              15U
          );

          OLED_WriteString(
              "STM32 BOOT OK"
          );

          OLED_SetCursor(
              0U,
              30U
          );

          OLED_WriteString(
              "WAIT ROCK 5C"
          );

          OLED_Update();
      }
  }


  /*
   * Continue your existing initialization below.
   */


  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
      Error_Handler();
  }

  if (JC_CAN_Init() != HAL_OK)
  {
      const char error_message[] =
          "[ERR] CAN init failed\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)error_message,
          sizeof(error_message) - 1U,
          100U
      );

      Error_Handler();
  }

  can_started = 1U;

  {
      const char startup_message[] =
          "[OK] CAN started @ 1Mbps\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)startup_message,
          sizeof(startup_message) - 1U,
          100U
      );
  }

  /*
   * Put both motors into idle first.
   */
  HAL_Delay(100U);
  JC_Idle(JC_NODE_LEFT);
  HAL_Delay(100U);
  JC_Idle(JC_NODE_RIGHT);
  HAL_Delay(100U);

  JC_EnterClosedLoop(JC_NODE_LEFT);
  HAL_Delay(100U);
  JC_EnterClosedLoop(JC_NODE_RIGHT);
  HAL_Delay(100U);

/*
 * Mode 5 = JC native low-speed high-torque mode.
 *
 * This mode uses register 0x0027 rather than
 * the normal velocity register 0x0021.
 */
if (JC_SetMode(
        JC_NODE_LEFT,
        JC_MODE_LOW_SPEED_TORQUE) != HAL_OK)
{
    Error_Handler();
}

HAL_Delay(100U);

if (JC_SetMode(
        JC_NODE_RIGHT,
        JC_MODE_LOW_SPEED_TORQUE) != HAL_OK)
{
    Error_Handler();
}

HAL_Delay(100U);

  /*
   * Safe startup.
   */
    (void)JC_SetLowSpeedRpm(
        JC_NODE_LEFT,
        0.0f
    );

    (void)JC_SetLowSpeedRpm(
        JC_NODE_RIGHT,
        0.0f
    );

  Robot_Stop();

  last_cmd_ms = HAL_GetTick();
  cmd_watchdog_active = 1U;

  HAL_UART_Receive_IT(
      &huart2,
      (uint8_t *)&uart_rx_byte,
      1U
  );

  {
      const char message[] =
          "\r\n"
          "=================================\r\n"
          " ROS CAR BASE CONTROLLER\r\n"
          "=================================\r\n"
          "JC mode : velocity closed loop\r\n"
          "CAN     : 1 Mbps\r\n"
          "Left    : Node 1\r\n"
          "Right   : Node 3\r\n"
          "\r\n"
          "ROS interface:\r\n"
          "V,<left_rpm_x100>,<right_rpm_x100>\r\n"
          "\r\n"
          "Debug:\r\n"
          "W/S/A/D/X\r\n"
          "=================================\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)message,
          sizeof(message) - 1U,
          200U
      );
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_motor_tick = 0U;
  uint32_t last_telemetry_ms = 0U;
  uint32_t last_debug_ms = 0U;
  uint32_t last_oled_ms = 0U;

  uint32_t last_left_rx_count = 0U;
  uint32_t last_right_rx_count = 0U;

  while (1)
  {
      uint32_t tick =
          app_tick;

      /*
       * 1. Safety
       */
      Robot_CheckWatchdog();


      /*
       * 2. Motor command @ 100 Hz
       */
      if ((uint32_t)(
              tick - last_motor_tick
          ) >= 10U)
      {
          last_motor_tick = tick;

          (void)JC_SetLowSpeedRpm(
              JC_NODE_LEFT,
              left_target_rpm
          );

          (void)JC_SetLowSpeedRpm(
              JC_NODE_RIGHT,
              right_target_rpm
          );
      }


      /*
       * 3. Formal telemetry @ 50 Hz
       */
      {
          uint32_t now =
              HAL_GetTick();

          if ((uint32_t)(
                  now - last_telemetry_ms
              ) >= TELEMETRY_PERIOD_MS)
          {
              last_telemetry_ms = now;

              Robot_SendTelemetry();
          }
      }

      /*
       * ============================================================
       * OLED STATUS @ 5 Hz
       * ============================================================
       */

      {
          uint32_t now =
              HAL_GetTick();

          if ((uint32_t)(
                  now -
                  last_oled_ms
              ) >= OLED_PERIOD_MS)
          {
              last_oled_ms = now;

              Robot_UpdateOLED();
          }
      }


      /*
       * 4. Human-readable debug @ 2 Hz
       */
      {
          uint32_t now =
              HAL_GetTick();

          if ((uint32_t)(
                  now - last_debug_ms
              ) >= 500U)
          {
              last_debug_ms = now;

              char buffer[192];

              int length = snprintf(
                  buffer,
                  sizeof(buffer),

                  "DBG,WD=%s "
                  "TARGET L=%7.2f R=%7.2f "
                  "ACT L=%7.2f R=%7.2f "
                  "RX=%lu/%lu\r\n",

                  cmd_watchdog_active ?
                      "STOP" :
                      "OK",

                  left_target_rpm,
                  right_target_rpm,

                  motor_left.vel_rpm,
                  motor_right.vel_rpm,

                  (unsigned long)
                      motor_left.rx_count,

                  (unsigned long)
                      motor_right.rx_count
              );

              if (length > 0)
              {
                  if ((size_t)length >
                      sizeof(buffer))
                  {
                      length =
                          sizeof(buffer);
                  }

                  HAL_UART_Transmit(
                      &huart2,
                      (uint8_t *)buffer,
                      (uint16_t)length,
                      50U
                  );
              }
          }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) &&
        (htim->Instance == TIM2))
    {
        app_tick++;

        if ((app_tick % 500U) == 0U)
        {
            HAL_GPIO_TogglePin(
                STATUS_LED_GPIO_Port,
                STATUS_LED_Pin
            );
        }
    }
}

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t c =
            uart_rx_byte;

        /*
         * Keep W/S/A/D/X as immediate
         * local debug commands.
         */
        if ((c == 'w') ||
            (c == 'W') ||
            (c == 's') ||
            (c == 'S') ||
            (c == 'a') ||
            (c == 'A') ||
            (c == 'd') ||
            (c == 'D') ||
            (c == 'x') ||
            (c == 'X') ||
            (c == ' '))
        {
            Robot_HandleSingleCommand(c);
        }
        else if ((c == '\r') ||
                 (c == '\n'))
        {
            /*
             * End of line.
             */
            if (uart_cmd_index > 0U)
            {
                uart_cmd_buffer[
                    uart_cmd_index
                ] = '\0';

                Robot_ProcessUartLine(
                    uart_cmd_buffer
                );

                uart_cmd_index = 0U;
            }
        }
        else
        {
            /*
             * Build line.
             */
            if (uart_cmd_index <
                (UART_CMD_BUFFER_SIZE - 1U))
            {
                uart_cmd_buffer[
                    uart_cmd_index++
                ] = (char)c;
            }
            else
            {
                /*
                 * Overflow -> discard line.
                 */
                uart_cmd_index = 0U;
            }
        }

        /*
         * Arm receive for next byte.
         */
        HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&uart_rx_byte,
            1U
        );
    }
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
