#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/joint_state.h>
#include <uxr/client/transport.h>

#define PI_F                         3.14159265358979323846f
#define WHEEL_DIAMETER_M             0.072f
#define TRACK_WIDTH_M                0.155f
#define PWM_PERIOD_COUNTS            6400U
#define MOTOR_STOP_DUTY_PERMILLE     1000U
#define MOTOR_MIN_RPM                6.5f
#define MOTOR_MAX_RPM                30.0f
#define MOTOR_DIRECTION_DELAY_MS     300U
#define MOTOR_COMMAND_TIMEOUT_MS     200U
#define WHEEL_STATE_PERIOD_MS        100U
#define TIME_SYNC_RETRY_MS           10000U

#define FG_PPR_FL                    1121.1f
#define FG_PPR_RL                    1123.5f
#define FG_PPR_FR                    1173.4f
#define FG_PPR_RR                    1174.1f

#define LEFT_FORWARD_LEVEL           GPIO_PIN_SET
#define LEFT_REVERSE_LEVEL           GPIO_PIN_RESET
#define RIGHT_FORWARD_LEVEL          GPIO_PIN_RESET
#define RIGHT_REVERSE_LEVEL          GPIO_PIN_SET

typedef enum
{
    WHEEL_FL = 0,
    WHEEL_RL,
    WHEEL_FR,
    WHEEL_RR,
    WHEEL_COUNT
} WheelIndex;

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

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

static volatile uint32_t fg_total[WHEEL_COUNT];
static volatile uint32_t fg_window[WHEEL_COUNT];
static volatile int8_t motor_direction[2];

static MotorState motor_state = MOTOR_STATE_READY;
static int8_t pending_direction[2];
static uint16_t pending_duty[2] = {1000U, 1000U};
static uint32_t direction_change_started_ms;
static uint32_t last_motor_command_ms;
static uint8_t motor_watchdog_enabled;
static uint8_t ros_time_synchronized;

static const float fg_ppr[WHEEL_COUNT] =
{
    FG_PPR_FL, FG_PPR_RL, FG_PPR_FR, FG_PPR_RR
};

static const MotorCalibrationPoint left_forward_curve[] =
{
    {5.3f,970U}, {6.8f,960U}, {8.9f,950U}, {14.2f,925U},
    {20.5f,900U}, {25.9f,875U}, {31.1f,850U}
};
static const MotorCalibrationPoint right_forward_curve[] =
{
    {6.4f,970U}, {8.5f,960U}, {10.5f,950U}, {15.9f,925U},
    {21.6f,900U}, {26.6f,875U}, {31.7f,850U}
};
static const MotorCalibrationPoint left_reverse_curve[] =
{
    {5.7f,970U}, {7.8f,960U}, {10.0f,950U}, {15.2f,925U},
    {21.6f,900U}, {26.9f,875U}, {32.2f,850U}
};
static const MotorCalibrationPoint right_reverse_curve[] =
{
    {5.5f,970U}, {7.5f,960U}, {9.5f,950U}, {14.8f,925U},
    {20.6f,900U}, {25.6f,875U}, {30.7f,850U}
};

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t cmd_vel_subscription;
static rcl_publisher_t wheel_state_publisher;
static geometry_msgs__msg__Twist cmd_vel_message;
static sensor_msgs__msg__JointState wheel_state_message;
static double wheel_position[WHEEL_COUNT];
static double wheel_velocity[WHEEL_COUNT];
static rosidl_runtime_c__String wheel_names[WHEEL_COUNT];

static const char *const wheel_name_text[WHEEL_COUNT] =
{
    "front_left_wheel_joint", "rear_left_wheel_joint",
    "front_right_wheel_joint", "rear_right_wheel_joint"
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
    const uint8_t *buffer, size_t length, uint8_t *error);
size_t cubemx_transport_read(struct uxrCustomTransport *transport,
    uint8_t *buffer, size_t length, int timeout_ms, uint8_t *error);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t count, size_t size, void *state);

static float ClampFloat(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static uint32_t Motor_DutyToCompare(uint16_t duty)
{
    if (duty > 1000U) duty = 1000U;
    if ((duty < 20U) && (duty != 1000U)) duty = 20U;
    return ((uint32_t)duty * PWM_PERIOD_COUNTS) / 1000U;
}

static void Motor_SetDuty(uint16_t left, uint16_t right)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Motor_DutyToCompare(left));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Motor_DutyToCompare(right));
}

static void Motor_OutputStop(void)
{
    Motor_SetDuty(MOTOR_STOP_DUTY_PERMILLE, MOTOR_STOP_DUTY_PERMILLE);
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

static void Motor_SetDirection(int8_t left, int8_t right)
{
    if (left > 0) HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, LEFT_FORWARD_LEVEL);
    else if (left < 0) HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, LEFT_REVERSE_LEVEL);
    if (right > 0) HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, RIGHT_FORWARD_LEVEL);
    else if (right < 0) HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, RIGHT_REVERSE_LEVEL);
    if (left != 0) motor_direction[0] = left;
    if (right != 0) motor_direction[1] = right;
}

static uint16_t Motor_InterpolateDuty(float rpm,
    const MotorCalibrationPoint *curve, uint32_t count)
{
    if (rpm <= curve[0].rpm) return curve[0].duty_permille;
    for (uint32_t i = 1U; i < count; ++i)
    {
        if (rpm <= curve[i].rpm)
        {
            float ratio = (rpm - curve[i-1U].rpm) /
                (curve[i].rpm - curve[i-1U].rpm);
            return (uint16_t)((float)curve[i-1U].duty_permille + ratio *
                ((float)curve[i].duty_permille -
                 (float)curve[i-1U].duty_permille) + 0.5f);
        }
    }
    return curve[count-1U].duty_permille;
}

static uint16_t Motor_RPMToDuty(float rpm, bool left)
{
    const MotorCalibrationPoint *curve;
    uint32_t count;
    float magnitude = rpm < 0.0f ? -rpm : rpm;
    magnitude = ClampFloat(magnitude, MOTOR_MIN_RPM, MOTOR_MAX_RPM);
    if (left)
    {
        curve = rpm > 0.0f ? left_forward_curve : left_reverse_curve;
        count = rpm > 0.0f ?
            sizeof(left_forward_curve)/sizeof(left_forward_curve[0]) :
            sizeof(left_reverse_curve)/sizeof(left_reverse_curve[0]);
    }
    else
    {
        curve = rpm > 0.0f ? right_forward_curve : right_reverse_curve;
        count = rpm > 0.0f ?
            sizeof(right_forward_curve)/sizeof(right_forward_curve[0]) :
            sizeof(right_reverse_curve)/sizeof(right_reverse_curve[0]);
    }
    return Motor_InterpolateDuty(magnitude, curve, count);
}

static void Motor_SetWheelRPM(float left_rpm, float right_rpm)
{
    int8_t direction[2] = {
        left_rpm > 0.0f ? 1 : (left_rpm < 0.0f ? -1 : 0),
        right_rpm > 0.0f ? 1 : (right_rpm < 0.0f ? -1 : 0)
    };
    uint16_t duty[2] = {
        direction[0] == 0 ? 1000U : Motor_RPMToDuty(left_rpm, true),
        direction[1] == 0 ? 1000U : Motor_RPMToDuty(right_rpm, false)
    };

    last_motor_command_ms = HAL_GetTick();
    motor_watchdog_enabled = 1U;
    if ((direction[0] == 0) && (direction[1] == 0))
    {
        Motor_Stop();
        return;
    }
    pending_direction[0] = direction[0];
    pending_direction[1] = direction[1];
    pending_duty[0] = duty[0];
    pending_duty[1] = duty[1];
    if (motor_state == MOTOR_STATE_WAIT_DIRECTION) return;
    if (((direction[0] != 0) && (direction[0] != motor_direction[0])) ||
        ((direction[1] != 0) && (direction[1] != motor_direction[1])))
    {
        Motor_OutputStop();
        direction_change_started_ms = HAL_GetTick();
        motor_state = MOTOR_STATE_WAIT_DIRECTION;
        return;
    }
    Motor_SetDirection(direction[0], direction[1]);
    Motor_SetDuty(duty[0], duty[1]);
}

static void Motor_Update(void)
{
    uint32_t now = HAL_GetTick();
    if (motor_watchdog_enabled &&
        ((now - last_motor_command_ms) > MOTOR_COMMAND_TIMEOUT_MS))
    {
        Motor_Stop();
        motor_watchdog_enabled = 0U;
        return;
    }
    if ((motor_state == MOTOR_STATE_WAIT_DIRECTION) &&
        ((now - direction_change_started_ms) >= MOTOR_DIRECTION_DELAY_MS))
    {
        Motor_SetDirection(pending_direction[0], pending_direction[1]);
        Motor_SetDuty(pending_duty[0], pending_duty[1]);
        motor_state = MOTOR_STATE_READY;
    }
}

static void UpdateHeaderStamp(void)
{
    if (!ros_time_synchronized) return;
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    if (epoch_ns <= 0) return;
    wheel_state_message.header.stamp.sec = (int32_t)(epoch_ns / 1000000000LL);
    wheel_state_message.header.stamp.nanosec =
        (uint32_t)(epoch_ns % 1000000000LL);
}

static void CmdVel_Callback(const void *input)
{
    const geometry_msgs__msg__Twist *msg = input;
    float linear = ClampFloat((float)msg->linear.x, -0.1131f, 0.1131f);
    float angular = ClampFloat((float)msg->angular.z, -1.45f, 1.45f);
    float left_mps = linear - angular * TRACK_WIDTH_M * 0.5f;
    float right_mps = linear + angular * TRACK_WIDTH_M * 0.5f;
    float rpm_factor = 60.0f / (PI_F * WHEEL_DIAMETER_M);
    float left_rpm = left_mps * rpm_factor;
    float right_rpm = right_mps * rpm_factor;
    float scale = 1.0f;
    float maximum = left_rpm < 0.0f ? -left_rpm : left_rpm;
    float right_abs = right_rpm < 0.0f ? -right_rpm : right_rpm;
    if (right_abs > maximum) maximum = right_abs;
    if (maximum > MOTOR_MAX_RPM) scale = MOTOR_MAX_RPM / maximum;
    Motor_SetWheelRPM(left_rpm * scale, right_rpm * scale);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
        (left_rpm == 0.0f && right_rpm == 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void FG_UpdateAndPublish(uint32_t elapsed_ms)
{
    uint32_t pulses[WHEEL_COUNT];
    int8_t direction[WHEEL_COUNT];
    __disable_irq();
    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        pulses[i] = fg_window[i];
        fg_window[i] = 0U;
    }
    direction[WHEEL_FL] = motor_direction[0];
    direction[WHEEL_RL] = motor_direction[0];
    direction[WHEEL_FR] = motor_direction[1];
    direction[WHEEL_RR] = motor_direction[1];
    __enable_irq();

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        double delta = ((double)pulses[i] * 2.0 * (double)PI_F /
            (double)fg_ppr[i]) * (double)direction[i];
        wheel_position[i] += delta;
        wheel_velocity[i] = delta * 1000.0 / (double)elapsed_ms;
    }
    UpdateHeaderStamp();
    (void)rcl_publish(&wheel_state_publisher, &wheel_state_message, NULL);
}

static bool MicroROS_Init(void)
{
    if (rmw_uros_set_custom_transport(true, &huart1,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read) != RMW_RET_OK) return false;

    allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&allocator)) return false;
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    ros_time_synchronized =
        (rmw_uros_sync_session(1000U) == RMW_RET_OK) ? 1U : 0U;
    node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, "g431_rover_base", "", &support) != RCL_RET_OK) return false;

    cmd_vel_subscription = rcl_get_zero_initialized_subscription();
    wheel_state_publisher = rcl_get_zero_initialized_publisher();
    if (rclc_subscription_init_default(&cmd_vel_subscription, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) return false;
    if (rclc_publisher_init_default(&wheel_state_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/wheel_states") != RCL_RET_OK) return false;

    memset(&cmd_vel_message, 0, sizeof(cmd_vel_message));
    memset(&wheel_state_message, 0, sizeof(wheel_state_message));
    wheel_state_message.position.data = wheel_position;
    wheel_state_message.position.size = WHEEL_COUNT;
    wheel_state_message.position.capacity = WHEEL_COUNT;
    wheel_state_message.velocity.data = wheel_velocity;
    wheel_state_message.velocity.size = WHEEL_COUNT;
    wheel_state_message.velocity.capacity = WHEEL_COUNT;
    wheel_state_message.name.data = wheel_names;
    wheel_state_message.name.size = WHEEL_COUNT;
    wheel_state_message.name.capacity = WHEEL_COUNT;
    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        if (!rosidl_runtime_c__String__init(&wheel_names[i])) return false;
        if (!rosidl_runtime_c__String__assign(&wheel_names[i], wheel_name_text[i])) return false;
    }
    if (!rosidl_runtime_c__String__init(
        &wheel_state_message.header.frame_id)) return false;
    if (!rosidl_runtime_c__String__assign(
        &wheel_state_message.header.frame_id, "base_link")) return false;

    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &cmd_vel_subscription,
        &cmd_vel_message, CmdVel_Callback, ON_NEW_DATA) != RCL_RET_OK) return false;
    return true;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    Motor_SetDirection(1, 1);
    Motor_Stop();
    HAL_Delay(2000U);
    if (!MicroROS_Init()) Error_Handler();

    uint32_t last_wheel_ms = HAL_GetTick();
    uint32_t last_time_sync_ms = HAL_GetTick();
    while (1)
    {
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        Motor_Update();
        uint32_t now = HAL_GetTick();
        if ((now - last_wheel_ms) >= WHEEL_STATE_PERIOD_MS)
        {
            uint32_t elapsed = now - last_wheel_ms;
            last_wheel_ms = now;
            FG_UpdateAndPublish(elapsed);
        }
        if (!ros_time_synchronized &&
            ((now - last_time_sync_ms) >= TIME_SYNC_RETRY_MS))
        {
            last_time_sync_ms = now;
            ros_time_synchronized =
                (rmw_uros_sync_session(100U) == RMW_RET_OK) ? 1U : 0U;
        }
        HAL_Delay(1U);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = RCC_PLLM_DIV2;
    osc.PLL.PLLN = 40;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 6399;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK) Error_Handler();
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim2);
}

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
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, DIR_L_Pin | DIR_R_Pin, GPIO_PIN_RESET);
    gpio.Pin = LED_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &gpio);
    gpio.Pin = DIR_L_Pin | DIR_R_Pin;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = FG_FL_Pin | FG_RL_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = FG_RR_Pin | FG_FR_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    WheelIndex wheel;
    if (pin == FG_FL_Pin) wheel = WHEEL_FL;
    else if (pin == FG_RL_Pin) wheel = WHEEL_RL;
    else if (pin == FG_FR_Pin) wheel = WHEEL_FR;
    else if (pin == FG_RR_Pin) wheel = WHEEL_RR;
    else return;
    fg_total[wheel]++;
    fg_window[wheel]++;
}

void Error_Handler(void)
{
    if (htim2.Instance == TIM2)
    {
        Motor_OutputStop();
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    __disable_irq();
    while (1) {}
}
