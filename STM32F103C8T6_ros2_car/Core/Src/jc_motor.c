#include "jc_motor.h"

extern CAN_HandleTypeDef hcan;

#define JC_TORQUE_LIMIT_NM  0.10f

JC_MotorState motor_left =
{
    .node_id = JC_NODE_LEFT,
    .pos_deg = 0.0f,
    .vel_rpm = 0.0f,
    .rx_count = 0U,
    .last_rx_ms = 0U
};

JC_MotorState motor_right =
{
    .node_id = JC_NODE_RIGHT,
    .pos_deg = 0.0f,
    .vel_rpm = 0.0f,
    .rx_count = 0U,
    .last_rx_ms = 0U
};

static uint16_t JC_TxId(uint8_t node_id)
{
    return (uint16_t)(0x600U + node_id);
}

static float JC_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

static int16_t JC_ReadBigEndianS16(
    const uint8_t *data)
{
    uint16_t value =
        ((uint16_t)data[0] << 8) |
        ((uint16_t)data[1]);

    return (int16_t)value;
}

static int32_t JC_ReadBigEndianS24(
    const uint8_t *data)
{
    int32_t value =
        ((int32_t)data[0] << 16) |
        ((int32_t)data[1] << 8) |
        ((int32_t)data[2]);

    if ((value & 0x00800000L) != 0)
    {
        value |= (int32_t)0xFF000000L;
    }

    return value;
}

static HAL_StatusTypeDef JC_Send8(
    uint16_t standard_id,
    const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0U;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        return HAL_BUSY;
    }

    tx_header.StdId = standard_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(
        &hcan,
        &tx_header,
        (uint8_t *)data,
        &tx_mailbox
    );
}

HAL_StatusTypeDef JC_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    /*
     * POC stage:
     * Accept all CAN frames into FIFO0.
     */
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;
    filter.FilterMaskIdLow = 0x0000U;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(
            &hcan,
            &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(
            &hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef JC_Idle(uint8_t node_id)
{
    const uint8_t data[8] =
    {
        0x2B,
        0x00,
        0xA0,
        0x00,
        0x00,
        0x01,
        0x00,
        0x00
    };

    return JC_Send8(
        JC_TxId(node_id),
        data
    );
}

HAL_StatusTypeDef JC_EnterClosedLoop(
    uint8_t node_id)
{
    const uint8_t data[8] =
    {
        0x2B,
        0x00,
        0xA2,
        0x00,
        0x00,
        0x01,
        0x00,
        0x00
    };

    return JC_Send8(
        JC_TxId(node_id),
        data
    );
}

HAL_StatusTypeDef JC_SetMode(
    uint8_t node_id,
    uint16_t mode)
{
    uint8_t data[8] =
    {
        0x2B,
        0x00,
        0x60,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };

    data[4] =
        (uint8_t)((mode >> 8) & 0xFFU);

    data[5] =
        (uint8_t)(mode & 0xFFU);

    return JC_Send8(
        JC_TxId(node_id),
        data
    );
}

HAL_StatusTypeDef JC_SetTorqueNm(
    uint8_t node_id,
    float torque_nm)
{
    int16_t raw_torque;

    uint8_t data[8] =
    {
        0x2B,
        0x00,
        0x20,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };

    torque_nm = JC_ClampFloat(
        torque_nm,
        -JC_TORQUE_LIMIT_NM,
        JC_TORQUE_LIMIT_NM
    );

    raw_torque =
        (int16_t)(torque_nm * 100.0f);

    data[4] =
        (uint8_t)(((uint16_t)raw_torque >> 8) & 0xFFU);

    data[5] =
        (uint8_t)((uint16_t)raw_torque & 0xFFU);

    return JC_Send8(
        JC_TxId(node_id),
        data
    );
}

HAL_StatusTypeDef JC_SetVelocityRpm(
    uint8_t node_id,
    float velocity_rpm)
{
    /*
     * JC velocity register:
     *
     * Register : 0x0021
     * Command  : 0x23
     * Format   : signed int32
     * Scale    : rpm * 100
     *
     * Example:
     *
     * +500 rpm
     * 50000 = 0x0000C350
     *
     * 23 00 21 00 00 00 C3 50
     */

    int32_t velocity_raw;

    uint8_t data[8] =
    {
        0x23,
        0x00,
        0x21,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };

    /*
     * Temporary robot safety limit.
     *
     * This is NOT the JC motor maximum.
     * It is our robot firmware limit.
     */
    if (velocity_rpm > 150.0f)
    {
        velocity_rpm = 150.0f;
    }

    if (velocity_rpm < -150.0f)
    {
        velocity_rpm = -150.0f;
    }

    velocity_raw =
        (int32_t)(velocity_rpm * 100.0f);

    data[4] =
        (uint8_t)(
            ((uint32_t)velocity_raw >> 24) &
            0xFFU
        );

    data[5] =
        (uint8_t)(
            ((uint32_t)velocity_raw >> 16) &
            0xFFU
        );

    data[6] =
        (uint8_t)(
            ((uint32_t)velocity_raw >> 8) &
            0xFFU
        );

    data[7] =
        (uint8_t)(
            (uint32_t)velocity_raw &
            0xFFU
        );

    return JC_Send8(
        JC_TxId(node_id),
        data
    );
}

void JC_ParseFeedback(
    uint16_t rx_id,
    const uint8_t *data,
    uint8_t len)
{
    JC_MotorState *motor = NULL;

    if ((data == NULL) || (len != 8U))
    {
        return;
    }

    /*
     * Node 1:
     * leader_gripper → left wheel
     *
     * Node 3:
     * leader_wrist → right wheel
     */
    if (rx_id ==
        (uint16_t)(0x580U + JC_NODE_LEFT))
    {
        motor = &motor_left;
    }
    else if (rx_id ==
             (uint16_t)(0x580U + JC_NODE_RIGHT))
    {
        motor = &motor_right;
    }
    else
    {
        return;
    }

    if (data[0] == 0x2AU)
    {
        int32_t position_raw =
            JC_ReadBigEndianS24(&data[1]);

        int16_t velocity_raw =
            JC_ReadBigEndianS16(&data[4]);

        motor->pos_deg =
            (float)position_raw / 100.0f;

        motor->vel_rpm =
            (float)velocity_raw / 100.0f;

        motor->rx_count++;

        motor->last_rx_ms =
            HAL_GetTick();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan_callback)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if ((hcan_callback == NULL) ||
        (hcan_callback->Instance != CAN1))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(
               hcan_callback,
               CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(
                hcan_callback,
                CAN_RX_FIFO0,
                &rx_header,
                rx_data) != HAL_OK)
        {
            break;
        }

        if (rx_header.IDE != CAN_ID_STD)
        {
            continue;
        }

        JC_ParseFeedback(
            (uint16_t)rx_header.StdId,
            rx_data,
            (uint8_t)rx_header.DLC
        );
    }
}
