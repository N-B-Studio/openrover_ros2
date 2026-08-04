#ifndef INC_JC_MOTOR_H_
#define INC_JC_MOTOR_H_

#include "main.h"

#define JC_NODE_LEFT   1U   /* leader_gripper */
#define JC_NODE_RIGHT  3U   /* leader_wrist */

#define JC_MODE_TORQUE              0U
#define JC_MODE_VELOCITY            1U
#define JC_MODE_POSITION_TRAPEZOID  2U
#define JC_MODE_POSITION_FILTER     3U
#define JC_MODE_POSITION_DIRECT     4U
#define JC_MODE_LOW_SPEED_TORQUE    5U

typedef struct
{
    uint8_t node_id;

    float pos_deg;
    float vel_rpm;

    uint32_t rx_count;
    uint32_t last_rx_ms;

} JC_MotorState;

extern JC_MotorState motor_left;
extern JC_MotorState motor_right;

HAL_StatusTypeDef JC_CAN_Init(void);

HAL_StatusTypeDef JC_Idle(uint8_t node_id);
HAL_StatusTypeDef JC_EnterClosedLoop(uint8_t node_id);
HAL_StatusTypeDef JC_SetMode(uint8_t node_id, uint16_t mode);
HAL_StatusTypeDef JC_SetTorqueNm(uint8_t node_id, float torque_nm);
HAL_StatusTypeDef JC_SetVelocityRpm(uint8_t node_id, float velocity_rpm);
HAL_StatusTypeDef JC_SetLowSpeedRpm(uint8_t node_id,float velocity_rpm);

void JC_ParseFeedback(
    uint16_t rx_id,
    const uint8_t *data,
    uint8_t len
);


#endif
