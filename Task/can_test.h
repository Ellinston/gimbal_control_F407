#ifndef TASK_CAN_TEST_H
#define TASK_CAN_TEST_H

#include "can.h"

/* 设备拨码编号、协议 ID 和控制槽位均只在 Task 中定义。 */
#define CAN_TEST_YAW_MOTOR_ID 1U
#define CAN_TEST_PITCH_MOTOR_ID 2U
#define CAN_TEST_YAW_FEEDBACK_ID (0x204U + CAN_TEST_YAW_MOTOR_ID)
#define CAN_TEST_PITCH_FEEDBACK_ID (0x204U + CAN_TEST_PITCH_MOTOR_ID)
#define CAN_TEST_CONTROL_ID 0x1FFU

typedef struct
{
    uint8_t data[CAN_DATA_CAPACITY];
    uint8_t length;
    uint8_t received;
    uint32_t frame_id;
    uint32_t updates;
    uint32_t last_rx_ms;
} CAN_TestRxBuffer;

extern CAN_TestRxBuffer yaw_rx_buffer;
extern CAN_TestRxBuffer pitch_rx_buffer;
extern HAL_StatusTypeDef can_test_init_status;

void CAN_TestInit(void);
void CAN_Test(void);

#endif /* TASK_CAN_TEST_H */
