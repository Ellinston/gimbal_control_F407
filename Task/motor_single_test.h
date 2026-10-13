#ifndef TASK_MOTOR_SINGLE_TEST_H
#define TASK_MOTOR_SINGLE_TEST_H

#include "motor.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_SINGLE_COMMAND 1000
#define MOTOR_SINGLE_TX_PERIOD_MS 10U

typedef struct
{
    HAL_StatusTypeDef init_status, rx_status, tx_status;
    int16_t command, submitted_command;
    uint32_t tx_submitted, log_dropped;
} MotorSingle_Status;

extern Motor single_yaw_motor;
extern MotorSingle_Status motor_single_status;
HAL_StatusTypeDef Task_Motor_Single_Test_Init(void);
/* 上电即每10ms发送固定控制码1000，其他槽为零；反馈只用于串口显示。
 * 每次提交后排队打印实际ID+DATA、控制值、转速、角度与反馈龄期。
 * 无阶梯加档、串口启动/停止、断联/行程停机。 */
void Task_Motor_Single_Test(void);

#ifdef __cplusplus
}
#endif
#endif
