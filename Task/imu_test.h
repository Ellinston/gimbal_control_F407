#ifndef TASK_IMU_TEST_H
#define TASK_IMU_TEST_H

#include <stdint.h>

/* 默认保留独立测试的文本日志；VOFA 模式在进入循环前传0，禁止混流。 */
void IMU_TestSetTextOutput(uint8_t enabled);

/* 外设初始化后调用一次，准备 SPI 通信并启用 DWT。 */
void IMU_TestInit(void);

/* 读取两条驱动接口的共享 CHIP_ID，并通过 usart_Tx 打印。
 * 先调用 IMU_TestInit；单次尝试，串口忙时立即返回。 */
void Task_IMU_Read_ID(void);

/* 首次调用完整初始化，之后仅回读配置并打印；失败需复位后重试。
 * 包含阻塞初始化过程，仅用于当前独立测试任务。 */
void Task_IMU_Init_Test(void);

/* 原始物理量测试，保留供单独切换验证；不要与角度测试同时调度。 */
void Task_IMU_Data_Test(void);

/* 二轴卡尔曼姿态测试：主循环反复调用，上电静止约 2 秒标定。
 * Pitch 为重力参考角，Yaw 为标定后归零的相对角，单位 deg。
 * 默认芯片 X 向前、Y 为 Pitch 轴、水平时 Z 向上；假设无 Roll。
 * 更新 imu_vofa_data；文本启用时每0.5秒输出一行。
 * 首次包含初始化检查，异常不输出旧角度。 */
void IMU_Angle_Test(void);

/* 主循环反复调用，当前调度 IMU_Angle_Test。 */
void IMU_Test(void);

#endif /* TASK_IMU_TEST_H */

