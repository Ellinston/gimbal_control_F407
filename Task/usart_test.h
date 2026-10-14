#ifndef TASK_USART_TEST_H
#define TASK_USART_TEST_H

typedef struct {
    float pitch_angle;    /* Pitch，deg；标定未完成/数据无效为 NaN。 */
    float yaw_angle;      /* 相对 Yaw，deg；标定后归零。 */
    float accel_x;        /* BMI270 X轴加速度，m/s^2，未滤波物理量。 */
    float accel_y;        /* BMI270 Y轴加速度，m/s^2，未滤波物理量。 */
} imu_vofa_data_t;

/* IMU_Angle_Test 更新，VOFA 任务只取快照，不重复读取传感器。 */
extern imu_vofa_data_t imu_vofa_data;

typedef struct {
    float target_angle;   /* 目标角度，deg，沿用位置环临时参考。 */
    float current_angle;  /* 当前角度，deg；未建立参考/反馈过期为 NaN。 */
    float target_speed;   /* 目标速度，rpm；来自速度给定或位置外环。 */
    float current_speed;  /* 当前速度，rpm；无有效反馈为 NaN。 */
    float velocity_ff_output; /* 前馈分量，GM6020电压控制码，最终限幅前。 */
    float speed_pid_output;   /* PID输出分量，GM6020电压控制码，求和前。 */
} motor_channel_t;

/* 电机测试更新；正弦速度模式使用YAW速度与PID/前馈分量。 */
extern motor_channel_t yaw_channel;
extern motor_channel_t pitch_channel;

/* USART1 初始化后调用；不启动串口接收。 */
void vofa_task_init(void);
/* 正弦速度测试保留八路：CH0 Pitch、CH1 Yaw、CH2 Acc X、CH3 Acc Y；
 * CH4目标rpm、CH5实际rpm、CH6前馈、CH7 PID。 */
void vofa_task_enable_yaw_speed(void);
/* 位置测试切回默认六路诊断映射。 */
void vofa_task_enable_motor_position(void);
/* 每100ms非阻塞JustFloat：默认CH0 IMU Yaw、CH1 IMU Pitch、
 * CH2编码器Yaw、CH3编码器Pitch（deg）；CH4 Yaw输出、CH5 Pitch输出（电压控制码）。
 * IMU未就绪/失效、编码器无参考/过期的角度为NaN。
 * 末尾00 00 80 7F，六路28字节；正弦速度模式八路36字节。
 * 串口忙/失败时丢弃本周期，不补发积压数据；不可混调文本输出任务。 */
void vofa_task_update(void);

/**
 * @brief 临时串口测试任务：收到 PING 加 LF 换行后，回复 PONG 加 LF。
 * @note 在 USART1 初始化后，由主循环反复调用；内部负责启动/恢复接收。
 *       每次只处理一小批数据，不等待接收或发送完成。
 */
void USART_PingTest(void);

#endif /* TASK_USART_TEST_H */

