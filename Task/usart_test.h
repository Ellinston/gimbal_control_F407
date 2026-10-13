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
    float target_speed;   /* 目标速度，rpm；当前直接位置环不生成速度目标。 */
    float current_speed;  /* 当前速度，rpm；无有效反馈为 NaN。 */
} motor_channel_t;

/* 保留电机任务的数据接口；当前 IMU 模式不发送这两组数据。 */
extern motor_channel_t yaw_channel;
extern motor_channel_t pitch_channel;

/* USART1 初始化后调用；不启动串口接收。 */
void vofa_task_init(void);
/* 非阻塞 JustFloat：Pitch、Yaw、Acc X、Acc Y，末尾00 00 80 7F。
 * 串口忙/失败时丢弃本周期，不补发积压数据；不可混调文本输出任务。 */
void vofa_task_update(void);

/**
 * @brief 临时串口测试任务：收到 PING 加 LF 换行后，回复 PONG 加 LF。
 * @note 在 USART1 初始化后，由主循环反复调用；内部负责启动/恢复接收。
 *       每次只处理一小批数据，不等待接收或发送完成。
 */
void USART_PingTest(void);

#endif /* TASK_USART_TEST_H */

