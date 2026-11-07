#ifndef TASK_USART_TEST_H
#define TASK_USART_TEST_H

typedef struct {
    float pitch_angle;    /* Pitch，deg；标定未完成/数据无效为 NaN。 */
    float yaw_angle;      /* 相对 Yaw，deg；标定后归零。 */
    float accel_x;        /* BMI270 X轴加速度，m/s^2，未滤波物理量。 */
    float accel_y;        /* BMI270 Y轴加速度，m/s^2，未滤波物理量。 */
} imu_vofa_data_t;

/* IMU_Angle_Test 的旧快照接口；当前 M3508 VOFA 不读取这些数据。 */
extern imu_vofa_data_t imu_vofa_data;

typedef struct {
    float target_angle;   /* 目标角度，deg，沿用位置环临时参考。 */
    float current_angle;  /* 当前角度，deg；未建立参考/反馈过期为 NaN。 */
    float target_speed;   /* 目标速度，rpm；来自速度给定或位置外环。 */
    float current_speed;  /* 当前速度，rpm；无有效反馈为 NaN。 */
    float velocity_ff_output; /* 前馈分量，GM6020电压控制码，最终限幅前。 */
    float speed_pid_output;   /* PID输出分量，GM6020电压控制码，求和前。 */
} motor_channel_t;

/* 旧电机测试仍更新这些快照；当前 M3508 VOFA 不输出它们。 */
extern motor_channel_t yaw_channel;
extern motor_channel_t pitch_channel;

/* USART1 接收完整 STOP\n 或 STOP\r\n 后，直接调用 M3508_Task_Stop()。
 * 主循环在电机 Update 前调用；内部启动/恢复接收，每轮最多处理32字节。
 * 支持分批/多行接收；异常或超长行丢弃至LF。不依赖CAN/VOFA发送状态。
 * 不回复额外文本；停机保持到电机任务重新初始化，不保存到Flash。
 * 与其他消费 usart_Rx 的任务（如 PING 测试）不能同时调度。 */
void usart_send_STOP(void);

/* USART1 初始化后调用；不启动串口接收。 */
void vofa_task_init(void);
/* 兼容旧任务的入口，当前为空操作，不切换输出通道。 */
void vofa_task_enable_yaw_speed(void);
/* 兼容旧任务的入口，当前为空操作，不切换输出通道。 */
void vofa_task_enable_motor_position(void);
/* 主循环先更新 M3508，再调用本函数；每100ms通过USART1发送FireWater：
 * M3508:<fixed_output>,<output_before_limit>\n
 * CH0 固定电流偏置；CH1 P+I+偏置（最终限幅前），均为C620电流控制码。
 * CAN初始化成功、收到过有效反馈、串口接收启动且发送空闲后，
 * 仅在 m3508_status.state != M3508_STOPPED 时提交新包。
 * 串口忙/失败时跳过本周期，不补发；不混调 PING/PONG 等输出任务。
 * 本函数不解析电脑 STOP 命令，停机状态由 M3508 任务维护。 */
void vofa_task_update(void);

/**
 * @brief 临时串口测试任务：收到 PING 加 LF 换行后，回复 PONG 加 LF。
 * @note 在 USART1 初始化后，由主循环反复调用；内部负责启动/恢复接收。
 *       每次只处理一小批数据，不等待接收或发送完成。
 */
void USART_PingTest(void);

#endif /* TASK_USART_TEST_H */

