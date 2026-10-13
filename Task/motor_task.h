#ifndef TASK_MOTOR_TASK_H
#define TASK_MOTOR_TASK_H

#include "motor.h"
#include "task_interface.h"

/* ================= 两轴调参区（修改后重新编译、烧录）=================
 * 每个轴的参数和 PID 状态独立。当前位置任务使用位置环，直接输出电压码。
 * Kp/Ki/Kd 沿用原值，由用户分别整定；输出与积分限幅单位均为原始控制码。
 * 目标角度沿用启动参考：YAW=0°，PITCH=100°。 */

/* YAW：位置环 PID 与固定目标角度。 */
#define YAW_POSITION_KP              40.0f
#define YAW_POSITION_KI              0.0f
#define YAW_POSITION_KD              0.0f
#define YAW_POSITION_MAX_OUTPUT      1500.0f
#define YAW_POSITION_INTEGRAL_LIMIT  1500.0f
#define YAW_TARGET_ANGLE_DEG         30.0f

/* PITCH：位置环 PID 与固定目标角度。 */
#define PITCH_POSITION_KP            40.0f
#define PITCH_POSITION_KI            0.0f
#define PITCH_POSITION_KD            0.0f
#define PITCH_POSITION_MAX_OUTPUT    1500.0f
#define PITCH_POSITION_INTEGRAL_LIMIT 1500.0f
#define PITCH_TARGET_ANGLE_DEG       70.0f

/* 独立速度环：仅 Motor_Set_Speed 模式使用，当前位置任务不经过速度环。 */
#define YAW_SPEED_KP                 100.0f
#define YAW_SPEED_KI                 40.0f
#define YAW_SPEED_KD                 0.0f
#define YAW_SPEED_MAX_OUTPUT         1500.0f
#define YAW_SPEED_INTEGRAL_LIMIT     1500.0f

#define PITCH_SPEED_KP               100.0f
#define PITCH_SPEED_KI               40.0f
#define PITCH_SPEED_KD               0.0f
#define PITCH_SPEED_MAX_OUTPUT       1500.0f
#define PITCH_SPEED_INTEGRAL_LIMIT   1500.0f
/* ======================= 调参区结束 ======================= */

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_TASK_YAW_ID 1U
#define MOTOR_TASK_PITCH_ID 2U
#define MOTOR_TASK_TX_PERIOD_MS 10U
#define MOTOR_TASK_MAX_RUN_MS 5000U /* 初期限时开环接口，非实机验收参数。 */

typedef enum
{
    MOTOR_TASK_STOPPED = 0,
    MOTOR_TASK_RUNNING,
    MOTOR_TASK_DURATION_END,
    MOTOR_TASK_FEEDBACK_LOST,
    MOTOR_TASK_RX_ERROR,
    MOTOR_TASK_TX_ERROR
} MotorTask_State;

typedef struct
{
    uint8_t initialized;
    MotorTask_State state;
    HAL_StatusTypeDef init_status;
    HAL_StatusTypeDef tx_status;
    HAL_StatusTypeDef yaw_rx_status;
    HAL_StatusTypeDef pitch_rx_status;
} MotorTask_Status;

/* 以下对象供主循环/调试器读取；业务代码通过接口修改任务状态。 */
extern Motor yaw_motor;
extern Motor pitch_motor;
extern MotorTask_Status motor_task_status;

/* 在 MX_CAN1_Init 后调用：启动 CAN，初始化两轴，并提交全零帧。
 * HAL_BUSY 表示零帧暂未提交，initialized=1 时 Run 会继续重试。 */
HAL_StatusTypeDef motor_task_Init(void);

/* 限时开环电压给定，不是速度/角度目标，不包含 PID。
 * 两轴均须有新鲜反馈；duration_ms 为 1..MOTOR_TASK_MAX_RUN_MS。
 * 首次实机需确认电压模式、机构固定和小输出值；Device 发送前再限幅到 ±3000。
 * 只登记命令，实际发送由 Run 完成；运行中再次 Start 返回 HAL_BUSY。 */
HAL_StatusTypeDef motor_task_Start(int16_t yaw_voltage_command,
                                  int16_t pitch_voltage_command,
                                  uint32_t duration_ms);

/* 立即清除两轴命令并尝试提交全零帧，失败时 Run 继续重试。
 * 不关闭 CAN，继续接收反馈；HAL_OK 不等于实机已停止。 */
HAL_StatusTypeDef motor_task_Stop(void);

/* 在 while(1) 中持续调用，非阻塞：收反馈、查超时、每 10ms 发送整组。
 * 停止/故障后持续发送零帧，恢复反馈不会自动重新启动。
 * 不可与 CAN_Test 同时调度：两者会争用同一反馈和控制帧。 */
void motor_task_Run(void);

typedef enum
{
    DOUBLE_MOTOR_WAIT_FEEDBACK = 0,
    DOUBLE_MOTOR_RUNNING,
    DOUBLE_MOTOR_STOPPED,
    DOUBLE_MOTOR_FEEDBACK_LOST,
    DOUBLE_MOTOR_RX_ERROR,
    DOUBLE_MOTOR_TX_ERROR,
    DOUBLE_MOTOR_ANGLE_LIMIT,
    DOUBLE_MOTOR_CONTROL_LATE
} DoubleMotor_State;

typedef struct
{
    uint8_t initialized;
    DoubleMotor_State state;
    HAL_StatusTypeDef init_status, tx_status, log_status;
    float yaw_target_deg, pitch_target_deg;
    int8_t yaw_direction, pitch_direction;
    uint32_t tx_submitted, log_submitted, log_dropped;
} DoubleMotor_Status;

extern DoubleMotor_Status double_motor_status;
extern const Task_Interface_t Double_Motor_Task;
/* 双轴位置测试：YAW 0↔180°，PITCH 100°起步，在55↔145°往返。
 * 目标每10ms步进1°；到达端点后等待实际角度及转速满足条件再反向。
 * 仅此任务占用 CAN1/USART1；故障与 Stop 锁止，重新 Init 才可恢复。
 * 主循环持续调用以收反馈，控制内部10ms门控；日志100ms异步输出。 */
void Task_Double_Motor_Test_Init(void);
void Task_Double_Motor_Test(void);
void Task_Double_Motor_Test_Stop(void);

/* 固定目标位置环：角度取本文件顶部调参区；与往返任务互斥。
 * 复用 double_motor_status、10ms控制、100ms异步日志和锁止保护。
 * 必须先 Init，再于主循环持续调用 Test；Stop 后须重新 Init。 */
extern const Task_Interface_t Position_Loop_Task;
void Task_Position_Loop_Test_Init(void);
void Task_Position_Loop_Test(void);
void Task_Position_Loop_Test_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MOTOR_TASK_H */

