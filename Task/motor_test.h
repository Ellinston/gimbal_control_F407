#ifndef TASK_MOTOR_TEST_H
#define TASK_MOTOR_TEST_H

#include "motor.h"
#include "task_interface.h"
#include "gimbal_level.h"

#define YAW_SINE_AMPLITUDE_RPM       10.0f
#define YAW_SINE_FREQUENCY_HZ        0.2f
#define YAW_SINE_PERIOD_MS           MOTOR_TASK_TX_PERIOD_MS
#define YAW_SINE_START_DELAY_MS      3000U
#define YAW_SINE_MIN_ANGLE_DEG       -10.0f
#define YAW_SINE_MAX_ANGLE_DEG       190.0f
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
 * 首次实机需确认电压模式、机构固定和小输出值；Device 按 motor.h 再次限幅。
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
    DOUBLE_MOTOR_CONTROL_LATE,
    DOUBLE_MOTOR_SERIAL_ERROR,
    DOUBLE_MOTOR_IMU_ERROR,
    DOUBLE_MOTOR_LEVEL_ERROR
} DoubleMotor_State;

typedef struct
{
    uint8_t initialized;
    DoubleMotor_State state;
    HAL_StatusTypeDef init_status, tx_status, log_status;
    float yaw_target_deg, pitch_target_deg;
    int8_t yaw_direction, pitch_direction;
    uint32_t tx_submitted, log_submitted, log_dropped;
    GimbalLevelResult level_result;
    float level_goal_yaw_deg, level_goal_pitch_deg, level_residual_deg;
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

/* 固定目标位置环：角度取motor.h 配置区；与往返任务互斥。
 * 复用 double_motor_status、10ms控制和锁止保护；控制后更新 VOFA 通道。
 * 本任务不发送文本；另调 vofa_task_update，每100ms输出 JustFloat。
 * 必须先 Init，再于主循环持续调用 Test；Stop 后须重新 Init。 */
extern const Task_Interface_t Position_Loop_Task;
void Task_Position_Loop_Test_Init(void);
void Task_Position_Loop_Test(void);
void Task_Position_Loop_Test_Stop(void);

typedef enum
{
    YAW_SINE_WAIT_FEEDBACK = 0,
    YAW_SINE_RUNNING,
    YAW_SINE_STOPPED,
    YAW_SINE_FEEDBACK_LOST,
    YAW_SINE_RX_ERROR,
    YAW_SINE_TX_ERROR,
    YAW_SINE_ANGLE_LIMIT,
    YAW_SINE_CONTROL_LATE,
    YAW_SINE_CONFIG_ERROR
} YawSine_State;

typedef struct
{
    uint8_t initialized;
    YawSine_State state;
    HAL_StatusTypeDef init_status, tx_status;
    float target_speed_rpm;
    uint32_t tx_submitted;
} YawSine_Status;

extern YawSine_Status yaw_sine_status;
/* 调试器写1，请求主循环执行Stop；处理后清0，任务仍锁止。 */
extern volatile uint8_t yaw_sine_stop_requested;
extern const Task_Interface_t Sine_Speed_Task;
/* 独占 CAN 控制；vofa_task_init 必须先调用。仅启用 YAW（拨码1）。 */
void Task_Sine_Speed_Test_Init(void);
void Task_Sine_Speed_Test(void);
/* 清输出并取消旧邮箱；持续 Update 重试零帧。故障/Stop 后须重新 Init。 */
void Task_Sine_Speed_Test_Stop(void);

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


extern const Task_Interface_t Serial_Position_Task;
/* USART1: SET <yaw_deg> <pitch_deg>\n 或 STOP\n；允许CRLF，角度按motor.h限幅。
 * 上电等完整指令及双轴反馈；Stop/故障锁止，重新Init恢复。
 * 先调用vofa_task_init，主循环另调vofa_task_update。 */
void Task_Serial_Position_Test_Init(void);
void Task_Serial_Position_Test(void);
void Task_Serial_Position_Test_Stop(void);
extern uint32_t motor_serial_accepted, motor_serial_rejected;

/* 自稳任务：安装/几何配置必须已标定，IMU与双轴反馈有效后找平。
 * 重力方向经几何求解成为编码器目标，允许改变Yaw朝向；不捕获启动水平。
 * STOP/IMU/CAN/几何不可达故障锁止。沿用PID、前馈、行程及六路VOFA。 */
extern const Task_Interface_t IMU_Stabilize_Task;
extern volatile uint8_t imu_stabilize_stop_requested;
void Task_IMU_Stabilize_Test_Init(void);
void Task_IMU_Stabilize_Test(void);
void Task_IMU_Stabilize_Test_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MOTOR_TEST_H */
