#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_VOLTAGE_COMMAND_MAX 25000
#define MOTOR_VOLTAGE_SAFE_LIMIT 6000 /* 本轮位置响应实测限幅，单位为电压控制码。 */
#define MOTOR_FEEDBACK_TIMEOUT_MS 100U

/* GM6020 CAN 电压模式。只在主循环调用，CAN 中断仍由 Bsp 处理。 */
typedef struct
{
    uint8_t id;                 /* 拨码编号 1..7，反馈 ID = 0x204 + id。 */
    uint8_t feedback_valid;     /* 至少收到过一帧合法反馈；是否在线另查时间戳。 */
    uint16_t angle_raw;         /* 单圈机械角度 0..8191。 */
    float angle_deg;            /* 单圈绝对角度 [0, 360)，不是相对机械零点。 */
    int16_t speed_rpm;
    int16_t torque_current_raw; /* 手册未给反馈电流比例，保留有符号原始值。 */
    uint8_t temperature_c;
    uint32_t last_rx_ms;
    uint32_t rx_updates;
} Motor;

/* 初始化一个电机的数据，不启动 CAN、不发送命令；调用者先执行 can_Init。 */
HAL_StatusTypeDef motor_Init(Motor *motor, uint8_t id);

/* 整组发送，first_id 只能为 1（0x1FF）或 5（0x2FF）。
 * 数组四项分别对应连续四个拨码编号；5 组第四项必须为 0。
 * 每项为电压给定原始值，不是伏特；发送前按 MOTOR_VOLTAGE_SAFE_LIMIT 限幅。
 * HAL_OK 仅表示提交 CAN 邮箱；HAL_BUSY/ERROR 原样返回。
 * 同一组必须由一个调用者统一填写所有槽，避免覆盖其他电机命令。 */
HAL_StatusTypeDef motor_Tx(uint8_t first_id, const int16_t voltage_command[4]);
int16_t motor_LimitVoltage(int32_t requested_command);

typedef struct
{
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    HAL_StatusTypeDef status;
} Motor_TxFrame;
/* 主循环只读：最近一次实际传给 can_Tx 的报文与提交结果，用于 Task 调试打印。 */
const Motor_TxFrame *motor_GetLastTxFrame(void);

/* 从 Bsp 取最新反馈；无新帧返回 HAL_BUSY，非法 DLC/角度返回 HAL_ERROR。
 * 仅合法帧更新 Motor；保留最后有效数据供诊断，不把旧数据当新反馈。 */
HAL_StatusTypeDef motor_Rx(Motor *motor);
uint8_t motor_IsOnline(const Motor *motor, uint32_t now_ms);

/* ================= 两轴调参区（修改后重新编译、烧录）=================
 * 每个轴的参数和 PID 状态独立。位置环输出目标 rpm，速度环输出电压码。
 * 以下位置环增益是串级控制的保守初值，尚需实机整定。
 * 目标角度沿用启动参考：YAW=0°，PITCH=100°。 */

/* YAW：位置环 PID 与固定目标角度。 */
#define YAW_POSITION_KP              5.0f
#define YAW_POSITION_KI              0.0f
#define YAW_POSITION_KD              0.0f
#define YAW_POSITION_MAX_OUTPUT      100.0f
#define YAW_POSITION_INTEGRAL_LIMIT  50.0f
#define YAW_TARGET_ANGLE_DEG         30.0f

/* PITCH：位置环 PID 与固定目标角度。 */
#define PITCH_POSITION_KP            5.0f
#define PITCH_POSITION_KI            0.0f
#define PITCH_POSITION_KD            0.0f
#define PITCH_POSITION_MAX_OUTPUT    100.0f
#define PITCH_POSITION_INTEGRAL_LIMIT 50.0f
#define PITCH_TARGET_ANGLE_DEG       70.0f

/* 速度环及前馈参数供所有闭环任务共用；前馈独立于PID状态。 */
#define YAW_SPEED_KP                 30.0f
#define YAW_SPEED_KI                 1.0f
#define YAW_SPEED_KD                 0.0f
#define YAW_SPEED_MAX_OUTPUT         6000.0f
#define YAW_SPEED_INTEGRAL_LIMIT     3000.0f
#define YAW_VELOCITY_FF_GAIN         15.0f /* GM6020 voltage command per rpm；需实机整定。 */

#define PITCH_SPEED_KP               30.0f
#define PITCH_SPEED_KI               1.0f
#define PITCH_SPEED_KD               0.0f
#define PITCH_SPEED_MAX_OUTPUT       6000.0f
#define PITCH_SPEED_INTEGRAL_LIMIT   3000.0f
#define PITCH_VELOCITY_FF_GAIN       15.0f

/* 双轴闭环接口：逻辑轴与当前工程拨码对应，不能与其他电机任务同时调度。 */
typedef enum { YAW_MOTOR = 0, PITCH_MOTOR, MOTOR_AXIS_COUNT } Motor_ID;
#define MOTOR_YAW_CAN_ID 2U
#define MOTOR_PITCH_CAN_ID 1U
#define MOTOR_CONTROL_SPEED_LIMIT_RPM 30.0f
#define MOTOR_YAW_MIN_DEG 0.0f
#define MOTOR_YAW_MAX_DEG 180.0f
#define MOTOR_PITCH_MIN_DEG 55.0f
#define MOTOR_PITCH_MAX_DEG 145.0f
#define MOTOR_PITCH_START_DEG 100.0f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_output;     /* 位置环为 rpm，速度环为电压原始控制码。 */
    float integral_limit; /* I 项输出限幅，与本环输出同单位。 */
} PID_Config_t;

typedef struct
{
    float yaw;   /* 度，沿用启动时建立的临时参考。 */
    float pitch;
} Target_Angles_t;

#ifndef CLAMP
/* 参数不得带自增或其他副作用；浮点目标的 NaN/Inf 另行拒绝。 */
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif

typedef enum { MOTOR_CONTROL_DISABLED = 0, MOTOR_CONTROL_SPEED, MOTOR_CONTROL_ANGLE,
               MOTOR_CONTROL_IMU_ANGLE } Motor_ControlMode;
typedef struct
{
    Motor feedback;
    HAL_StatusTypeDef rx_status;
    Motor_ControlMode mode;
    uint8_t reference_valid;
    float actual_angle_deg, target_angle_deg, target_speed_rpm;
    float speed_pid_output, velocity_ff_output; /* 电压原始控制码，求和前各自独立。 */
    int16_t voltage_command;
} Motor_ControlStatus;

/* Task 传入两轴各自的位置/速度 PID 配置，数组按 Motor_ID 索引，初始化时复制。
 * 只初始化控制对象，不启动 CAN；空配置保持未初始化，Update 返回 HAL_ERROR。
 * 首帧为 YAW 0°、PITCH 100°的临时参考。
 * PITCH 不是编码器绝对角度或已标定的机械角度；接实物后需确认。 */
void Motor_Control_Init(const PID_Config_t angle_config[MOTOR_AXIS_COUNT],
                        const PID_Config_t speed_config[MOTOR_AXIS_COUNT]);
void Motor_Control_ReadFeedback(void);
const Motor_ControlStatus *Motor_Get_Status(Motor_ID motor);
/* speed 为浮点 rpm，按 MOTOR_CONTROL_SPEED_LIMIT_RPM 限幅；非有限速度停止两轴。
 * angle 单位度，YAW [0,180]，PITCH [55,145]；非有限角度停止两轴。
 * Set 仅登记目标，Update 才计算闭环并提交完整控制帧。 */
void Motor_Set_Speed(Motor_ID motor, float speed);
float Motor_Generate_Sine_Speed(float amplitude_rpm, float frequency_hz, float time_s);
void Motor_Set_Angle(Motor_ID motor, float angle);
/* IMU位置模式：目标为IMU参考角deg，不套用编码器目标范围；
 * 最短角差生成目标rpm，两轴方向按本轮实测同号。机械行程由Task检查编码器。
 * 无效目标停止两轴；IMU失效/电机反馈丢失由Update清零。 */
void Motor_Set_IMU_Angle(Motor_ID motor, float angle);
/* 每轴独立设置前馈系数，单位为电压原始控制码/rpm；默认 0。
 * 无效轴或非有限系数被忽略；速度模式和位置串级模式共用。 */
void Motor_Set_Velocity_Feedforward_Gain(Motor_ID motor, float gain);
/* 位置目标入口；复用角度限幅。先设置两轴目标，再统一调用
 * Motor_Control_Update(dt_s) 计算位置/速度串级与前馈并提交一帧。 */
void Motor_PID_Control(Motor_ID motor, float target_angle);
float Motor_Get_Angle(Motor_ID motor); /* 无临时参考/无效轴返回 NaN，在线状态另查。 */
float Get_Current_Angle(Motor_ID motor); /* 统一位置反馈入口，当前返回编码器角度。 */
/* IMU角度deg，与编码器启动参考不同；有效轴的数据无效时返回NaN，无效轴返回0。 */
float Get_Current_IMU_Angle(Motor_ID motor);
/* PID与前馈相加并限幅后的电压控制码；停止/无效轴返回0，不是伏特或实测电压。 */
float Motor_Get_Output(Motor_ID motor);
HAL_StatusTypeDef Motor_Control_Update(float dt_s);
void Motor_Control_Stop(void); /* 清目标模式、输出和 PID 历史，不提交 CAN。 */

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_MOTOR_H */
