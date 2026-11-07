#ifndef TASK_M3508_H
#define TASK_M3508_H

#include "can.h"
#include "task_interface.h"

/* ================= 速度环调参区 =================
 * target_speed 是减速器输出轴 rpm；P/I 使用转子 rpm 误差。
 * 10 ms 离散积分：I += KI * error，KI 已包含采样周期，不再乘 dt。
 * P/I/FIXED_CURRENT 的输出单位均为 C620 CAN 电流控制码。 */
#define target_speed                15.0f
#define KP                          10.0f
#define KI                          0.5f
#define I_OUTPUT_LIMIT              2500.0f
#ifndef FIXED_CURRENT
#define FIXED_CURRENT               500.0f /* 自行填写；0 和非0均直接参与相加。 */
#endif

/* 历史固定输出参数，保留但不再参与控制：
 * #define M3508_FIXED_CURRENT_RAW   2000
 */
#define M3508_ESC_ID                 1U
#define M3508_GEAR_RATIO             19.0f
#define M3508_CURRENT_LIMIT_RAW      6000U /* Full C620 protocol range: +/-20 A. */
#define M3508_PERIOD_MS              10U

typedef enum {
    M3508_WAIT_FEEDBACK = 0, M3508_RUNNING, M3508_STOPPED,
    M3508_FEEDBACK_LOST, M3508_RX_ERROR, M3508_TX_ERROR,
    M3508_CONTROL_LATE, M3508_OVERTEMPERATURE, M3508_CAN_ERROR
} M3508_State;

typedef struct {
    M3508_State state;
    HAL_StatusTypeDef init_status, tx_status;
    uint32_t feedback_count, last_feedback_ms, tx_submitted;
    int16_t rotor_speed_rpm, torque_current_raw, command_raw;
    uint16_t angle_raw;
    uint8_t temperature_c;
    float output_speed_rpm, target_output_rpm;
    float speed_error_rpm; /* 转子转速误差，target_speed*减速比 - 反馈转子rpm。 */
    float p_output, i_output, fixed_output, output_before_limit;
} M3508_Status;

extern M3508_Status m3508_status; /* Main-loop owned, inspect in debugger. */
extern volatile uint8_t m3508_stop_requested; /* Write 1: latched stop. */
extern const Task_Interface_t M3508_Task;
void M3508_Task_Init(void);
void M3508_Task_Update(void);
void M3508_Task_Stop(void);
/* Sole CAN1 control task. Every 10 ms: command = P + I + FIXED_CURRENT.
 * I output clamps at +/-I_OUTPUT_LIMIT; final command clamps at protocol range.
 * Optional protection conditions are separately commented out in Update.
 * No current-exceeded, unresponsive-motor or feedback-loss stop conditions.
 * Transmit busy/error is retried on the next period. Only explicit Stop stops.
 * Clamping changes the value, never enters STOPPED. */
#endif
