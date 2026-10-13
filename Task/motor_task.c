#include "motor_task.h"
#include "can.h"
#include "usart.h"
#include "usart_test.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 参数值统一在 motor_task.h 顶部修改，此处只负责组装配置并传入 Device。 */
static const PID_Config_t angle_pid_config[MOTOR_AXIS_COUNT] =
{
    [YAW_MOTOR] = {
        .kp = YAW_POSITION_KP, .ki = YAW_POSITION_KI, .kd = YAW_POSITION_KD,
        .max_output = YAW_POSITION_MAX_OUTPUT,
        .integral_limit = YAW_POSITION_INTEGRAL_LIMIT
    },
    [PITCH_MOTOR] = {
        .kp = PITCH_POSITION_KP, .ki = PITCH_POSITION_KI, .kd = PITCH_POSITION_KD,
        .max_output = PITCH_POSITION_MAX_OUTPUT,
        .integral_limit = PITCH_POSITION_INTEGRAL_LIMIT
    }
};

static const PID_Config_t speed_pid_config[MOTOR_AXIS_COUNT] =
{
    [YAW_MOTOR] = {
        .kp = YAW_SPEED_KP, .ki = YAW_SPEED_KI, .kd = YAW_SPEED_KD,
        .max_output = YAW_SPEED_MAX_OUTPUT,
        .integral_limit = YAW_SPEED_INTEGRAL_LIMIT
    },
    [PITCH_MOTOR] = {
        .kp = PITCH_SPEED_KP, .ki = PITCH_SPEED_KI, .kd = PITCH_SPEED_KD,
        .max_output = PITCH_SPEED_MAX_OUTPUT,
        .integral_limit = PITCH_SPEED_INTEGRAL_LIMIT
    }
};

static const Target_Angles_t target_angles =
{
    .yaw = YAW_TARGET_ANGLE_DEG,
    .pitch = PITCH_TARGET_ANGLE_DEG
};

Motor yaw_motor;
Motor pitch_motor;
MotorTask_Status motor_task_status;

static int16_t voltage_command[4];
static uint32_t start_ms;
static uint32_t run_duration_ms;
static uint32_t last_tx_ms;
static uint32_t last_attempt_ms;
static uint8_t zero_pending;

static void motor_task_ClearOutput(MotorTask_State state)
{
    memset(voltage_command, 0, sizeof(voltage_command));
    motor_task_status.state = state;
    zero_pending = 1U;
}

static HAL_StatusTypeDef motor_task_Send(uint32_t now)
{
    last_attempt_ms = now;
    motor_task_status.tx_status = motor_Tx(1U, voltage_command);
    if (motor_task_status.tx_status == HAL_OK)
    {
        last_tx_ms = now;
        zero_pending = 0U;
    }
    else
    {
        /* 连邮箱忙也取消本次运行，避免总线恢复后重放非零任务。 */
        motor_task_ClearOutput(MOTOR_TASK_TX_ERROR);
    }
    return motor_task_status.tx_status;
}

HAL_StatusTypeDef motor_task_Init(void)
{
    motor_task_ClearOutput(MOTOR_TASK_STOPPED);
    motor_task_status.initialized = 0U;
    motor_task_status.tx_status = HAL_ERROR;
    motor_task_status.yaw_rx_status = HAL_BUSY;
    motor_task_status.pitch_rx_status = HAL_BUSY;
    (void)motor_Init(&yaw_motor, MOTOR_TASK_YAW_ID);
    (void)motor_Init(&pitch_motor, MOTOR_TASK_PITCH_ID);
    motor_task_status.init_status = can_Init();
    if (motor_task_status.init_status != HAL_OK) { return motor_task_status.init_status; }
    motor_task_status.initialized = 1U;
    last_tx_ms = HAL_GetTick();
    return motor_task_Send(last_tx_ms);
}

HAL_StatusTypeDef motor_task_Start(int16_t yaw_voltage_command,
                                  int16_t pitch_voltage_command,
                                  uint32_t duration_ms)
{
    uint32_t now = HAL_GetTick();
    if ((motor_task_status.initialized == 0U) || (duration_ms == 0U) ||
        (duration_ms > MOTOR_TASK_MAX_RUN_MS) ||
        (yaw_voltage_command < -MOTOR_VOLTAGE_COMMAND_MAX) ||
        (yaw_voltage_command > MOTOR_VOLTAGE_COMMAND_MAX) ||
        (pitch_voltage_command < -MOTOR_VOLTAGE_COMMAND_MAX) ||
        (pitch_voltage_command > MOTOR_VOLTAGE_COMMAND_MAX)) { return HAL_ERROR; }
    if ((motor_task_status.state == MOTOR_TASK_RUNNING) || (zero_pending != 0U) ||
        !motor_IsOnline(&yaw_motor, now) || !motor_IsOnline(&pitch_motor, now))
    {
        return HAL_BUSY;
    }
    voltage_command[0] = yaw_voltage_command;
    voltage_command[1] = pitch_voltage_command;
    voltage_command[2] = 0;
    voltage_command[3] = 0;
    start_ms = now;
    run_duration_ms = duration_ms;
    motor_task_status.state = MOTOR_TASK_RUNNING;
    return HAL_OK;
}

HAL_StatusTypeDef motor_task_Stop(void)
{
    motor_task_ClearOutput(MOTOR_TASK_STOPPED);
    if (motor_task_status.initialized == 0U) { return HAL_ERROR; }
    return motor_task_Send(HAL_GetTick());
}

void motor_task_Run(void)
{
    if (motor_task_status.initialized == 0U) { return; }
    motor_task_status.yaw_rx_status = motor_Rx(&yaw_motor);
    motor_task_status.pitch_rx_status = motor_Rx(&pitch_motor);
    uint32_t now = HAL_GetTick();

    if (motor_task_status.state == MOTOR_TASK_RUNNING)
    {
        if ((motor_task_status.yaw_rx_status == HAL_ERROR) ||
            (motor_task_status.pitch_rx_status == HAL_ERROR))
        {
            motor_task_ClearOutput(MOTOR_TASK_RX_ERROR);
        }
        else if (!motor_IsOnline(&yaw_motor, now) || !motor_IsOnline(&pitch_motor, now))
        {
            motor_task_ClearOutput(MOTOR_TASK_FEEDBACK_LOST);
        }
        else if ((uint32_t)(now - start_ms) >= run_duration_ms)
        {
            motor_task_ClearOutput(MOTOR_TASK_DURATION_END);
        }
    }

    /* 发送失败后以至多 1ms 一次重试零帧，避免主循环忙转灌满邮箱。 */
    if (((zero_pending != 0U) ||
         ((uint32_t)(now - last_tx_ms) >= MOTOR_TASK_TX_PERIOD_MS)) &&
        ((uint32_t)(now - last_attempt_ms) >= 1U))
    {
        (void)motor_task_Send(now);
    }
}

/* 双轴闭环测试与上面的旧开环接口互斥调度。 */
DoubleMotor_Status double_motor_status;
static uint32_t double_last_control_ms, double_last_attempt_ms, double_last_log_ms;
static uint8_t double_zero_pending;
static char double_log_buffer[256];
static uint8_t fixed_position_test;

static void Double_ClearOutput(DoubleMotor_State state)
{
    Motor_Control_Stop();
    double_motor_status.state = state;
    double_zero_pending = 1U;
}

static void Double_SendZero(uint32_t now)
{
    int16_t commands[4] = {0};
    double_last_attempt_ms = now;
    /* 先取消旧邮箱，防止总线恢复后继续重发旧的非零命令。 */
    HAL_StatusTypeDef result = double_zero_pending ? can_AbortTx() : HAL_OK;
    if (result == HAL_OK) { result = motor_Tx(1U, commands); }
    double_motor_status.tx_status = result;
    if (result == HAL_OK)
    {
        double_zero_pending = 0U;
        ++double_motor_status.tx_submitted;
    }
    else if (result != HAL_BUSY)
    {
        Double_ClearOutput(DOUBLE_MOTOR_TX_ERROR);
    }
}

void Task_Double_Motor_Test_Init(void)
{
    fixed_position_test = 0U;
    memset(&double_motor_status, 0, sizeof(double_motor_status));
    Motor_Control_Init(angle_pid_config, speed_pid_config);
    double_motor_status.tx_status = double_motor_status.log_status = HAL_BUSY;
    double_motor_status.yaw_target_deg = MOTOR_YAW_MIN_DEG;
    double_motor_status.pitch_target_deg = MOTOR_PITCH_START_DEG;
    double_motor_status.yaw_direction = double_motor_status.pitch_direction = 1;
    uint32_t now = HAL_GetTick();
    double_last_control_ms = now;
    double_last_attempt_ms = now - 1U;
    double_last_log_ms = now - 100U;
    Double_ClearOutput(DOUBLE_MOTOR_WAIT_FEEDBACK);
    double_motor_status.init_status = can_Init();
    double_motor_status.initialized = (double_motor_status.init_status == HAL_OK);
    if (double_motor_status.initialized) { Double_SendZero(now); }
    else { double_motor_status.state = DOUBLE_MOTOR_TX_ERROR; }
}

void Task_Double_Motor_Test_Stop(void)
{
    Double_ClearOutput(DOUBLE_MOTOR_STOPPED);
    if (double_motor_status.initialized) { Double_SendZero(HAL_GetTick()); }
}

static float Double_StepTarget(Motor_ID id, float target, int8_t *direction,
                               float minimum, float maximum)
{
    float endpoint = (*direction > 0) ? maximum : minimum;
    if (target == endpoint)
    {
        const Motor_ControlStatus *axis = Motor_Get_Status(id);
        /* 计数只是目标步数；必须用真实反馈判断是否完成行程。 */
        if (fabsf(Motor_Get_Angle(id) - endpoint) <= 1.0f &&
            axis->feedback.speed_rpm >= -2 && axis->feedback.speed_rpm <= 2)
        {
            *direction = (int8_t)-*direction;
        }
        return target;
    }
    return CLAMP(target + (float)*direction, minimum, maximum);
}

static void Double_FormatAngle(char *buffer, size_t capacity, float angle)
{
    if (!isfinite(angle)) { (void)snprintf(buffer, capacity, "NA"); return; }
    int32_t value = (int32_t)(angle * 10.0f);
    uint32_t magnitude = (uint32_t)(value < 0 ? -value : value);
    (void)snprintf(buffer, capacity, "%s%lu.%lu", value < 0 ? "-" : "",
        (unsigned long)(magnitude / 10U), (unsigned long)(magnitude % 10U));
}

static void Double_Log(uint32_t now)
{
    if ((uint32_t)(now - double_last_log_ms) < 100U) { return; }
    double_last_log_ms = now;
    if (!usart_TxReady())
    {
        double_motor_status.log_status = HAL_BUSY;
        ++double_motor_status.log_dropped;
        return;
    }
    char yaw[24], pitch[24];
    Double_FormatAngle(yaw, sizeof(yaw), Motor_Get_Angle(YAW_MOTOR));
    Double_FormatAngle(pitch, sizeof(pitch), Motor_Get_Angle(PITCH_MOTOR));
    const Motor_ControlStatus *y = Motor_Get_Status(YAW_MOTOR);
    const Motor_ControlStatus *p = Motor_Get_Status(PITCH_MOTOR);
    int length = snprintf(double_log_buffer, sizeof(double_log_buffer),
            "[MOTOR] state=%u YAW target=%ld actual=%sdeg PITCH target=%ld actual=%sdeg rpm=%d,%d cmd=%d,%d\r\n",
            (unsigned int)double_motor_status.state,
            (long)double_motor_status.yaw_target_deg, yaw,
            (long)double_motor_status.pitch_target_deg, pitch,
            y->feedback.speed_rpm, p->feedback.speed_rpm, y->voltage_command, p->voltage_command);
    if (length <= 0 || (size_t)length >= sizeof(double_log_buffer))
    {
        double_motor_status.log_status = HAL_ERROR;
        ++double_motor_status.log_dropped;
        return;
    }
    double_motor_status.log_status = usart_Tx((const uint8_t *)double_log_buffer, (uint16_t)length);
    if (double_motor_status.log_status == HAL_OK) { ++double_motor_status.log_submitted; }
    else { ++double_motor_status.log_dropped; }
}

void Task_Double_Motor_Test(void)
{
    if (!double_motor_status.initialized) { return; }
    Motor_Control_ReadFeedback();
    uint32_t now = HAL_GetTick();
    const Motor_ControlStatus *yaw = Motor_Get_Status(YAW_MOTOR);
    const Motor_ControlStatus *pitch = Motor_Get_Status(PITCH_MOTOR);
    uint8_t online = motor_IsOnline(&yaw->feedback, now) && motor_IsOnline(&pitch->feedback, now);
    if (double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK ||
        double_motor_status.state == DOUBLE_MOTOR_RUNNING)
    {
        if ((yaw->rx_status != HAL_OK && yaw->rx_status != HAL_BUSY) ||
            (pitch->rx_status != HAL_OK && pitch->rx_status != HAL_BUSY))
        {
            Double_ClearOutput(DOUBLE_MOTOR_RX_ERROR);
        }
        else if (double_motor_status.state == DOUBLE_MOTOR_RUNNING && !online)
        {
            Double_ClearOutput(DOUBLE_MOTOR_FEEDBACK_LOST);
        }
        else if (double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK &&
                 online && !double_zero_pending)
        {
            double_motor_status.state = DOUBLE_MOTOR_RUNNING;
            double_last_control_ms = now;
        }
    }
    if (double_motor_status.state == DOUBLE_MOTOR_RUNNING)
    {
        /* 临时软件越界保护：相对测试行程外留10°；不是已验证的机械限位。 */
        if (yaw->actual_angle_deg < MOTOR_YAW_MIN_DEG - 10.0f ||
            yaw->actual_angle_deg > MOTOR_YAW_MAX_DEG + 10.0f ||
            pitch->actual_angle_deg < MOTOR_PITCH_MIN_DEG - 10.0f ||
            pitch->actual_angle_deg > MOTOR_PITCH_MAX_DEG + 10.0f)
        {
            Double_ClearOutput(DOUBLE_MOTOR_ANGLE_LIMIT);
        }
        else if ((uint32_t)(now - double_last_control_ms) > 100U)
        {
            Double_ClearOutput(DOUBLE_MOTOR_CONTROL_LATE);
        }
    }
    uint32_t elapsed = (uint32_t)(now - double_last_control_ms);
    if (elapsed >= MOTOR_TASK_TX_PERIOD_MS)
    {
        double_last_control_ms = now;
        if (double_motor_status.state == DOUBLE_MOTOR_RUNNING)
        {
            if (!fixed_position_test)
            {
                double_motor_status.yaw_target_deg = Double_StepTarget(YAW_MOTOR,
                    double_motor_status.yaw_target_deg, &double_motor_status.yaw_direction,
                    MOTOR_YAW_MIN_DEG, MOTOR_YAW_MAX_DEG);
                double_motor_status.pitch_target_deg = Double_StepTarget(PITCH_MOTOR,
                    double_motor_status.pitch_target_deg, &double_motor_status.pitch_direction,
                    MOTOR_PITCH_MIN_DEG, MOTOR_PITCH_MAX_DEG);
            }
            Motor_PID_Control(YAW_MOTOR, double_motor_status.yaw_target_deg);
            Motor_PID_Control(PITCH_MOTOR, double_motor_status.pitch_target_deg);
            /* 日志显示 Device 实际接受的限幅目标。 */
            double_motor_status.yaw_target_deg = yaw->target_angle_deg;
            double_motor_status.pitch_target_deg = pitch->target_angle_deg;
            double_last_attempt_ms = now;
            double_motor_status.tx_status = Motor_Control_Update((float)elapsed * 0.001f);
            if (double_motor_status.tx_status == HAL_OK) { ++double_motor_status.tx_submitted; }
            else { Double_ClearOutput(DOUBLE_MOTOR_TX_ERROR); }
        }
        else if (!double_zero_pending) { Double_SendZero(now); }
    }
    if (double_zero_pending && (uint32_t)(now - double_last_attempt_ms) >= 1U)
    {
        Double_SendZero(now);
    }
    /* 固定位置模式由 VOFA 任务独占串口，避免文本污染 JustFloat。 */
    if (!fixed_position_test) { Double_Log(now); }
}

const Task_Interface_t Double_Motor_Task =
{
    .Init = Task_Double_Motor_Test_Init,
    .Update = Task_Double_Motor_Test
};

static void Position_UpdateChannels(void)
{
    const Motor_ControlStatus *yaw = Motor_Get_Status(YAW_MOTOR);
    const Motor_ControlStatus *pitch = Motor_Get_Status(PITCH_MOTOR);
    uint32_t now = usart_GetTick();
    yaw_channel.target_angle = double_motor_status.yaw_target_deg;
    pitch_channel.target_angle = double_motor_status.pitch_target_deg;
    yaw_channel.target_speed = yaw->target_speed_rpm;
    pitch_channel.target_speed = pitch->target_speed_rpm;
    /* 失联不伪装为稳定角度；目标仍保留任务设定，便于对照。 */
    yaw_channel.current_angle = motor_IsOnline(&yaw->feedback, now) ? Motor_Get_Angle(YAW_MOTOR) : NAN;
    pitch_channel.current_angle = motor_IsOnline(&pitch->feedback, now) ? Motor_Get_Angle(PITCH_MOTOR) : NAN;
    yaw_channel.current_speed = motor_IsOnline(&yaw->feedback, now) ? (float)yaw->feedback.speed_rpm : NAN;
    pitch_channel.current_speed = motor_IsOnline(&pitch->feedback, now) ? (float)pitch->feedback.speed_rpm : NAN;
}

void Task_Position_Loop_Test_Init(void)
{
    Task_Double_Motor_Test_Init();
    fixed_position_test = 1U;
    Motor_Set_Angle(YAW_MOTOR, target_angles.yaw);
    Motor_Set_Angle(PITCH_MOTOR, target_angles.pitch);
    double_motor_status.yaw_target_deg = Motor_Get_Status(YAW_MOTOR)->target_angle_deg;
    double_motor_status.pitch_target_deg = Motor_Get_Status(PITCH_MOTOR)->target_angle_deg;
    Position_UpdateChannels();
}

void Task_Position_Loop_Test(void)
{
    Task_Double_Motor_Test();
    Position_UpdateChannels();
}

void Task_Position_Loop_Test_Stop(void)
{
    Task_Double_Motor_Test_Stop();
    Position_UpdateChannels();
}

const Task_Interface_t Position_Loop_Task =
{
    .Init = Task_Position_Loop_Test_Init,
    .Update = Task_Position_Loop_Test
};

