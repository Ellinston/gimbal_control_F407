#include "motor_test.h"
#include "can.h"
#include "usart.h"
#include "usart_test.h"
#include "imu_test.h"
#include "imu_level_config.h"
#include "velocity_feedforward.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
/* PID 参数唯一来源是 motor.h；测试只按轴组装配置，不覆盖参数。 */
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

typedef struct
{
    Motor_ID motor_id;
    float kp;
    float ki;
    float kd;
    float max_output;
    float integral_limit;
    float ff_gain;
} Motor_PID_Params_t;

static const Motor_PID_Params_t motor_pid_params[] =
{
    /* 行序不代表电机ID；初始化始终使用motor_id索引。 */
    {PITCH_MOTOR, PITCH_SPEED_KP, PITCH_SPEED_KI, PITCH_SPEED_KD,
     PITCH_SPEED_MAX_OUTPUT, PITCH_SPEED_INTEGRAL_LIMIT, PITCH_VELOCITY_FF_GAIN},
    {YAW_MOTOR, YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD,
     YAW_SPEED_MAX_OUTPUT, YAW_SPEED_INTEGRAL_LIMIT, YAW_VELOCITY_FF_GAIN}
};

static HAL_StatusTypeDef MotorTask_PID_Init(void)
{
    PID_Config_t speed_config[MOTOR_AXIS_COUNT] = {0};
    uint8_t mapped[MOTOR_AXIS_COUNT] = {0};
    const size_t count = sizeof(motor_pid_params) / sizeof(motor_pid_params[0]);

    for (size_t row = 0U; row < count; ++row)
    {
        const Motor_PID_Params_t *params = &motor_pid_params[row];
        if ((uint32_t)params->motor_id >= MOTOR_AXIS_COUNT || mapped[params->motor_id])
        {
            Motor_Control_Init(NULL, NULL);
            return HAL_ERROR;
        }
        mapped[params->motor_id] = 1U;
        speed_config[params->motor_id] = (PID_Config_t){
            .kp = params->kp, .ki = params->ki, .kd = params->kd,
            .max_output = params->max_output,
            .integral_limit = params->integral_limit
        };
    }

    for (uint32_t id = 0U; id < MOTOR_AXIS_COUNT; ++id)
    {
        if (!mapped[id])
        {
            Motor_Control_Init(NULL, NULL);
            return HAL_ERROR;
        }
    }

    Motor_Control_Init(angle_pid_config, speed_config);
    for (size_t row = 0U; row < count; ++row)
    {
        const Motor_PID_Params_t *params = &motor_pid_params[row];
        Motor_Set_Velocity_Feedforward_Gain(params->motor_id, params->ff_gain);
    }
    return HAL_OK;
}

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
static uint8_t serial_position_test, serial_command_valid;
static uint8_t imu_stabilize_test;
GimbalLevelConfig imu_level_config = IMU_LEVEL_CONFIG_INITIALIZER;
static GimbalLevelConfig level_config; /* immutable copy for this run */
static GimbalLevelState level_state;
static uint8_t level_reference_valid;
volatile uint8_t imu_stabilize_stop_requested;

static uint8_t Stabilize_SnapshotValid(const IMU_AttitudeSnapshot *snapshot, uint32_t now)
{
    if (!snapshot->valid || (uint32_t)(now-snapshot->sample_ms)>100U) { return 0U; }
    for (uint32_t i=0U; i<3U; ++i)
    { if (!isfinite(snapshot->up_sensor[i]) || !isfinite(snapshot->gyro_sensor_rads[i])) { return 0U; } }
    return 1U;
}

static uint8_t Stabilize_Targets(const IMU_AttitudeSnapshot *snapshot, float dt)
{
    float current[2]={Get_Current_Angle(YAW_MOTOR),Get_Current_Angle(PITCH_MOTOR)};
    if (!level_reference_valid)
    {
        for (uint32_t i=0U; i<2U; ++i)
        {
            const Motor_ControlStatus *axis=Motor_Get_Status((Motor_ID)i);
            if (!axis->reference_valid) { return 0U; }
            /* Absolute mechanical zero -> startup logical coordinates.
             * Its 360-degree representative does not affect Rz*Ry or IK. */
            level_config.zero_deg[i]=current[i]+remainderf(
                level_config.zero_deg[i]-axis->feedback.angle_deg,360.0f);
        }
        level_reference_valid=1U;
    }
    /* No extrapolation from a stale IMU while joints move. Both CAN samples
     * and gravity must lie within a 30 ms observation window (tick-wrap safe). */
    for (uint32_t i=0U; i<2U; ++i)
    {
        uint32_t rx=Motor_Get_Status((Motor_ID)i)->feedback.last_rx_ms;
        if ((uint32_t)(rx-snapshot->sample_ms)>30U && (uint32_t)(snapshot->sample_ms-rx)>30U)
        { double_motor_status.level_result=LEVEL_INPUT_INVALID; return 0U; }
    }
    double_motor_status.level_result=GimbalLevel_Update(&level_config,&level_state,
        snapshot->up_sensor,current,dt);
    if (double_motor_status.level_result!=LEVEL_OK &&
        double_motor_status.level_result!=LEVEL_NEAR_VERTICAL) { return 0U; }
    double_motor_status.yaw_target_deg=level_state.command_deg[0];
    double_motor_status.pitch_target_deg=level_state.command_deg[1];
    double_motor_status.level_goal_yaw_deg=level_state.goal_deg[0];
    double_motor_status.level_goal_pitch_deg=level_state.goal_deg[1];
    double_motor_status.level_residual_deg=level_state.residual_deg;
    /* Project the top-mounted gyro onto the physical joint axes first.
     * For Rz(yaw)*Ry(pitch), outer Yaw in top coordinates is (-sin(p),0,cos(p));
     * Pitch is +Y. Convert rad/s to encoder-positive rpm before subtraction. */
    float gyro_top[3]={0};
    for (uint32_t i=0U; i<3U; ++i)
    {
        for (uint32_t j=0U; j<3U; ++j)
        { gyro_top[i]+=level_config.sensor_to_top[i][j]*snapshot->gyro_sensor_rads[j]; }
    }
    float pitch_rad=level_config.direction[1]*(current[1]-level_config.zero_deg[1])*0.0174532925199f;
    float imu_rpm[2]={
        (-sinf(pitch_rad)*gyro_top[0]+cosf(pitch_rad)*gyro_top[2])*9.54929658551f/level_config.direction[0],
        gyro_top[1]*9.54929658551f/level_config.direction[1]
    };
    for (uint32_t i=0U; i<MOTOR_AXIS_COUNT; ++i)
    {
        float encoder_rpm=(float)Motor_Get_Status((Motor_ID)i)->feedback.speed_rpm;
        float compensation_rpm=calculate_base_velocity_ff(imu_rpm[i],encoder_rpm);
        if (!isfinite(compensation_rpm))
        { double_motor_status.level_result=LEVEL_INPUT_INVALID; return 0U; }
        Motor_Set_Base_Velocity_Feedforward((Motor_ID)i,compensation_rpm);
    }
    return 1U;
}

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
    serial_position_test = serial_command_valid = imu_stabilize_test = 0U;
    memset(&double_motor_status, 0, sizeof(double_motor_status));
    HAL_StatusTypeDef pid_status = MotorTask_PID_Init();
    double_motor_status.tx_status = double_motor_status.log_status = HAL_BUSY;
    double_motor_status.yaw_target_deg = MOTOR_YAW_MIN_DEG;
    double_motor_status.pitch_target_deg = MOTOR_PITCH_START_DEG;
    double_motor_status.yaw_direction = double_motor_status.pitch_direction = 1;
    uint32_t now = HAL_GetTick();
    double_last_control_ms = now;
    double_last_attempt_ms = now - 1U;
    double_last_log_ms = now - 100U;
    Double_ClearOutput(DOUBLE_MOTOR_WAIT_FEEDBACK);
    double_motor_status.init_status = (pid_status == HAL_OK) ? can_Init() : pid_status;
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
        if (fabsf(Get_Current_Angle(id) - endpoint) <= 1.0f &&
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
    Double_FormatAngle(yaw, sizeof(yaw), Get_Current_Angle(YAW_MOTOR));
    Double_FormatAngle(pitch, sizeof(pitch), Get_Current_Angle(PITCH_MOTOR));
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
    const IMU_AttitudeSnapshot attitude = imu_attitude_snapshot;
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
                 online && !double_zero_pending && (!serial_position_test || serial_command_valid) &&
                 (!imu_stabilize_test || Stabilize_SnapshotValid(&attitude, now)))
        {
            if (imu_stabilize_test)
            {
                double_motor_status.yaw_target_deg = Get_Current_Angle(YAW_MOTOR);
                double_motor_status.pitch_target_deg = Get_Current_Angle(PITCH_MOTOR);
            }
            double_motor_status.state = DOUBLE_MOTOR_RUNNING;
            double_last_control_ms = now;
        }
    }
    if (double_motor_status.state == DOUBLE_MOTOR_RUNNING)
    {
        /* 临时软件越界保护：相对测试行程外留10°；不是已验证的机械限位。 */
        if (imu_stabilize_test && !Stabilize_SnapshotValid(&attitude, now))
        {
            Double_ClearOutput(DOUBLE_MOTOR_IMU_ERROR);
        }
        else if (Get_Current_Angle(YAW_MOTOR) < MOTOR_YAW_MIN_DEG - 10.0f ||
            Get_Current_Angle(YAW_MOTOR) > MOTOR_YAW_MAX_DEG + 10.0f ||
            Get_Current_Angle(PITCH_MOTOR) < MOTOR_PITCH_MIN_DEG - 10.0f ||
            Get_Current_Angle(PITCH_MOTOR) > MOTOR_PITCH_MAX_DEG + 10.0f)
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
        if (double_motor_status.state == DOUBLE_MOTOR_RUNNING && imu_stabilize_test &&
            !Stabilize_Targets(&attitude,(float)elapsed*0.001f))//算目标
        { Double_ClearOutput(DOUBLE_MOTOR_LEVEL_ERROR); }
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
            if (imu_stabilize_test)
            {
                //登记目标
                // 找平模式
                Motor_Set_Angle(YAW_MOTOR, double_motor_status.yaw_target_deg);
                Motor_Set_Angle(PITCH_MOTOR, double_motor_status.pitch_target_deg);
            }
            else
            {
                // 串口位置等其他模式
                Motor_PID_Control(YAW_MOTOR, double_motor_status.yaw_target_deg);
                Motor_PID_Control(PITCH_MOTOR, double_motor_status.pitch_target_deg);
            }
            /* 日志显示 Device 实际接受的限幅目标。 */
            double_motor_status.yaw_target_deg = yaw->target_angle_deg;
            double_motor_status.pitch_target_deg = pitch->target_angle_deg;
            double_last_attempt_ms = now;
            double_motor_status.tx_status = Motor_Control_Update((float)elapsed * 0.001f);//计算和发送的执行者
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
    yaw_channel.velocity_ff_output = yaw->velocity_ff_output;
    yaw_channel.speed_pid_output = yaw->speed_pid_output;
    pitch_channel.velocity_ff_output = pitch->velocity_ff_output;
    pitch_channel.speed_pid_output = pitch->speed_pid_output;
    /* 失联不伪装为稳定角度；目标仍保留任务设定，便于对照。 */
    yaw_channel.current_angle = motor_IsOnline(&yaw->feedback, now) ? Get_Current_Angle(YAW_MOTOR) : NAN;
    pitch_channel.current_angle = motor_IsOnline(&pitch->feedback, now) ? Get_Current_Angle(PITCH_MOTOR) : NAN;
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

/* 串口位置测试复用双轴控制周期及锁止保护，解析只在主循环执行。 */
#define MOTOR_SERIAL_LINE_CAPACITY 64U
static char motor_serial_line[MOTOR_SERIAL_LINE_CAPACITY];
static uint8_t motor_serial_length, motor_serial_invalid;
static uint32_t motor_serial_fault_count;
uint32_t motor_serial_accepted, motor_serial_rejected;

static char *Serial_SkipSpace(char *cursor)
{
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r') { ++cursor; }
    return cursor;
}

static void Serial_ParseLine(void)
{
    char *cursor = Serial_SkipSpace(motor_serial_line);
    if (strncmp(cursor, "STOP", 4U) == 0 && *Serial_SkipSpace(cursor + 4U) == '\0')
    {
        Task_Serial_Position_Test_Stop();
        ++motor_serial_accepted;
        return;
    }
    /* 停机/故障锁止后，SET 不能重新使能。完整Init才允许重新测试。 */
    if (double_motor_status.state != DOUBLE_MOTOR_WAIT_FEEDBACK &&
        double_motor_status.state != DOUBLE_MOTOR_RUNNING)
    {
        ++motor_serial_rejected;
        return;
    }
    if (strncmp(cursor, "SET", 3U) != 0 ||
        (cursor[3] != ' ' && cursor[3] != '\t'))
    {
        ++motor_serial_rejected;
        return;
    }
    cursor = Serial_SkipSpace(cursor + 3U);
    char *end;
    float yaw = strtof(cursor, &end);
    if (end == cursor || (*end != ' ' && *end != '\t'))
    {
        ++motor_serial_rejected;
        return;
    }
    cursor = Serial_SkipSpace(end);
    float pitch = strtof(cursor, &end);
    if (end == cursor || *Serial_SkipSpace(end) != '\0' ||
        !isfinite(yaw) || !isfinite(pitch))
    {
        ++motor_serial_rejected;
        return;
    }
    /* 两个值全部通过校验后一次接受，禁止半条命令只更新一轴。 */
    Motor_Set_Angle(YAW_MOTOR, yaw);
    Motor_Set_Angle(PITCH_MOTOR, pitch);
    double_motor_status.yaw_target_deg = Motor_Get_Status(YAW_MOTOR)->target_angle_deg;
    double_motor_status.pitch_target_deg = Motor_Get_Status(PITCH_MOTOR)->target_angle_deg;
    serial_command_valid = 1U;
    ++motor_serial_accepted;
}

void Task_Serial_Position_Test_Init(void)
{
    Task_Double_Motor_Test_Init();
    fixed_position_test = serial_position_test = 1U;
    serial_command_valid = 0U;
    motor_serial_length = motor_serial_invalid = 0U;
    motor_serial_accepted = motor_serial_rejected = 0U;
    /* 清理旧测试遗留字节：旧命令不能在重新Init后启动电机。 */
    uint8_t stale[64];
    for (uint32_t i = 0U; i < 4U; ++i) { (void)usart_Rx(stale, sizeof(stale)); }
    motor_serial_fault_count = usart_GetRxFaultCount();
    if (usart_Start() != HAL_OK) { Double_ClearOutput(DOUBLE_MOTOR_SERIAL_ERROR); }
    vofa_task_enable_motor_position();
    Position_UpdateChannels();
}

void Task_Serial_Position_Test_Stop(void)
{
    serial_command_valid = 0U;
    Task_Position_Loop_Test_Stop();
}

static void MotorTask_ReadSerial(void (*parse_line)(void))
{
    if (!double_motor_status.initialized) { return; }
    uint8_t buffer[64];
    HAL_StatusTypeDef rx_status = usart_Start();
    uint16_t length = usart_Rx(buffer, sizeof(buffer));
    if (rx_status != HAL_OK || usart_GetRxFaultCount() != motor_serial_fault_count)
    {
        /* 丢字节后整次测试锁止，不能把截断/拼接的内容解释成新角度。 */
        if (double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK ||
            double_motor_status.state == DOUBLE_MOTOR_RUNNING)
        {
            Double_ClearOutput(DOUBLE_MOTOR_SERIAL_ERROR);
        }
        motor_serial_length = 0U;
        motor_serial_invalid = 1U;
        ++motor_serial_rejected;
    }
    else
    {
        for (uint16_t i = 0U; i < length; ++i)
        {
            uint8_t byte = buffer[i];
            if (byte == '\n')
            {
                motor_serial_line[motor_serial_length] = '\0';
                if (!motor_serial_invalid) { parse_line(); }
                else { ++motor_serial_rejected; }
                motor_serial_length = motor_serial_invalid = 0U;
            }
            else if (!motor_serial_invalid)
            {
                if (byte < 0x20U && byte != '\t' && byte != '\r') { motor_serial_invalid = 1U; }
                else if (byte > 0x7EU || motor_serial_length >= MOTOR_SERIAL_LINE_CAPACITY - 1U)
                {
                    motor_serial_invalid = 1U;
                }
                else { motor_serial_line[motor_serial_length++] = (char)byte; }
            }
        }
    }
}

void Task_Serial_Position_Test(void)
{
    MotorTask_ReadSerial(Serial_ParseLine);
    Task_Double_Motor_Test();
    Position_UpdateChannels();
}

const Task_Interface_t Serial_Position_Task = {
    .Init = Task_Serial_Position_Test_Init,
    .Update = Task_Serial_Position_Test
};


void Task_IMU_Stabilize_Test_Init(void)
{
    Task_Serial_Position_Test_Init();
    imu_stabilize_test = 1U;
    serial_position_test = 0U; /* 标定和双轴反馈有效后自动开始，不等待SET。 */
    imu_stabilize_stop_requested = 0U;
    level_config=imu_level_config;
    /* Travel remains owned by motor.h, never a second editable source. */
    level_config.minimum_deg[0]=MOTOR_YAW_MIN_DEG;
    level_config.maximum_deg[0]=MOTOR_YAW_MAX_DEG;
    level_config.minimum_deg[1]=MOTOR_PITCH_MIN_DEG;
    level_config.maximum_deg[1]=MOTOR_PITCH_MAX_DEG;
    level_state=(GimbalLevelState){0}; level_reference_valid=0U;
    double_motor_status.yaw_target_deg = double_motor_status.pitch_target_deg = NAN;
    if (!GimbalLevel_ConfigValid(&level_config) || level_config.zero_deg[0]<0.0f ||
        level_config.zero_deg[0]>=360.0f || level_config.zero_deg[1]<0.0f || level_config.zero_deg[1]>=360.0f)
    {
        double_motor_status.level_result=LEVEL_CONFIG_INVALID;
        if (double_motor_status.state==DOUBLE_MOTOR_WAIT_FEEDBACK)
        { Double_ClearOutput(DOUBLE_MOTOR_LEVEL_ERROR); }
    }
    Position_UpdateChannels();
}

void Task_IMU_Stabilize_Test_Stop(void)
{
    Task_Position_Loop_Test_Stop();
}

static void Stabilize_ParseLine(void)
{
    char *cursor = Serial_SkipSpace(motor_serial_line);
    if (strncmp(cursor, "STOP", 4U) == 0 && *Serial_SkipSpace(cursor + 4U) == '\0')
    {
        Task_IMU_Stabilize_Test_Stop();
        ++motor_serial_accepted;
    }
    else { ++motor_serial_rejected; } /* 不把编码器SET误当IMU姿态目标。 */
}

void Task_IMU_Stabilize_Test(void)
{
    if (imu_stabilize_stop_requested)
    {
        imu_stabilize_stop_requested = 0U;
        Task_IMU_Stabilize_Test_Stop();
    }
    MotorTask_ReadSerial(Stabilize_ParseLine);
    Task_Double_Motor_Test();
    Position_UpdateChannels();
}

const Task_Interface_t IMU_Stabilize_Task = {
    .Init = Task_IMU_Stabilize_Test_Init,
    .Update = Task_IMU_Stabilize_Test
};

YawSine_Status yaw_sine_status;
volatile uint8_t yaw_sine_stop_requested;
static uint32_t sine_init_ms, sine_last_control_ms, sine_last_attempt_ms;
static float sine_elapsed_s;
static uint8_t sine_zero_pending;

static void Sine_UpdateChannels(void)
{
    const Motor_ControlStatus *yaw = Motor_Get_Status(YAW_MOTOR);
    uint8_t online = motor_IsOnline(&yaw->feedback, HAL_GetTick());
    yaw_sine_status.target_speed_rpm = yaw->target_speed_rpm;
    yaw_channel.target_angle = NAN; /* 速度模式没有位置目标。 */
    yaw_channel.current_angle = online ? Get_Current_Angle(YAW_MOTOR) : NAN;
    yaw_channel.target_speed = yaw->target_speed_rpm;
    yaw_channel.current_speed = online ? (float)yaw->feedback.speed_rpm : NAN;
    /* 分别传递ff_out与pid_out的快照，不从最终电压反推分量。 */
    yaw_channel.velocity_ff_output = yaw->velocity_ff_output;
    yaw_channel.speed_pid_output = yaw->speed_pid_output;
}

static void Sine_ClearOutput(YawSine_State state)
{
    Motor_Control_Stop();
    yaw_sine_status.state = state;
    sine_zero_pending = 1U;
    Sine_UpdateChannels();
}

static void Sine_SendZero(uint32_t now)
{
    const int16_t commands[4] = {0};
    sine_last_attempt_ms = now;
    HAL_StatusTypeDef result = sine_zero_pending ? can_AbortTx() : HAL_OK;
    if (result == HAL_OK) { result = motor_Tx(1U, commands); }
    yaw_sine_status.tx_status = result;
    if (result == HAL_OK)
    {
        sine_zero_pending = 0U;
        ++yaw_sine_status.tx_submitted;
    }
    else
    {
        /* 重试前仍需取消可能残留的邮箱；故障原因保持可读。 */
        sine_zero_pending = 1U;
        if (yaw_sine_status.state == YAW_SINE_WAIT_FEEDBACK && result != HAL_BUSY)
        {
            Sine_ClearOutput(YAW_SINE_TX_ERROR);
        }
    }
}

void Task_Sine_Speed_Test_Init(void)
{
    yaw_sine_status = (YawSine_Status){0};
    yaw_sine_stop_requested = 0U;
    HAL_StatusTypeDef pid_status = MotorTask_PID_Init();
    sine_elapsed_s = 0.0f;
    sine_init_ms = sine_last_control_ms = HAL_GetTick();
    sine_last_attempt_ms = sine_init_ms - 1U;
    vofa_task_enable_yaw_speed();
    Sine_ClearOutput(YAW_SINE_WAIT_FEEDBACK);
    yaw_sine_status.init_status = (pid_status == HAL_OK) ? can_Init() : pid_status;
    yaw_sine_status.initialized = (yaw_sine_status.init_status == HAL_OK);
    if (!isfinite(YAW_SINE_AMPLITUDE_RPM) || YAW_SINE_AMPLITUDE_RPM <= 0.0f ||
        YAW_SINE_AMPLITUDE_RPM > MOTOR_CONTROL_SPEED_LIMIT_RPM ||
        !isfinite(YAW_SINE_FREQUENCY_HZ) || YAW_SINE_FREQUENCY_HZ <= 0.0f)
    {
        Sine_ClearOutput(YAW_SINE_CONFIG_ERROR);
    }
    if (yaw_sine_status.initialized) { Sine_SendZero(sine_init_ms); }
    else { yaw_sine_status.state = YAW_SINE_TX_ERROR; }
}

void Task_Sine_Speed_Test_Stop(void)
{
    Sine_ClearOutput(YAW_SINE_STOPPED);
    if (yaw_sine_status.initialized) { Sine_SendZero(HAL_GetTick()); }
}

void Task_Sine_Speed_Test(void)
{
    if (!yaw_sine_status.initialized)
    {
        Sine_UpdateChannels();
        vofa_task_update();
        return;
    }
    if (yaw_sine_stop_requested)
    {
        yaw_sine_stop_requested = 0U;
        Task_Sine_Speed_Test_Stop();
    }
    Motor_Control_ReadFeedback();
    const Motor_ControlStatus *yaw = Motor_Get_Status(YAW_MOTOR);
    uint32_t now = HAL_GetTick();
    uint8_t online = motor_IsOnline(&yaw->feedback, now);
    if (yaw_sine_status.state == YAW_SINE_WAIT_FEEDBACK ||
        yaw_sine_status.state == YAW_SINE_RUNNING)
    {
        if (yaw->rx_status != HAL_OK && yaw->rx_status != HAL_BUSY)
        {
            Sine_ClearOutput(YAW_SINE_RX_ERROR);
        }
        else if (yaw_sine_status.state == YAW_SINE_RUNNING && !online)
        {
            Sine_ClearOutput(YAW_SINE_FEEDBACK_LOST);
        }
        else if (yaw_sine_status.state == YAW_SINE_WAIT_FEEDBACK && online &&
                 !sine_zero_pending && (uint32_t)(now - sine_init_ms) >= YAW_SINE_START_DELAY_MS)
        {
            yaw_sine_status.state = YAW_SINE_RUNNING;
            sine_last_control_ms = now;
            Motor_Set_Speed(YAW_MOTOR, 0.0f);
        }
    }
    uint32_t elapsed = (uint32_t)(now - sine_last_control_ms);
    if (yaw_sine_status.state == YAW_SINE_RUNNING)
    {
        if (Get_Current_Angle(YAW_MOTOR) < YAW_SINE_MIN_ANGLE_DEG ||
            Get_Current_Angle(YAW_MOTOR) > YAW_SINE_MAX_ANGLE_DEG)
        {
            Sine_ClearOutput(YAW_SINE_ANGLE_LIMIT);
        }
        else if (elapsed > 100U) { Sine_ClearOutput(YAW_SINE_CONTROL_LATE); }
    }
    if (elapsed >= YAW_SINE_PERIOD_MS)
    {
        sine_last_control_ms = now;
        if (yaw_sine_status.state == YAW_SINE_RUNNING)
        {
            float dt_s = (float)elapsed * 0.001f;
            sine_elapsed_s = fmodf(sine_elapsed_s + dt_s, 1.0f / YAW_SINE_FREQUENCY_HZ);
            float target_speed = Motor_Generate_Sine_Speed(YAW_SINE_AMPLITUDE_RPM,
                YAW_SINE_FREQUENCY_HZ, sine_elapsed_s);
            Motor_Set_Speed(YAW_MOTOR, target_speed);
            sine_last_attempt_ms = now;
            yaw_sine_status.tx_status = Motor_Control_Update(dt_s);
            if (yaw_sine_status.tx_status == HAL_OK) { ++yaw_sine_status.tx_submitted; }
            else { Sine_ClearOutput(YAW_SINE_TX_ERROR); }
        }
        else if (!sine_zero_pending) { Sine_SendZero(now); }
    }
    if (sine_zero_pending && (uint32_t)(now - sine_last_attempt_ms) >= 1U) { Sine_SendZero(now); }
    Sine_UpdateChannels();
    vofa_task_update();
}

const Task_Interface_t Sine_Speed_Task = {
    .Init = Task_Sine_Speed_Test_Init,
    .Update = Task_Sine_Speed_Test
};

Motor single_yaw_motor;
MotorSingle_Status motor_single_status;
static uint32_t init_ms, single_last_tx_ms;

/* 逐帧日志异步排队，发送中的缓冲区直到UART完成才释放；不阻塞10ms控制周期。 */
#define LOG_SLOTS 8U
#define LOG_CAPACITY 160U
static char logs[LOG_SLOTS][LOG_CAPACITY];
static uint16_t log_lengths[LOG_SLOTS];
static uint8_t log_head, log_tail, log_count, log_inflight;

static void Single_FlushLog(void)
{
    if (log_inflight && huart1.gState == HAL_UART_STATE_READY)
    {
        log_inflight = 0U;
        log_head = (uint8_t)((log_head + 1U) % LOG_SLOTS);
        --log_count;
    }
    if (!log_inflight && log_count && huart1.gState == HAL_UART_STATE_READY &&
        usart_Tx((const uint8_t *)logs[log_head], log_lengths[log_head]) == HAL_OK)
    {
        log_inflight = 1U;
    }
}

static void Single_LogTx(uint32_t now)
{
    Single_FlushLog();
    if (log_count == LOG_SLOTS) { ++motor_single_status.log_dropped; return; }
    const Motor_TxFrame *frame = motor_GetLastTxFrame();
    const uint8_t *d = frame->data;
    uint32_t deg = (uint32_t)single_yaw_motor.angle_raw * 3600U / 8192U;
    uint32_t age = (uint32_t)(now - (single_yaw_motor.feedback_valid ? single_yaw_motor.last_rx_ms : init_ms));
    int length = snprintf(logs[log_tail], LOG_CAPACITY,
        "TX %lu ID=%03lX DATA=%02X %02X %02X %02X %02X %02X %02X %02X "
        "s=%u c=%d rpm=%d raw=%u deg=%lu.%lu v=%u a=%lu\r\n",
        (unsigned long)now, (unsigned long)frame->id,
        (unsigned int)d[0], (unsigned int)d[1], (unsigned int)d[2], (unsigned int)d[3],
        (unsigned int)d[4], (unsigned int)d[5], (unsigned int)d[6], (unsigned int)d[7],
        (unsigned int)frame->status, (int)motor_single_status.command,
        (int)single_yaw_motor.speed_rpm, (unsigned int)single_yaw_motor.angle_raw,
        (unsigned long)(deg / 10U), (unsigned long)(deg % 10U),
        (unsigned int)single_yaw_motor.feedback_valid,
        (unsigned long)age);
    if (length <= 0 || (unsigned int)length >= LOG_CAPACITY)
    {
        ++motor_single_status.log_dropped;
        return;
    }
    log_lengths[log_tail] = (uint16_t)length;
    log_tail = (uint8_t)((log_tail + 1U) % LOG_SLOTS);
    ++log_count;
    Single_FlushLog();
}

HAL_StatusTypeDef Task_Motor_Single_Test_Init(void)
{
    memset(&motor_single_status, 0, sizeof(motor_single_status));
    (void)motor_Init(&single_yaw_motor, 1U);
    motor_single_status.command = motor_LimitVoltage(MOTOR_SINGLE_COMMAND);
    motor_single_status.rx_status = motor_single_status.tx_status = HAL_BUSY;
    motor_single_status.init_status = can_Init();
    init_ms = HAL_GetTick();
    single_last_tx_ms = init_ms - MOTOR_SINGLE_TX_PERIOD_MS;
    log_head = log_tail = log_count = log_inflight = 0U;
    return motor_single_status.init_status;
}

void Task_Motor_Single_Test(void)
{
    Single_FlushLog();
    motor_single_status.rx_status = motor_Rx(&single_yaw_motor);
    uint8_t unused[8], length;
    (void)can_Rx(0x206U, unused, &length);
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - single_last_tx_ms) >= MOTOR_SINGLE_TX_PERIOD_MS)
    {
        const int16_t command[4] = {motor_LimitVoltage(MOTOR_SINGLE_COMMAND), 0, 0, 0};
        motor_single_status.command = command[0];
        single_last_tx_ms = now;
        motor_single_status.tx_status = motor_Tx(1U, command);
        if (motor_single_status.tx_status == HAL_OK)
        {
            motor_single_status.submitted_command = command[0];
            ++motor_single_status.tx_submitted;
        }
        Single_LogTx(now);
    }
    Single_FlushLog();
}
