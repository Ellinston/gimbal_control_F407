#include "motor.h"
#include "can.h"
#include "pid.h"
#include "velocity_feedforward.h"
#include "imu_test.h"
#include <math.h>
#include <string.h>

static Motor_TxFrame last_tx_frame;

float Motor_Generate_Sine_Speed(float amplitude_rpm, float frequency_hz, float time_s)
{
    if (!isfinite(amplitude_rpm) || !isfinite(frequency_hz) || !isfinite(time_s) ||
        amplitude_rpm < 0.0f || frequency_hz <= 0.0f || time_s < 0.0f)
    {
        return NAN;
    }
    return amplitude_rpm * sinf(6.2831853071795864769f * frequency_hz * time_s);
}

const Motor_TxFrame *motor_GetLastTxFrame(void)
{
    return &last_tx_frame;
}

_Static_assert(MOTOR_VOLTAGE_SAFE_LIMIT > 0 &&
               MOTOR_VOLTAGE_SAFE_LIMIT <= MOTOR_VOLTAGE_COMMAND_MAX,
               "Motor safety limit must stay within the protocol range");

int16_t motor_LimitVoltage(int32_t requested_command)
{
    if (requested_command > MOTOR_VOLTAGE_SAFE_LIMIT) { return MOTOR_VOLTAGE_SAFE_LIMIT; }
    if (requested_command < -MOTOR_VOLTAGE_SAFE_LIMIT) { return -MOTOR_VOLTAGE_SAFE_LIMIT; }
    return (int16_t)requested_command;
}

static uint16_t motor_ReadU16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t motor_ReadI16(const uint8_t *data)
{
    uint16_t raw = motor_ReadU16(data);
    /* 用宽整数显式恢复负数，避免超范围 unsigned -> signed 转换。 */
    return (int16_t)((raw <= 32767U) ? (int32_t)raw : (int32_t)raw - 65536);
}

HAL_StatusTypeDef motor_Init(Motor *motor, uint8_t id)
{
    if ((motor == NULL) || (id < 1U) || (id > 7U)) { return HAL_ERROR; }
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    return HAL_OK;
}

HAL_StatusTypeDef motor_Tx(uint8_t first_id, const int16_t voltage_command[4])
{
    uint8_t data[8] = {0};
    if ((voltage_command == NULL) || ((first_id != 1U) && (first_id != 5U)))
    {
        return HAL_ERROR;
    }
    if ((first_id == 5U) && (voltage_command[3] != 0)) { return HAL_ERROR; }
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        uint16_t raw = (uint16_t)motor_LimitVoltage(voltage_command[i]);
        data[i * 2U] = (uint8_t)(raw >> 8);
        data[i * 2U + 1U] = (uint8_t)raw;
    }
    last_tx_frame.id = (first_id == 1U) ? 0x1FFU : 0x2FFU;
    last_tx_frame.length = sizeof(data);
    memcpy(last_tx_frame.data, data, sizeof(data));
    last_tx_frame.status = can_Tx(last_tx_frame.id, data, sizeof(data));
    return last_tx_frame.status;
}

HAL_StatusTypeDef motor_Rx(Motor *motor)
{
    uint8_t data[8];
    uint8_t length;
    if ((motor == NULL) || (motor->id < 1U) || (motor->id > 7U)) { return HAL_ERROR; }
    HAL_StatusTypeDef status = can_Rx(0x204U + motor->id, data, &length);
    if (status != HAL_OK) { return status; }
    if (length != sizeof(data)) { return HAL_ERROR; }
    uint16_t angle = motor_ReadU16(data);
    if (angle > 8191U) { return HAL_ERROR; }

    motor->angle_raw = angle;
    motor->angle_deg = (float)angle * (360.0f / 8192.0f);
    motor->speed_rpm = motor_ReadI16(data + 2U);
    motor->torque_current_raw = motor_ReadI16(data + 4U);
    motor->temperature_c = data[6];
    motor->last_rx_ms = HAL_GetTick();
    ++motor->rx_updates;
    motor->feedback_valid = 1U;
    return HAL_OK;
}

uint8_t motor_IsOnline(const Motor *motor, uint32_t now_ms)
{
    return (uint8_t)((motor != NULL) && (motor->id >= 1U) && (motor->id <= 7U) &&
        (motor->feedback_valid != 0U) &&
        ((uint32_t)(now_ms - motor->last_rx_ms) <= MOTOR_FEEDBACK_TIMEOUT_MS));
}

static Motor_ControlStatus axes[MOTOR_AXIS_COUNT];
static PID_Position angle_pid[MOTOR_AXIS_COUNT], speed_pid[MOTOR_AXIS_COUNT];
static float velocity_ff_gain[MOTOR_AXIS_COUNT];
static float previous_absolute_deg[MOTOR_AXIS_COUNT];
static uint8_t control_initialized;

_Static_assert(MOTOR_YAW_CAN_ID >= 1U && MOTOR_YAW_CAN_ID <= 4U &&
               MOTOR_PITCH_CAN_ID >= 1U && MOTOR_PITCH_CAN_ID <= 4U &&
               MOTOR_YAW_CAN_ID != MOTOR_PITCH_CAN_ID,
               "Dual axes must use distinct slots in the 0x1FF group");

static uint8_t valid_axis(Motor_ID motor)
{
    return (uint8_t)(motor == YAW_MOTOR || motor == PITCH_MOTOR);
}

const Motor_ControlStatus *Motor_Get_Status(Motor_ID motor)
{
    return valid_axis(motor) ? &axes[motor] : NULL;
}

void Motor_Control_Init(const PID_Config_t angle_config[MOTOR_AXIS_COUNT],
                        const PID_Config_t speed_config[MOTOR_AXIS_COUNT])
{
    control_initialized = 0U;
    memset(axes, 0, sizeof(axes));
    memset(previous_absolute_deg, 0, sizeof(previous_absolute_deg));
    memset(velocity_ff_gain, 0, sizeof(velocity_ff_gain));
    (void)motor_Init(&axes[YAW_MOTOR].feedback, MOTOR_YAW_CAN_ID);
    (void)motor_Init(&axes[PITCH_MOTOR].feedback, MOTOR_PITCH_CAN_ID);
    if (angle_config == NULL || speed_config == NULL) { return; }
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        axes[i].rx_status = HAL_BUSY;
        const PID_Config_t *angle = &angle_config[i];
        const PID_Config_t *speed = &speed_config[i];
        PID_Position_Init(&angle_pid[i], angle->kp, angle->ki, angle->kd,
                          angle->integral_limit, angle->max_output);
        PID_Position_Init(&speed_pid[i], speed->kp, speed->ki, speed->kd,
                          speed->integral_limit, speed->max_output);
    }
    control_initialized = 1U;
}

void Motor_Control_Stop(void)
{
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        axes[i].mode = MOTOR_CONTROL_DISABLED;
        axes[i].target_angle_deg = axes[i].actual_angle_deg;
        axes[i].target_speed_rpm = 0.0f;
        axes[i].speed_pid_output = 0.0f;
        axes[i].velocity_ff_output = 0.0f;
        axes[i].base_velocity_ff_rpm = 0.0f;
        axes[i].voltage_command = 0;
        PID_Position_Reset(&angle_pid[i]);
        PID_Position_Reset(&speed_pid[i]);
    }
}

void Motor_Control_ReadFeedback(void)
{
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        Motor_ControlStatus *axis = &axes[i];
        axis->rx_status = motor_Rx(&axis->feedback);
        if (axis->rx_status != HAL_OK) { continue; }
        float absolute = axis->feedback.angle_deg;
        if (!axis->reference_valid)
        {
            axis->actual_angle_deg = (i == PITCH_MOTOR) ? MOTOR_PITCH_START_DEG : 0.0f;
            axis->reference_valid = 1U;
        }
        else
        {
            /* 相邻反馈跨 0/360°时解卷绕，YAW 的180°行程不走最短路。 */
            float delta = absolute - previous_absolute_deg[i];
            if (delta > 180.0f) { delta -= 360.0f; }
            if (delta < -180.0f) { delta += 360.0f; }
            axis->actual_angle_deg += delta;
        }
        previous_absolute_deg[i] = absolute;
    }
}

static void select_mode(Motor_ID motor, Motor_ControlMode mode)
{
    if (axes[motor].mode != mode)
    {
        PID_Position_Reset(&angle_pid[motor]);
        PID_Position_Reset(&speed_pid[motor]);
    }
    axes[motor].mode = mode;
}

void Motor_Set_Speed(Motor_ID motor, float speed)
{
    if (!valid_axis(motor)) { return; }
    if (!isfinite(speed)) { Motor_Control_Stop(); return; }
    select_mode(motor, MOTOR_CONTROL_SPEED);
    axes[motor].target_speed_rpm = CLAMP((float)speed,
        -MOTOR_CONTROL_SPEED_LIMIT_RPM, MOTOR_CONTROL_SPEED_LIMIT_RPM);
}

void Motor_Set_Angle(Motor_ID motor, float angle)
{
    if (!valid_axis(motor)) { return; }
    if (!isfinite(angle)) { Motor_Control_Stop(); return; }
    select_mode(motor, MOTOR_CONTROL_ANGLE);
    axes[motor].target_angle_deg = (motor == YAW_MOTOR) ?
        CLAMP(angle, MOTOR_YAW_MIN_DEG, MOTOR_YAW_MAX_DEG) :
        CLAMP(angle, MOTOR_PITCH_MIN_DEG, MOTOR_PITCH_MAX_DEG);
}

static float motor_WrapDegrees(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle >= 180.0f) { angle -= 360.0f; }
    if (angle < -180.0f) { angle += 360.0f; }
    return angle;
}

void Motor_Set_IMU_Angle(Motor_ID motor, float angle)
{
    if (!valid_axis(motor)) { return; }
    if (!isfinite(angle)) { Motor_Control_Stop(); return; }
    select_mode(motor, MOTOR_CONTROL_IMU_ANGLE);
    axes[motor].target_angle_deg = motor_WrapDegrees(angle);
}

void Motor_Set_Base_Velocity_Feedforward(Motor_ID motor, float speed_rpm)
{
    if (!valid_axis(motor)) { return; }
    if (!isfinite(speed_rpm)) { Motor_Control_Stop(); return; }
    axes[motor].base_velocity_ff_rpm = CLAMP(speed_rpm,
        -MOTOR_CONTROL_SPEED_LIMIT_RPM, MOTOR_CONTROL_SPEED_LIMIT_RPM);
}

void Motor_Set_Velocity_Feedforward_Gain(Motor_ID motor, float gain)
{
    if (!valid_axis(motor) || !isfinite(gain)) { return; }
    velocity_ff_gain[motor] = gain;
}

void Motor_PID_Control(Motor_ID motor, float target_angle)
{
    Motor_Set_Angle(motor, target_angle);
}

float Motor_Get_Angle(Motor_ID motor)
{
    return valid_axis(motor) && axes[motor].reference_valid ? axes[motor].actual_angle_deg : NAN;
}

HAL_StatusTypeDef Motor_Control_Update(float dt_s)
{
    if (!control_initialized) { return HAL_ERROR; }
    int16_t commands[4] = {0};
    uint32_t now = HAL_GetTick();
    float angle_feedback[MOTOR_AXIS_COUNT] = {NAN, NAN};
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f)
    {
        Motor_Control_Stop();
    }
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        /* 单轴速度测试不依赖禁用轴的反馈；启用两轴时仍检查两轴。 */
        if (axes[i].mode == MOTOR_CONTROL_DISABLED) { continue; }
        if (axes[i].mode == MOTOR_CONTROL_ANGLE)
        {
            angle_feedback[i] = Get_Current_Angle((Motor_ID)i);
        }
        else if (axes[i].mode == MOTOR_CONTROL_IMU_ANGLE)
        {
            angle_feedback[i] = Get_Current_IMU_Angle((Motor_ID)i);
        }
        if (!axes[i].reference_valid || !motor_IsOnline(&axes[i].feedback, now) ||
            (axes[i].rx_status != HAL_OK && axes[i].rx_status != HAL_BUSY) ||
            ((axes[i].mode == MOTOR_CONTROL_ANGLE || axes[i].mode == MOTOR_CONTROL_IMU_ANGLE) &&
             !isfinite(angle_feedback[i])))
        {
            Motor_Control_Stop();
            break;
        }
    }
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        Motor_ControlStatus *axis = &axes[i];
        float output = 0.0f;
        axis->speed_pid_output = 0.0f;
        axis->velocity_ff_output = 0.0f;
        if (axis->mode == MOTOR_CONTROL_ANGLE || axis->mode == MOTOR_CONTROL_IMU_ANGLE)
        {
            /* IMU坐标与机械坐标独立；只在IMU模式中取跨±180°的最短角差。 */
            float error = axis->target_angle_deg - angle_feedback[i];
            if (axis->mode == MOTOR_CONTROL_IMU_ANGLE) { error = motor_WrapDegrees(error); }
            axis->target_speed_rpm = CLAMP(PID_Position_Calculate(&angle_pid[i],
                error, dt_s) + axis->base_velocity_ff_rpm,//目标转速 = 位置 PID(角度误差) + 底座运动补偿 rpm
                -MOTOR_CONTROL_SPEED_LIMIT_RPM, MOTOR_CONTROL_SPEED_LIMIT_RPM);
        }
        if (axis->mode == MOTOR_CONTROL_ANGLE || axis->mode == MOTOR_CONTROL_IMU_ANGLE ||
            axis->mode == MOTOR_CONTROL_SPEED)
        {
            /* PID输入只含目标与反馈之差；前馈旁路叠加在PID输出端。 */
            float pid_out;
            float ff_out;

            pid_out = PID_Position_Calculate(&speed_pid[i],
                axis->target_speed_rpm - (float)axis->feedback.speed_rpm, dt_s);
            ff_out = calculate_velocity_ff(axis->target_speed_rpm, velocity_ff_gain[i]);

            axis->speed_pid_output = pid_out;
            axis->velocity_ff_output = ff_out;
            output = pid_out + ff_out;
        }
        if (!isfinite(output))
        {
            Motor_Control_Stop();
            memset(commands, 0, sizeof(commands));
            break;
        }
        /* 限幅只作用于求和后的电压命令，不改变 PID 积分或前馈计算。 */
        axis->voltage_command = motor_LimitVoltage((int32_t)CLAMP(output,
            -(float)MOTOR_VOLTAGE_SAFE_LIMIT, (float)MOTOR_VOLTAGE_SAFE_LIMIT));
        commands[axis->feedback.id - 1U] = axis->voltage_command;
    }
    HAL_StatusTypeDef result = motor_Tx(1U, commands);
    if (result != HAL_OK) { Motor_Control_Stop(); }
    return result;
}

float Get_Current_Angle(Motor_ID motor)
{
    return Motor_Get_Angle(motor);
}

float Get_Current_IMU_Angle(Motor_ID motor)
{
    if (motor == YAW_MOTOR) { return imu_test_get_yaw_angle(); }
    if (motor == PITCH_MOTOR) { return imu_test_get_pitch_angle(); }
    return 0.0f;
}

float Motor_Get_Output(Motor_ID motor)
{
    return valid_axis(motor) ? (float)axes[motor].voltage_command : 0.0f;
}
