#include "motor.h"
#include "pid.h"
#include <math.h>
#include <string.h>

static Motor_ControlStatus axes[MOTOR_AXIS_COUNT];
static PID_Position angle_pid[MOTOR_AXIS_COUNT], speed_pid[MOTOR_AXIS_COUNT];
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

void Motor_Set_Speed(Motor_ID motor, int16_t speed)
{
    if (!valid_axis(motor)) { return; }
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
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f)
    {
        Motor_Control_Stop();
    }
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        if (!axes[i].reference_valid || !motor_IsOnline(&axes[i].feedback, now) ||
            (axes[i].rx_status != HAL_OK && axes[i].rx_status != HAL_BUSY))
        {
            Motor_Control_Stop();
            break;
        }
    }
    for (uint32_t i = 0U; i < MOTOR_AXIS_COUNT; ++i)
    {
        Motor_ControlStatus *axis = &axes[i];
        float output = 0.0f;
        if (axis->mode == MOTOR_CONTROL_ANGLE)
        {
            output = PID_Position_Calculate(&angle_pid[i],
                axis->target_angle_deg - axis->actual_angle_deg, dt_s);
        }
        else if (axis->mode == MOTOR_CONTROL_SPEED)
        {
            output = PID_Position_Calculate(&speed_pid[i],
                axis->target_speed_rpm - (float)axis->feedback.speed_rpm, dt_s);
        }
        axis->voltage_command = motor_LimitVoltage((int32_t)output);
        commands[axis->feedback.id - 1U] = axis->voltage_command;
    }
    HAL_StatusTypeDef result = motor_Tx(1U, commands);
    if (result != HAL_OK) { Motor_Control_Stop(); }
    return result;
}
