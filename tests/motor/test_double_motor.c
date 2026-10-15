#include "motor_task.h"
#include "pid.h"
#include "can.h"
#include "usart.h"
#include "usart_test.h"
#include "velocity_feedforward.h"
#include "imu_test.h"
#include "imu_level_config.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

UART_HandleTypeDef huart1 = {HAL_UART_STATE_READY};
static uint32_t tick, attempts, aborts;
static HAL_StatusTypeDef init_result, tx_result, abort_result;
static uint8_t pending[2], length[2], feedback[2][8], sent[8];
static const uint8_t *inflight;
static uint16_t inflight_length;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef can_Init(void) { return init_result; }
HAL_StatusTypeDef can_AbortTx(void) { ++aborts; return abort_result; }
HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t size)
{
    assert(id == 0x1FFU && size == 8U);
    ++attempts;
    memcpy(sent, data, 8U);
    for (uint32_t i = 4U; i < 8U; ++i) { assert(sent[i] == 0U); }
    return tx_result;
}
HAL_StatusTypeDef can_Rx(uint32_t id, uint8_t *data, uint8_t *size)
{
    assert(id == 0x205U || id == 0x206U);
    uint32_t i = id == 0x204U + MOTOR_YAW_CAN_ID ? YAW_MOTOR : PITCH_MOTOR;
    if (!pending[i]) { return HAL_BUSY; }
    pending[i] = 0U; *size = length[i]; memcpy(data, feedback[i], 8U);
    return HAL_OK;
}
HAL_StatusTypeDef usart_Tx(const uint8_t *data, uint16_t size)
{
    assert(huart1.gState == HAL_UART_STATE_READY && size < 192U);
    inflight = data; inflight_length = size;
    huart1.gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}
static int16_t submitted_command(Motor_ID axis)
{
    uint32_t slot = axis == YAW_MOTOR ? MOTOR_YAW_CAN_ID - 1U : MOTOR_PITCH_CAN_ID - 1U;
    uint16_t raw = (uint16_t)(((uint16_t)sent[2U * slot] << 8) | sent[2U * slot + 1U]);
    return (int16_t)(raw <= 32767U ? (int32_t)raw : (int32_t)raw - 65536);
}

static void inject(uint32_t axis, float degrees, int16_t rpm)
{
    while (degrees < 0.0f) { degrees += 360.0f; }
    while (degrees >= 360.0f) { degrees -= 360.0f; }
    uint16_t raw = (uint16_t)(degrees * (8192.0f / 360.0f));
    memset(feedback[axis], 0, 8U);
    feedback[axis][0] = (uint8_t)(raw >> 8); feedback[axis][1] = (uint8_t)raw;
    feedback[axis][2] = (uint8_t)((uint16_t)rpm >> 8); feedback[axis][3] = (uint8_t)rpm;
    pending[axis] = 1U; length[axis] = 8U;
}
static void zero(void)
{
    for (uint32_t i = 0U; i < 8U; ++i) { assert(sent[i] == 0U); }
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 0);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 0);
}
static void host_control_defaults(void)
{
    /* 历史断言使用固定测试参数，避免受用户正在整定的宏影响。 */
    const PID_Config_t angle[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 0.5f, .max_output = 10.0f, .integral_limit = 10.0f},
        [PITCH_MOTOR] = {.kp = 0.5f, .max_output = 10.0f, .integral_limit = 10.0f}
    };
    const PID_Config_t speed[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 100.0f, .ki = 40.0f, .max_output = 1500.0f, .integral_limit = 1500.0f},
        [PITCH_MOTOR] = {.kp = 100.0f, .ki = 40.0f, .max_output = 1500.0f, .integral_limit = 1500.0f}
    };
    Motor_Control_Init(angle, speed);
}

static void reset(uint32_t now)
{
    tick = now; attempts = aborts = 0U;
    init_result = tx_result = abort_result = HAL_OK;
    memset(pending, 0, sizeof(pending)); memset(sent, 0xFF, sizeof(sent));
    huart1.gState = HAL_UART_STATE_READY;
    inflight = NULL; inflight_length = 0U;
    vofa_task_init();
    Double_Motor_Task.Init();
    host_control_defaults();
    assert(double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK); zero();
}
static void sample(float yaw, float pitch)
{
    inject(0U, yaw, 0); inject(1U, pitch - 100.0f, 0);
    Double_Motor_Task.Update();
}
static void start(void)
{
    reset(0U); sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_RUNNING);
}

static void position_sample(float yaw, float pitch)
{
    inject(0U, yaw, 0); inject(1U, pitch - 100.0f, 0);
    Position_Loop_Task.Update();
}

static void position_start(void)
{
    reset(0U);
    Position_Loop_Task.Init();
    host_control_defaults();
    Motor_Set_Angle(YAW_MOTOR, YAW_TARGET_ANGLE_DEG);
    Motor_Set_Angle(PITCH_MOTOR, PITCH_TARGET_ANGLE_DEG);
    position_sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_RUNNING);
}

static void test_position_loop(void)
{
    reset(0U); Position_Loop_Task.Init();
    host_control_defaults();
    Motor_Set_Angle(YAW_MOTOR, YAW_TARGET_ANGLE_DEG);
    Motor_Set_Angle(PITCH_MOTOR, PITCH_TARGET_ANGLE_DEG);
    Position_Loop_Task.Update(); zero();
    assert(inflight == NULL);
    assert(isnan(yaw_channel.current_angle) && isnan(pitch_channel.current_angle));
    tick = 10U; inject(0U, 0.0f, 0); Position_Loop_Task.Update(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK);
    tick = 20U; inject(1U, 0.0f, 0); Position_Loop_Task.Update(); zero();
    tick = 29U; position_sample(0.0f, 100.0f); zero();
    uint32_t before = attempts;
    tick = 30U; position_sample(0.0f, 100.0f);
    assert(attempts == before + 1U);
    assert(submitted_command(YAW_MOTOR) == 1004 && submitted_command(PITCH_MOTOR) == -1004);
    assert(Motor_Get_Status(YAW_MOTOR)->target_angle_deg == 30.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_angle_deg == 70.0f);
    tick = 100U; position_sample(0.0f, 100.0f);
    assert(inflight == NULL && double_motor_status.log_submitted == 0U);
    assert(yaw_channel.target_angle == 30.0f && yaw_channel.current_angle == 0.0f);
    assert(pitch_channel.target_angle == 70.0f && pitch_channel.current_angle == 100.0f);
    tick = 200U; position_sample(1.0f, 99.0f);
    assert(inflight == NULL);
    for (tick = 210U; tick <= 2000U; tick += 10U)
    {
        position_sample(30.0f, 70.0f);
        assert(double_motor_status.yaw_target_deg == 30.0f && double_motor_status.pitch_target_deg == 70.0f);
        /* 模拟反馈直接跳到目标，速度 PI 会保留此前积累的控制码。 */
        assert(abs(Motor_Get_Status(YAW_MOTOR)->voltage_command) <= 150);
        assert(abs(Motor_Get_Status(PITCH_MOTOR)->voltage_command) <= 150);
    }
    Motor_PID_Control(PITCH_MOTOR, 999.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_angle_deg == MOTOR_PITCH_MAX_DEG);
    Motor_PID_Control(YAW_MOTOR, -999.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->target_angle_deg == MOTOR_YAW_MIN_DEG);
    inject(0U, 30.0f, -12); inject(1U, -30.0f, 34); Position_Loop_Task.Update();
    assert(yaw_channel.current_speed == -12.0f && pitch_channel.current_speed == 34.0f);
    assert(fabsf(yaw_channel.target_speed) < 0.1f &&
           fabsf(pitch_channel.target_speed) < 0.1f);
    puts("PASS fixed 30/70 targets, 10ms grouped output, retained angle/speed snapshots and no text output");

    position_start(); tick = 10U; position_sample(0.0f, 100.0f);
    tick = 111U; inject(0U, 0.0f, 0); Position_Loop_Task.Update(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST);
    assert(isnan(pitch_channel.current_angle) && isnan(pitch_channel.current_speed));
    tick = 112U; position_sample(0.0f, 100.0f); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST);
    position_start(); tick = 10U; tx_result = HAL_BUSY; position_sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_TX_ERROR);
    tick = 11U; tx_result = HAL_OK; Position_Loop_Task.Update(); zero();
    position_start(); tick = 101U; position_sample(0.0f, 100.0f); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_CONTROL_LATE);
    position_start(); tick = 10U; position_sample(0.0f, 100.0f);
    Task_Position_Loop_Test_Stop(); zero();
    tick = 20U; position_sample(0.0f, 100.0f); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_STOPPED);
    reset(UINT32_MAX - 5U); Position_Loop_Task.Init();
    host_control_defaults();
    Motor_Set_Angle(YAW_MOTOR, YAW_TARGET_ANGLE_DEG);
    Motor_Set_Angle(PITCH_MOTOR, PITCH_TARGET_ANGLE_DEG);
    position_sample(0.0f, 100.0f);
    tick = 3U; position_sample(0.0f, 100.0f); zero();
    tick = 4U; position_sample(0.0f, 100.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1004);
    puts("PASS fixed position feedback loss, TX fault, late scheduling, stop latch and tick wrap");
}

static void test_task_pid_mapping(void)
{
    const PID_Config_t expected[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD,
            YAW_SPEED_MAX_OUTPUT, YAW_SPEED_INTEGRAL_LIMIT},
        [PITCH_MOTOR] = {PITCH_SPEED_KP, PITCH_SPEED_KI, PITCH_SPEED_KD,
            PITCH_SPEED_MAX_OUTPUT, PITCH_SPEED_INTEGRAL_LIMIT}
    };
    const float ff_gain[MOTOR_AXIS_COUNT] = {YAW_VELOCITY_FF_GAIN, PITCH_VELOCITY_FF_GAIN};
    PID_Position reference[MOTOR_AXIS_COUNT];
    reset(0U);
    Position_Loop_Task.Init(); /* 使用真实Task映射，不覆盖生产配置。 */
    position_sample(0.0f, 100.0f);
    for (uint32_t id = 0U; id < MOTOR_AXIS_COUNT; ++id)
    {
        PID_Position_Init(&reference[id], expected[id].kp, expected[id].ki,
            expected[id].kd, expected[id].integral_limit, expected[id].max_output);
    }
    assert(Motor_Get_Status(YAW_MOTOR)->feedback.id == MOTOR_YAW_CAN_ID);
    assert(Motor_Get_Status(PITCH_MOTOR)->feedback.id == MOTOR_PITCH_CAN_ID);
    for (uint32_t step = 0U; step < 10U; ++step)
    {
        const float target[MOTOR_AXIS_COUNT] = {step % 2U ? -3.0f : 3.0f, 4.0f};
        inject(0U, 0.0f, 0); inject(1U, 0.0f, 0); Motor_Control_ReadFeedback();
        Motor_Set_Speed(YAW_MOTOR, target[YAW_MOTOR]);
        Motor_Set_Speed(PITCH_MOTOR, target[PITCH_MOTOR]);
        assert(Motor_Control_Update(0.01f) == HAL_OK);
        for (uint32_t id = 0U; id < MOTOR_AXIS_COUNT; ++id)
        {
            const Motor_ControlStatus *axis = Motor_Get_Status((Motor_ID)id);
            float pid = PID_Position_Calculate(&reference[id], target[id], 0.01f);
            float ff = calculate_velocity_ff(target[id], ff_gain[id]);
            assert(axis->speed_pid_output == pid && axis->velocity_ff_output == ff);
            assert(axis->voltage_command == (int16_t)CLAMP(pid + ff,
                -(float)MOTOR_VOLTAGE_SAFE_LIMIT, (float)MOTOR_VOLTAGE_SAFE_LIMIT));
        }
        Position_Loop_Task.Update(); /* 同tick只同步VOFA快照。 */
        assert(yaw_channel.speed_pid_output == Motor_Get_Status(YAW_MOTOR)->speed_pid_output);
        assert(pitch_channel.speed_pid_output == Motor_Get_Status(PITCH_MOTOR)->speed_pid_output);
    }
    vofa_task_enable_yaw_speed();
    huart1.gState = HAL_UART_STATE_READY;
    tick = 100U; vofa_task_update();
    assert(inflight_length == 36U);
    float packet[8]; memcpy(packet, inflight, sizeof(packet));
    assert(packet[6] == yaw_channel.velocity_ff_output);
    assert(packet[7] == yaw_channel.speed_pid_output);
    puts("PASS Task motor_id mapping independent of row order, per-axis macro gains, PID state and YAW VOFA");
}

static void test_independent_pid_config(void)
{
    /* 固定外环输出10rpm，保证速度积分断言独立于生产速度上限。 */
    PID_Config_t angle[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 20.0f, .max_output = 10.0f, .integral_limit = 1500.0f},
        [PITCH_MOTOR] = {.kp = 60.0f, .max_output = 10.0f, .integral_limit = 1500.0f}
    };
    PID_Config_t speed[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 10.0f, .ki = 20.0f, .max_output = 1500.0f, .integral_limit = 1500.0f},
        [PITCH_MOTOR] = {.kp = 30.0f, .max_output = 1500.0f, .integral_limit = 1500.0f}
    };
    reset(0U);
    Motor_Control_Init(angle, speed);
    inject(0U, 0.0f, 0); inject(1U, 0.0f, 0); Motor_Control_ReadFeedback();
    Motor_Set_Angle(YAW_MOTOR, 10.0f); Motor_Set_Angle(PITCH_MOTOR, 110.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == 10.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_speed_rpm == 10.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 102);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 300);
    /* 初始化复制参数；改变调用方配置和 YAW 目标不影响 PITCH。 */
    angle[YAW_MOTOR].kp = 999.0f;
    Motor_Set_Angle(YAW_MOTOR, 5.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 104);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 300);
    Motor_Set_Angle(YAW_MOTOR, 180.0f); Motor_Set_Angle(PITCH_MOTOR, 55.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 106);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == -300);
    Motor_Set_Speed(YAW_MOTOR, 4); Motor_Set_Speed(PITCH_MOTOR, 4);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 40);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 120);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 41);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 120);
    Motor_Control_Stop(); assert(Motor_Control_Update(0.01f) == HAL_OK); zero();
    Motor_Control_Init(NULL, speed); assert(Motor_Control_Update(0.01f) == HAL_ERROR);
    Motor_Control_Init(angle, NULL); assert(Motor_Control_Update(0.01f) == HAL_ERROR);
    puts("PASS independent axis gains, copied configuration, speed integrals, +/-1500 limits and null config");
}

static void test_velocity_feedforward(void)
{
    PID_Config_t angle[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 0.5f, .max_output = 10.0f, .integral_limit = 10.0f},
        [PITCH_MOTOR] = {.kp = 0.5f, .max_output = 10.0f, .integral_limit = 10.0f}
    };
    PID_Config_t speed[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 100.0f, .max_output = 1500.0f, .integral_limit = 1500.0f},
        [PITCH_MOTOR] = {.kp = 100.0f, .max_output = 1500.0f, .integral_limit = 1500.0f}
    };
    assert(calculate_velocity_ff(4.0f, 5.0f) == 20.0f);
    assert(calculate_velocity_ff(-4.0f, 5.0f) == -20.0f);
    assert(calculate_velocity_ff(4.0f, 0.0f) == 0.0f);
    assert(calculate_velocity_ff(NAN, 5.0f) == 0.0f);
    assert(calculate_velocity_ff(FLT_MAX, FLT_MAX) == 0.0f);

    reset(0U);
    Motor_Control_Init(angle, speed);
    inject(0U, 0.0f, 0); inject(1U, 0.0f, 0); Motor_Control_ReadFeedback();
    Motor_Set_Speed(YAW_MOTOR, 4);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->speed_pid_output == 400.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->velocity_ff_output == 0.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 400);
    Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, 5.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->speed_pid_output == 400.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->velocity_ff_output == 20.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 420);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 0);
    Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, NAN); /* 无效增益不覆盖已有值。 */
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 420);
    Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, (float)MOTOR_VOLTAGE_SAFE_LIMIT);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->speed_pid_output == 400.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == MOTOR_VOLTAGE_SAFE_LIMIT);
    assert(Motor_Get_Output(YAW_MOTOR) == (float)MOTOR_VOLTAGE_SAFE_LIMIT);
    Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, 0.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 400);

    Motor_Set_Angle(YAW_MOTOR, 10.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == 5.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->speed_pid_output == 500.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 500);
    Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, 5.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->speed_pid_output == 500.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->velocity_ff_output == 25.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 525);
    Motor_Control_Stop();
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    zero();
    puts("PASS independent velocity FF, zero-gain speed regression, cascade and final safety clamp");

    /* 同一误差序列，两种前馈增益：PID的P/I/D历史必须完全相同。 */
    speed[YAW_MOTOR].ki = 40.0f;
    speed[YAW_MOTOR].kd = 0.1f;
    float pid_history[20];
    for (uint32_t pass = 0U; pass < 2U; ++pass)
    {
        Motor_Control_Init(angle, speed);
        Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, pass ? 5.0f : 0.0f);
        for (uint32_t step = 0U; step < 20U; ++step)
        {
            tick += 10U;
            inject(0U, 0.0f, 1); Motor_Control_ReadFeedback();
            float target = step < 10U ? 4.0f : -4.0f;
            Motor_Set_Speed(YAW_MOTOR, target);
            assert(Motor_Control_Update(0.01f) == HAL_OK);
            const Motor_ControlStatus *yaw = Motor_Get_Status(YAW_MOTOR);
            if (!pass) { pid_history[step] = yaw->speed_pid_output; }
            else
            {
                assert(yaw->speed_pid_output == pid_history[step]);
                assert(yaw->velocity_ff_output == target * 5.0f);
                assert(yaw->voltage_command == (int16_t)(pid_history[step] + target * 5.0f));
            }
        }
    }
    puts("PASS FF bypass: identical PID P/I/D history with zero/nonzero gain and signed output sum");
}

static void sine_start(void)
{
    reset(0U);
    Sine_Speed_Task.Init();
    tick = YAW_SINE_START_DELAY_MS - 1U;
    inject(0U, 0.0f, 0); Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_WAIT_FEEDBACK);
    tick++;
    inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    assert(yaw_sine_status.state == YAW_SINE_RUNNING);
    assert(Motor_Get_Status(PITCH_MOTOR)->mode == MOTOR_CONTROL_DISABLED);
}

static void test_sine_feedforward_only(void)
{
    /* 测试配置独立于用户宏，验证实际Task到JustFloat的完整数据链。 */
    const PID_Config_t angle[MOTOR_AXIS_COUNT] = {0};
    const PID_Config_t speed[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .max_output = 1500.0f, .integral_limit = 1500.0f},
        [PITCH_MOTOR] = {.max_output = 1500.0f, .integral_limit = 1500.0f}
    };
    reset(0U);
    Sine_Speed_Task.Init();
    inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    float safe_angle = (YAW_SINE_MIN_ANGLE_DEG + YAW_SINE_MAX_ANGLE_DEG) * 0.5f;
    tick = YAW_SINE_START_DELAY_MS;
    inject(0U, safe_angle, 0); Sine_Speed_Task.Update();
    assert(yaw_sine_status.state == YAW_SINE_RUNNING);
    Motor_Control_Init(angle, speed);
    Motor_Set_Velocity_Feedforward_Gain(YAW_MOTOR, 100.0f);
    inject(0U, 0.0f, 0); Motor_Control_ReadFeedback();
    uint32_t packets = 0U;
    for (uint32_t step = 1U; step <= 1000U; ++step)
    {
        tick += 10U;
        huart1.gState = HAL_UART_STATE_READY;
        inject(0U, safe_angle, (int16_t)((int32_t)(step % 15U) - 7));
        Sine_Speed_Task.Update();
        const Motor_ControlStatus *yaw = Motor_Get_Status(YAW_MOTOR);
        assert(yaw_sine_status.state == YAW_SINE_RUNNING);
        assert(yaw->speed_pid_output == 0.0f);
        assert(yaw_channel.speed_pid_output == 0.0f);
        assert(yaw->velocity_ff_output == calculate_velocity_ff(yaw->target_speed_rpm, 100.0f));
        assert(yaw->voltage_command == (int16_t)yaw->velocity_ff_output);
        if (step % 10U == 0U)
        {
            float packet[8];
            assert(inflight_length == 36U);
            memcpy(packet, inflight, sizeof(packet));
            assert(packet[4] == yaw->target_speed_rpm);
            assert(packet[5] == (float)yaw->feedback.speed_rpm);
            assert(packet[6] == yaw->velocity_ff_output);
            assert(packet[7] == 0.0f);
            ++packets;
        }
    }
    assert(packets == 100U);
    Task_Sine_Speed_Test_Stop(); zero();
    assert(yaw_channel.speed_pid_output == 0.0f && yaw_channel.velocity_ff_output == 0.0f);
    puts("PASS FF-only sine: 1000 updates with changing feedback; 100 JustFloat frames with PID CH7 exactly zero");
}

static void test_sine_speed(void)
{
    reset(0U); Sine_Speed_Task.Init();
    tick = 4000U; Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_WAIT_FEEDBACK);
    assert(isnan(yaw_channel.current_speed));
    sine_start();
    uint32_t start_ms = tick, before = attempts;
    tick += 9U; Sine_Speed_Task.Update(); assert(attempts == before);
    for (uint32_t t = 10U; t <= 10000U; t += 10U)
    {
        tick = start_ms + t;
        inject(0U, 0.0f, 0); /* PITCH 整段无反馈也可运行。 */
        Sine_Speed_Task.Update();
        float expected = YAW_SINE_AMPLITUDE_RPM * sinf(6.28318530718f * YAW_SINE_FREQUENCY_HZ * (float)t * 0.001f);
        assert(fabsf(yaw_channel.target_speed - expected) < 0.002f);
        assert(yaw_sine_status.state == YAW_SINE_RUNNING);
        assert(Motor_Get_Status(YAW_MOTOR)->mode == MOTOR_CONTROL_SPEED);
        assert(fabsf(Motor_Get_Status(YAW_MOTOR)->velocity_ff_output -
                     yaw_channel.target_speed * YAW_VELOCITY_FF_GAIN) < 0.002f);
        assert(Motor_Get_Status(PITCH_MOTOR)->mode == MOTOR_CONTROL_DISABLED);
        assert(submitted_command(PITCH_MOTOR) == 0);
        for (uint32_t i = 4U; i < 8U; ++i) { assert(sent[i] == 0U); }
    }
    /* 分数rpm进入PI，非有限值停机；大幅值仍由Device限幅。 */
    Motor_Set_Speed(YAW_MOTOR, 0.25f);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == 0.25f);
    Motor_Set_Speed(YAW_MOTOR, MOTOR_CONTROL_SPEED_LIMIT_RPM + 1.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == MOTOR_CONTROL_SPEED_LIMIT_RPM);
    Motor_Set_Speed(YAW_MOTOR, INFINITY);
    assert(Motor_Control_Update(0.01f) == HAL_OK); zero();
    Motor_Set_Speed(YAW_MOTOR, NAN);
    assert(Motor_Control_Update(0.01f) == HAL_OK); zero();
    puts("PASS sine: two periods, fractional rpm, both polarities, 10ms gate and YAW-only CAN slots");

    sine_start(); tick += 100U; huart1.gState = HAL_UART_STATE_READY;
    inject(0U, 0.0f, -3); Sine_Speed_Task.Update();
    assert(inflight_length == 36U);
    float packet[8]; memcpy(packet, inflight, sizeof(packet));
    assert(packet[4] == yaw_channel.target_speed && packet[5] == -3.0f);
    assert(packet[6] == Motor_Get_Status(YAW_MOTOR)->velocity_ff_output);
    assert(packet[7] == Motor_Get_Status(YAW_MOTOR)->speed_pid_output);
    assert(packet[6] == calculate_velocity_ff(packet[4], YAW_VELOCITY_FF_GAIN));
    assert(packet[6] != packet[7]);
    assert(inflight[32] == 0 && inflight[33] == 0 && inflight[34] == 0x80 && inflight[35] == 0x7F);
    uint8_t snapshot[36]; memcpy(snapshot, inflight, 36U);
    yaw_channel.target_speed = 99.0f; tick += 100U; vofa_task_update();
    assert(memcmp(snapshot, inflight, 36U) == 0);
    puts("PASS sine VOFA: eight-channel 36-byte JustFloat, CH4/5 rpm, CH6/7 FF/PID and busy buffer lifetime");

    sine_start(); tick += 101U; Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_FEEDBACK_LOST);
    tick++; inject(0U, 0.0f, 0); Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_FEEDBACK_LOST);
    sine_start(); tick++; inject(0U, 0.0f, 0); length[0] = 7U;
    Sine_Speed_Task.Update(); zero(); assert(yaw_sine_status.state == YAW_SINE_RX_ERROR);
    sine_start(); tick += 101U; inject(0U, 0.0f, 0); Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_CONTROL_LATE);
    sine_start(); tick++; inject(0U, -11.0f, 0); Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_ANGLE_LIMIT);
    sine_start(); tick += 10U; inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    Task_Sine_Speed_Test_Stop(); zero();
    tick += 10U; inject(0U, 0.0f, 0); Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_STOPPED && yaw_channel.target_speed == 0.0f);
    assert(yaw_channel.velocity_ff_output == 0.0f && yaw_channel.speed_pid_output == 0.0f);
    sine_start(); yaw_sine_stop_requested = 1U; Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_STOPPED && !yaw_sine_stop_requested);
    puts("PASS sine: stale/malformed feedback, late control, angle guard and explicit stop latch");

    sine_start(); tick += 10U; tx_result = HAL_BUSY;
    inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    assert(yaw_sine_status.state == YAW_SINE_TX_ERROR);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 0);
    before = attempts; abort_result = HAL_BUSY; tx_result = HAL_OK;
    tick++; Sine_Speed_Task.Update(); assert(attempts == before);
    abort_result = HAL_OK; tick++; Sine_Speed_Task.Update(); zero();
    assert(yaw_sine_status.state == YAW_SINE_TX_ERROR);
    sine_start(); tick += 10U; tx_result = HAL_ERROR;
    inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    assert(yaw_sine_status.state == YAW_SINE_TX_ERROR);
    tx_result = HAL_OK; tick++; Sine_Speed_Task.Update(); zero();
    reset(UINT32_MAX - 3500U); Sine_Speed_Task.Init();
    tick += 3000U; inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    for (uint32_t i = 0; i < 100U; ++i)
    {
        tick += 10U; inject(0U, 0.0f, 0); Sine_Speed_Task.Update();
    }
    assert(yaw_sine_status.state == YAW_SINE_RUNNING);
    assert(fabsf(yaw_channel.target_speed - YAW_SINE_AMPLITUDE_RPM * 0.95105652f) < 0.002f);
    init_result = HAL_ERROR; Sine_Speed_Task.Init(); before = attempts;
    Sine_Speed_Task.Update(); assert(!yaw_sine_status.initialized && attempts == before);
    puts("PASS sine: TX busy/error latch and abort-before-zero retry, tick wrap and init failure");
}

extern void mock_usart_feed(const uint8_t *data, uint16_t length);
extern uint32_t mock_usart_fault_count;
extern float mock_imu_yaw, mock_imu_pitch;

static void serial_bytes(const uint8_t *data, uint16_t size)
{
    mock_usart_feed(data, size);
    Task_Serial_Position_Test();
}

static void serial_line(const char *text)
{
    serial_bytes((const uint8_t *)text, (uint16_t)strlen(text));
}

static void serial_sample(uint32_t elapsed)
{
    tick += elapsed;
    inject(0U, 0.0f, 0);
    inject(1U, 0.0f, 0);
    Task_Serial_Position_Test();
}

static void serial_start(uint32_t now)
{
    mock_usart_fault_count = 0U;
    reset(now);
    Task_Serial_Position_Test_Init();
    serial_sample(0U);
    assert(double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK);
    zero();
}

static void test_serial_position(void)
{
    serial_start(0U);
    mock_imu_yaw = -11.0f; mock_imu_pitch = 12.0f;
    assert(Get_Current_IMU_Angle(YAW_MOTOR) == -11.0f);
    assert(Get_Current_IMU_Angle(PITCH_MOTOR) == 12.0f);
    assert(Get_Current_IMU_Angle((Motor_ID)-1) == 0.0f);
    assert(Get_Current_IMU_Angle(MOTOR_AXIS_COUNT) == 0.0f);
    assert(Motor_Get_Output((Motor_ID)-1) == 0.0f);
    assert(Motor_Get_Output(MOTOR_AXIS_COUNT) == 0.0f);
    assert(Get_Current_Angle(YAW_MOTOR) == Motor_Get_Angle(YAW_MOTOR));
    assert(Get_Current_Angle(PITCH_MOTOR) == 100.0f);
    assert(isnan(Get_Current_Angle((Motor_ID)-1)));
    serial_sample(10U); zero();
    serial_line("SET 30");
    serial_sample(10U); zero();
    assert(motor_serial_accepted == 0U);
    serial_line(" 120\r\n");
    assert(motor_serial_accepted == 1U && double_motor_status.state == DOUBLE_MOTOR_RUNNING);
    uint32_t before = attempts;
    serial_sample(9U); assert(attempts == before);
    serial_sample(1U);
    assert(Motor_Get_Status(YAW_MOTOR)->mode == MOTOR_CONTROL_ANGLE);
    assert(Motor_Get_Status(PITCH_MOTOR)->mode == MOTOR_CONTROL_ANGLE);
    assert(yaw_channel.target_angle == 30.0f && pitch_channel.target_angle == 120.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command > 0);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command > 0);
    serial_line("SET 45 90\nSET 60 110\n");
    assert(motor_serial_accepted == 3U);
    assert(double_motor_status.yaw_target_deg == 60.0f && double_motor_status.pitch_target_deg == 110.0f);
    serial_line("SET nan 100\nSET 90 inf\nSET 20 90junk\nSET 20\nSET20 90\nSET 30,100\n");
    Task_Serial_Position_Test(); /* 有界消费64字节，处理余下输入。 */
    assert(motor_serial_accepted == 3U && motor_serial_rejected == 6U);
    assert(double_motor_status.yaw_target_deg == 60.0f && double_motor_status.pitch_target_deg == 110.0f);
    const uint8_t embedded_nul[] = {'S','E','T',' ','4','0',' ','1','0','0',0,'x','\n'};
    serial_bytes(embedded_nul, sizeof(embedded_nul));
    char oversized[100]; memset(oversized, '1', sizeof(oversized)); oversized[99] = '\n';
    serial_bytes((const uint8_t *)oversized, sizeof(oversized));
    Task_Serial_Position_Test();
    assert(motor_serial_rejected == 8U && motor_serial_accepted == 3U);
    serial_line("SET 999 -999\n");
    assert(yaw_channel.target_angle == MOTOR_YAW_MAX_DEG && pitch_channel.target_angle == MOTOR_PITCH_MIN_DEG);
    serial_line("SET 25 115\n");
    serial_sample(10U);
    huart1.gState = HAL_UART_STATE_READY;
    tick = 100U; serial_sample(0U); vofa_task_update();
    assert(inflight_length == 28U);
    float packet[6]; memcpy(packet, inflight, sizeof(packet));
    assert(packet[0] == -11.0f && packet[1] == 12.0f);
    assert(packet[2] == 0.0f && packet[3] == 100.0f);
    assert(packet[4] == Motor_Get_Output(YAW_MOTOR));
    assert(packet[5] == Motor_Get_Output(PITCH_MOTOR));
    assert(packet[4] > 0.0f && packet[5] > 0.0f);
    assert(inflight[24] == 0U && inflight[25] == 0U && inflight[26] == 0x80U && inflight[27] == 0x7FU);
    uint8_t snapshot[28]; memcpy(snapshot, inflight, sizeof(snapshot));
    serial_line("SET 70 130\n"); serial_sample(100U); vofa_task_update();
    assert(memcmp(snapshot, inflight, sizeof(snapshot)) == 0);
    serial_line("STOP\nSET 80 140\n"); zero();
    assert(Motor_Get_Output(YAW_MOTOR) == 0.0f && Motor_Get_Output(PITCH_MOTOR) == 0.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_STOPPED);
    serial_sample(10U); zero();
    puts("PASS serial: startup zero, split/glued/CRLF commands, atomic targets, finite input, bounds, STOP latch and six-channel IMU/encoder/output JustFloat");

    serial_start(0U); serial_line("SET 30 120\n"); serial_sample(10U);
    tick += 101U; Task_Serial_Position_Test(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST);
    assert(isnan(yaw_channel.current_angle) && isnan(pitch_channel.current_angle));
    huart1.gState = HAL_UART_STATE_READY; tick += 100U; vofa_task_update();
    memcpy(packet, inflight, sizeof(packet));
    assert(isnan(packet[2]) && isnan(packet[3]));
    assert(packet[4] == 0.0f && packet[5] == 0.0f);
    serial_line("SET 40 110\n"); serial_sample(1U); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST);
    serial_start(0U); serial_line("SET 30 120\n"); serial_sample(10U);
    mock_usart_fault_count++;
    tick++; /* 零帧重试门控为1ms，模拟下一次主循环。 */
    serial_line("SET 40 110\n"); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_SERIAL_ERROR);
    serial_sample(1U); zero();
    serial_start(0U); serial_line("SET 30 120\n");
    tx_result = HAL_BUSY; serial_sample(10U);
    assert(double_motor_status.state == DOUBLE_MOTOR_TX_ERROR);
    tx_result = HAL_OK; serial_sample(1U); zero();
    serial_start(0U); serial_line("SET 30 120\n");
    tick += 101U; serial_sample(0U); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_CONTROL_LATE);
    serial_start(UINT32_MAX - 5U); serial_line("SET 30 120\n"); serial_sample(10U);
    assert(double_motor_status.state == DOUBLE_MOTOR_RUNNING);
    Task_Serial_Position_Test_Stop(); zero();
    serial_start(0U); serial_line("SET 30 120\n");
    tick++; inject(0U, 0.0f, 0); length[0] = 7U;
    Task_Serial_Position_Test(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_RX_ERROR);
    serial_start(0U); serial_line("SET 30 120\n");
    tick++; inject(0U, 200.0f, 0); inject(1U, 0.0f, 0);
    Task_Serial_Position_Test(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_ANGLE_LIMIT);
    puts("PASS serial: feedback loss, RX corruption, malformed CAN, angle guard, TX failure, late cycle, stop output and tick wrap");
}


/* Independent physical chain Rx(base roll)*Ry(base pitch)*Rz(joint yaw)*Ry(joint pitch).
 * Inverse elementary rotations produce sensor observations; no solver helper. */
static float base_roll, base_pitch;
static void stabilize_observation(float yaw_joint, float pitch_joint)
{
    const double rad=3.141592653589793/180.0;
    double br=base_roll*rad,bp=base_pitch*rad,y=yaw_joint*rad,p=pitch_joint*rad;
    double x=-sin(bp)*cos(br),v=sin(br),z=cos(bp)*cos(br);
    double rx=cos(y)*x+sin(y)*v,ry=-sin(y)*x+cos(y)*v;
    imu_attitude_snapshot=(IMU_AttitudeSnapshot){.valid=1,.sample_ms=tick,
        .up_sensor={(float)(cos(p)*rx-sin(p)*z),(float)ry,(float)(sin(p)*rx+cos(p)*z)}};
}
static void stabilize_sample(float yaw_joint, float pitch_joint, int16_t yaw_rpm, int16_t pitch_rpm)
{
    tick += MOTOR_TASK_TX_PERIOD_MS;
    inject(YAW_MOTOR,yaw_joint,yaw_rpm);inject(PITCH_MOTOR,pitch_joint,pitch_rpm);
    stabilize_observation(yaw_joint,pitch_joint);
    huart1.gState=HAL_UART_STATE_READY;IMU_Stabilize_Task.Update();
}
static void stabilize_config(void)
{
    imu_level_config=(GimbalLevelConfig)IMU_LEVEL_CONFIG_INITIALIZER;
    /* Synthetic fixture uses its own identity mounting and joint directions,
     * independent of the measured installation in the application config. */
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
    { imu_level_config.sensor_to_top[i][j]=(i==j)?1.0f:0.0f; }
    imu_level_config.platform_up_top[0]=imu_level_config.platform_up_top[1]=0.0f;
    imu_level_config.platform_up_top[2]=1.0f;
    imu_level_config.zero_deg[0]=imu_level_config.zero_deg[1]=0;
    imu_level_config.direction[0]=imu_level_config.direction[1]=1;
    imu_level_config.calibrated=1;
}
static void stabilize_start(uint32_t now,float roll,float pitch)
{
    reset(now);stabilize_config();base_roll=roll;base_pitch=pitch;
    mock_imu_yaw=mock_imu_pitch=NAN; /* proves legacy getters/VOFA are not control inputs */
    mock_usart_fault_count=0U;imu_attitude_snapshot=(IMU_AttitudeSnapshot){0};
    IMU_Stabilize_Task.Init();stabilize_sample(0,0,0,0);
}
static double platform_error(float yaw,float pitch)
{
    /* Forward normal in world, independent of inverse observation above. */
    double r=3.141592653589793/180.0;
    double x=cos(yaw*r)*sin(pitch*r),y=sin(yaw*r)*sin(pitch*r),z=cos(pitch*r);
    double bx=cos(base_pitch*r)*x+sin(base_pitch*r)*z;
    double bz=-sin(base_pitch*r)*x+cos(base_pitch*r)*z;
    double wy=cos(base_roll*r)*y-sin(base_roll*r)*bz;
    double wz=sin(base_roll*r)*y+cos(base_roll*r)*bz;
    return atan2(hypot(bx,wy),wz)/r;
}
static void test_imu_stabilize(void)
{
    reset(0);imu_level_config=(GimbalLevelConfig)IMU_LEVEL_CONFIG_INITIALIZER;
    imu_level_config.calibrated=0;
    IMU_Stabilize_Task.Init();tick+=10;IMU_Stabilize_Task.Update();zero();
    assert(double_motor_status.state==DOUBLE_MOTOR_LEVEL_ERROR);
    assert(double_motor_status.level_result==LEVEL_CONFIG_INVALID);
    stabilize_config();IMU_Stabilize_Task.Init();imu_attitude_snapshot.valid=0;
    for(unsigned i=0;i<220;++i)
    {
        tick+=10;inject(0,0,0);inject(1,0,0);IMU_Stabilize_Task.Update();zero();
        assert(double_motor_status.state==DOUBLE_MOTOR_WAIT_FEEDBACK);
    }
    base_roll=15;base_pitch=-10;
    stabilize_sample(0,0,0,0);stabilize_sample(0,0,0,0);
    assert(double_motor_status.state==DOUBLE_MOTOR_RUNNING);
    for(unsigned i=0;i<2;++i) { assert(Motor_Get_Status((Motor_ID)i)->mode==MOTOR_CONTROL_ANGLE); }
    assert(Motor_Get_Output(YAW_MOTOR)>0 && Motor_Get_Output(PITCH_MOTOR)>0);
    assert(fabsf(double_motor_status.yaw_target_deg-.2f)<.001);
    assert(fabsf(double_motor_status.pitch_target_deg-100.2f)<.001);
    for(unsigned i=0;i<2;++i)
    {
        const Motor_ControlStatus *axis=Motor_Get_Status((Motor_ID)i);
        float expected=(axis->target_angle_deg-axis->actual_angle_deg)*
            (i==0?YAW_POSITION_KP:PITCH_POSITION_KP);
        assert(fabsf(axis->target_speed_rpm-expected)<.001);
        assert(fabsf(axis->velocity_ff_output-expected*(i==0?YAW_VELOCITY_FF_GAIN:PITCH_VELOCITY_FF_GAIN))<.001);
        assert(submitted_command((Motor_ID)i)==axis->voltage_command);
    }
    puts("PASS leveling: uncalibrated config blocked; wait gravity; encoder PID + rpm + feedforward + unified CAN");

    /* Plant is illustrative, not a validated motor model or physical tuning. */
    stabilize_start(0,15,-10);float joint[2]={0},rpm[2]={0};
    for(unsigned i=0;i<2400;++i)
    {
        if(i==800 || i==1600)
        {
            printf("plant %u: joint %.4f %.4f goal %.4f %.4f error %.4f\n",i,
                (double)joint[0],(double)joint[1],(double)double_motor_status.level_goal_yaw_deg,
                (double)double_motor_status.level_goal_pitch_deg,platform_error(joint[0],joint[1]));
            assert(platform_error(joint[0],joint[1])<1);
            base_roll=i==800?0:-18;base_pitch=i==800?12:-8;
        }
        stabilize_sample(joint[0],joint[1],(int16_t)lroundf(rpm[0]),(int16_t)lroundf(rpm[1]));
        assert(double_motor_status.state==DOUBLE_MOTOR_RUNNING);
        for(unsigned j=0;j<2;++j)
        {
            float output=Motor_Get_Output((Motor_ID)j);assert(fabsf(output)<=MOTOR_VOLTAGE_SAFE_LIMIT);
            rpm[j]+=.1f*(output/15.0f-rpm[j]);joint[j]+=rpm[j]*.06f;
        }
    }
    double error=platform_error(joint[0],joint[1]);assert(error<1);
    printf("PASS independent 3D base/joint plant: mixed/Pitch/opposite Roll; final normal error %.4f deg\n",error);
    mock_usart_feed((const uint8_t *)"ST",2);IMU_Stabilize_Task.Update();
    assert(double_motor_status.state==DOUBLE_MOTOR_RUNNING);
    mock_usart_feed((const uint8_t *)"OP\r\n",4);IMU_Stabilize_Task.Update();zero();
    stabilize_sample(joint[0],joint[1],0,0);zero();assert(double_motor_status.state==DOUBLE_MOTOR_STOPPED);

    stabilize_start(0,0,0);stabilize_sample(0,0,0,0);imu_attitude_snapshot.valid=0;
    ++tick;IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_IMU_ERROR);
    stabilize_sample(0,0,0,0);zero();assert(double_motor_status.state==DOUBLE_MOTOR_IMU_ERROR);
    stabilize_start(0,0,0);stabilize_sample(0,0,0,0);imu_attitude_snapshot.up_sensor[0]=NAN;
    ++tick;IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_IMU_ERROR);
    stabilize_start(0,0,0);stabilize_sample(0,0,0,0);imu_attitude_snapshot.sample_ms=tick-101;
    ++tick;IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_IMU_ERROR);
    stabilize_start(0,0,0);stabilize_sample(0,0,0,0);
    tick+=10;inject(0,0,0);inject(1,0,0);imu_attitude_snapshot.sample_ms=tick-40;
    IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_LEVEL_ERROR);
    stabilize_start(0,0,0);stabilize_sample(0,0,0,0);
    tick+=101;stabilize_observation(0,0);IMU_Stabilize_Task.Update();zero();
    assert(double_motor_status.state==DOUBLE_MOTOR_FEEDBACK_LOST);
    stabilize_start(0,0,0);stabilize_sample(0,0,0,0);
    tick+=10;inject(0,0,0);length[0]=7;inject(1,0,0);stabilize_observation(0,0);
    IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_RX_ERROR);
    stabilize_start(0,0,0);tx_result=HAL_BUSY;stabilize_sample(0,0,0,0);
    assert(double_motor_status.state==DOUBLE_MOTOR_TX_ERROR);
    tx_result=HAL_OK;++tick;IMU_Stabilize_Task.Update();zero();
    stabilize_start(0,0,0);tx_result=HAL_ERROR;stabilize_sample(0,0,0,0);
    assert(double_motor_status.state==DOUBLE_MOTOR_TX_ERROR);
    tx_result=HAL_OK;++tick;IMU_Stabilize_Task.Update();zero();assert(aborts>0);
    stabilize_start(0,0,70);stabilize_sample(0,0,0,0);zero();
    assert(double_motor_status.state==DOUBLE_MOTOR_LEVEL_ERROR && double_motor_status.level_result==LEVEL_UNREACHABLE);
    base_pitch=0;stabilize_sample(0,0,0,0);zero();assert(double_motor_status.state==DOUBLE_MOTOR_LEVEL_ERROR);
    stabilize_start(0,0,0);tick+=101;inject(0,0,0);inject(1,0,0);stabilize_observation(0,0);
    IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_CONTROL_LATE);
    stabilize_start(0,0,0);stabilize_sample(170,0,0,0);stabilize_sample(200,0,0,0);zero();
    assert(double_motor_status.state==DOUBLE_MOTOR_ANGLE_LIMIT);
    stabilize_start(0,0,0);++mock_usart_fault_count;IMU_Stabilize_Task.Update();zero();
    assert(double_motor_status.state==DOUBLE_MOTOR_SERIAL_ERROR);
    stabilize_start(UINT32_MAX-5U,0,0);stabilize_sample(0,0,0,0);
    mock_usart_feed((const uint8_t *)"SET 50 120\n",11);IMU_Stabilize_Task.Update();
    assert(double_motor_status.yaw_target_deg==0 && double_motor_status.pitch_target_deg==100);
    imu_stabilize_stop_requested=1;IMU_Stabilize_Task.Update();zero();assert(double_motor_status.state==DOUBLE_MOTOR_STOPPED);
    /* Boot at arbitrary physical joints: startup 0/100 is NOT geometric zero. */
    reset(0);stabilize_config();base_roll=base_pitch=0;IMU_Stabilize_Task.Init();
    stabilize_sample(27,-12,0,0);stabilize_sample(27,-12,0,0);
    assert(double_motor_status.state==DOUBLE_MOTOR_RUNNING);
    assert(fabsf(double_motor_status.level_goal_pitch_deg-112)<.05);
    assert(fabsf(double_motor_status.level_goal_yaw_deg)<.05);
    puts("PASS IMU invalid/NaN/stale/skew, feedback loss/malformed, TX busy/error, limit/late/unreachable, STOP/serial locks, tick wrap and absolute-zero startup conversion");
}

static void test_abort(int sig) { (void)sig; _Exit(1); }
int main(void)
{
    signal(SIGABRT,test_abort); /* No Windows crash-report dialog in host CI. */
    setvbuf(stdout, NULL, _IONBF, 0);
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
#endif
    assert(Motor_Control_Update(0.01f) == HAL_ERROR);
    reset(0U);
    assert(isnan(Motor_Get_Angle(YAW_MOTOR)) && isnan(Motor_Get_Angle((Motor_ID)-1)));
    for (tick = 10U; tick <= 200U; tick += 10U) { Double_Motor_Task.Update(); zero(); }
    inject(0U, 359.0f, 0); Double_Motor_Task.Update(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK);
    inject(1U, 358.0f, 0); Double_Motor_Task.Update();
    assert(double_motor_status.state == DOUBLE_MOTOR_RUNNING);
    tick += 9U; sample(359.0f, 100.0f); zero();
    /* 首帧参考分别是359°/358°；跨编码器零点后应累加为+2°。 */
    tick += 1U; inject(0U, 1.0f, 0); inject(1U, 0.0f, 0); Double_Motor_Task.Update();
    assert(fabsf(Motor_Get_Angle(YAW_MOTOR) - 2.0f) < 0.05f);
    assert(fabsf(Motor_Get_Angle(PITCH_MOTOR) - 102.0f) < 0.05f);
    puts("PASS no output before both feedback references; 10ms gating; encoder wrap");

    start(); tick = 10U; sample(0.0f, 100.0f);
    assert(sent[0] == 0U && sent[1] == 50U && sent[2] == 0U && sent[3] == 50U);
    Motor_Set_Angle(YAW_MOTOR, 999.0f); Motor_Set_Angle(PITCH_MOTOR, -999.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->target_angle_deg == 180.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_angle_deg == 55.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == 10.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_speed_rpm == -10.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1004);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command <= -1000 &&
           Motor_Get_Status(PITCH_MOTOR)->voltage_command >= -1005);
    Motor_Set_Speed(YAW_MOTOR, 1000); Motor_Set_Speed(PITCH_MOTOR, -1000);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == MOTOR_CONTROL_SPEED_LIMIT_RPM);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_speed_rpm == -MOTOR_CONTROL_SPEED_LIMIT_RPM);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1500);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == -1500);
    Motor_Set_Angle(YAW_MOTOR, NAN); assert(Motor_Control_Update(0.01f) == HAL_OK); zero();
    Motor_Set_Angle(YAW_MOTOR, 10.0f); Motor_Set_Angle(PITCH_MOTOR, 110.0f);
    assert(Motor_Control_Update(NAN) == HAL_OK); zero();
    puts("PASS shared CAN slots, position and speed loops, limits and nonfinite targets/dt");

    start();
    for (uint32_t i = 1U; i <= 200U; ++i) { tick = i * 10U; sample(0.0f, 100.0f); }
    assert(double_motor_status.yaw_target_deg == 180.0f && double_motor_status.yaw_direction == 1);
    assert(double_motor_status.pitch_target_deg == 145.0f && double_motor_status.pitch_direction == 1);
    /* 固定反馈即使经过180次也不换向；随后逐步模拟真实行程。 */
    for (uint32_t i = 1U; i <= 180U; ++i)
    {
        tick += 10U; sample((float)i, 100.0f + (float)(i > 45U ? 45U : i));
    }
    assert(double_motor_status.yaw_direction == -1 && double_motor_status.pitch_direction == -1);
    uint8_t saw_pitch_lower = 0U, saw_yaw_lower = 0U;
    for (uint32_t i = 0U; i < 450U; ++i)
    {
        tick += 10U; sample(double_motor_status.yaw_target_deg, double_motor_status.pitch_target_deg);
        assert(double_motor_status.yaw_target_deg >= 0.0f && double_motor_status.yaw_target_deg <= 180.0f);
        assert(double_motor_status.pitch_target_deg >= 55.0f && double_motor_status.pitch_target_deg <= 145.0f);
        if (double_motor_status.yaw_target_deg == 0.0f && double_motor_status.yaw_direction == 1) { saw_yaw_lower = 1U; }
        if (double_motor_status.pitch_target_deg == 55.0f && double_motor_status.pitch_direction == 1) { saw_pitch_lower = 1U; }
    }
    assert(saw_yaw_lower && saw_pitch_lower);
    puts("PASS both endpoint reversals use feedback; pitch/yaw targets remain inside limits");

    start(); tick = 10U; sample(0.0f, 100.0f);
    tick = 111U; inject(0U, 0.0f, 0); Double_Motor_Task.Update();
    assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST); zero();
    tick = 112U; sample(0.0f, 100.0f); assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST); zero();
    start(); tick = 10U; sample(0.0f, 100.0f);
    tick = 11U; inject(1U, 0.0f, 0); length[1] = 7U; Double_Motor_Task.Update();
    assert(double_motor_status.state == DOUBLE_MOTOR_RX_ERROR); zero();
    start(); tick = 10U; sample(0.0f, 100.0f);
    tick = 11U; sample(0.0f, 160.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_ANGLE_LIMIT); zero();
    start(); tick = 101U; sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_CONTROL_LATE); zero();
    puts("PASS stale/malformed feedback, angle bounds, late scheduling stop and latch both axes");

    start(); tick = 10U; tx_result = HAL_BUSY; sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_TX_ERROR);
    uint32_t before = attempts; abort_result = HAL_BUSY; tx_result = HAL_OK;
    tick = 11U; Double_Motor_Task.Update(); assert(attempts == before);
    abort_result = HAL_OK; tick = 12U; Double_Motor_Task.Update(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_TX_ERROR && aborts >= 3U);
    start(); tick = 10U; sample(0.0f, 100.0f); Task_Double_Motor_Test_Stop(); zero();
    tick = 20U; sample(0.0f, 100.0f); assert(double_motor_status.state == DOUBLE_MOTOR_STOPPED); zero();
    start(); tick = 10U; tx_result = HAL_ERROR; sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_TX_ERROR);
    tick = 11U; tx_result = HAL_OK; Double_Motor_Task.Update(); zero();
    puts("PASS TX busy/error latch; abort queued commands before zero retry; explicit stop");

    start(); char snapshot[192]; memcpy(snapshot, inflight, inflight_length);
    uint16_t snapshot_length = inflight_length;
    for (uint32_t i = 1U; i <= 20U; ++i) { tick = i * 10U; sample(0.0f, 100.0f); }
    assert(memcmp(snapshot, inflight, snapshot_length) == 0 && double_motor_status.log_dropped > 0U);
    huart1.gState = HAL_UART_STATE_READY; tick = 300U; sample(0.0f, 100.0f);
    assert(strstr((const char *)inflight, "[MOTOR]") != NULL);
    reset(UINT32_MAX - 5U); sample(0.0f, 100.0f);
    tick = 3U; sample(0.0f, 100.0f); zero();
    tick = 4U; sample(0.0f, 100.0f); assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 50);
    init_result = HAL_ERROR; Double_Motor_Task.Init(); before = attempts;
    Double_Motor_Task.Update(); assert(!double_motor_status.initialized && attempts == before);
    puts("PASS asynchronous UART buffer ownership, tick wrap, init failure");
    test_independent_pid_config();
    test_task_pid_mapping();
    test_velocity_feedforward();
    test_position_loop();
    test_sine_feedforward_only();
    if (YAW_SINE_MIN_ANGLE_DEG <= 0.0f && YAW_SINE_MAX_ANGLE_DEG >= 0.0f)
    {
        test_sine_speed();
    }
    else
    {
        puts("SKIP historical zero-angle sine startup: current user angle bounds exclude zero");
    }
    test_serial_position();
    test_imu_stabilize();
    puts("ALL DOUBLE/FIXED/SERIAL POSITION HOST CHECKS PASSED (mock CAN/UART; not physical motion proof)");
    return 0;
}

