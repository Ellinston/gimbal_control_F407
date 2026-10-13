#include "motor_task.h"
#include "can.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    uint32_t i = id - 0x205U;
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
static void reset(uint32_t now)
{
    tick = now; attempts = aborts = 0U;
    init_result = tx_result = abort_result = HAL_OK;
    memset(pending, 0, sizeof(pending)); memset(sent, 0xFF, sizeof(sent));
    huart1.gState = HAL_UART_STATE_READY;
    Double_Motor_Task.Init();
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
    position_sample(0.0f, 100.0f);
    assert(double_motor_status.state == DOUBLE_MOTOR_RUNNING);
}

static void test_position_loop(void)
{
    reset(0U); Position_Loop_Task.Init();
    Position_Loop_Task.Update(); zero();
    assert(strstr((const char *)inflight, "Current=NAdeg") != NULL);
    tick = 10U; inject(0U, 0.0f, 0); Position_Loop_Task.Update(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_WAIT_FEEDBACK);
    tick = 20U; inject(1U, 0.0f, 0); Position_Loop_Task.Update(); zero();
    tick = 29U; position_sample(0.0f, 100.0f); zero();
    uint32_t before = attempts;
    tick = 30U; position_sample(0.0f, 100.0f);
    assert(attempts == before + 1U);
    assert(sent[0] == 0x04U && sent[1] == 0xB0U && sent[2] == 0xFBU && sent[3] == 0x50U);
    assert(Motor_Get_Status(YAW_MOTOR)->target_angle_deg == 30.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_angle_deg == 70.0f);
    char snapshot[192]; uint16_t saved_length = inflight_length;
    memcpy(snapshot, inflight, saved_length);
    tick = 100U; position_sample(0.0f, 100.0f);
    assert(memcmp(snapshot, inflight, saved_length) == 0 && double_motor_status.log_dropped > 0U);
    huart1.gState = HAL_UART_STATE_READY;
    tick = 200U; position_sample(0.0f, 100.0f);
    assert(strstr((const char *)inflight, "[PID] state=1 YAW: Target=30.0deg, Current=0.0deg, Error=30.0deg") != NULL);
    assert(strstr((const char *)inflight, "PITCH: Target=70.0deg, Current=100.0deg, Error=-30.0deg") != NULL);
    for (tick = 210U; tick <= 2000U; tick += 10U)
    {
        position_sample(30.0f, 70.0f);
        assert(double_motor_status.yaw_target_deg == 30.0f && double_motor_status.pitch_target_deg == 70.0f);
        assert(abs(Motor_Get_Status(YAW_MOTOR)->voltage_command) <= 2);
        assert(abs(Motor_Get_Status(PITCH_MOTOR)->voltage_command) <= 2);
    }
    Motor_PID_Control(PITCH_MOTOR, 999.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_angle_deg == MOTOR_PITCH_MAX_DEG);
    Motor_PID_Control(YAW_MOTOR, -999.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->target_angle_deg == MOTOR_YAW_MIN_DEG);
    puts("PASS fixed 30/70 targets, 10ms grouped output, angle error logs and UART buffer ownership");

    position_start(); tick = 10U; position_sample(0.0f, 100.0f);
    tick = 111U; inject(0U, 0.0f, 0); Position_Loop_Task.Update(); zero();
    assert(double_motor_status.state == DOUBLE_MOTOR_FEEDBACK_LOST);
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
    reset(UINT32_MAX - 5U); Position_Loop_Task.Init(); position_sample(0.0f, 100.0f);
    tick = 3U; position_sample(0.0f, 100.0f); zero();
    tick = 4U; position_sample(0.0f, 100.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1200);
    puts("PASS fixed position feedback loss, TX fault, late scheduling, stop latch and tick wrap");
}

static void test_independent_pid_config(void)
{
    PID_Config_t angle[MOTOR_AXIS_COUNT] = {
        [YAW_MOTOR] = {.kp = 20.0f, .max_output = 1500.0f, .integral_limit = 1500.0f},
        [PITCH_MOTOR] = {.kp = 60.0f, .max_output = 1500.0f, .integral_limit = 1500.0f}
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
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 200);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 600);
    /* 初始化复制参数；改变调用方配置和 YAW 目标不影响 PITCH。 */
    angle[YAW_MOTOR].kp = 999.0f;
    Motor_Set_Angle(YAW_MOTOR, 5.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 100);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == 600);
    Motor_Set_Angle(YAW_MOTOR, 180.0f); Motor_Set_Angle(PITCH_MOTOR, 55.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1500);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == -1500);
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

int main(void)
{
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
    assert(sent[0] == 0U && sent[1] == 40U && sent[2] == 0U && sent[3] == 40U);
    Motor_Set_Angle(YAW_MOTOR, 999.0f); Motor_Set_Angle(PITCH_MOTOR, -999.0f);
    assert(Motor_Get_Status(YAW_MOTOR)->target_angle_deg == 180.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_angle_deg == 55.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1500);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == -1500);
    Motor_Set_Speed(YAW_MOTOR, 1000); Motor_Set_Speed(PITCH_MOTOR, -1000);
    assert(Motor_Get_Status(YAW_MOTOR)->target_speed_rpm == 10.0f);
    assert(Motor_Get_Status(PITCH_MOTOR)->target_speed_rpm == -10.0f);
    assert(Motor_Control_Update(0.01f) == HAL_OK);
    assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 1004);
    assert(Motor_Get_Status(PITCH_MOTOR)->voltage_command == -1004);
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
    tick = 4U; sample(0.0f, 100.0f); assert(Motor_Get_Status(YAW_MOTOR)->voltage_command == 40);
    init_result = HAL_ERROR; Double_Motor_Task.Init(); before = attempts;
    Double_Motor_Task.Update(); assert(!double_motor_status.initialized && attempts == before);
    puts("PASS asynchronous UART buffer ownership, tick wrap, init failure");
    test_independent_pid_config();
    test_position_loop();
    puts("ALL DOUBLE/FIXED POSITION HOST CHECKS PASSED (mock CAN/UART; not physical motion proof)");
    return 0;
}

