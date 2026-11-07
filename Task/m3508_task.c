#include "m3508_task.h"
#include <string.h>

M3508_Status m3508_status;
volatile uint8_t m3508_stop_requested;
static uint32_t last_control_ms;
static uint8_t initialized, abort_pending;

_Static_assert(M3508_ESC_ID >= 1U && M3508_ESC_ID <= 4U,
               "Demo 0x200 supports ESC IDs 1..4");
_Static_assert(M3508_CURRENT_LIMIT_RAW <= 16384U, "C620 current range");
/* 历史固定输出参数检查，保留但不生效：
_Static_assert(M3508_FIXED_CURRENT_RAW >= -16384 && M3508_FIXED_CURRENT_RAW <= 16384,
               "Fixed current must be in the C620 protocol range");
*/

/* 数值限幅，只裁剪输出；不是超限停机保护。 */
static float clamp_output(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

static int16_t read_i16(const uint8_t *p)
{
    uint16_t raw = ((uint16_t)p[0] << 8) | p[1];
    return (int16_t)((raw <= 32767U) ? (int32_t)raw : (int32_t)raw - 65536);
}

/* Same big-endian packing as Demo set_moto_current(); new HAL via BSP. */
static HAL_StatusTypeDef send_current(int16_t current)
{
    uint8_t data[8] = {0};
    uint32_t slot = 2U * (M3508_ESC_ID - 1U);
    data[slot] = (uint8_t)((uint16_t)current >> 8);
    data[slot + 1U] = (uint8_t)current;
    m3508_status.tx_status = can_Tx(0x200U, data, sizeof(data));
    if (m3508_status.tx_status == HAL_OK) { ++m3508_status.tx_submitted; }
    return m3508_status.tx_status;
}

void M3508_Task_Init(void)
{
    memset(&m3508_status, 0, sizeof(m3508_status));
    last_control_ms = HAL_GetTick();
    m3508_stop_requested = 0U;
    m3508_status.init_status = can_Init();
    initialized = (m3508_status.init_status == HAL_OK);
    m3508_status.tx_status = HAL_BUSY;
    m3508_status.state = initialized ? M3508_RUNNING : M3508_WAIT_FEEDBACK;
    m3508_status.target_output_rpm = target_speed;
    abort_pending = 0U;
}

void M3508_Task_Stop(void)
{
    m3508_status.state = M3508_STOPPED;
    m3508_status.command_raw = 0;
    m3508_status.target_output_rpm = 0.0f;
    m3508_status.speed_error_rpm = 0.0f;
    m3508_status.p_output = 0.0f;
    m3508_status.i_output = 0.0f;
    m3508_status.fixed_output = 0.0f;
    m3508_status.output_before_limit = 0.0f;
    abort_pending = 1U; /* Explicit manual stop only. */
    m3508_stop_requested = 0U;
    if (initialized && can_AbortTx() == HAL_OK)
    {
        abort_pending = 0U;
        (void)send_current(0);
    }
}

void M3508_Task_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t data[8] = {0}, length;
    /* 1. 显式手动停机：独立于下面已注释的自动保护。 */
    if (m3508_stop_requested) { M3508_Task_Stop(); }
    /* 2. 接收层：校验报文格式，错误帧仅丢弃，不停机。 */
    if (can_Rx(0x200U + M3508_ESC_ID, data, &length) == HAL_OK)
    {
        uint16_t angle = ((uint16_t)data[0] << 8) | data[1];
        /* Reject malformed telemetry without stopping current commands. */
        if (length == 8U && angle <= 8191U)
        {
            m3508_status.angle_raw = angle;
            m3508_status.rotor_speed_rpm = read_i16(data + 2);
            m3508_status.torque_current_raw = read_i16(data + 4);
            m3508_status.temperature_c = data[6];
            m3508_status.output_speed_rpm = m3508_status.rotor_speed_rpm / M3508_GEAR_RATIO;
            m3508_status.last_feedback_ms = now;
            ++m3508_status.feedback_count;
        }
    }
    /* 3. 调度/通信层：10 ms 控制周期；CAN 初始化失败自动重试。 */
    uint32_t control_elapsed_ms = now - last_control_ms;
    if (control_elapsed_ms < M3508_PERIOD_MS) { return; }
    last_control_ms = now;
    if (!initialized)
    {
        m3508_status.init_status = can_Init();
        initialized = (m3508_status.init_status == HAL_OK);
        if (!initialized) { return; }
        if (m3508_status.state != M3508_STOPPED) { m3508_status.state = M3508_RUNNING; }
    }
    /* 4. 可选自动保护：默认全部注释，各条件可独立取消注释或删去。
     * 不设置电流超限停机、电机不转/无响应停机、反馈超时停机。 */

    /* 4A. 温度保护（关闭）：
    if (m3508_status.temperature_c >= 80U)
    {
        M3508_Task_Stop();
    }
    */

    /* 4B. CAN 总线错误保护（关闭）：
    extern CAN_HandleTypeDef hcan1;
    if ((hcan1.Instance->ESR & 0x7U) != 0U)
    {
        M3508_Task_Stop();
    }
    */

    /* 4C. 控制周期迟到保护（关闭）：
    if (control_elapsed_ms > 50U)
    {
        M3508_Task_Stop();
    }
    */

    /* 5. 执行手动停机：先取消旧邮箱，再发送零帧。 */
    if (abort_pending)
    {
        if (can_AbortTx() != HAL_OK) { return; }
        abort_pending = 0U;
        (void)send_current(0);
        return;
    }

    if (m3508_status.state == M3508_RUNNING)
    {
        /* 历史固定输出代码，按要求仅注释，不删除：
        m3508_status.command_raw = M3508_FIXED_CURRENT_RAW;
        */

        /* 6A. 目标和误差：输出轴 rpm 换算为转子 rpm。 */
        m3508_status.target_output_rpm = target_speed;
        m3508_status.speed_error_rpm = target_speed * M3508_GEAR_RATIO
                                     - (float)m3508_status.rotor_speed_rpm;

        /* 6B. P 项与离散积分项；I 项限制在 +/-I_OUTPUT_LIMIT。 */
        m3508_status.p_output = KP * m3508_status.speed_error_rpm;
        m3508_status.i_output += KI * m3508_status.speed_error_rpm;
        m3508_status.i_output = clamp_output(m3508_status.i_output, I_OUTPUT_LIMIT);

        /* 6C. 始终相加：FIXED_CURRENT 为0时闭环照常运行。 */
        m3508_status.fixed_output = FIXED_CURRENT;
        m3508_status.output_before_limit = m3508_status.p_output
                                        + m3508_status.i_output
                                        + m3508_status.fixed_output;

        /* 6D. C620 协议编码范围限制，超出仅裁剪，不停机。 */
        m3508_status.command_raw = (int16_t)clamp_output(
            m3508_status.output_before_limit, (float)M3508_CURRENT_LIMIT_RAW);
    }
    /* 7. 发送层：失败仅记录，下一周期继续发送。 */
    (void)send_current(m3508_status.command_raw);

    /* 7A. 发送失败保护（关闭）：
    if (m3508_status.tx_status != HAL_OK)
    {
        M3508_Task_Stop();
    }
    */
}

const Task_Interface_t M3508_Task = { M3508_Task_Init, M3508_Task_Update };
