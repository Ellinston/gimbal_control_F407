#include "usart_test.h"
#include "usart.h"
#include "m3508_task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define VOFA_PERIOD_MS 100U

imu_vofa_data_t imu_vofa_data;

motor_channel_t yaw_channel;
motor_channel_t pitch_channel;
static uint32_t vofa_last_sample_ms;
static uint8_t vofa_initialized;
/* 必须持久保存到中断发送完成；更新前先经 BSP 确认发送空闲。 */
static char vofa_packet[64];
static volatile HAL_StatusTypeDef vofa_tx_status = HAL_BUSY;
static volatile uint32_t vofa_tx_submitted, vofa_tx_dropped;
/* STOP 接收启动状态只供调试器查看，不往 VOFA 数据流中插入回复文本。 */
static volatile HAL_StatusTypeDef stop_rx_status = HAL_BUSY;

void usart_send_STOP(void)
{
    /* 解析状态跨主循环保留：串口可能先收到 ST，下一轮才收到 OP\n。 */
    static char command[8];
    static uint8_t command_length, discard_line;
    static uint32_t last_rx_fault_count;
    uint8_t buffer[32];

    /* 1. 独立启动/恢复 USART1 接收，不等待 CAN 反馈、VOFA 周期或发送空闲。
     * 即使重新启动接收失败，仍处理队列中已有的字节；状态可在调试器查看。 */
    stop_rx_status = usart_Start();
    uint32_t rx_fault_count = usart_GetRxFaultCount();
    if (rx_fault_count != last_rx_fault_count)
    {
        /* 丢字节/接收出错后丢弃当前行，避免把损坏的命令拼成 STOP。 */
        last_rx_fault_count = rx_fault_count;
        command_length = 0U;
        discard_line = 1U;
    }

    /* 2. 每轮最多取 32 字节，不等待完整命令，不阻塞电机任务。 */
    uint16_t length = usart_Rx(buffer, sizeof(buffer));
    for (uint16_t i = 0U; i < length; ++i)
    {
        if (buffer[i] == '\n')
        {
            /* LF 表示一行结束；只移除末尾 CR，兼容 STOP\r\n。 */
            if (command_length > 0U && command[command_length - 1U] == '\r')
            {
                --command_length;
            }
            command[command_length] = '\0';

            /* 3. 仅完整、大写 STOP 触发完整停机流程，不直接修改 state。
             * Stop 清零控制量并取消旧 CAN 邮箱；未完成的取消由 Update 重试。 */
            if (!discard_line && strcmp(command, "STOP") == 0)
            {
                M3508_Task_Stop();
            }
            command_length = 0U;
            discard_line = 0U;
        }
        else if (!discard_line)
        {
            /* 超长行或嵌入 NUL 的行整行忽略，直到下一个 LF 再重新识别。 */
            if (buffer[i] == '\0' || command_length >= sizeof(command) - 1U)
            {
                discard_line = 1U;
            }
            else
            {
                command[command_length++] = (char)buffer[i];
            }
        }
    }
}

void vofa_task_init(void)
{
    imu_vofa_data = (imu_vofa_data_t){NAN, NAN, NAN, NAN};
    /* 保留旧任务依赖的快照变量；本次 VOFA 不再输出 IMU/双轴数据。 */
    yaw_channel = (motor_channel_t){0};
    yaw_channel.current_angle = NAN;
    yaw_channel.current_speed = NAN;
    pitch_channel = yaw_channel;
    vofa_last_sample_ms = usart_GetTick();
    vofa_tx_status = HAL_BUSY;
    vofa_tx_submitted = vofa_tx_dropped = 0U;
    vofa_initialized = 1U;
    /* 不改写 vofa_packet，重复初始化也不能破坏正在发送的帧。 */
}

void vofa_task_enable_yaw_speed(void)
{
    /* 兼容旧电机测试的调用；不再切换 VOFA 通道映射。 */
}

void vofa_task_enable_motor_position(void)
{
    /* 兼容旧电机测试的调用；不再切换 VOFA 通道映射。 */
}

void vofa_task_update(void)
{
    /* 1. 按原来的 100 ms 周期采样；忙时跳过，不积压、不阻塞控制环。 */
    uint32_t now = usart_GetTick();
    if (!vofa_initialized || (uint32_t)(now - vofa_last_sample_ms) < VOFA_PERIOD_MS)
    {
        return;
    }
    vofa_last_sample_ms = now;
    /* 2. M3508 任务先更新 CAN 反馈。本任务只读控制量，不再次消费 CAN 帧。
     * feedback_count 非零表示收到过有效反馈，不表示反馈一直新鲜。
     * 串口接收启动成功即可，不要求电脑先发数据才显示波形。 */
    if (m3508_status.init_status != HAL_OK || m3508_status.feedback_count == 0U)
    {
        return;
    }
    if (usart_Start() != HAL_OK || !usart_TxReady())
    {
        vofa_tx_status = HAL_BUSY;
        ++vofa_tx_dropped;
        return;
    }
    /* 3. 用 M3508 实际停机状态替代旧的正弦速度模式开关。
     * 停机后不提交新包；已经交给中断发送的包允许正常完成。 */
    if (m3508_status.state != M3508_STOPPED)
    {
        /* 4. 同一次主循环中取两路快照，单位均为 C620 电流控制码。
         * CH0 = 固定偏置；CH1 = P + I + 固定偏置，尚未最终限幅。 */
        const float fixed_output = m3508_status.fixed_output;
        const float output_before_limit = m3508_status.output_before_limit;
        /* FireWater：ASCII，逗号分隔，换行结束；不使用 JustFloat 二进制尾。 */
        int length = snprintf(vofa_packet, sizeof(vofa_packet), "M3508:%.2f,%.2f\n",
                              (double)fixed_output, (double)output_before_limit);
        if (length <= 0 || (size_t)length >= sizeof(vofa_packet))
        {
            vofa_tx_status = HAL_ERROR;
            ++vofa_tx_dropped;
            return;
        }
        /* 5. 异步发送；静态缓冲区在发送完成前不会改写。 */
        vofa_tx_status = usart_Tx((const uint8_t *)vofa_packet, (uint16_t)length);
        if (vofa_tx_status == HAL_OK) { ++vofa_tx_submitted; }
        else { ++vofa_tx_dropped; }
    }
}

/* 调试器中分别观察接收启动状态、发送提交状态和错误次数。 */
static volatile HAL_StatusTypeDef usart_start_status = HAL_ERROR;
static volatile HAL_StatusTypeDef usart_tx_status = HAL_OK;
static volatile uint32_t usart_tx_error_count;
static uint32_t pending_pong_count;

void USART_PingTest(void)
{
    /* 调试标志：0 = 尚未通过，1 = 曾经通过；保持到程序复位。 */
    static volatile uint8_t rx_started = 0U;
    static volatile uint8_t rx_data_received = 0U;
    static volatile uint8_t ping_matched = 0U;
    static volatile uint8_t pong_submitted = 0U;

    static const uint8_t ping[] = "PING";
    static const uint8_t pong[] = "PONG\n";
    static uint8_t matched_length;
    static uint8_t invalid_line;
    uint8_t buffer[32];
    uint16_t length;

    /* 第 1 层：启动接收成功，rx_started 变为 1。 */
    usart_start_status = usart_Start();
    if ((usart_start_status == HAL_OK) && (rx_started == 0U))
    {
        rx_started = 1U;
    }

    /* 第 2 层：从 BSP 读到字节，rx_data_received 变为 1。 */
    length = usart_Rx(buffer, sizeof(buffer));
    if ((length != 0U) && (rx_data_received == 0U))
    {
        rx_data_received = 1U;
    }

    /* 按换行分帧：允许分批到达和多条粘连，异常/过长行整行忽略。 */
    for (uint16_t i = 0U; i < length; ++i)
    {
        if (buffer[i] == '\n')
        {
            if ((invalid_line == 0U) &&
                (matched_length == (sizeof(ping) - 1U)) &&
                (pending_pong_count < UINT32_MAX))
            {
                /* 第 3 层：识别到完整 PING 加 LF，ping_matched 变为 1。 */
                if (ping_matched == 0U)
                {
                    ping_matched = 1U;
                }
                ++pending_pong_count;
            }
            matched_length = 0U;
            invalid_line = 0U;
        }
        else if (invalid_line == 0U)
        {
            if ((matched_length < (sizeof(ping) - 1U)) &&
                (buffer[i] == ping[matched_length]))
            {
                ++matched_length;
            }
            else
            {
                invalid_line = 1U;
            }
        }
    }

    if (pending_pong_count != 0U)
    {
        usart_tx_status = usart_Tx(pong, sizeof(pong) - 1U);
        if (usart_tx_status == HAL_OK)
        {
            /* 第 4 层：回复已提交发送，pong_submitted 变为 1。
             * 这里只表示提交成功，电脑是否收到仍要看串口助手。 */
            if (pong_submitted == 0U)
            {
                pong_submitted = 1U;
            }
            --pending_pong_count;
        }
        else if (usart_tx_status != HAL_BUSY)
        {
            ++usart_tx_error_count;
        }
        /* HAL_BUSY 保留待回复计数，下一轮重试；pong 始终有效且不改写。 */
    }
}

