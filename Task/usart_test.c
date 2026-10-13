#include "usart_test.h"
#include "usart.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define VOFA_PERIOD_MS 100U

imu_vofa_data_t imu_vofa_data;

motor_channel_t yaw_channel;
motor_channel_t pitch_channel;
static uint32_t vofa_last_sample_ms;
static uint8_t vofa_initialized;
/* 必须持久保存到中断发送完成；更新前先经 BSP 确认发送空闲。 */
static uint8_t vofa_packet[20];
static volatile HAL_StatusTypeDef vofa_tx_status = HAL_BUSY;
static volatile uint32_t vofa_tx_submitted, vofa_tx_dropped;
/* 调试器查看本次打包值及 vofa_packet[0..19]，不向 USART1 插入 printf。 */
static volatile imu_vofa_data_t vofa_debug_sample;

_Static_assert(sizeof(float) == 4U, "JustFloat requires 32-bit floats");
_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && FLT_MIN_EXP == -125,
               "JustFloat requires binary32 float format");
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
               "JustFloat memcpy packing requires little-endian byte order");

void vofa_task_init(void)
{
    imu_vofa_data = (imu_vofa_data_t){NAN, NAN, NAN, NAN};
    vofa_debug_sample = imu_vofa_data;
    yaw_channel = (motor_channel_t){0.0f, NAN, 0.0f, NAN};
    pitch_channel = yaw_channel;
    vofa_last_sample_ms = usart_GetTick();
    vofa_tx_status = HAL_BUSY;
    vofa_tx_submitted = vofa_tx_dropped = 0U;
    vofa_initialized = 1U;
    /* 不改写 vofa_packet，重复初始化也不能破坏正在发送的帧。 */
}

void vofa_task_update(void)
{
    uint32_t now = usart_GetTick();
    if (!vofa_initialized || (uint32_t)(now - vofa_last_sample_ms) < VOFA_PERIOD_MS)
    {
        return;
    }
    vofa_last_sample_ms = now;
    if (!usart_TxReady())
    {
        vofa_tx_status = HAL_BUSY;
        ++vofa_tx_dropped;
        return;
    }
    const imu_vofa_data_t sample = imu_vofa_data;
    vofa_debug_sample = sample;
    /* 逐通道写入固定偏移；上面的编译期检查保证32位float与小端字节序。 */
    memcpy(&vofa_packet[0],  &sample.pitch_angle, 4U); /* CH0: Pitch deg */
    memcpy(&vofa_packet[4],  &sample.yaw_angle,   4U); /* CH1: Yaw deg */
    memcpy(&vofa_packet[8],  &sample.accel_x,     4U); /* CH2: Acc X m/s^2 */
    memcpy(&vofa_packet[12], &sample.accel_y,     4U); /* CH3: Acc Y m/s^2 */
    vofa_packet[16] = 0x00U;
    vofa_packet[17] = 0x00U;
    vofa_packet[18] = 0x80U;
    vofa_packet[19] = 0x7FU;
    vofa_tx_status = usart_Tx(vofa_packet, sizeof(vofa_packet));
    if (vofa_tx_status == HAL_OK) { ++vofa_tx_submitted; }
    else { ++vofa_tx_dropped; }
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

