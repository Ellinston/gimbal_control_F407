#include "motor_single_test.h"
#include "can.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
Motor single_yaw_motor;
MotorSingle_Status motor_single_status;
static uint32_t init_ms, last_tx_ms;

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
    last_tx_ms = init_ms - MOTOR_SINGLE_TX_PERIOD_MS;
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
    if ((uint32_t)(now - last_tx_ms) >= MOTOR_SINGLE_TX_PERIOD_MS)
    {
        const int16_t command[4] = {motor_LimitVoltage(MOTOR_SINGLE_COMMAND), 0, 0, 0};
        motor_single_status.command = command[0];
        last_tx_ms = now;
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

