#include "can_test.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

#define CAN_TEST_TX_PERIOD_MS 10U
#define CAN_TEST_PRINT_PERIOD_MS 500U
#define CAN_TEST_RX_TIMEOUT_MS 100U

CAN_TestRxBuffer yaw_rx_buffer;
CAN_TestRxBuffer pitch_rx_buffer;
HAL_StatusTypeDef can_test_init_status = HAL_ERROR;

static uint32_t last_tx_ms;
static uint32_t last_print_ms;
static uint32_t last_retry_ms;
static char message[640]; /* usart_Tx 是异步接口，发送完成前保持内容不变。 */

void CAN_TestInit(void)
{
    memset(&yaw_rx_buffer, 0, sizeof(yaw_rx_buffer));
    memset(&pitch_rx_buffer, 0, sizeof(pitch_rx_buffer));
    can_test_init_status = can_Init();
    last_tx_ms = HAL_GetTick();
    last_print_ms = last_tx_ms;
    last_retry_ms = last_tx_ms;
}

static void CAN_TestReceive(uint32_t id, CAN_TestRxBuffer *buffer, uint32_t now)
{
    uint8_t data[CAN_DATA_CAPACITY];
    uint8_t length;
    if (can_Rx(id, data, &length) == HAL_OK)
    {
        /* 手册规定 GM6020 反馈 DLC=8；短帧不替换有效电机缓存。 */
        if (length != CAN_DATA_CAPACITY) { return; }
        memcpy(buffer->data, data, sizeof(buffer->data));
        buffer->length = length;
        buffer->received = 1U;
        buffer->frame_id = id;
        buffer->last_rx_ms = now;
        ++buffer->updates;
    }
}

static int CAN_TestFormatRx(char *text, size_t capacity, const char *axis,
                            uint32_t motor_id, uint32_t frame_id,
                            const CAN_TestRxBuffer *buffer, uint32_t now)
{
    if (buffer->received == 0U)
    {
        return snprintf(text, capacity, "%s: motor=%lu ID=0x%03lX WAIT RX\r\n",
                        axis, (unsigned long)motor_id, (unsigned long)frame_id);
    }
    /* 最多 8 个十六进制字节，不把原始 CAN 数据作为 C 字符串使用。 */
    char hex[CAN_DATA_CAPACITY * 3U + 1U] = {0};
    for (uint32_t i = 0U; i < buffer->length; ++i)
    {
        (void)snprintf(hex + i * 3U, sizeof(hex) - i * 3U,
                       "%02X ", (unsigned int)buffer->data[i]);
    }
    uint32_t age = (uint32_t)(now - buffer->last_rx_ms);
    return snprintf(text, capacity,
        "%s: motor=%lu ID=0x%03lX DLC=%u DATA=%supdates=%lu age=%lums %s\r\n",
        axis, (unsigned long)motor_id, (unsigned long)buffer->frame_id,
        (unsigned int)buffer->length, hex,
        (unsigned long)buffer->updates, (unsigned long)age,
        (age <= CAN_TEST_RX_TIMEOUT_MS) ? "FRESH" : "STALE");
}

void CAN_Test(void)
{
    uint32_t now = HAL_GetTick();
    if (can_test_init_status == HAL_OK)
    {
        CAN_TestReceive(CAN_TEST_YAW_FEEDBACK_ID, &yaw_rx_buffer, now);
        CAN_TestReceive(CAN_TEST_PITCH_FEEDBACK_ID, &pitch_rx_buffer, now);

        /* GM6020 拨码 1/2 共用 0x1FF，必须在一帧中更新两个槽位，
         * 避免分轴发帧时另一轴被清零。本阶段两个控制量固定为零。 */
        if (((uint32_t)(now - last_tx_ms) >= CAN_TEST_TX_PERIOD_MS) &&
            ((uint32_t)(now - last_retry_ms) >= 1U))
        {
            const int16_t yaw_voltage_command = 0;
            const int16_t pitch_voltage_command = 0;
            uint8_t payload[CAN_DATA_CAPACITY] = {0};
            payload[0] = (uint8_t)((uint16_t)yaw_voltage_command >> 8);
            payload[1] = (uint8_t)yaw_voltage_command;
            payload[2] = (uint8_t)((uint16_t)pitch_voltage_command >> 8);
            payload[3] = (uint8_t)pitch_voltage_command;
            last_retry_ms = now;
            if (can_Tx(CAN_TEST_CONTROL_ID, payload, sizeof(payload)) == HAL_OK)
            {
                last_tx_ms = now;
            }
        }
    }

    if (((uint32_t)(now - last_print_ms) < CAN_TEST_PRINT_PERIOD_MS) ||
        (huart1.gState != HAL_UART_STATE_READY)) { return; }

    int length = snprintf(message, sizeof(message),
        "CAN: init=%u submit=%lu complete=%lu busy=%lu tx_errors=%lu "
        "rx=%lu last=0x%03lX dropped=%lu fifo_overrun=%lu ESR=0x%08lX\r\n",
        (unsigned int)can_test_init_status, (unsigned long)can_diag.tx_submitted,
        (unsigned long)can_diag.tx_completed, (unsigned long)can_diag.tx_busy,
        (unsigned long)can_diag.tx_errors, (unsigned long)can_diag.rx_frames,
        (unsigned long)can_diag.last_rx_id, (unsigned long)can_diag.rx_dropped,
        (unsigned long)can_diag.fifo_overruns,
        (unsigned long)can_diag.error_status_register);
    if ((length <= 0) || ((size_t)length >= sizeof(message))) { return; }
    size_t used = (size_t)length;
    length = CAN_TestFormatRx(message + used, sizeof(message) - used, "YAW",
                              CAN_TEST_YAW_MOTOR_ID, CAN_TEST_YAW_FEEDBACK_ID,
                              &yaw_rx_buffer, now);
    if ((length <= 0) || ((size_t)length >= sizeof(message) - used)) { return; }
    used += (size_t)length;
    length = CAN_TestFormatRx(message + used, sizeof(message) - used, "PITCH",
                              CAN_TEST_PITCH_MOTOR_ID, CAN_TEST_PITCH_FEEDBACK_ID,
                              &pitch_rx_buffer, now);
    if ((length <= 0) || ((size_t)length >= sizeof(message) - used)) { return; }
    used += (size_t)length;
    if (usart_Tx((const uint8_t *)message, (uint16_t)used) == HAL_OK)
    {
        last_print_ms = now;
    }
}

