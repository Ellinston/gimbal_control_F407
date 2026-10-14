#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"

UART_HandleTypeDef huart1;
uint32_t mock_primask;
static uint8_t *receive_target;
static const uint8_t *transmit_target;
static uint16_t transmit_length;
static unsigned receive_calls, transmit_calls;
static HAL_StatusTypeDef receive_result = HAL_OK;
static HAL_StatusTypeDef transmit_result = HAL_OK;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                    uint8_t *data, uint16_t size)
{
    ++receive_calls;
    assert(size == 1U);
    if (receive_result != HAL_OK) return receive_result;
    if (uart->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
    receive_target = data;
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                     const uint8_t *data, uint16_t size)
{
    if (uart->gState != HAL_UART_STATE_READY) return HAL_BUSY;
    if (transmit_result != HAL_OK) return transmit_result;
    transmit_target = data;
    transmit_length = size;
    ++transmit_calls;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    uart->RxState = HAL_UART_STATE_READY;
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    return HAL_OK;
}

/* 使用实际 BSP 和 Task 源码，避免维护第二份解析器。 */
#include "../../Bsp/usart/usart.c"
#include "../../Task/usart_test.c"

/* 本套件验证BSP异步发送和打包；电机/IMU端使用可辨认的独立数值。 */
static Motor_ControlStatus mock_axes[MOTOR_AXIS_COUNT];
const Motor_ControlStatus *Motor_Get_Status(Motor_ID motor) { return &mock_axes[motor]; }
uint8_t motor_IsOnline(const Motor *motor, uint32_t now) { (void)motor; (void)now; return 1U; }
float Get_Current_Angle(Motor_ID motor) { return motor == YAW_MOTOR ? 70.0f : 100.0f; }
float Motor_Get_Output(Motor_ID motor) { return motor == YAW_MOTOR ? 500.0f : -1000.0f; }
float imu_test_get_yaw_angle(void) { return imu_vofa_data.yaw_angle; }
float imu_test_get_pitch_angle(void) { return imu_vofa_data.pitch_angle; }

static void feed(uint8_t value)
{
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);
    *receive_target = value;
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(&huart1);
}

static void feed_text(const char *text)
{
    while (*text != '\0') feed((uint8_t)*text++);
}

static void finish_tx(void)
{
    assert(transmit_length == 5U);
    assert(memcmp(transmit_target, "PONG\n", 5U) == 0);
    huart1.gState = HAL_UART_STATE_READY;
}

static void test_vofa(void)
{
    const uint8_t expected[28] = {
        0x00, 0x00, 0xA0, 0xBF, /* IMU Yaw -1.25 */
        0x00, 0x00, 0xF0, 0x41, /* IMU Pitch 30 */
        0x00, 0x00, 0x8C, 0x42, /* 70 */
        0x00, 0x00, 0xC8, 0x42, /* 100 */
        0x00, 0x00, 0xFA, 0x43, /* Yaw output 500 */
        0x00, 0x00, 0x7A, 0xC4, /* Pitch output -1000 */
        0x00, 0x00, 0x80, 0x7F
    };
    tick = 0U; transmit_calls = 0U;
    vofa_task_init();
    assert(isnan(imu_vofa_data.pitch_angle) && isnan(imu_vofa_data.accel_y));
    imu_vofa_data = (imu_vofa_data_t){30.0f, -1.25f, 70.0f, 100.0f};
    for (tick = 0U; tick < 100U; tick += 10U) { vofa_task_update(); }
    assert(transmit_calls == 0U);
    vofa_task_update();
    assert(transmit_calls == 1U && transmit_length == sizeof(expected));
    assert(memcmp(transmit_target, expected, sizeof(expected)) == 0);
    assert(vofa_debug_sample.pitch_angle == 30.0f && vofa_debug_sample.yaw_angle == -1.25f);
    assert(vofa_debug_sample.accel_x == 70.0f && vofa_debug_sample.accel_y == 100.0f);
    /* 仅在主机测试控制台打印，不会进入固件 USART1 数据流。 */
    printf("IMU test snapshot: Pitch=%.2f Yaw=%.2f AccelX=%.2f AccelY=%.2f\n",
           (double)vofa_debug_sample.pitch_angle, (double)vofa_debug_sample.yaw_angle,
           (double)vofa_debug_sample.accel_x, (double)vofa_debug_sample.accel_y);
    printf("JustFloat bytes (%u):", (unsigned)transmit_length);
    for (uint16_t i = 0U; i < transmit_length; ++i) { printf(" %02X", (unsigned)transmit_target[i]); }
    puts("");
    /* 同一时刻重复调用，以及连续两周期发送未结束，不能覆盖活动帧。 */
    imu_vofa_data.yaw_angle = 42.0f;
    vofa_task_update();
    tick = 200U; vofa_task_update();
    tick = 300U; vofa_task_update();
    assert(transmit_calls == 1U && vofa_tx_dropped == 2U);
    assert(memcmp(transmit_target, expected, sizeof(expected)) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    tick = 310U; vofa_task_update(); assert(transmit_calls == 1U);
    tick = 400U; vofa_task_update();
    float actual;
    memcpy(&actual, transmit_target + 0U, sizeof(actual));
    assert(transmit_calls == 2U && actual == 42.0f);
    huart1.gState = HAL_UART_STATE_READY;
    transmit_result = HAL_ERROR; tick = 500U; vofa_task_update();
    assert(vofa_tx_status == HAL_ERROR && vofa_tx_dropped == 3U);
    transmit_result = HAL_OK; tick = 600U; vofa_task_update();
    assert(transmit_calls == 3U && vofa_tx_submitted == 3U);
    uint8_t snapshot[28]; memcpy(snapshot, transmit_target, sizeof(snapshot));
    tick = UINT32_MAX - 49U; vofa_task_init();
    assert(memcmp(snapshot, transmit_target, sizeof(snapshot)) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    tick = 49U; vofa_task_update(); assert(transmit_calls == 3U);
    tick = 50U; vofa_task_update(); assert(transmit_calls == 4U);
    memcpy(&actual, transmit_target + 0U, sizeof(actual)); assert(isnan(actual));
    huart1.gState = HAL_UART_STATE_READY;
    puts("PASS VOFA: exact 28-byte six-channel JustFloat, channel order, 100ms cadence, busy buffer lifetime,");
    puts("           dropped periods, TX error recovery, reinitialization and tick wrap.");
}

int main(void)
{
    uint8_t data[300], value = 42U;
    unsigned calls;
    UART_HandleTypeDef other = {0};
    assert(usart_Start() == HAL_ERROR);
    assert(usart_Tx(&value, 1U) == HAL_ERROR);
    huart1.Instance = USART1;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.gState = HAL_UART_STATE_READY;
    receive_result = HAL_BUSY;
    assert(usart_Start() == HAL_BUSY);
    receive_result = HAL_OK;
    assert(usart_Start() == HAL_OK);
    calls = receive_calls;
    feed(value);
    assert(usart_Start() == HAL_OK);
    assert(receive_calls == calls + 1U);
    assert(usart_Rx(NULL, 3U) == 0U);
    assert(usart_Rx(data, 0U) == 0U);
    assert(usart_Rx(data, sizeof(data)) == 1U && data[0] == value);
    assert(usart_Rx(data, sizeof(data)) == 0U);

    for (unsigned i = 0; i < 255U; ++i) feed((uint8_t)i);
    feed(99U);
    assert(rx_overflow_count == 1U);
    assert(usart_Rx(data, 37U) == 37U);
    for (unsigned i = 0; i < 37U; ++i) assert(data[i] == (uint8_t)i);
    for (unsigned i = 255U; i < 292U; ++i) feed((uint8_t)i);
    assert(usart_Rx(data, sizeof(data)) == 255U);
    for (unsigned i = 0; i < 255U; ++i) assert(data[i] == (uint8_t)(i + 37U));
    for (unsigned i = 0; i < 4096U; ++i) {
        feed((uint8_t)i);
        assert(usart_Rx(data, 1U) == 1U && data[0] == (uint8_t)i);
    }

    assert(usart_Tx(NULL, 1U) == HAL_ERROR);
    assert(usart_Tx(&value, 0U) == HAL_ERROR);
    mock_primask = 1U;
    assert(usart_Tx(&value, 1U) == HAL_OK);
    assert(mock_primask == 1U && transmit_target == &value);
    assert(usart_Tx(&value, 1U) == HAL_BUSY);
    mock_primask = 0U;
    huart1.gState = HAL_UART_STATE_READY;
    calls = receive_calls;
    HAL_UART_RxCpltCallback(&other);
    HAL_UART_ErrorCallback(&other);
    assert(receive_calls == calls);

    huart1.ErrorCode = HAL_UART_ERROR_ORE;
    *receive_target = 0xFFU;
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(&huart1);
    assert(usart_Rx(data, sizeof(data)) == 0U);
    HAL_UART_ErrorCallback(&huart1);
    assert(rx_error_count == 1U && rx_running == 1U);
    receive_result = HAL_ERROR;
    feed(11U);
    assert(rx_running == 0U && rx_restart_failure_count == 1U);
    receive_result = HAL_OK;
    assert(usart_Start() == HAL_OK);
    assert(usart_Rx(data, sizeof(data)) == 1U && data[0] == 11U);

    transmit_calls = 0U;
    feed_text("PI");
    USART_PingTest();
    assert(transmit_calls == 0U);
    feed_text("NG\n");
    USART_PingTest();
    assert(transmit_calls == 1U);
    for (unsigned i = 0; i < 10U; ++i) feed_text("PING\n");
    USART_PingTest();
    USART_PingTest();
    assert(usart_tx_status == HAL_BUSY && pending_pong_count == 10U);
    while (pending_pong_count != 0U) {
        finish_tx();
        USART_PingTest();
    }
    finish_tx();
    assert(transmit_calls == 11U);
    feed_text("PING\r\nXPING\nPINGX\n\nPING\n");
    USART_PingTest();
    finish_tx();
    assert(transmit_calls == 12U);
    for (unsigned i = 0; i < 20U; ++i) {
        feed_text("abcdefghijklmnop");
        USART_PingTest();
    }
    feed_text("\nPING\n");
    USART_PingTest();
    finish_tx();
    assert(transmit_calls == 13U);
    assert(usart_tx_error_count == 0U && usart_start_status == HAL_OK);
    assert(mock_primask == 0U);
    puts("PASS: RX start/retry, bounds, overflow, wrap, 4096 cycles, TX busy/lifetime,");
    puts("      IRQ mask, callback routing, error recovery, split/burst/invalid/long PING lines.");
    test_vofa();
    return 0;
}

