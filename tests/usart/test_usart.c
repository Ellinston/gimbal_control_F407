#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
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

/* 联合使用实际串口 BSP、命令解析、M3508 控制/停机源码；只模拟 CAN 外设。 */
#include "../../Task/m3508_task.c"
static HAL_StatusTypeDef can_init_result = HAL_OK;
static HAL_StatusTypeDef can_abort_result = HAL_OK;
static HAL_StatusTypeDef can_tx_result = HAL_OK;
static unsigned can_tx_calls, can_abort_calls;
static uint8_t can_sent[8];
HAL_StatusTypeDef can_Init(void) { return can_init_result; }
HAL_StatusTypeDef can_Rx(uint32_t id, uint8_t *data, uint8_t *length)
{
    (void)id; (void)data; (void)length;
    return HAL_BUSY; /* STOP 必须不依赖电机反馈。 */
}
HAL_StatusTypeDef can_AbortTx(void)
{
    ++can_abort_calls;
    return can_abort_result;
}
HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t length)
{
    assert(id == 0x200U && length == sizeof(can_sent));
    ++can_tx_calls;
    memcpy(can_sent, data, sizeof(can_sent));
    return can_tx_result;
}

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
    const char expected[] = "M3508:-1400.00,8000.25\n";
    m3508_status = (M3508_Status){
        .state = M3508_RUNNING, .init_status = HAL_OK, .feedback_count = 1U,
        .fixed_output = -1400.0f, .output_before_limit = 8000.25f,
        .command_raw = 6000 /* CH1 must preserve the value before clamping. */
    };
    tick = 0U; transmit_calls = 0U;
    vofa_task_init();
    for (tick = 0U; tick < 100U; tick += 10U) { vofa_task_update(); }
    assert(transmit_calls == 0U);
    vofa_task_update();
    assert(transmit_calls == 1U && transmit_length == sizeof(expected) - 1U);
    assert(memcmp(transmit_target, expected, transmit_length) == 0);
    /* 同一时刻重复调用，以及连续两周期发送未结束，不能覆盖活动帧。 */
    m3508_status.fixed_output = 0.0f;
    m3508_status.output_before_limit = -123.5f;
    vofa_task_update();
    tick = 200U; vofa_task_update();
    tick = 300U; vofa_task_update();
    assert(transmit_calls == 1U && vofa_tx_dropped == 2U);
    assert(memcmp(transmit_target, expected, sizeof(expected) - 1U) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    tick = 310U; vofa_task_update(); assert(transmit_calls == 1U);
    tick = 400U; vofa_task_update();
    const char changed[] = "M3508:0.00,-123.50\n";
    assert(transmit_calls == 2U && transmit_length == sizeof(changed) - 1U);
    assert(memcmp(transmit_target, changed, transmit_length) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    transmit_result = HAL_ERROR; tick = 500U; vofa_task_update();
    assert(vofa_tx_status == HAL_ERROR && vofa_tx_dropped == 3U);
    transmit_result = HAL_OK; tick = 600U; vofa_task_update();
    assert(transmit_calls == 3U && vofa_tx_submitted == 3U);
    uint8_t snapshot[64];
    const uint16_t snapshot_length = transmit_length;
    memcpy(snapshot, transmit_target, snapshot_length);
    tick = UINT32_MAX - 49U; vofa_task_init();
    assert(memcmp(snapshot, transmit_target, snapshot_length) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    tick = 49U; vofa_task_update(); assert(transmit_calls == 3U);
    tick = 50U; vofa_task_update(); assert(transmit_calls == 4U);
    assert(memcmp(transmit_target, changed, sizeof(changed) - 1U) == 0);
    huart1.gState = HAL_UART_STATE_READY;

    /* STOP 仅抑制新包，不改写已经提交的缓冲区。 */
    m3508_status.state = M3508_STOPPED;
    tick += 100U; vofa_task_update(); assert(transmit_calls == 4U);
    assert(memcmp(transmit_target, changed, sizeof(changed) - 1U) == 0);
    m3508_status.state = M3508_RUNNING;
    m3508_status.init_status = HAL_ERROR;
    tick += 100U; vofa_task_update(); assert(transmit_calls == 4U);
    m3508_status.init_status = HAL_OK;
    m3508_status.feedback_count = 0U;
    tick += 100U; vofa_task_update(); assert(transmit_calls == 4U);
    m3508_status.feedback_count = 1U;

    /* 串口接收启动失败时不输出，恢复后无需电脑先发数据即可输出。 */
    rx_running = 0U; huart1.RxState = HAL_UART_STATE_READY;
    receive_result = HAL_ERROR;
    tick += 100U; vofa_task_update(); assert(transmit_calls == 4U);
    receive_result = HAL_OK;
    tick += 100U; vofa_task_update(); assert(transmit_calls == 5U);
    huart1.gState = HAL_UART_STATE_READY;

    /* 超长浮点文本必须整包丢弃，不能发送截断的半行。 */
    m3508_status.fixed_output = FLT_MAX;
    m3508_status.output_before_limit = -FLT_MAX;
    tick += 100U; vofa_task_update();
    assert(transmit_calls == 5U && vofa_tx_status == HAL_ERROR);
    m3508_status.fixed_output = 1400.25f;
    m3508_status.output_before_limit = 2500.5f;
    vofa_task_enable_yaw_speed(); vofa_task_enable_motor_position();
    tick += 100U; vofa_task_update(); assert(transmit_calls == 6U);
    const char positive[] = "M3508:1400.25,2500.50\n";
    assert(transmit_length == sizeof(positive) - 1U);
    assert(memcmp(transmit_target, positive, transmit_length) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    puts("PASS VOFA: two-channel FireWater, pre-limit value, signed/zero values, 100ms cadence,");
    puts("           busy buffer lifetime, TX/RX recovery, reinit/wrap, STOP/CAN gates and truncation.");
}

static void test_serial_stop(void)
{
    tick = 0U;
    M3508_Task_Init();
    assert(m3508_status.state == M3508_RUNNING);
    m3508_status.command_raw = -1400;
    m3508_status.p_output = 10.0f;
    m3508_status.i_output = 20.0f;
    m3508_status.fixed_output = -1400.0f;
    m3508_status.output_before_limit = -1370.0f;
    can_tx_calls = can_abort_calls = 0U;

    /* 启动接收不依赖 CAN 成功/反馈、100ms 周期或串口发送完成。 */
    rx_running = 0U;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.gState = HAL_UART_STATE_BUSY_TX;
    receive_result = HAL_BUSY;
    usart_send_STOP();
    assert(can_tx_calls == 0U);
    receive_result = HAL_OK;
    usart_send_STOP();
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);
    feed_text("\n"); usart_send_STOP(); /* 同步此前 BSP 故障后的行边界。 */

    feed_text("ST"); usart_send_STOP();
    assert(m3508_status.state == M3508_RUNNING);
    feed_text("OP"); usart_send_STOP();
    assert(m3508_status.state == M3508_RUNNING); /* 没有换行，尚非完整命令。 */
    feed_text("\n"); usart_send_STOP();
    assert(tick == 0U && m3508_status.state == M3508_STOPPED);
    assert(can_abort_calls == 1U && can_tx_calls == 1U);
    assert(m3508_status.command_raw == 0 && m3508_status.p_output == 0.0f);
    assert(m3508_status.i_output == 0.0f && m3508_status.fixed_output == 0.0f);
    assert(m3508_status.output_before_limit == 0.0f);
    for (unsigned i = 0U; i < sizeof(can_sent); ++i) { assert(can_sent[i] == 0U); }
    tick = 10U; M3508_Task_Update();
    assert(m3508_status.state == M3508_STOPPED && m3508_status.command_raw == 0);

    /* 非完整匹配、嵌入NUL、超长行均不得误触发，下一条合法命令仍可接收。 */
    unsigned aborts_before = can_abort_calls;
    feed_text("XSTOP\nSTOPX\nstop\n\nSTOP\rX\n"); usart_send_STOP();
    const uint8_t embedded_nul[] = {'S','T','O','P',0U,'X','\n'};
    for (unsigned i = 0U; i < sizeof(embedded_nul); ++i) { feed(embedded_nul[i]); }
    usart_send_STOP();
    for (unsigned i = 0U; i < 80U; ++i) { feed('X'); }
    feed_text("STOP\n");
    usart_send_STOP(); usart_send_STOP(); usart_send_STOP();
    assert(can_abort_calls == aborts_before);
    feed_text("STOP\r"); usart_send_STOP();
    assert(can_abort_calls == aborts_before);
    feed_text("\nSTOP\n"); usart_send_STOP();
    assert(can_abort_calls == aborts_before + 2U);

    /* 接收中断出错后，不能把错误前后的碎片拼成 STOP。 */
    aborts_before = can_abort_calls;
    feed_text("ST"); usart_send_STOP();
    HAL_UART_ErrorCallback(&huart1);
    feed_text("OP\n"); usart_send_STOP();
    assert(can_abort_calls == aborts_before);
    feed_text("STOP\n"); usart_send_STOP();
    assert(can_abort_calls == aborts_before + 1U);

    /* CAN旧邮箱未取消完成时先清零控制量，Update 后续重试，只发送零帧。 */
    M3508_Task_Init();
    m3508_status.command_raw = -1400;
    can_abort_result = HAL_BUSY;
    unsigned tx_before = can_tx_calls;
    feed_text("STOP\n"); usart_send_STOP();
    assert(m3508_status.state == M3508_STOPPED && m3508_status.command_raw == 0);
    assert(abort_pending == 1U && can_tx_calls == tx_before);
    tick += 10U; M3508_Task_Update(); assert(can_tx_calls == tx_before);
    can_abort_result = HAL_OK;
    tick += 10U; M3508_Task_Update();
    assert(abort_pending == 0U && can_tx_calls == tx_before + 1U);
    for (unsigned i = 0U; i < sizeof(can_sent); ++i) { assert(can_sent[i] == 0U); }

    /* CAN初始化失败也能锁住 STOP；恢复通信后不能自动回到 RUNNING。 */
    can_init_result = HAL_ERROR; M3508_Task_Init();
    feed_text("STOP\n"); usart_send_STOP();
    assert(m3508_status.state == M3508_STOPPED);
    can_init_result = HAL_OK;
    tick += 10U; M3508_Task_Update();
    assert(m3508_status.state == M3508_STOPPED && m3508_status.command_raw == 0);

    /* 重新初始化恢复正常启动，不持久保存本次停机。 */
    M3508_Task_Init();
    assert(m3508_status.state == M3508_RUNNING && m3508_stop_requested == 0U);
    huart1.gState = HAL_UART_STATE_READY;
    puts("PASS serial STOP: real IRQ/RX/parser/M3508 stop, split/LF/CRLF, malformed lines,");
    puts("                  busy UART/no CAN feedback, RX recovery, mailbox retry, zero frames and reinit.");
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
    test_serial_stop();
    return 0;
}

