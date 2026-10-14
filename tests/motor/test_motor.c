#include "motor_task.h"
#include "can.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick, tx_attempts, tx_submits, sent_id;
static HAL_StatusTypeDef init_result, tx_result;
static uint8_t sent_data[8];
static uint8_t pending[7], lengths[7], feedback[7][8];
UART_HandleTypeDef huart1 = {HAL_UART_STATE_READY};
HAL_StatusTypeDef usart_Tx(const uint8_t *data, uint16_t length)
{
    (void)data; (void)length; return HAL_OK;
}
HAL_StatusTypeDef can_AbortTx(void) { return HAL_OK; }

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef can_Init(void) { return init_result; }
HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t length)
{
    ++tx_attempts;
    assert(length == 8U);
    if (tx_result == HAL_OK)
    {
        ++tx_submits;
        sent_id = id;
        memcpy(sent_data, data, 8U);
    }
    return tx_result;
}
HAL_StatusTypeDef can_Rx(uint32_t id, uint8_t *data, uint8_t *length)
{
    assert(id >= 0x205U && id <= 0x20BU);
    uint32_t slot = id - 0x205U;
    if (!pending[slot]) { return HAL_BUSY; }
    pending[slot] = 0U;
    memcpy(data, feedback[slot], 8U);
    *length = lengths[slot];
    return HAL_OK;
}

static void inject(uint8_t id)
{
    const uint8_t data[8] = {0x1F, 0xFF, 0xFF, 0x85, 0x80, 0x00, 42, 0xFF};
    pending[id - 1U] = 1U;
    lengths[id - 1U] = 8U;
    memcpy(feedback[id - 1U], data, 8U);
}

static void assert_zero(void)
{
    assert(sent_id == 0x1FFU);
    for (uint32_t i = 0U; i < 8U; ++i) { assert(sent_data[i] == 0U); }
}

static void reset(void)
{
    tick = tx_attempts = tx_submits = sent_id = 0U;
    init_result = tx_result = HAL_OK;
    memset(pending, 0, sizeof(pending));
    memset(sent_data, 0xFF, sizeof(sent_data));
    assert(motor_task_Init() == HAL_OK);
    assert_zero();
    assert(motor_task_status.state == MOTOR_TASK_STOPPED);
}

static void receive_both(void)
{
    inject(1U);
    inject(2U);
    motor_task_Run();
}

int main(void)
{
    Motor motor;
    assert(motor_Init(NULL, 1U) == HAL_ERROR);
    assert(motor_Init(&motor, 0U) == HAL_ERROR);
    assert(motor_Init(&motor, 8U) == HAL_ERROR);
    assert(motor_Init(&motor, 1U) == HAL_OK && !motor.feedback_valid);
    assert(motor_Rx(NULL) == HAL_ERROR);
    assert(!motor_IsOnline(NULL, 0U));
    assert(motor_Rx(&motor) == HAL_BUSY && !motor.feedback_valid);
    for (uint8_t id = 1U; id <= 7U; ++id)
    {
        assert(motor_Init(&motor, id) == HAL_OK);
        inject(id); tick = id;
        assert(motor_Rx(&motor) == HAL_OK);
        assert(motor.angle_raw == 8191U);
        assert(motor.angle_deg > 359.95f && motor.angle_deg < 360.0f);
        assert(motor.speed_rpm == -123 && motor.torque_current_raw == -32768);
        assert(motor.temperature_c == 42U && motor.last_rx_ms == tick);
        assert(motor.rx_updates == 1U && motor.feedback_valid);
        assert(motor_IsOnline(&motor, tick + 100U));
        assert(!motor_IsOnline(&motor, tick + 101U));
    }
    Motor snapshot = motor;
    inject(7U); lengths[6] = 7U;
    assert(motor_Rx(&motor) == HAL_ERROR && memcmp(&snapshot, &motor, sizeof(motor)) == 0);
    inject(7U); feedback[6][0] = 0x20U; feedback[6][1] = 0U;
    assert(motor_Rx(&motor) == HAL_ERROR && memcmp(&snapshot, &motor, sizeof(motor)) == 0);
    inject(7U); memset(feedback[6], 0, 8U);
    feedback[6][2] = feedback[6][4] = 0x7FU;
    feedback[6][3] = feedback[6][5] = 0xFFU;
    tick = UINT32_MAX - 50U;
    assert(motor_Rx(&motor) == HAL_OK && motor.angle_raw == 0U);
    assert(motor.speed_rpm == 32767 && motor.torque_current_raw == 32767);
    assert(motor_IsOnline(&motor, 49U) && !motor_IsOnline(&motor, 50U));
    puts("PASS all feedback IDs, angle endpoints, signed fields, invalid DLC/angle, stale/wrapped tick");

    int16_t command[4] = {25000, -25000, 1, -1};
    const uint16_t positive_limit = (uint16_t)MOTOR_VOLTAGE_SAFE_LIMIT;
    const uint16_t negative_limit = (uint16_t)(-MOTOR_VOLTAGE_SAFE_LIMIT);
    const uint8_t expected[8] = {
        (uint8_t)(positive_limit >> 8), (uint8_t)positive_limit,
        (uint8_t)(negative_limit >> 8), (uint8_t)negative_limit, 0, 1, 0xFF, 0xFF};
    tx_result = HAL_OK;
    assert(motor_Tx(1U, command) == HAL_OK && sent_id == 0x1FFU);
    assert(memcmp(sent_data, expected, 8U) == 0);
    const Motor_TxFrame *frame = motor_GetLastTxFrame();
    assert(frame->id == sent_id && frame->length == 8U && frame->status == HAL_OK);
    assert(memcmp(frame->data, sent_data, 8U) == 0);
    uint32_t attempts = tx_attempts;
    assert(motor_Tx(0U, command) == HAL_ERROR && motor_Tx(2U, command) == HAL_ERROR);
    assert(motor_Tx(1U, NULL) == HAL_ERROR && motor_Tx(5U, command) == HAL_ERROR);
    assert(tx_attempts == attempts);
    command[0] = 32767; assert(motor_Tx(1U, command) == HAL_OK);
    assert(sent_data[0] == expected[0] && sent_data[1] == expected[1]);
    command[0] = -32768; assert(motor_Tx(1U, command) == HAL_OK);
    assert(sent_data[0] == expected[2] && sent_data[1] == expected[3]);
    assert(motor_LimitVoltage(INT32_MAX) == MOTOR_VOLTAGE_SAFE_LIMIT &&
           motor_LimitVoltage(INT32_MIN) == -MOTOR_VOLTAGE_SAFE_LIMIT);
    assert(motor_LimitVoltage(MOTOR_VOLTAGE_SAFE_LIMIT) == MOTOR_VOLTAGE_SAFE_LIMIT &&
           motor_LimitVoltage(-MOTOR_VOLTAGE_SAFE_LIMIT) == -MOTOR_VOLTAGE_SAFE_LIMIT);
    assert(motor_LimitVoltage(1250) == 1250 && motor_LimitVoltage(-1250) == -1250);
    assert(motor_LimitVoltage(0) == 0 && motor_LimitVoltage(100) == 100);
    command[0] = 0; command[3] = 0;
    assert(motor_Tx(5U, command) == HAL_OK && sent_id == 0x2FFU);
    assert(sent_data[6] == 0U && sent_data[7] == 0U);
    tx_result = HAL_BUSY; assert(motor_Tx(1U, command) == HAL_BUSY);
    tx_result = HAL_ERROR; assert(motor_Tx(1U, command) == HAL_ERROR);
    puts("PASS voltage groups, full payload, big endian, bounds, reserved slot, CAN status forwarding");

    reset();
    assert(motor_task_Start(100, -200, 100U) == HAL_BUSY);
    receive_both();
    assert(motor_task_Start(25001, 0, 100U) == HAL_ERROR);
    assert(motor_task_Start(0, -25001, 100U) == HAL_ERROR);
    assert(motor_task_Start(0, 0, 0U) == HAL_ERROR);
    assert(motor_task_Start(0, 0, MOTOR_TASK_MAX_RUN_MS + 1U) == HAL_ERROR);
    assert(motor_task_Start(100, -200, 100U) == HAL_OK);
    assert(motor_task_Start(0, 0, 100U) == HAL_BUSY);
    tick = 9U; motor_task_Run(); assert(tx_submits == 1U);
    tick = 10U; motor_task_Run(); assert(tx_submits == 2U);
    const uint8_t task_expected[8] = {0, 100, 0xFF, 0x38, 0, 0, 0, 0};
    assert(memcmp(sent_data, task_expected, 8U) == 0);
    assert(motor_task_Stop() == HAL_OK && motor_task_status.state == MOTOR_TASK_STOPPED);
    assert_zero();
    tick = 20U; motor_task_Run(); assert_zero();
    assert(motor_task_status.state == MOTOR_TASK_STOPPED);
    puts("PASS start prerequisites, duration/command bounds, grouped dual output, period, immediate stop");

    reset(); receive_both(); assert(motor_task_Start(100, 0, 500U) == HAL_OK);
    tick = 10U; motor_task_Run();
    tick = 101U; inject(1U); motor_task_Run();
    assert(motor_task_status.state == MOTOR_TASK_FEEDBACK_LOST); assert_zero();
    tick = 102U; receive_both(); assert(motor_task_status.state == MOTOR_TASK_FEEDBACK_LOST);
    assert(motor_task_Start(100, 0, 20U) == HAL_OK);
    tick = 122U; receive_both();
    assert(motor_task_status.state == MOTOR_TASK_DURATION_END); assert_zero();
    puts("PASS one-axis feedback loss stops both, no automatic restart, finite duration");

    reset(); receive_both(); assert(motor_task_Start(100, 0, 100U) == HAL_OK);
    tick = 1U; inject(2U); lengths[1] = 0U; motor_task_Run();
    assert(motor_task_status.state == MOTOR_TASK_RX_ERROR); assert_zero();
    reset(); receive_both(); assert(motor_task_Start(100, 0, 100U) == HAL_OK);
    tick = 10U; tx_result = HAL_BUSY; motor_task_Run();
    assert(motor_task_status.state == MOTOR_TASK_TX_ERROR);
    attempts = tx_attempts;
    motor_task_Run(); assert(tx_attempts == attempts);
    tick = 11U; tx_result = HAL_OK; motor_task_Run(); assert_zero();
    assert(motor_task_status.state == MOTOR_TASK_TX_ERROR);
    assert(motor_task_Start(100, 0, 100U) == HAL_OK);
    tick = 21U; tx_result = HAL_ERROR; motor_task_Run();
    assert(motor_task_status.state == MOTOR_TASK_TX_ERROR);
    tick = 22U; tx_result = HAL_OK; motor_task_Run(); assert_zero();
    assert(motor_task_Start(100, 0, 100U) == HAL_OK);
    tx_result = HAL_BUSY; assert(motor_task_Stop() == HAL_BUSY);
    assert(motor_task_Start(100, 0, 100U) == HAL_BUSY);
    tick = 23U; tx_result = HAL_OK; motor_task_Run(); assert_zero();
    puts("PASS malformed feedback stop, TX busy/error cancel, zero retries, pending-stop blocks start");

    init_result = HAL_ERROR; assert(motor_task_Init() == HAL_ERROR);
    attempts = tx_attempts; motor_task_Run(); assert(tx_attempts == attempts);
    assert(motor_task_Start(1, 0, 1U) == HAL_ERROR);
    assert(motor_task_Stop() == HAL_ERROR);
    init_result = HAL_OK; tx_result = HAL_BUSY;
    assert(motor_task_Init() == HAL_BUSY && motor_task_status.initialized);
    ++tick; tx_result = HAL_OK; motor_task_Run(); assert_zero();
    reset(); tick = UINT32_MAX - 5U; receive_both();
    assert(motor_task_Start(100, 0, 10U) == HAL_OK);
    tick = 4U; motor_task_Run();
    assert(motor_task_status.state == MOTOR_TASK_DURATION_END); assert_zero();
    puts("PASS init failure, initial zero retry, run duration across tick wrap");
    puts("ALL MOTOR HOST CHECKS PASSED (mock CAN; not hardware proof)");
    return 0;
}

