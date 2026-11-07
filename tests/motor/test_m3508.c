#include "m3508_task.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

CAN_TypeDef mock_can;
CAN_HandleTypeDef hcan1 = { .Instance = CAN1 };
static uint32_t tick, sent_id, aborts;
static uint8_t pending, rx_data[8], rx_length, sent[8];
static HAL_StatusTypeDef init_result, tx_result, abort_result;
static int16_t encoded(float value)
{
    if (value > 16384.0f) { return 16384; }
    if (value < -16384.0f) { return -16384; }
    return (int16_t)value;
}

static void check_command(float pi_output)
{
    int16_t expected = encoded(pi_output + FIXED_CURRENT);
    assert(m3508_status.command_raw == expected);
    assert(sent[0] == (uint8_t)((uint16_t)expected >> 8));
    assert(sent[1] == (uint8_t)expected);
    for (unsigned i=2;i<8;i++) { assert(sent[i] == 0); }
}
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef can_Init(void) { return init_result; }
HAL_StatusTypeDef can_AbortTx(void) { ++aborts; return abort_result; }
HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t length)
{
    assert(length == 8U);
    sent_id = id;
    if (tx_result == HAL_OK) { memcpy(sent, data, 8); }
    return tx_result;
}
HAL_StatusTypeDef can_Rx(uint32_t id, uint8_t *data, uint8_t *length)
{
    assert(id == 0x201U);
    if (!pending) { return HAL_BUSY; }
    memcpy(data, rx_data, 8); *length = rx_length; pending = 0;
    return HAL_OK;
}
static void feedback(int16_t rpm)
{
    memset(rx_data, 0, 8);
    rx_data[0] = 0x12; rx_data[1] = 0x34;
    rx_data[2] = (uint8_t)((uint16_t)rpm >> 8); rx_data[3] = (uint8_t)rpm;
    rx_data[4] = 0xFF; rx_data[5] = 0x9C; /* -100 */
    rx_data[6] = 30; rx_length = 8; pending = 1;
}
static void reset(void)
{
    tick = 0; pending = 0; mock_can.ESR = 0; aborts = 0;
    init_result = tx_result = abort_result = HAL_OK;
    memset(sent, 0xCC, 8);
    M3508_Task_Init(); tick=10; M3508_Task_Update();
    assert(sent_id == 0x200U);
    assert(m3508_status.state == M3508_RUNNING);
    assert(m3508_status.target_output_rpm == 5.0f);
    assert(m3508_status.p_output == 95.0f);
    assert(m3508_status.i_output == 47.5f);
    check_command(142.5f);
    for (unsigned i=2;i<8;i++) { assert(sent[i] == 0); }
}
static void run_to(uint32_t end, int16_t rpm)
{
    while (tick < end) { tick += 10; feedback(rpm); M3508_Task_Update(); }
}
static void check_zero(void)
{
    assert(m3508_status.command_raw == 0);
    assert(m3508_status.target_output_rpm == 0);
    for (unsigned i=0;i<8;i++) { assert(sent[i] == 0); }
}
int main(void)
{
    reset();
    /* Fixed bias never selects a different controller, including when zero. */
    assert(m3508_status.fixed_output == FIXED_CURRENT);
    assert(m3508_status.output_before_limit == 142.5f + FIXED_CURRENT);
    float saved_i = m3508_status.i_output;
    M3508_Task_Update();
    assert(m3508_status.i_output == saved_i); /* 10 ms gate */
    feedback(95); tick+=10; M3508_Task_Update();
    assert(m3508_status.p_output == 0.0f && m3508_status.i_output == saved_i);
    check_command(47.5f);

    /* Motor not responding / feedback absent must not stop control. */
    reset();
    for (unsigned i=0;i<2000;i++) { tick+=10; M3508_Task_Update(); }
    assert(m3508_status.state == M3508_RUNNING);
    assert(m3508_status.target_output_rpm == 5.0f);
    assert(m3508_status.i_output == 4000.0f);
    check_command(4095.0f);
    reset(); run_to(100, -190);
    assert(m3508_status.rotor_speed_rpm == -190);
    assert(m3508_status.p_output == 285.0f);
    assert(fabsf(m3508_status.output_speed_rpm + 10) < 0.001f);
    assert(m3508_status.angle_raw == 0x1234 && m3508_status.torque_current_raw == -100);
    feedback(95); tick+=10; M3508_Task_Update();
    assert(m3508_status.output_speed_rpm == 5.0f);
    assert(m3508_status.p_output == 0.0f);

    reset(); run_to(1100, 0); m3508_stop_requested=1; M3508_Task_Update();
    assert(m3508_status.state == M3508_STOPPED); check_zero();
    run_to(1300, 0); assert(m3508_status.state == M3508_STOPPED); check_zero();

    reset(); run_to(1100, 0); tx_result=HAL_BUSY;
    tick+=10; feedback(0); M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    assert(m3508_status.tx_status == HAL_BUSY);
    tx_result=HAL_ERROR; tick+=10; M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    tx_result=HAL_OK; tick+=10; M3508_Task_Update();
    assert(m3508_status.tx_status == HAL_OK && m3508_status.state == M3508_RUNNING);

    reset(); run_to(1100, 0); unsigned old_aborts=aborts;
    abort_result=HAL_BUSY; M3508_Task_Stop();
    assert(m3508_status.state == M3508_STOPPED && m3508_status.command_raw == 0);
    tick+=10; M3508_Task_Update(); assert(aborts > old_aborts);
    abort_result=HAL_OK; tick+=10; M3508_Task_Update(); check_zero();

    reset(); run_to(1100, 0); feedback(0); rx_length=7; tick+=10; M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    reset(); feedback(0); rx_data[0]=0x20; M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    reset(); run_to(1100, 0); feedback(0); rx_data[6]=80; tick+=10; M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    reset(); run_to(1100, 0); mock_can.ESR=4; tick+=10; M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    reset(); run_to(1100, 0); tick+=51; feedback(0); M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    reset(); run_to(1100, 30000);
    assert(m3508_status.i_output == -4000.0f);
    assert(m3508_status.state == M3508_RUNNING); /* Output saturation is not STOP. */
    check_command(-33905.0f);
    run_to(2200, -30000);
    assert(m3508_status.i_output == 4000.0f && m3508_status.state == M3508_RUNNING);
    check_command(34095.0f);
    reset(); init_result=HAL_ERROR; M3508_Task_Init();
    assert(m3508_status.init_status == HAL_ERROR);
    tick+=10; M3508_Task_Update(); assert(m3508_status.init_status == HAL_ERROR);
    init_result=HAL_OK; tick+=10; M3508_Task_Update();
    assert(m3508_status.state == M3508_RUNNING);
    assert(m3508_status.i_output == 47.5f); /* Init resets integral. */
    printf("M3508 PI+bias passed: FIXED_CURRENT=%.1f, I limits, retry, no automatic stop.\n",
           (double)FIXED_CURRENT);
    return 0;
}
