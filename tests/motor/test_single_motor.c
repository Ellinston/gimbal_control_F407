#include "motor_single_test.h"
#include "can.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1 = {HAL_UART_STATE_READY};
volatile CAN_Diagnostics can_diag;
static uint32_t tick, attempts, submits;
static HAL_StatusTypeDef tx_result;
static uint8_t pending, dlc, feedback[8];
static int16_t sent_command;
static char printed[160];
static const uint8_t *inflight;
static uint16_t inflight_length;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef can_Init(void) { return HAL_OK; }
HAL_StatusTypeDef can_AbortTx(void) { return HAL_OK; }
HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t length)
{
    ++attempts;
    assert(id == 0x1FFU && length == 8U);
    sent_command = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    assert(sent_command == 1000);
    for (uint32_t i=2U;i<8U;++i) { assert(data[i] == 0U); }
    const Motor_TxFrame *frame = motor_GetLastTxFrame();
    assert(frame->id == id && frame->length == length && memcmp(frame->data,data,8U)==0);
    if (tx_result == HAL_OK) { ++submits; }
    return tx_result;
}
HAL_StatusTypeDef can_Rx(uint32_t id,uint8_t *data,uint8_t *length)
{
    assert(id==0x205U || id==0x206U);
    if (id!=0x205U || !pending) { return HAL_BUSY; }
    pending=0U; memcpy(data,feedback,8U); *length=dlc; return HAL_OK;
}
HAL_StatusTypeDef usart_Tx(const uint8_t *data,uint16_t length)
{
    assert(huart1.gState==HAL_UART_STATE_READY && length<sizeof(printed));
    memcpy(printed,data,length); printed[length]='\0';
    inflight=data; inflight_length=length; huart1.gState=HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}
static void inject(uint16_t angle,int16_t speed)
{
    memset(feedback,0,8U); feedback[0]=(uint8_t)(angle>>8); feedback[1]=(uint8_t)angle;
    feedback[2]=(uint8_t)((uint16_t)speed>>8); feedback[3]=(uint8_t)speed;
    feedback[6]=28U; pending=1U; dlc=8U;
}
static void reset(void)
{
    tick=attempts=submits=0U; tx_result=HAL_OK; pending=0U;
    huart1.gState=HAL_UART_STATE_READY; printed[0]='\0';
    assert(Task_Motor_Single_Test_Init()==HAL_OK);
}
static void sample(uint32_t now,uint16_t angle,int16_t speed)
{
    tick=now; inject(angle,speed); huart1.gState=HAL_UART_STATE_READY;
    Task_Motor_Single_Test();
}
int main(void)
{
    reset(); Task_Motor_Single_Test();
    assert(sent_command==1000 && submits==1U);
    assert(strstr(printed,"ID=1FF DATA=03 E8 00 00 00 00 00 00 s=0 c=1000")!=NULL);
    tick=9U; Task_Motor_Single_Test(); assert(submits==1U);
    tick=10U; Task_Motor_Single_Test(); assert(submits==2U);
    for(tick=2000U;tick<=12000U;tick+=2000U){Task_Motor_Single_Test(); assert(sent_command==1000);}
    tick=100000U; Task_Motor_Single_Test(); assert(sent_command==1000);
    puts("PASS first-loop fixed1000, 10ms TX, no ramp/feedback/time gating");

    reset(); sample(0U,8190U,0); sample(10U,20U,3); sample(20U,4000U,100);
    sample(30U,4676U,0); assert(sent_command==1000);
    sample(40U,8192U,0); assert(motor_single_status.rx_status==HAL_ERROR && sent_command==1000);
    assert(single_yaw_motor.angle_raw==4676U);
    puts("PASS feedback angle/speed and invalid frames never change the fixed command");

    reset(); Task_Motor_Single_Test();
    char snapshot[160]; memcpy(snapshot,inflight,inflight_length); uint16_t snapshot_length=inflight_length;
    for(tick=10U;tick<=100U;tick+=10U){Task_Motor_Single_Test();}
    assert(memcmp(snapshot,inflight,snapshot_length)==0 && motor_single_status.log_dropped>0U);
    uint32_t count=submits; huart1.gState=HAL_UART_STATE_READY; tick=120U; Task_Motor_Single_Test();
    assert(submits==count+1U && sent_command==1000);
    puts("PASS asynchronous raw-frame logs preserve buffers and do not block control");

    reset(); Task_Motor_Single_Test(); tick=10U; tx_result=HAL_BUSY; Task_Motor_Single_Test();
    assert(motor_single_status.tx_status==HAL_BUSY);
    tick=20U; tx_result=HAL_ERROR; Task_Motor_Single_Test();
    tick=30U; tx_result=HAL_OK; Task_Motor_Single_Test(); assert(submits==2U && sent_command==1000);
    puts("PASS CAN busy/error retry without changing command");

    reset(); tick=UINT32_MAX-5U; assert(Task_Motor_Single_Test_Init()==HAL_OK);
    Task_Motor_Single_Test(); assert(submits==1U);
    tick=3U; Task_Motor_Single_Test(); assert(submits==1U);
    tick=4U; Task_Motor_Single_Test(); assert(submits==2U);
    tick=1994U; Task_Motor_Single_Test(); assert(sent_command==1000);
    puts("PASS fixed output across tick wrap");
    puts("ALL FIXED1000 HOST CHECKS PASSED (mock hardware)");
    return 0;
}

