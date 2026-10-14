#include "usart.h"
#include <assert.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
uint8_t usart_TxReady(void) { return huart1.gState == HAL_UART_STATE_READY; }
uint32_t usart_GetTick(void) { return HAL_GetTick(); }
HAL_StatusTypeDef usart_Start(void) { return HAL_OK; }
static uint8_t serial_input[1024];
static uint16_t serial_count;
uint32_t mock_usart_fault_count;
uint32_t usart_GetRxFaultCount(void) { return mock_usart_fault_count; }
void mock_usart_feed(const uint8_t *data, uint16_t length)
{
    assert(serial_count + length <= sizeof(serial_input));
    memcpy(serial_input + serial_count, data, length);
    serial_count += length;
}
uint16_t usart_Rx(uint8_t *buffer, uint16_t capacity)
{
    uint16_t length = serial_count < capacity ? serial_count : capacity;
    memcpy(buffer, serial_input, length);
    serial_count -= length;
    memmove(serial_input, serial_input + length, serial_count);
    return length;
}

/* 电机主机测试不连接传感器；两轴可独立注入，避免误用编码器数据。 */
#include "imu_test.h"
#include <math.h>
float mock_imu_yaw = NAN, mock_imu_pitch = NAN;
float imu_test_get_yaw_angle(void) { return mock_imu_yaw; }
float imu_test_get_pitch_angle(void) { return mock_imu_pitch; }
