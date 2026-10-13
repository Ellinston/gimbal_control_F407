#include "motor.h"
#include "can.h"
#include <string.h>

static Motor_TxFrame last_tx_frame;

const Motor_TxFrame *motor_GetLastTxFrame(void)
{
    return &last_tx_frame;
}

_Static_assert(MOTOR_VOLTAGE_SAFE_LIMIT > 0 &&
               MOTOR_VOLTAGE_SAFE_LIMIT <= MOTOR_VOLTAGE_COMMAND_MAX,
               "Motor safety limit must stay within the protocol range");

int16_t motor_LimitVoltage(int32_t requested_command)
{
    if (requested_command > MOTOR_VOLTAGE_SAFE_LIMIT) { return MOTOR_VOLTAGE_SAFE_LIMIT; }
    if (requested_command < -MOTOR_VOLTAGE_SAFE_LIMIT) { return -MOTOR_VOLTAGE_SAFE_LIMIT; }
    return (int16_t)requested_command;
}

static uint16_t motor_ReadU16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t motor_ReadI16(const uint8_t *data)
{
    uint16_t raw = motor_ReadU16(data);
    /* 用宽整数显式恢复负数，避免超范围 unsigned -> signed 转换。 */
    return (int16_t)((raw <= 32767U) ? (int32_t)raw : (int32_t)raw - 65536);
}

HAL_StatusTypeDef motor_Init(Motor *motor, uint8_t id)
{
    if ((motor == NULL) || (id < 1U) || (id > 7U)) { return HAL_ERROR; }
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    return HAL_OK;
}

HAL_StatusTypeDef motor_Tx(uint8_t first_id, const int16_t voltage_command[4])
{
    uint8_t data[8] = {0};
    if ((voltage_command == NULL) || ((first_id != 1U) && (first_id != 5U)))
    {
        return HAL_ERROR;
    }
    if ((first_id == 5U) && (voltage_command[3] != 0)) { return HAL_ERROR; }
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        uint16_t raw = (uint16_t)motor_LimitVoltage(voltage_command[i]);
        data[i * 2U] = (uint8_t)(raw >> 8);
        data[i * 2U + 1U] = (uint8_t)raw;
    }
    last_tx_frame.id = (first_id == 1U) ? 0x1FFU : 0x2FFU;
    last_tx_frame.length = sizeof(data);
    memcpy(last_tx_frame.data, data, sizeof(data));
    last_tx_frame.status = can_Tx(last_tx_frame.id, data, sizeof(data));
    return last_tx_frame.status;
}

HAL_StatusTypeDef motor_Rx(Motor *motor)
{
    uint8_t data[8];
    uint8_t length;
    if ((motor == NULL) || (motor->id < 1U) || (motor->id > 7U)) { return HAL_ERROR; }
    HAL_StatusTypeDef status = can_Rx(0x204U + motor->id, data, &length);
    if (status != HAL_OK) { return status; }
    if (length != sizeof(data)) { return HAL_ERROR; }
    uint16_t angle = motor_ReadU16(data);
    if (angle > 8191U) { return HAL_ERROR; }

    motor->angle_raw = angle;
    motor->angle_deg = (float)angle * (360.0f / 8192.0f);
    motor->speed_rpm = motor_ReadI16(data + 2U);
    motor->torque_current_raw = motor_ReadI16(data + 4U);
    motor->temperature_c = data[6];
    motor->last_rx_ms = HAL_GetTick();
    ++motor->rx_updates;
    motor->feedback_valid = 1U;
    return HAL_OK;
}

uint8_t motor_IsOnline(const Motor *motor, uint32_t now_ms)
{
    return (uint8_t)((motor != NULL) && (motor->id >= 1U) && (motor->id <= 7U) &&
        (motor->feedback_valid != 0U) &&
        ((uint32_t)(now_ms - motor->last_rx_ms) <= MOTOR_FEEDBACK_TIMEOUT_MS));
}
