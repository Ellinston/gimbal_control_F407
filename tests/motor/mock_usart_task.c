#include "usart.h"

extern UART_HandleTypeDef huart1;
uint8_t usart_TxReady(void) { return huart1.gState == HAL_UART_STATE_READY; }
uint32_t usart_GetTick(void) { return HAL_GetTick(); }
HAL_StatusTypeDef usart_Start(void) { return HAL_OK; }
uint16_t usart_Rx(uint8_t *buffer, uint16_t capacity)
{
    (void)buffer; (void)capacity;
    return 0U;
}
