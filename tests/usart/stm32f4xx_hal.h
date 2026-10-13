#ifndef MOCK_STM32F4XX_HAL_H
#define MOCK_STM32F4XX_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
enum { HAL_UART_STATE_RESET, HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_RX,
       HAL_UART_STATE_BUSY_TX };
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_ORE 8U
#define USART1 ((void *)(uintptr_t)0x40011000U)
typedef struct {
    void *Instance;
    uint32_t RxState, gState, ErrorCode;
} UART_HandleTypeDef;
extern uint32_t mock_primask;
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1U; }
static inline void __set_PRIMASK(uint32_t value) { mock_primask = value; }
#define __DMB() __asm__ volatile ("" ::: "memory")
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);
#endif
