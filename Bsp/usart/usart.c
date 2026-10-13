#include "usart.h"

extern UART_HandleTypeDef huart1;

#define USART_RX_BUFFER_SIZE 256U

/* 单生产者（USART1 中断）、单消费者（主循环）的接收队列。 */
static uint8_t rx_buffer[USART_RX_BUFFER_SIZE];
static uint8_t rx_byte;
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_running;

/* 可通过调试器观察；串口模块不向通信流中插入诊断文本。 */
static volatile uint32_t rx_overflow_count;
static volatile uint32_t rx_error_count;
static volatile uint32_t rx_restart_failure_count;

static void usart_RearmRx(void)
{
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) == HAL_OK)
    {
        rx_running = 1U;
    }
    else
    {
        rx_running = 0U;
        ++rx_restart_failure_count;
    }
}

HAL_StatusTypeDef usart_Start(void)
{
    HAL_StatusTypeDef status;
    uint32_t primask = __get_PRIMASK();

    /* HAL 的锁同时用于 TX/RX；避免主循环调用被接收回调抢占。 */
    __disable_irq();
    if ((huart1.Instance != USART1) ||
        (huart1.RxState == HAL_UART_STATE_RESET))
    {
        status = HAL_ERROR;
    }
    else if ((rx_running != 0U) &&
             (huart1.RxState == HAL_UART_STATE_BUSY_RX))
    {
        status = HAL_OK;
    }
    else
    {
        status = HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
        rx_running = (status == HAL_OK) ? 1U : 0U;
    }
    __set_PRIMASK(primask);
    return status;
}

HAL_StatusTypeDef usart_Tx(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;
    uint32_t primask;

    if ((data == NULL) || (length == 0U) ||
        (huart1.Instance != USART1) ||
        (huart1.gState == HAL_UART_STATE_RESET))
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    status = HAL_UART_Transmit_IT(&huart1, data, length);
    __set_PRIMASK(primask);
    return status;
}

uint16_t usart_Rx(uint8_t *buffer, uint16_t capacity)
{
    uint16_t count = 0U;
    uint16_t head = rx_head;
    uint16_t tail = rx_tail;

    if ((buffer == NULL) || (capacity == 0U))
    {
        return 0U;
    }

    /* 获取队首快照，仅消费调用时已有的数据，保证读取时间有界。 */
    __DMB();
    while ((tail != head) && (count < capacity))
    {
        buffer[count++] = rx_buffer[tail];
        tail = (uint16_t)((tail + 1U) % USART_RX_BUFFER_SIZE);
        /* 先完成读取再释放槽位，中断不覆盖尚未读取的字节。 */
        __DMB();
        rx_tail = tail;
    }
    return count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart != &huart1)
    {
        return;
    }

    rx_running = 0U;
    /* HAL 可能先调用完成回调、再处理接收错误；交由错误回调恢复。 */
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        return;
    }

    next_head = (uint16_t)((rx_head + 1U) % USART_RX_BUFFER_SIZE);
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = rx_byte;
        __DMB();
        rx_head = next_head;
    }
    else
    {
        ++rx_overflow_count;
    }
    usart_RearmRx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }

    ++rx_error_count;
    rx_running = 0U;
    /* 当前无 DMA：该调用只清理接收状态/错误标志，不等待外部数据。 */
    if (HAL_UART_AbortReceive(huart) == HAL_OK)
    {
        usart_RearmRx();
    }
    else
    {
        ++rx_restart_failure_count;
    }
}

