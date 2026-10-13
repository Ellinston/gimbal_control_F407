#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 USART1 单字节中断接收，需先完成 MX_USART1_UART_Init。
 * @return HAL_OK：接收已启动；HAL_BUSY：外设忙；HAL_ERROR：启动失败。
 * @note 主循环可重复调用以恢复接收，不会清空已收到的数据。
 */
HAL_StatusTypeDef usart_Start(void);

/**
 * @brief 通过 USART1 异步发送字节数组，不等待发送完成。
 * @param data 数据起始地址；发送完成前数组必须有效且不得改写。
 * @param length 发送字节数，必须大于 0。
 * @return HAL_OK：已开始发送；HAL_BUSY：正在发送，请稍后重试；
 *         HAL_ERROR：参数无效或发送失败。
 * @note HAL_OK 表示提交成功；最终完成由 HAL 的发送中断处理。
 */
HAL_StatusTypeDef usart_Tx(const uint8_t *data, uint16_t length);

/* 主循环发送者在改写异步发送缓冲区前查询；发送均须由主循环提交。 */
uint8_t usart_TxReady(void);
/* 串口任务使用的毫秒时基，按 uint32_t 差值处理回绕。 */
uint32_t usart_GetTick(void);

/**
 * @brief 取出已接收的数据，立即返回，不等待新数据。
 * @param buffer 接收数据的目标缓冲区。
 * @param capacity 目标缓冲区容量，单位为字节。
 * @return 实际读取字节数；无数据、空指针或容量为 0 时返回 0。
 * @note 不添加字符串结束符。接收队列可存 255 字节，满时丢弃新字节。
 *       三个接口均由主循环调用；USART1 的 HAL 回调由本模块统一定义。
 */
uint16_t usart_Rx(uint8_t *buffer, uint16_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USART_H */

