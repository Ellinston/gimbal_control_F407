#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_DATA_CAPACITY 8U
#define CAN_RX_SLOT_COUNT 32U

/* 仅通信诊断，不包含电机编号或业务数据。可在调试器中观察。 */
typedef struct
{
    uint32_t tx_submitted;
    uint32_t tx_completed;
    uint32_t tx_busy;
    uint32_t tx_errors;
    uint32_t rx_frames;
    uint32_t rx_replaced;
    uint32_t rx_dropped;
    uint32_t rx_invalid;
    uint32_t rx_errors;
    uint32_t fifo_overruns;
    uint32_t last_rx_id;
    uint32_t hal_error;
    uint32_t error_status_register;
} CAN_Diagnostics;

extern volatile CAN_Diagnostics can_diag;

/**
 * @brief 在 MX_CAN1_Init 后配置标准数据帧过滤器、启动 CAN1 和收发通知。
 * @return HAL_OK：已启动；HAL_ERROR：配置或启动失败。重复调用不清空缓存。
 */
HAL_StatusTypeDef can_Init(void);

/**
 * @brief 按报文 ID 提交一帧标准数据帧，不等待总线发送完成。
 * @param id CAN 标准报文 ID，范围 0x000..0x7FF，不是设备拨码编号。
 * @param data 数据地址；length 为 0 时允许 NULL。HAL 会立即复制数据。
 * @param length 数据长度，范围 0..8 字节。
 * @return HAL_OK：已提交邮箱（不代表对端收到）；HAL_BUSY：邮箱满；
 *         HAL_ERROR：参数无效、CAN 未启动或提交失败。
 * @note 通过 can_diag.tx_completed 观察总线实际发送完成数。
 */
HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t length);

/* 取消 CAN1 三个邮箱的待发送报文，非阻塞；HAL_BUSY 表示仍在取消，需重试。
 * HAL_OK 表示三个邮箱已空，此后可提交零帧；不代表电机已经停转。
 * 用于本轮独占 CAN1 的电机停机，不能与其他发送任务同时使用。 */
HAL_StatusTypeDef can_AbortTx(void);

/**
 * @brief 按指定报文 ID 取出最新一帧，并清除该 ID 的待取标志，不等待。
 * @param id CAN 标准报文 ID，范围 0x000..0x7FF。
 * @param data 目标缓冲区，必须至少有 CAN_DATA_CAPACITY（8）字节。
 * @param length 返回实际 DLC；无数据或出错时不修改目标参数。
 * @return HAL_OK：取得一帧；HAL_BUSY：该 ID 暂无新帧；HAL_ERROR：参数无效。
 * @note 中断缓存最多 32 个不同的未读取 ID；同 ID 新帧覆盖旧帧。
 *       can_diag 记录覆盖和满缓存丢帧；以上接口均由主循环调用。
 */
HAL_StatusTypeDef can_Rx(uint32_t id, uint8_t *data, uint8_t *length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
