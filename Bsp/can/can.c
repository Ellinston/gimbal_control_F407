#include "can.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

typedef struct
{
    uint32_t id;
    uint8_t data[CAN_DATA_CAPACITY];
    uint8_t length;
} CAN_RxSlot;

/* 中断写入，主循环在短临界区内取出。只缓存报文，不识别设备。 */
static CAN_RxSlot rx_slots[CAN_RX_SLOT_COUNT];
static volatile uint32_t rx_slot_count;
volatile CAN_Diagnostics can_diag;

HAL_StatusTypeDef can_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING |
                             CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_TX_MAILBOX_EMPTY;

    if (hcan1.Instance != CAN1) { return HAL_ERROR; }
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING)
    {
        return HAL_CAN_ActivateNotification(&hcan1, notifications);
    }
    if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY) { return HAL_ERROR; }

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    /* 屏蔽 IDE/RTR，仅接收标准数据帧；报文 ID 不在 BSP 中限定。 */
    filter.FilterMaskIdLow = 0x0006U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK) { return status; }
    status = HAL_CAN_Start(&hcan1);
    if (status != HAL_OK) { return status; }
    status = HAL_CAN_ActivateNotification(&hcan1, notifications);
    if (status != HAL_OK) { (void)HAL_CAN_Stop(&hcan1); }
    return status;
}

HAL_StatusTypeDef can_Tx(uint32_t id, const uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t payload[CAN_DATA_CAPACITY] = {0};
    uint32_t mailbox;
    HAL_StatusTypeDef status;

    if ((id > 0x7FFU) || (length > CAN_DATA_CAPACITY) ||
        ((length != 0U) && (data == NULL)) || (hcan1.Instance != CAN1) ||
        (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING))
    {
        ++can_diag.tx_errors;
        return HAL_ERROR;
    }
    can_diag.hal_error = HAL_CAN_GetError(&hcan1);
    can_diag.error_status_register = hcan1.Instance->ESR;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        ++can_diag.tx_busy;
        return HAL_BUSY;
    }
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    if (length != 0U) { memcpy(payload, data, length); }
    status = HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox);
    if (status == HAL_OK) { ++can_diag.tx_submitted; }
    else { ++can_diag.tx_errors; }
    return status;
}

HAL_StatusTypeDef can_AbortTx(void)
{
    if ((hcan1.Instance != CAN1) ||
        (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING)) { return HAL_ERROR; }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3U) { return HAL_OK; }
    HAL_StatusTypeDef status = HAL_CAN_AbortTxRequest(&hcan1,
        CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    /* HAL 只置取消请求位；下一轮确认邮箱空了再允许提交新的零帧。 */
    return (status == HAL_OK) ? HAL_BUSY : status;
}

HAL_StatusTypeDef can_Rx(uint32_t id, uint8_t *data, uint8_t *length)
{
    if ((id > 0x7FFU) || (data == NULL) || (length == NULL)) { return HAL_ERROR; }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0U; i < rx_slot_count; ++i)
    {
        if (rx_slots[i].id == id)
        {
            memcpy(data, rx_slots[i].data, CAN_DATA_CAPACITY);
            *length = rx_slots[i].length;
            /* 顺序不属于接口约定；移入末项，避免搬移整段缓存。 */
            --rx_slot_count;
            rx_slots[i] = rx_slots[rx_slot_count];
            __set_PRIMASK(primask);
            return HAL_OK;
        }
    }
    __set_PRIMASK(primask);
    return HAL_BUSY;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) { return; }

    /* 硬件 FIFO 最多 3 帧；限定一次中断的工作量，串口打印留在 Task。 */
    for (uint32_t n = 0U; n < 3U; ++n)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[CAN_DATA_CAPACITY] = {0};
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U) { break; }
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            ++can_diag.rx_errors;
            break;
        }
        if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA) ||
            (header.StdId > 0x7FFU) || (header.DLC > CAN_DATA_CAPACITY))
        {
            ++can_diag.rx_invalid;
            continue;
        }
        ++can_diag.rx_frames;
        can_diag.last_rx_id = header.StdId;
        uint32_t slot;
        for (slot = 0U; slot < rx_slot_count; ++slot)
        {
            if (rx_slots[slot].id == header.StdId) { break; }
        }
        if (slot == rx_slot_count)
        {
            if (rx_slot_count == CAN_RX_SLOT_COUNT)
            {
                ++can_diag.rx_dropped;
                continue;
            }
            ++rx_slot_count;
        }
        else { ++can_diag.rx_replaced; }
        rx_slots[slot].id = header.StdId;
        rx_slots[slot].length = (uint8_t)header.DLC;
        memset(rx_slots[slot].data, 0, CAN_DATA_CAPACITY);
        memcpy(rx_slots[slot].data, data, header.DLC);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) { ++can_diag.tx_completed; }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) { ++can_diag.tx_completed; }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) { ++can_diag.tx_completed; }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) { return; }
    can_diag.hal_error = HAL_CAN_GetError(hcan);
    can_diag.error_status_register = hcan->Instance->ESR;
    if ((can_diag.hal_error & HAL_CAN_ERROR_RX_FOV0) != 0U)
    {
        ++can_diag.fifo_overruns;
        /* HAL 错误码是累计位；清除此位才能正确统计后续溢出事件。 */
        (void)HAL_CAN_ResetError(hcan);
    }
}
