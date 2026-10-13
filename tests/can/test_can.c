#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

CAN_TypeDef mock_can;
CAN_HandleTypeDef hcan1 = {CAN1, HAL_CAN_STATE_READY, 0};
UART_HandleTypeDef huart1 = {HAL_UART_STATE_READY};
uint32_t mock_primask;
static uint32_t tick, free_mailboxes = 3U, stops, submits;
static HAL_StatusTypeDef filter_result, start_result, notify_result, tx_result, rx_result;
static CAN_FilterTypeDef configured_filter;
static CAN_RxHeaderTypeDef incoming_header;
static uint8_t incoming_data[8];
static uint32_t incoming_count;
static uint32_t sent_ids[64];
static uint8_t sent_payloads[64][8], sent_lengths[64];
static char uart_text[640];

HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *h) { return h->State; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{ (void)h; configured_filter = *f; return filter_result; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{ if (start_result == HAL_OK) { h->State = HAL_CAN_STATE_LISTENING; } return start_result; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{ ++stops; h->State = HAL_CAN_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailboxes)
{ assert(h == &hcan1 && mailboxes == 7U); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t flags)
{ (void)h; assert(flags == 7U); return notify_result; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h) { (void)h; return free_mailboxes; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header,
                                     uint8_t *data, uint32_t *mailbox)
{
    (void)h; assert(header->IDE == CAN_ID_STD && header->RTR == CAN_RTR_DATA);
    assert(header->DLC <= 8U); assert(data != NULL); *mailbox = 1U;
    if (tx_result == HAL_OK)
    {
        assert(submits < 64U); sent_ids[submits] = header->StdId;
        memcpy(sent_payloads[submits], data, 8U);
        sent_lengths[submits++] = (uint8_t)header->DLC;
    }
    return tx_result;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; assert(fifo == CAN_RX_FIFO0); return incoming_count; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    if (rx_result != HAL_OK) { return rx_result; }
    assert(incoming_count > 0U); --incoming_count;
    *header = incoming_header; memcpy(data, incoming_data, 8U); return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { h->ErrorCode = 0U; return HAL_OK; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef usart_Tx(const uint8_t *data, uint16_t length)
{
    assert(huart1.gState == HAL_UART_STATE_READY);
    assert(length < sizeof(uart_text)); memcpy(uart_text, data, length); uart_text[length] = '\0';
    huart1.gState = HAL_UART_STATE_BUSY_TX; return HAL_OK;
}

/* 同一编译单元允许清理测试现场，不在生产模块添加测试专用接口。 */
#include "../../Bsp/can/can.c"
#include "../../Task/can_test.c"

static void reset(void)
{
    memset((void *)&can_diag, 0, sizeof(can_diag));
    memset(rx_slots, 0, sizeof(rx_slots)); rx_slot_count = 0U;
    hcan1.Instance = CAN1; hcan1.State = HAL_CAN_STATE_READY; hcan1.ErrorCode = 0U;
    tick = 0U; mock_primask = 0U; free_mailboxes = 3U; stops = 0U; submits = 0U;
    filter_result = start_result = notify_result = tx_result = rx_result = HAL_OK;
    incoming_count = 0U; huart1.gState = HAL_UART_STATE_READY;
    memset(uart_text, 0, sizeof(uart_text));
}

static void inject(uint32_t id, uint8_t value, uint32_t dlc)
{
    incoming_header = (CAN_RxHeaderTypeDef){id, 0U, CAN_ID_STD, CAN_RTR_DATA, dlc};
    memset(incoming_data, value, sizeof(incoming_data)); incoming_count = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

int main(void)
{
    uint8_t data[8], length;
    reset();
    filter_result = HAL_ERROR; assert(can_Init() == HAL_ERROR);
    filter_result = HAL_OK; start_result = HAL_ERROR; assert(can_Init() == HAL_ERROR);
    start_result = HAL_OK; notify_result = HAL_ERROR;
    assert(can_Init() == HAL_ERROR && stops == 1U && hcan1.State == HAL_CAN_STATE_READY);
    notify_result = HAL_OK; assert(can_Init() == HAL_OK);
    assert(can_AbortTx() == HAL_OK);
    free_mailboxes = 2U; assert(can_AbortTx() == HAL_BUSY);
    free_mailboxes = 3U; assert(can_AbortTx() == HAL_OK);
    assert(configured_filter.FilterIdHigh == 0U && configured_filter.FilterIdLow == 0U);
    assert(configured_filter.FilterMaskIdLow == 6U && configured_filter.FilterMaskIdHigh == 0U);
    inject(0x123U, 0xABU, 8U); assert(can_Init() == HAL_OK);
    assert(can_Rx(0x123U, data, &length) == HAL_OK && length == 8U && data[7] == 0xABU);
    puts("PASS init failures, rollback, all-ID standard filter, repeated init preserves data");

    memset(data, 0, sizeof(data));
    assert(can_Tx(0x800U, data, 8U) == HAL_ERROR);
    assert(can_Tx(1U, NULL, 8U) == HAL_ERROR);
    assert(can_Tx(1U, data, 9U) == HAL_ERROR);
    assert(can_Tx(0U, NULL, 0U) == HAL_OK);
    assert(can_Tx(0x7FFU, data, 8U) == HAL_OK);
    assert(can_diag.tx_submitted == 2U && can_diag.tx_completed == 0U);
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    HAL_CAN_TxMailbox1CompleteCallback(&hcan1);
    HAL_CAN_TxMailbox2CompleteCallback(&hcan1);
    assert(can_diag.tx_completed == 3U);
    free_mailboxes = 0U; assert(can_Tx(1U, data, 8U) == HAL_BUSY);
    free_mailboxes = 3U; tx_result = HAL_ERROR; assert(can_Tx(1U, data, 8U) == HAL_ERROR);
    hcan1.State = HAL_CAN_STATE_READY; assert(can_Tx(1U, data, 8U) == HAL_ERROR);
    assert(can_AbortTx() == HAL_ERROR);
    puts("PASS TX ID/DLC bounds, empty frame, busy/error, submit versus complete");

    reset();
    inject(2U, 0x22U, 8U); inject(0x456U, 0x45U, 8U); inject(1U, 0x11U, 8U);
    inject(1U, 0x33U, 3U);
    mock_primask = 1U;
    assert(can_Rx(1U, data, &length) == HAL_OK && length == 3U && data[0] == 0x33U && data[3] == 0U);
    assert(mock_primask == 1U); mock_primask = 0U;
    assert(can_Rx(2U, data, &length) == HAL_OK && data[7] == 0x22U);
    assert(mock_primask == 0U);
    assert(can_Rx(0x456U, data, &length) == HAL_OK && data[0] == 0x45U);
    length = 99U; data[0] = 99U;
    assert(can_Rx(1U, data, &length) == HAL_BUSY && length == 99U && data[0] == 99U);
    assert(can_Rx(0x800U, data, &length) == HAL_ERROR);
    assert(can_Rx(1U, NULL, &length) == HAL_ERROR && can_Rx(1U, data, NULL) == HAL_ERROR);
    inject(0U, 0U, 0U); assert(can_Rx(0U, data, &length) == HAL_OK && length == 0U);
    inject(1U, 0U, 9U); assert(can_diag.rx_invalid == 1U);
    incoming_header.IDE = CAN_ID_EXT; incoming_count = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); assert(can_diag.rx_invalid == 2U);
    incoming_header.IDE = CAN_ID_STD; incoming_header.RTR = CAN_RTR_REMOTE; incoming_count = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); assert(can_diag.rx_invalid == 3U);
    rx_result = HAL_ERROR; incoming_count = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); assert(can_diag.rx_errors == 1U);
    puts("PASS interleaved IDs, latest frame, consume, short/empty DLC, invalid frames, IRQ restoration");

    reset();
    for (uint32_t id = 0U; id < CAN_RX_SLOT_COUNT; ++id) { inject(id, (uint8_t)id, 8U); }
    inject(0x700U, 0U, 8U); assert(can_diag.rx_dropped == 1U);
    inject(2U, 0xEFU, 8U); assert(can_Rx(2U, data, &length) == HAL_OK && data[0] == 0xEFU);
    inject(0x700U, 0x70U, 8U); assert(can_Rx(0x700U, data, &length) == HAL_OK && data[0] == 0x70U);
    hcan1.ErrorCode = HAL_CAN_ERROR_RX_FOV0; HAL_CAN_ErrorCallback(&hcan1);
    assert(can_diag.fifo_overruns == 1U && hcan1.ErrorCode == 0U);
    puts("PASS full cache, matching-ID replacement, released-slot reuse, FIFO overflow diagnostic");

    reset(); CAN_TestInit(); assert(can_test_init_status == HAL_OK);
    free_mailboxes = 0U; tick = 10U; CAN_Test(); assert(submits == 0U);
    free_mailboxes = 3U; tick = 11U; CAN_Test();
    assert(submits == 1U && sent_ids[0] == 0x1FFU && sent_lengths[0] == 8U);
    for (uint32_t slot = 0U; slot < 8U; ++slot) { assert(sent_payloads[0][slot] == 0U); }
    tick = 20U; CAN_Test(); assert(submits == 1U);
    tick = 21U; CAN_Test(); assert(submits == 2U && sent_ids[1] == 0x1FFU);
    inject(0x206U, 0x22U, 8U); inject(0x205U, 0x11U, 8U); tick = 22U; CAN_Test();
    assert(yaw_rx_buffer.data[0] == 0x11U && pitch_rx_buffer.data[0] == 0x22U);
    assert(yaw_rx_buffer.updates == 1U && pitch_rx_buffer.updates == 1U);
    tick = 500U; inject(0x205U, 0xA1U, 8U); inject(0x206U, 0xB2U, 8U); CAN_Test();
    assert(strstr(uart_text, "YAW: motor=1 ID=0x205 DLC=8 DATA=A1") != NULL);
    assert(strstr(uart_text, "PITCH: motor=2 ID=0x206 DLC=8 DATA=B2") != NULL);
    assert(strstr(uart_text, "FRESH") != NULL);
    char snapshot[640]; memcpy(snapshot, message, sizeof(snapshot));
    tick = 1000U; CAN_Test(); assert(memcmp(snapshot, message, sizeof(snapshot)) == 0);
    huart1.gState = HAL_UART_STATE_READY; CAN_Test(); assert(strstr(uart_text, "STALE") != NULL);
    puts("PASS grouped 0x1FF zero command, period/retry, YAW/PITCH separation, serial lifetime, stale data");

    reset(); CAN_TestInit(); inject(1U, 0x11U, 8U); inject(2U, 0x22U, 8U);
    tick = 500U; CAN_Test();
    assert(strstr(uart_text, "YAW: motor=1 ID=0x205 WAIT RX") != NULL);
    assert(strstr(uart_text, "PITCH: motor=2 ID=0x206 WAIT RX") != NULL);
    assert(strstr(uart_text, "FRESH") == NULL);
    puts("PASS absent feedback never reported as valid data");

    reset(); CAN_TestInit();
    inject(0x206U, 0x62U, 8U); inject(0x205U, 0x51U, 8U); tick = 500U; CAN_Test();
    assert(yaw_rx_buffer.frame_id == 0x205U && pitch_rx_buffer.frame_id == 0x206U);
    assert(strstr(uart_text, "YAW: motor=1 ID=0x205 DLC=8 DATA=51") != NULL);
    assert(strstr(uart_text, "PITCH: motor=2 ID=0x206 DLC=8 DATA=62") != NULL);
    inject(0x205U, 0xFFU, 7U); tick = 501U; CAN_Test();
    assert(yaw_rx_buffer.data[0] == 0x51U && yaw_rx_buffer.updates == 1U);
    puts("PASS GM6020 Task mapping, real frame ID printing, malformed feedback rejection");
    puts("ALL CAN HOST CHECKS PASSED (mock HAL; not hardware proof)");
    return 0;
}

