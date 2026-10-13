#include "BMI270Middleware.h"
extern SPI_HandleTypeDef hspi1;

void BMI270_NS_H(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); }
void BMI270_NS_L(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); }
void BMI270_GPIO_init(void) { BMI270_NS_H(); }

void BMI270_com_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void BMI270_delay_ms(uint16_t ms) { HAL_Delay(ms); }

void BMI270_delay_us(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { __NOP(); }
}

HAL_StatusTypeDef BMI270_read_write_byte(uint8_t tx, uint8_t *rx)
{
    if (rx == NULL) { return HAL_ERROR; }
    return HAL_SPI_TransmitReceive(&hspi1, &tx, rx, 1U, 2U);
}
