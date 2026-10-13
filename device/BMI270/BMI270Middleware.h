#ifndef BMI270_MIDDLEWARE_H
#define BMI270_MIDDLEWARE_H
#include "main.h"

void BMI270_GPIO_init(void);
void BMI270_com_init(void);
void BMI270_NS_H(void);
void BMI270_NS_L(void);
void BMI270_delay_ms(uint16_t ms);
void BMI270_delay_us(uint16_t us);
/* rx receives one byte; HAL errors are returned, never disguised as data. */
HAL_StatusTypeDef BMI270_read_write_byte(uint8_t tx, uint8_t *rx);
#endif
