#ifndef BMI270_DRIVER_H
#define BMI270_DRIVER_H
#include <stdint.h>

#define BMI270_NO_ERROR       0U
#define BMI270_SPI_ERROR      1U
#define BMI270_ID_ERROR       2U
#define BMI270_CONFIG_ERROR   3U
#define BMI270_NOT_READY      4U
#define BMI270_ARGUMENT_ERROR 5U
#define BMI270_SENSOR_ERROR   6U

/* Read-only diagnostics for Task/debugger; raw values use native chip axes. */
typedef struct {
    uint8_t chip_id, internal_status, error_reg, read_status;
    uint8_t initialized;
    int16_t accel_raw[3], gyro_raw[3];
    uint32_t sensor_time; /* 24-bit counter, 39.0625 us/tick, wraps at 655.36 s. */
    uint32_t samples, spi_errors;
} BMI270_Diagnostics;
extern BMI270_Diagnostics bmi270_diag;

/* 单寄存器读取：0 表示成功，失败返回 BMI270_* 错误码。
 * 调用前须初始化 GPIO、SPI1 和 DWT；data 不得为空。
 * BMI270 accel/gyro 共用 CHIP_ID (0x00 -> 0x24)，可在 BMI270_init 前读取。 */
uint8_t accel_read_single_reg(uint8_t reg, uint8_t *data);
uint8_t gyro_read_single_reg(uint8_t reg, uint8_t *data);

/* Blocking startup only; 0 = success, otherwise one of the codes above. */
uint8_t BMI270_init(void);
/* gyro: rad/s, accel: m/s^2, temperature: degrees C. No axis remapping/bias.
 * Returns 0 for a fresh sample; 4 when no fresh pair exists. On failure all
 * output arguments and last successful raw sample remain unchanged.
 * Unlike the BMI088 reference, return status prevents stale-data acceptance. */
uint8_t BMI270_read(float gyro[3], float accel[3], float *temperature);
#endif
