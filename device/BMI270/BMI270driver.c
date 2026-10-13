#include "BMI270driver.h"
#include "BMI270Middleware.h"
#include "BMI270reg.h"
#include "BMI270config.h"
#include <stddef.h>

static const float accel_sen = 4.0f * 9.80665f / 32768.0f;
static const float gyro_sen = 2000.0f * 0.0174532925199433f / 32768.0f;
BMI270_Diagnostics bmi270_diag;
_Static_assert(sizeof(bmi270_config_file) == 8192U, "BMI270 configuration must be complete");

static uint8_t transfer(uint8_t tx, uint8_t *rx)
{
    if (BMI270_read_write_byte(tx, rx) != HAL_OK) {
        ++bmi270_diag.spi_errors;
        return BMI270_SPI_ERROR;
    }
    return BMI270_NO_ERROR;
}

static uint8_t read_multi_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t discard, result;
    BMI270_NS_L();
    result = transfer(reg | BMI270_SPI_READ, &discard);
    /* Every BMI270 SPI read has one dummy byte, for BOTH accel and gyro. */
    if (result == BMI270_NO_ERROR) { result = transfer(0U, &discard); }
    for (uint16_t i = 0U; (i < len) && (result == BMI270_NO_ERROR); ++i) {
        result = transfer(0U, &data[i]);
    }
    BMI270_NS_H();
    BMI270_delay_us(2U);
    return result;
}

static uint8_t write_multi_reg(uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t discard, result;
    BMI270_NS_L();
    result = transfer(reg, &discard);
    for (uint16_t i = 0U; (i < len) && (result == BMI270_NO_ERROR); ++i) {
        result = transfer(data[i], &discard);
    }
    BMI270_NS_H();
    BMI270_delay_us(2U);
    return result;
}

/* Shared BMI270 bus helpers avoid duplicating identical accel/gyro SPI logic. */
static uint8_t accel_read_muli_reg(uint8_t reg, uint8_t *data, uint16_t len) { return read_multi_reg(reg, data, len); }
static uint8_t gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint16_t len) { return read_multi_reg(reg, data, len); }
static uint8_t accel_write_single_reg(uint8_t reg, uint8_t data) { return write_multi_reg(reg, &data, 1U); }
static uint8_t gyro_write_single_reg(uint8_t reg, uint8_t data) { return write_multi_reg(reg, &data, 1U); }
uint8_t accel_read_single_reg(uint8_t reg, uint8_t *data)
{
    if (data == NULL) { return BMI270_ARGUMENT_ERROR; }
    return accel_read_muli_reg(reg, data, 1U);
}

uint8_t gyro_read_single_reg(uint8_t reg, uint8_t *data)
{
    if (data == NULL) { return BMI270_ARGUMENT_ERROR; }
    return gyro_read_muli_reg(reg, data, 1U);
}

#define CHECK_SPI(expr) do { if ((expr) != BMI270_NO_ERROR) { return BMI270_SPI_ERROR; } } while (0)

uint8_t BMI270_init(void)
{
    uint8_t dummy, actual;
    bmi270_diag = (BMI270_Diagnostics){0};
    BMI270_GPIO_init();
    BMI270_com_init();
    BMI270_delay_ms(10U);
    /* First read selects SPI. Soft reset reverts the interface, so repeat it. */
    CHECK_SPI(accel_read_single_reg(BMI270_CHIP_ID_REG, &dummy));
    CHECK_SPI(accel_write_single_reg(BMI270_CMD, BMI270_SOFTRESET));
    BMI270_delay_ms(10U);
    CHECK_SPI(accel_read_single_reg(BMI270_CHIP_ID_REG, &dummy));
    BMI270_delay_ms(1U);
    CHECK_SPI(accel_read_single_reg(BMI270_CHIP_ID_REG, &bmi270_diag.chip_id));
    if (bmi270_diag.chip_id != BMI270_CHIP_ID) { return BMI270_ID_ERROR; }
    CHECK_SPI(accel_write_single_reg(BMI270_PWR_CONF, 0U));
    BMI270_delay_us(500U);
    CHECK_SPI(accel_write_single_reg(BMI270_INIT_CTRL, 0U));
    /* Bosch config upload uses WORD addresses and even-size chunks. */
    for (uint16_t offset = 0U; offset < sizeof(bmi270_config_file); offset += 32U) {
        uint8_t address[2] = {(offset / 2U) & 0x0FU, (uint8_t)((offset / 2U) >> 4U)};
        CHECK_SPI(write_multi_reg(BMI270_INIT_ADDR_0, address, 2U));
        CHECK_SPI(write_multi_reg(BMI270_INIT_DATA, &bmi270_config_file[offset], 32U));
    }
    CHECK_SPI(accel_write_single_reg(BMI270_INIT_CTRL, 1U));
    BMI270_delay_ms(150U);
    CHECK_SPI(accel_read_single_reg(BMI270_INTERNAL_STATUS, &bmi270_diag.internal_status));
    if ((bmi270_diag.internal_status & BMI270_INIT_STATUS_MASK) != BMI270_INIT_OK) {
        return BMI270_CONFIG_ERROR;
    }
    CHECK_SPI(accel_write_single_reg(BMI270_ACC_CONF, BMI270_ACC_100HZ_NORMAL));
    CHECK_SPI(accel_write_single_reg(BMI270_ACC_RANGE, BMI270_ACC_RANGE_4G));
    CHECK_SPI(gyro_write_single_reg(BMI270_GYR_CONF, BMI270_GYR_100HZ_NORMAL));
    CHECK_SPI(gyro_write_single_reg(BMI270_GYR_RANGE, BMI270_GYR_RANGE_2000));
    CHECK_SPI(accel_read_single_reg(BMI270_ACC_CONF, &actual));
    if (actual != BMI270_ACC_100HZ_NORMAL) { return BMI270_CONFIG_ERROR; }
    CHECK_SPI(accel_read_single_reg(BMI270_ACC_RANGE, &actual));
    if (actual != BMI270_ACC_RANGE_4G) { return BMI270_CONFIG_ERROR; }
    CHECK_SPI(gyro_read_single_reg(BMI270_GYR_CONF, &actual));
    if (actual != BMI270_GYR_100HZ_NORMAL) { return BMI270_CONFIG_ERROR; }
    CHECK_SPI(gyro_read_single_reg(BMI270_GYR_RANGE, &actual));
    if (actual != BMI270_GYR_RANGE_2000) { return BMI270_CONFIG_ERROR; }
    CHECK_SPI(accel_write_single_reg(BMI270_PWR_CTRL, BMI270_ACC_GYR_TEMP_ON));
    BMI270_delay_ms(100U);
    CHECK_SPI(accel_read_single_reg(BMI270_PWR_CTRL, &actual));
    if (actual != BMI270_ACC_GYR_TEMP_ON) { return BMI270_CONFIG_ERROR; }
    CHECK_SPI(accel_read_single_reg(BMI270_ERR_REG, &bmi270_diag.error_reg));
    if (bmi270_diag.error_reg != 0U) { return BMI270_SENSOR_ERROR; }
    bmi270_diag.initialized = 1U;
    return BMI270_NO_ERROR;
}

static int16_t signed_le(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)((value < 0x8000U) ? (int32_t)value : (int32_t)value - 65536);
}

static uint8_t read_sample(float gyro[3], float accel[3], float *temperature)
{
    uint8_t status, data[15], temp[2];
    if ((gyro == NULL) || (accel == NULL) || (temperature == NULL)) { return BMI270_ARGUMENT_ERROR; }
    if (bmi270_diag.initialized == 0U) { return BMI270_CONFIG_ERROR; }
    CHECK_SPI(accel_read_single_reg(BMI270_STATUS_REG, &status));
    if ((status & BMI270_DATA_READY) != BMI270_DATA_READY) { return BMI270_NOT_READY; }
    /* One coherent burst: accel XYZ, gyro XYZ, then 24-bit sensor time. */
    CHECK_SPI(accel_read_muli_reg(BMI270_ACC_DATA_REG, data, sizeof(data)));
    CHECK_SPI(read_multi_reg(BMI270_TEMP_REG, temp, sizeof(temp)));
    CHECK_SPI(accel_read_single_reg(BMI270_ERR_REG, &bmi270_diag.error_reg));
    if (bmi270_diag.error_reg != 0U) { return BMI270_SENSOR_ERROR; }
    int16_t raw_temp = signed_le(temp);
    if (raw_temp == BMI270_TEMP_INVALID) { return BMI270_NOT_READY; }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        bmi270_diag.accel_raw[axis] = signed_le(&data[axis * 2U]);
        bmi270_diag.gyro_raw[axis] = signed_le(&data[6U + axis * 2U]);
        accel[axis] = bmi270_diag.accel_raw[axis] * accel_sen;
        gyro[axis] = bmi270_diag.gyro_raw[axis] * gyro_sen;
    }
    *temperature = 23.0f + raw_temp / 512.0f;
    bmi270_diag.sensor_time = (uint32_t)data[12] | ((uint32_t)data[13] << 8U) | ((uint32_t)data[14] << 16U);
    ++bmi270_diag.samples;
    return BMI270_NO_ERROR;
}

uint8_t BMI270_read(float gyro[3], float accel[3], float *temperature)
{
    bmi270_diag.read_status = read_sample(gyro, accel, temperature);
    return bmi270_diag.read_status;
}
