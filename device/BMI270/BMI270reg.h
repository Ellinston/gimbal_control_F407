#ifndef BMI270_REG_H
#define BMI270_REG_H

/* Bosch BMI270 datasheet BST-BMI270-DS000: one shared register map and CS. */
#define BMI270_CHIP_ID_REG       0x00U
#define BMI270_CHIP_ID           0x24U
#define BMI270_ERR_REG           0x02U
#define BMI270_STATUS_REG        0x03U
#define BMI270_ACC_DATA_REG      0x0CU
#define BMI270_GYR_DATA_REG      0x12U
#define BMI270_INTERNAL_STATUS   0x21U
#define BMI270_TEMP_REG          0x22U
#define BMI270_ACC_CONF          0x40U
#define BMI270_ACC_RANGE         0x41U //量程
#define BMI270_GYR_CONF          0x42U
#define BMI270_GYR_RANGE         0x43U //量程
#define BMI270_INIT_CTRL         0x59U
#define BMI270_INIT_ADDR_0       0x5BU
#define BMI270_INIT_DATA         0x5EU
#define BMI270_PWR_CONF          0x7CU
#define BMI270_PWR_CTRL          0x7DU
#define BMI270_CMD               0x7EU
#define BMI270_SOFTRESET         0xB6U
#define BMI270_SPI_READ          0x80U
#define BMI270_DATA_READY        0xC0U
#define BMI270_INIT_OK           0x01U
#define BMI270_INIT_STATUS_MASK  0x0FU
#define BMI270_ACC_100HZ_NORMAL  0xA8U
#define BMI270_ACC_RANGE_4G      0x01U
#define BMI270_GYR_100HZ_NORMAL  0xE8U
#define BMI270_GYR_RANGE_2000    0x00U
#define BMI270_ACC_GYR_TEMP_ON   0x0EU
#define BMI270_TEMP_INVALID      (-32768)

#endif
