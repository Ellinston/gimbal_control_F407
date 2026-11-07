#include "imu_test.h"
#include "gravity_estimator.h"
#include "main.h"
#include "BMI270driver.h"
#include "BMI270reg.h"
#include "BMI270Middleware.h"
#include "usart.h"
#include "usart_test.h"
#include <math.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

#define IMU_TEST_PERIOD_MS 1000U
#define IMU_READ_PERIOD_MS 2U     /* 500 Hz 调用；芯片当前输出新样本为 100 Hz。 */
#define IMU_PRINT_PERIOD_MS 500U  /* 每 0.5 秒打印一次，采样轮询仍保持 500 Hz。 */

/* 可在调试器中逐项查看；读取失败时 ID 不用于判断芯片型号。 */
static volatile uint8_t imu_accel_id;
static volatile uint8_t imu_gyro_id;
static volatile uint8_t imu_accel_status = BMI270_NOT_READY;
static volatile uint8_t imu_gyro_status = BMI270_NOT_READY;
static volatile uint8_t imu_dwt_ready;
static volatile HAL_StatusTypeDef imu_uart_status = HAL_OK;
static uint32_t last_test_ms;
static char message[1024];
static uint8_t imu_text_output = 1U;

void IMU_TestSetTextOutput(uint8_t enabled)
{
    imu_text_output = (enabled != 0U);
}

typedef struct
{
    const char *name;
    uint8_t address;
    uint8_t expected;
    uint8_t mask;
    uint8_t use_gyro_reader;
} IMU_RegisterCheck;

static const IMU_RegisterCheck init_checks[] =
{
    {"CHIP_ID", BMI270_CHIP_ID_REG, BMI270_CHIP_ID, 0xFFU, 0U},
    {"INTERNAL_STATUS", BMI270_INTERNAL_STATUS, BMI270_INIT_OK, BMI270_INIT_STATUS_MASK, 0U},
    {"ERR_REG", BMI270_ERR_REG, 0x00U, 0xFFU, 0U},
    {"ACC_CONF", BMI270_ACC_CONF, BMI270_ACC_100HZ_NORMAL, 0xFFU, 0U},
    {"ACC_RANGE", BMI270_ACC_RANGE, BMI270_ACC_RANGE_4G, 0xFFU, 0U},
    {"GYR_CONF", BMI270_GYR_CONF, BMI270_GYR_100HZ_NORMAL, 0xFFU, 1U},
    {"GYR_RANGE", BMI270_GYR_RANGE, BMI270_GYR_RANGE_2000, 0xFFU, 1U},
    {"PWR_CONF", BMI270_PWR_CONF, 0x00U, 0xFFU, 0U},
    {"PWR_CTRL", BMI270_PWR_CTRL, BMI270_ACC_GYR_TEMP_ON, 0xFFU, 0U},
};
#define IMU_INIT_CHECK_COUNT (sizeof(init_checks) / sizeof(init_checks[0]))

static volatile uint8_t imu_init_attempted;
static volatile uint8_t imu_init_status = BMI270_NOT_READY;
static volatile uint8_t imu_init_passed;
static volatile uint8_t imu_config_actual[IMU_INIT_CHECK_COUNT];
static volatile uint8_t imu_config_status[IMU_INIT_CHECK_COUNT];
static volatile uint8_t imu_config_passed[IMU_INIT_CHECK_COUNT];

/* 原始有符号值来自驱动诊断；物理量来自 BMI270_read 的换算结果。
 * 不做零偏扣除或姿态解算，方便观察传感器的真实静止/运动响应。 */
typedef struct
{
    float accel_ms2[3];
    float gyro_rads[3];
    float temperature_c;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    uint32_t sensor_time;
    uint32_t read_calls;
    uint32_t samples;
    uint32_t not_ready;
    uint32_t errors;
    uint32_t missed_periods;
    uint32_t last_sample_ms;
    uint8_t read_status;
} IMU_DataTestResults;

static volatile IMU_DataTestResults imu_data;
static uint32_t last_read_ms;
static uint32_t last_print_ms;
static uint8_t data_test_started;

void IMU_TestInit(void)
{
    uint8_t ignored_id;
    uint32_t start;

    BMI270_GPIO_init();
    BMI270_com_init();
    __DSB();
    __ISB();

    /* 驱动读取内部使用微秒延时；先有界检查 DWT，避免失效时死循环。 */
    start = DWT->CYCCNT;
    for (uint32_t i = 0U; i < 128U; ++i) { __NOP(); }
    imu_dwt_ready =
        ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U) &&
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) &&
        ((uint32_t)(DWT->CYCCNT - start) != 0U);

    HAL_Delay(10U);
    if (imu_dwt_ready)
    {
        /* 首次空读只用于切换 SPI 接口，不能把它作为 ID 验收结果。 */
        imu_accel_status = accel_read_single_reg(BMI270_CHIP_ID_REG, &ignored_id);
        HAL_Delay(1U);
    }
    /* 完整初始化由主循环中的 Task_IMU_Init_Test 首次运行时执行。 */
    last_test_ms = HAL_GetTick() - IMU_TEST_PERIOD_MS;
}

void Task_IMU_Read_ID(void)
{
    if (!imu_text_output) { return; }
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;
    int length;

    /* 静态缓冲区仍在发送时不改写它，也不发起另一轮读取。 */
    if (huart1.gState != HAL_UART_STATE_READY) { return; }

    if (imu_dwt_ready)
    {
        /* BMI270 是单芯片：两条接口读取同一个 CHIP_ID，均应为 0x24。 */
        HAL_Delay(1U);
        imu_accel_status = accel_read_single_reg(BMI270_CHIP_ID_REG, &accel_id);
        HAL_Delay(1U);
        imu_gyro_status = gyro_read_single_reg(BMI270_CHIP_ID_REG, &gyro_id);
        HAL_Delay(1U);
        imu_accel_id = accel_id;
        imu_gyro_id = gyro_id;

        length = snprintf(message, sizeof(message),
            "ACCEL ID=0x%02X status=%u %s\r\n"
            "GYRO  ID=0x%02X status=%u %s\r\n"
            "BMI270 shared CHIP_ID expected=0x24\r\n\r\n",
            (unsigned int)accel_id, (unsigned int)imu_accel_status,
            ((imu_accel_status == BMI270_NO_ERROR) &&
             (accel_id == BMI270_CHIP_ID)) ? "PASS" : "FAIL",
            (unsigned int)gyro_id, (unsigned int)imu_gyro_status,
            ((imu_gyro_status == BMI270_NO_ERROR) &&
             (gyro_id == BMI270_CHIP_ID)) ? "PASS" : "FAIL");
    }
    else
    {
        length = snprintf(message, sizeof(message),
                          "BMI270 ID pretest SKIP: DWT not running\r\n");
    }

    if ((length > 0) && ((size_t)length < sizeof(message)))
    {
        imu_uart_status = usart_Tx((const uint8_t *)message, (uint16_t)length);
    }
}

void Task_IMU_Init_Test(void)
{
    size_t used;
    int length;

    if (imu_text_output && !usart_TxReady()) { return; }
    if (!imu_dwt_ready)
    {
        static const uint8_t skipped[] = "IMU INIT CHECK SKIP: DWT not running\r\n";
        imu_init_passed = 0U;
        if (imu_text_output) { imu_uart_status = usart_Tx(skipped, sizeof(skipped) - 1U); }
        return;
    }

    /* 每次上电只初始化一次。失败保留现场，复位 MCU 后才重试。
     * 后续调用只回读，不反复软复位或上传配置。 */
    if (!imu_init_attempted)
    {
        imu_init_attempted = 1U;
        imu_init_status = BMI270_init();
    }
    imu_init_passed = (imu_init_status == BMI270_NO_ERROR) &&
                      (bmi270_diag.initialized == 1U);

    length = snprintf(message, sizeof(message),
        "BMI270_init status=%u initialized=%u %s\r\n",
        (unsigned int)imu_init_status, (unsigned int)bmi270_diag.initialized,
        imu_init_passed ? "PASS" : "FAIL");
    if ((length <= 0) || ((size_t)length >= sizeof(message))) { return; }
    used = (size_t)length;

    /* 使用驱动读函数重新访问硬件，不使用初始化过程缓存的寄存器值。 */
    for (size_t i = 0U; i < IMU_INIT_CHECK_COUNT; ++i)
    {
        const IMU_RegisterCheck *check = &init_checks[i];
        uint8_t actual = 0U;
        HAL_Delay(1U); /* 初始化失败时也保留省电模式下的通信间隔。 */
        uint8_t status = check->use_gyro_reader ?
            gyro_read_single_reg(check->address, &actual) :
            accel_read_single_reg(check->address, &actual);
        uint8_t passed = (status == BMI270_NO_ERROR) &&
            ((actual & check->mask) == (check->expected & check->mask));
        imu_config_actual[i] = actual;
        imu_config_status[i] = status;
        imu_config_passed[i] = passed;
        if (!passed) { imu_init_passed = 0U; }

        length = snprintf(message + used, sizeof(message) - used,
            "%s[0x%02X]=0x%02X expected=0x%02X mask=0x%02X status=%u %s\r\n",
            check->name, (unsigned int)check->address, (unsigned int)actual,
            (unsigned int)check->expected, (unsigned int)check->mask,
            (unsigned int)status, passed ? "PASS" : "FAIL");
        if ((length <= 0) || ((size_t)length >= sizeof(message) - used)) { return; }
        used += (size_t)length;
    }
    length = snprintf(message + used, sizeof(message) - used,
                      "IMU INIT CHECK %s\r\n\r\n", imu_init_passed ? "PASS" : "FAIL");
    if ((length <= 0) || ((size_t)length >= sizeof(message) - used)) { return; }
    used += (size_t)length;
    if (imu_text_output) { imu_uart_status = usart_Tx((const uint8_t *)message, (uint16_t)used); }
}

static void IMU_FormatMilli(char *text, size_t capacity, float value)
{
    /* nano printf 默认未启用 %f，用整数格式显示三位小数，不改链接选项。
     * BMI270 正常量程内的加速度/角速度远小于 int32_t 的溢出范围。 */
    int32_t milli = (int32_t)(value * 1000.0f + (value < 0.0f ? -0.5f : 0.5f));
    uint32_t magnitude = (uint32_t)(milli < 0 ? -milli : milli);
    (void)snprintf(text, capacity, "%s%lu.%03lu", milli < 0 ? "-" : "",
                   (unsigned long)(magnitude / 1000U),
                   (unsigned long)(magnitude % 1000U));
}

void Task_IMU_Data_Test(void)
{
    uint32_t now;
    int length;

    /* 每次复位只进行一次完整初始化和配置检查。
     * 失败时停止采样，继续报告初始化结果，不能打印占位数据冒充样本。 */
    if (!data_test_started)
    {
        if (huart1.gState != HAL_UART_STATE_READY) { return; }
        Task_IMU_Init_Test();
        data_test_started = 1U;
        last_read_ms = HAL_GetTick();
        last_print_ms = last_read_ms;
        last_test_ms = last_read_ms;
    }

    now = HAL_GetTick();
    if (!imu_init_passed)
    {
        if (((uint32_t)(now - last_test_ms) >= IMU_TEST_PERIOD_MS) &&
            (huart1.gState == HAL_UART_STATE_READY))
        {
            last_test_ms = now;
            Task_IMU_Init_Test(); /* 已有 attempted 标志，不重复软复位。 */
        }
        return;
    }

    uint32_t elapsed = (uint32_t)(now - last_read_ms);
    if (elapsed >= IMU_READ_PERIOD_MS)
    {
        float accel[3], gyro[3], temperature;
        /* 维持 2 ms 节拍；错过的周期记录下来，不突发补读旧样本。 */
        imu_data.missed_periods += elapsed / IMU_READ_PERIOD_MS - 1U;
        last_read_ms += (elapsed / IMU_READ_PERIOD_MS) * IMU_READ_PERIOD_MS;
        ++imu_data.read_calls;
        uint8_t status = BMI270_read(gyro, accel, &temperature);
        imu_data.read_status = status;

        if (status == BMI270_NO_ERROR)
        {
            /* 驱动内部已经 raw * accel_sen / gyro_sen，不能再次乘灵敏度。 */
            for (uint32_t axis = 0U; axis < 3U; ++axis)
            {
                imu_data.accel_ms2[axis] = accel[axis];
                imu_data.gyro_rads[axis] = gyro[axis];
                imu_data.accel_raw[axis] = bmi270_diag.accel_raw[axis];
                imu_data.gyro_raw[axis] = bmi270_diag.gyro_raw[axis];
            }
            imu_data.temperature_c = temperature;
            imu_data.sensor_time = bmi270_diag.sensor_time;
            imu_data.last_sample_ms = HAL_GetTick();
            ++imu_data.samples;
        }
        else if (status == BMI270_NOT_READY)
        {
            ++imu_data.not_ready; /* 500 Hz 轮询 100 Hz 芯片，未就绪是正常情况。 */
        }
        else
        {
            ++imu_data.errors; /* 不更新上次成功物理量和原始值。 */
        }
    }

    now = HAL_GetTick();
    if (!imu_text_output) { return; }
    if (((uint32_t)(now - last_print_ms) < IMU_PRINT_PERIOD_MS) ||
        (huart1.gState != HAL_UART_STATE_READY)) { return; }
    last_print_ms = now;

    if (imu_data.samples == 0U)
    {
        length = snprintf(message, sizeof(message),
            "IMU WAIT sample status=%u calls=%lu not_ready=%lu errors=%lu\r\n",
            (unsigned int)imu_data.read_status, (unsigned long)imu_data.read_calls,
            (unsigned long)imu_data.not_ready, (unsigned long)imu_data.errors);
    }
    else if (((imu_data.read_status != BMI270_NO_ERROR) &&
              (imu_data.read_status != BMI270_NOT_READY)) ||
             ((uint32_t)(now - imu_data.last_sample_ms) > 100U))
    {
        /* 读取失败或数据过期时显式提示，不将旧数据作为正常读数打印。 */
        length = snprintf(message, sizeof(message),
            "IMU data unavailable: status=%u age=%lums\r\n\r\n",
            (unsigned int)imu_data.read_status,
            (unsigned long)(uint32_t)(now - imu_data.last_sample_ms));
    }
    else
    {
        char accel_text[3][16], gyro_text[3][16];
        for (uint32_t axis = 0U; axis < 3U; ++axis)
        {
            IMU_FormatMilli(accel_text[axis], sizeof(accel_text[axis]), imu_data.accel_ms2[axis]);
            IMU_FormatMilli(gyro_text[axis], sizeof(gyro_text[axis]), imu_data.gyro_rads[axis]);
        }
        length = snprintf(message, sizeof(message),
            "Acc X: %s\r\n"
            "Acc Y: %s\r\n"
            "Acc Z: %s\r\n"
            "W X: %s\r\n"
            "W Y: %s\r\n"
            "W Z: %s\r\n\r\n",
            accel_text[0], accel_text[1], accel_text[2],
            gyro_text[0], gyro_text[1], gyro_text[2]);
    }

    /* 发送繁忙不会停止上面的读取；仅在空闲时改写并提交静态文本。 */
    if ((length > 0) && ((size_t)length < sizeof(message)))
    {
        imu_uart_status = usart_Tx((const uint8_t *)message, (uint16_t)length);
    }
}

/* Task owns timing/publication; Bsp/Algorithm owns gravity/bias mathematics. */
#define IMU_GRAVITY_MS2 GRAVITY_MS2
#define IMU_RAD_TO_DEG 57.2957795131f
#define IMU_DEG_TO_RAD 0.0174532925199f
#define IMU_CALIBRATION_SAMPLES GRAVITY_CALIBRATION_SAMPLES
#define IMU_MAX_SAMPLE_TICKS 1280U
#define IMU_SENSOR_TICK_S 0.0000390625f

typedef struct
{
    float pitch_deg, roll_deg, yaw_deg, accel_pitch_deg, accel_roll_deg;
    float gyro_bias_rads[3];
    float gyro_sensor_rads[3];
    uint32_t calibration_samples, samples, errors, not_ready, gap_resets;
    uint32_t last_sample_ms;
    uint8_t ready, read_status;
} IMU_AngleTestResults;
static IMU_AngleTestResults imu_angle;
static GravityEstimator gravity_estimator;
static uint32_t angle_last_sensor_time, angle_last_read_ms, angle_last_print_ms;
static uint8_t angle_started, angle_have_time;
static float angle_accel_ms2[2];
IMU_AttitudeSnapshot imu_attitude_snapshot;

static uint8_t IMU_AngleDataAvailable(uint32_t now)
{
    return (uint8_t)(imu_init_passed && angle_have_time && imu_angle.samples != 0U &&
        (imu_angle.read_status == BMI270_NO_ERROR || imu_angle.read_status == BMI270_NOT_READY) &&
        (uint32_t)(now - imu_angle.last_sample_ms) <= 100U);
}
float imu_test_get_yaw_angle(void)
{
    return imu_attitude_snapshot.valid &&
        (uint32_t)(HAL_GetTick()-imu_attitude_snapshot.sample_ms)<=100U ? imu_attitude_snapshot.yaw_deg : NAN;
}
float imu_test_get_pitch_angle(void)
{
    return imu_attitude_snapshot.valid &&
        (uint32_t)(HAL_GetTick()-imu_attitude_snapshot.sample_ms)<=100U ? imu_attitude_snapshot.pitch_deg : NAN;
}
static void IMU_UpdateVofaData(uint32_t now)
{
    IMU_AttitudeSnapshot snapshot = {0};
    snapshot.sample_ms=imu_angle.last_sample_ms;
    snapshot.sensor_time=angle_last_sensor_time;
    snapshot.valid=imu_angle.ready && IMU_AngleDataAvailable(now);
    snapshot.pitch_deg=imu_angle.pitch_deg; snapshot.roll_deg=imu_angle.roll_deg;
    snapshot.yaw_deg=imu_angle.yaw_deg;
    snapshot.accel_pitch_deg=imu_angle.accel_pitch_deg;
    snapshot.accel_roll_deg=imu_angle.accel_roll_deg;
    snapshot.accel_weight=gravity_estimator.accel_weight;
    for (uint32_t i=0; i<3; ++i)
    {
        snapshot.up_sensor[i]=gravity_estimator.up_sensor[i];
        snapshot.gyro_sensor_rads[i]=imu_angle.gyro_sensor_rads[i];
    }
    imu_attitude_snapshot=snapshot;
    imu_vofa_data = (imu_vofa_data_t){NAN, NAN, NAN, NAN};
    if (!IMU_AngleDataAvailable(now)) { return; }
    imu_vofa_data.accel_x = angle_accel_ms2[0];
    imu_vofa_data.accel_y = angle_accel_ms2[1];
    if (snapshot.valid)
    {
        imu_vofa_data.pitch_angle = snapshot.pitch_deg;
        imu_vofa_data.yaw_angle = snapshot.yaw_deg;
    }
}
static void IMU_AngleRestart(void)
{
    Gravity_Reset(&gravity_estimator);
    imu_angle.ready = 0U; imu_angle.calibration_samples = 0U;
    imu_attitude_snapshot.valid=0U;
}
static void IMU_AngleSample(const float accel[3], const float gyro[3],
                             uint32_t sensor_time, uint32_t now_ms)
{
    uint32_t ticks = (sensor_time - angle_last_sensor_time) & 0x00FFFFFFU;
    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        if (!isfinite(accel[axis]) || !isfinite(gyro[axis]))
        { ++imu_angle.errors; angle_have_time=0U; IMU_AngleRestart(); return; }
    }
    if (angle_have_time && ticks == 0U) { return; }
    if (angle_have_time && (ticks > IMU_MAX_SAMPLE_TICKS ||
        (uint32_t)(now_ms-imu_angle.last_sample_ms)>50U))
    { ++imu_angle.gap_resets; IMU_AngleRestart(); ticks=256U; }
    if (!angle_have_time) { ticks=256U; }
    angle_have_time=1U; angle_last_sensor_time=sensor_time;
    imu_angle.last_sample_ms=now_ms; ++imu_angle.samples;
    angle_accel_ms2[0]=accel[0]; angle_accel_ms2[1]=accel[1];
    //计算
    if (!Gravity_Update(&gravity_estimator,accel,gyro,(float)ticks*IMU_SENSOR_TICK_S))
    { ++imu_angle.errors; IMU_AngleRestart(); return; }
    imu_angle.ready=gravity_estimator.ready;
    imu_angle.calibration_samples=gravity_estimator.calibration_samples;
    imu_angle.roll_deg=gravity_estimator.roll_deg;
    imu_angle.pitch_deg=gravity_estimator.pitch_deg;
    imu_angle.yaw_deg=gravity_estimator.yaw_deg;
    imu_angle.accel_roll_deg=gravity_estimator.accel_roll_deg;
    imu_angle.accel_pitch_deg=gravity_estimator.accel_pitch_deg;
    for (uint32_t i=0; i<3; ++i)
    {
        imu_angle.gyro_bias_rads[i]=gravity_estimator.gyro_bias_rads[i];
        imu_angle.gyro_sensor_rads[i]=gyro[i]-gravity_estimator.gyro_bias_rads[i];
    }
}

static void IMU_FormatDegrees(char *text, size_t capacity, float degrees)
{
    /* 避免依赖 newlib-nano 的浮点 printf；先四舍五入再格式化两位小数。 */
    int32_t centi = (int32_t)(degrees * 100.0f + (degrees < 0.0f ? -0.5f : 0.5f));
    uint32_t magnitude = (uint32_t)(centi < 0 ? -centi : centi);
    (void)snprintf(text, capacity, "%s%lu.%02lu", centi < 0 ? "-" : "",
                   (unsigned long)(magnitude / 100U), (unsigned long)(magnitude % 100U));
}

void IMU_Angle_Test(void)
{
    if (!angle_started)
    {
        if (imu_text_output && !usart_TxReady()) { return; }
        Task_IMU_Init_Test();
        angle_started = 1U;
        angle_last_read_ms = HAL_GetTick();
        angle_last_print_ms = angle_last_read_ms;
    }
    uint32_t now = HAL_GetTick();
    if (imu_init_passed && ((uint32_t)(now - angle_last_read_ms) >= IMU_READ_PERIOD_MS))
    {
        float accel[3], gyro[3], temperature;
        angle_last_read_ms = now; /* 不突发补读，dt 使用芯片的 24 位时间戳。 */
        uint8_t status = BMI270_read(gyro, accel, &temperature);
        imu_angle.read_status = status;
        if (status == BMI270_NO_ERROR)
        {
            IMU_AngleSample(accel, gyro, bmi270_diag.sensor_time, HAL_GetTick());
        }
        else if (status == BMI270_NOT_READY) { ++imu_angle.not_ready; }
        else { ++imu_angle.errors; }
    }
    now = HAL_GetTick();
    IMU_UpdateVofaData(now);
    if (!imu_text_output) { return; }
    if (((uint32_t)(now - angle_last_print_ms) < IMU_PRINT_PERIOD_MS) ||
        (huart1.gState != HAL_UART_STATE_READY)) { return; }
    angle_last_print_ms = now;
    int length;
    if (!imu_init_passed)
    {
        length = snprintf(message, sizeof(message), "IMU init failed: status=%u; reset to retry\r\n",
                          (unsigned int)imu_init_status);
    }
    else if (((imu_angle.read_status != BMI270_NO_ERROR) &&
              (imu_angle.read_status != BMI270_NOT_READY)) ||
             ((uint32_t)(now - imu_angle.last_sample_ms) > 100U))
    {
        length = snprintf(message, sizeof(message), "IMU angle unavailable: status=%u\r\n",
                          (unsigned int)imu_angle.read_status);
    }
    else if (!imu_angle.ready)
    {
        length = snprintf(message, sizeof(message), "IMU calibrating: keep still (%lu/%u)\r\n",
            (unsigned long)imu_angle.calibration_samples, (unsigned int)IMU_CALIBRATION_SAMPLES);
    }
    else
    {
        char pitch[16], yaw[16];
        IMU_FormatDegrees(pitch, sizeof(pitch), imu_angle.pitch_deg);
        IMU_FormatDegrees(yaw, sizeof(yaw), imu_angle.yaw_deg);
        length = snprintf(message, sizeof(message), "Pitch: %s deg, Yaw: %s deg\r\n", pitch, yaw);
    }
    if ((length > 0) && ((size_t)length < sizeof(message)))
    {
        imu_uart_status = usart_Tx((const uint8_t *)message, (uint16_t)length);
    }
}

void IMU_Test(void)
{
    IMU_Angle_Test();
}

