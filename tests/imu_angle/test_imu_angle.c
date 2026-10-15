#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "BMI270driver.h"

/* Compile the actual Task implementation, including its private estimator.
 * Hardware stubs below never participate in the angle mathematics. */
#include "../../Task/imu_test.c"
#include "../../Task/usart_test.c"


/* 本套件只集成真实IMU/USART；电机端返回无反馈和零输出。 */
static Motor_ControlStatus mock_motor_status[MOTOR_AXIS_COUNT];
const Motor_ControlStatus *Motor_Get_Status(Motor_ID motor) { return &mock_motor_status[motor]; }
uint8_t motor_IsOnline(const Motor *motor, uint32_t now) { (void)motor; (void)now; return 0U; }
float Get_Current_Angle(Motor_ID motor) { (void)motor; return NAN; }
float Motor_Get_Output(Motor_ID motor) { (void)motor; return 0.0f; }

UART_HandleTypeDef huart1;
BMI270_Diagnostics bmi270_diag;
TestDWT test_dwt;
TestCoreDebug test_core_debug;
static uint32_t mock_ms, mock_sensor_time, read_calls, tx_calls, init_calls;
static uint8_t mock_read_status, mock_init_status;
static const uint8_t *tx_data;
static uint16_t tx_length;
static const float bias[3] = {0.01f, -0.015f, 0.02f};

uint32_t HAL_GetTick(void) { return mock_ms; }
uint32_t usart_GetTick(void) { return mock_ms; }
uint8_t usart_TxReady(void) { return huart1.gState == HAL_UART_STATE_READY; }
HAL_StatusTypeDef usart_Start(void) { return HAL_OK; }
uint16_t usart_Rx(uint8_t *data, uint16_t size) { (void)data; (void)size; return 0U; }
void HAL_Delay(uint32_t ms) { mock_ms += ms; }
void BMI270_GPIO_init(void) {}
void BMI270_com_init(void) {}
uint8_t BMI270_init(void)
{
    ++init_calls;
    bmi270_diag.initialized = mock_init_status == BMI270_NO_ERROR;
    return mock_init_status;
}
uint8_t accel_read_single_reg(uint8_t reg, uint8_t *value)
{
    for (size_t i = 0; i < IMU_INIT_CHECK_COUNT; ++i)
    {
        if (init_checks[i].address == reg) { *value = init_checks[i].expected; return 0U; }
    }
    return BMI270_ARGUMENT_ERROR;
}
uint8_t gyro_read_single_reg(uint8_t reg, uint8_t *value)
{
    return accel_read_single_reg(reg, value);
}
uint8_t BMI270_read(float gyro[3], float accel[3], float *temperature)
{
    ++read_calls;
    if (mock_read_status != BMI270_NO_ERROR) { return mock_read_status; }
    for (uint32_t i = 0; i < 3U; ++i) { gyro[i] = bias[i]; accel[i] = 0.0f; }
    accel[2] = IMU_GRAVITY_MS2;
    *temperature = 25.0f;
    mock_sensor_time = (mock_sensor_time + 256U) & 0xFFFFFFU;
    bmi270_diag.sensor_time = mock_sensor_time;
    return BMI270_NO_ERROR;
}
HAL_StatusTypeDef usart_Tx(const uint8_t *data, uint16_t length)
{
    assert(huart1.gState == HAL_UART_STATE_READY);
    assert(data != NULL && length > 0U && length < sizeof(message));
    ++tx_calls;
    tx_data = data; tx_length = length;
    huart1.gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

static float IMU_WrapDegrees(float x) { return remainderf(x,360.0f); }

static void reset_test(void)
{
    imu_angle = (IMU_AngleTestResults){0};
    Gravity_Reset(&gravity_estimator);
    imu_attitude_snapshot=(IMU_AttitudeSnapshot){0};
    angle_have_time = angle_started = 0U;
    angle_last_sensor_time = angle_last_read_ms = angle_last_print_ms = 0U;
    mock_ms = mock_sensor_time = read_calls = tx_calls = init_calls = 0U;
    mock_read_status = mock_init_status = BMI270_NO_ERROR;
    imu_init_attempted = imu_init_passed = 0U;
    imu_dwt_ready = 1U;
    bmi270_diag = (BMI270_Diagnostics){0};
    huart1.gState = HAL_UART_STATE_READY;
    IMU_TestSetTextOutput(1U);
    vofa_task_init();
    IMU_AngleRestart();
}

static void sample(float pitch_deg, float pitch_rate_dps, float yaw_rate_dps, float noise)
{
    float p = pitch_deg * IMU_DEG_TO_RAD;
    float yaw_rate = yaw_rate_dps * IMU_DEG_TO_RAD;
    float accel[3] = {-IMU_GRAVITY_MS2 * sinf(p) + noise, 0.0f,
                      IMU_GRAVITY_MS2 * cosf(p)};
    float gyro[3] = {bias[0] - yaw_rate * sinf(p),
                     bias[1] + pitch_rate_dps * IMU_DEG_TO_RAD,
                     bias[2] + yaw_rate * cosf(p)};
    mock_sensor_time = (mock_sensor_time + 256U) & 0xFFFFFFU;
    mock_ms += 10U;
    IMU_AngleSample(accel, gyro, mock_sensor_time, mock_ms);
}

static void calibrate(float pitch_deg)
{
    for (uint32_t i = 0; i < IMU_CALIBRATION_SAMPLES; ++i) { sample(pitch_deg, 0, 0, 0); }
    assert(imu_angle.ready && imu_angle.calibration_samples == IMU_CALIBRATION_SAMPLES);
    assert(fabsf(IMU_WrapDegrees(imu_angle.pitch_deg - pitch_deg)) < 0.001f);
    assert(imu_angle.yaw_deg == 0.0f);
    for (uint32_t i = 0; i < 3U; ++i) { assert(fabsf(imu_angle.gyro_bias_rads[i] - bias[i]) < 1e-6f); }
}

static void test_noise_and_pitch(void)
{
    reset_test();
    calibrate(0);
    double raw_squared = 0.0, filtered_squared = 0.0;
    for (uint32_t i = 0; i < 2000U; ++i)
    {
        float noise = (i & 1U) ? 0.15f : -0.15f;
        sample(0, 0, 0, noise);
        if (i >= 200U)
        {
            float raw_angle = atan2f(-noise, IMU_GRAVITY_MS2) * IMU_RAD_TO_DEG;
            raw_squared += raw_angle * raw_angle;
            filtered_squared += imu_angle.pitch_deg * imu_angle.pitch_deg;
        }
    }
    assert(filtered_squared < raw_squared * 0.1);
    printf("PASS stationary noise: raw RMS %.4f deg, filtered RMS %.4f deg\n",
           sqrt(raw_squared / 1800.0), sqrt(filtered_squared / 1800.0));
    for (uint32_t i = 1; i <= 100U; ++i) { sample(0.3f * (float)i, 30, 0, 0); }
    for (uint32_t i = 0; i < 300U; ++i) { sample(30, 0, 0, 0); }
    assert(fabsf(imu_angle.pitch_deg - 30.0f) < 0.2f);
    assert(fabsf(imu_angle.yaw_deg) < 0.01f);
    printf("PASS +30 deg Pitch ramp and hold: %.4f deg\n", (double)imu_angle.pitch_deg);
}

static void test_yaw_and_wrap(void)
{
    const float pitch_cases[] = {0.0f, 45.0f, 90.0f};
    for (uint32_t c = 0; c < 3U; ++c)
    {
        reset_test();
        /* Calibration deliberately crosses the 24-bit time counter boundary. */
        mock_sensor_time = 0xFFFF00U;
        calibrate(pitch_cases[c]);
        for (uint32_t i = 0; i < 300U; ++i) { sample(pitch_cases[c], 0, 30, 0); }
        for (uint32_t i = 0; i < 50U; ++i) { sample(pitch_cases[c], 0, 0, 0); }
        assert(fabsf(imu_angle.yaw_deg - 90.0f) < 0.15f);
        assert(fabsf(imu_angle.pitch_deg - pitch_cases[c]) < 0.01f);
        for (uint32_t i = 0; i < 600U; ++i) { sample(pitch_cases[c], 0, -30, 0); }
        for (uint32_t i = 0; i < 50U; ++i) { sample(pitch_cases[c], 0, 0, 0); }
        assert(fabsf(imu_angle.yaw_deg + 90.0f) < 0.15f);
        for (uint32_t i = 0; i < 900U; ++i) { sample(pitch_cases[c], 0, 30, 0); }
        for (uint32_t i = 0; i < 50U; ++i) { sample(pitch_cases[c], 0, 0, 0); }
        assert(fabsf(IMU_WrapDegrees(imu_angle.yaw_deg - 180.0f)) < 0.15f);
        assert(imu_angle.gap_resets == 0U);
        printf("PASS Yaw +90/-180/+270 deg, Pitch %.0f deg, sensor-time and angle wraps\n",
               (double)pitch_cases[c]);
    }
    reset_test();
    for (unsigned i=0;i<IMU_CALIBRATION_SAMPLES;++i) { sample(179,0,0,0); }
    assert(imu_angle.ready && fabsf(imu_angle.pitch_deg-1.0f)<0.001f);
    for (uint32_t i = 1; i <= 50U; ++i) { sample(179.0f + 0.04f * (float)i, 4, 0, 0); }
    for (uint32_t i = 0; i < 300U; ++i) { sample(181, 0, 0, 0); }
    assert(fabsf(imu_angle.pitch_deg + 1.0f) < 0.2f);
    assert(gravity_estimator.up_sensor[2]<-0.99f);
    puts("PASS inverted gravity crossing 180 deg (Pitch uses canonical +/-90 deg)");
}

static void test_rejection_and_gaps(void)
{
    reset_test();
    sample(0, 0, 0, 0);
    sample(0, 0, 30, 0);
    assert(!imu_angle.ready && imu_angle.calibration_samples == 0U);
    calibrate(0);
    uint32_t count = imu_angle.samples;
    float accel[3] = {0, 0, IMU_GRAVITY_MS2}, gyro[3] = {0, 0, 1};
    IMU_AngleSample(accel, gyro, mock_sensor_time, mock_ms);
    assert(imu_angle.samples == count && imu_angle.yaw_deg == 0);
    /* 2 g forward acceleration must not become a Pitch tilt observation. */
    accel[0] = 2.0f * IMU_GRAVITY_MS2;
    for (uint32_t i = 0; i < 100U; ++i)
    {
        mock_sensor_time += 256U; mock_ms += 10U;
        IMU_AngleSample(accel, bias, mock_sensor_time, mock_ms);
    }
    assert(fabsf(imu_angle.pitch_deg) < 0.001f);
    mock_sensor_time += 2560U; mock_ms += 100U;
    sample(0, 0, 0, 0);
    assert(!imu_angle.ready && imu_angle.gap_resets == 1U);
    assert(imu_angle.calibration_samples == 1U);
    accel[0] = NAN;
    IMU_AngleSample(accel, gyro, mock_sensor_time + 256U, mock_ms + 10U);
    assert(!imu_angle.ready && imu_angle.errors == 1U && !angle_have_time);
    puts("PASS moving calibration, duplicate sample, acceleration rejection, gap and NaN handling");
}

static void test_task_and_output(void)
{
    reset_test();
    IMU_Angle_Test();
    assert(init_calls == 1U && imu_init_passed);
    huart1.gState = HAL_UART_STATE_READY;
    for (uint32_t i = 0; i < 1100U; ++i)
    {
        mock_ms += 2U;
        mock_read_status = (i % 5U == 0U) ? BMI270_NO_ERROR : BMI270_NOT_READY;
        IMU_Angle_Test();
        huart1.gState = HAL_UART_STATE_READY;
    }
    assert(imu_angle.ready && init_calls == 1U && imu_angle.not_ready > 0U);
    assert(imu_angle.samples == 220U);
    imu_angle.pitch_deg = 12.34f; imu_angle.yaw_deg = -5.67f;
    angle_last_print_ms = mock_ms - IMU_PRINT_PERIOD_MS;
    mock_read_status = BMI270_NOT_READY;
    IMU_Angle_Test();
    assert(strcmp(message, "Pitch: 12.34 deg, Yaw: -5.67 deg\r\n") == 0);
    char saved[sizeof(message)];
    memcpy(saved, message, sizeof(message));
    uint32_t previous_read_calls = read_calls;
    mock_ms += 500U;
    IMU_Angle_Test();
    assert(read_calls > previous_read_calls && memcmp(saved, message, sizeof(message)) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    IMU_Angle_Test();
    assert(strstr(message, "unavailable") && !strstr(message, "Pitch:"));
    huart1.gState = HAL_UART_STATE_READY;
    mock_read_status = BMI270_SPI_ERROR; mock_ms += 500U;
    IMU_Angle_Test();
    assert(strstr(message, "unavailable") && imu_angle.errors == 1U);
    reset_test();
    mock_init_status = BMI270_CONFIG_ERROR;
    IMU_Angle_Test();
    huart1.gState = HAL_UART_STATE_READY; mock_ms += 500U;
    IMU_Angle_Test();
    assert(init_calls == 1U && read_calls == 0U && strstr(message, "init failed"));
    puts("PASS 500 Hz polling/100 Hz samples, NOT_READY, exact format, TX lifetime, stale/error/init failure");
}

static void test_vofa_integration(void)
{
    reset_test();
    IMU_TestSetTextOutput(0U);
    IMU_Angle_Test();
    assert(init_calls == 1U && tx_calls == 0U);
    assert(isnan(imu_vofa_data.pitch_angle) && isnan(imu_vofa_data.accel_x));
    mock_ms += 10U; IMU_Angle_Test();
    assert(isnan(imu_vofa_data.yaw_angle) && imu_vofa_data.accel_x == 0.0f);
    for (uint32_t i = 1U; i < 220U; ++i)
    {
        mock_ms += 10U;
        IMU_Angle_Test();
        uint32_t before = tx_calls;
        vofa_task_update();
        if (tx_calls != before) { assert(tx_length == 28U); }
        huart1.gState = HAL_UART_STATE_READY;
    }
    assert(imu_angle.ready && fabsf(imu_vofa_data.pitch_angle) < 0.001f);
    uint32_t reads_before_get = read_calls;
    assert(imu_test_get_yaw_angle() == 0.0f);
    assert(read_calls == reads_before_get);
    assert(imu_vofa_data.yaw_angle == 0.0f && init_calls == 1U);
    /* Synthetic data passes through the real estimator, snapshot and packer. */
    sample(20.0f, 0.0f, 10.0f, 0.0f);
    mock_read_status = BMI270_NOT_READY;
    IMU_Angle_Test();
    imu_vofa_data_t expected = imu_vofa_data;
    assert(expected.accel_x < -3.0f && expected.accel_y == 0.0f);
    assert(imu_test_get_yaw_angle() == expected.yaw_angle);
    assert(imu_test_get_pitch_angle() == expected.pitch_angle);
    vofa_last_sample_ms = mock_ms - 100U;
    vofa_task_update();
    assert(tx_length == 28U);
    float values[6]; memcpy(values, tx_data, sizeof(values));
    assert(values[0] == expected.yaw_angle && values[1] == expected.pitch_angle);
    assert(isnan(values[2]) && isnan(values[3]));
    assert(values[4] == 0.0f && values[5] == 0.0f);
    const uint8_t tail[4] = {0, 0, 0x80, 0x7F};
    assert(memcmp(tx_data + 24U, tail, sizeof(tail)) == 0);
    uint8_t saved[28]; memcpy(saved, tx_data, sizeof(saved));
    mock_ms += 101U;
    /* 即使采样任务未再调度，getter也必须自行拒绝过期角度。 */
    assert(isnan(imu_test_get_yaw_angle()) && isnan(imu_test_get_pitch_angle()));
    IMU_Angle_Test(); vofa_task_update();
    assert(isnan(imu_vofa_data.pitch_angle) && isnan(imu_vofa_data.accel_x));
    assert(memcmp(saved, tx_data, sizeof(saved)) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    mock_ms += 100U; vofa_task_update();
    memcpy(values, tx_data, sizeof(values));
    for (uint32_t i = 0; i < 4U; ++i) { assert(isnan(values[i])); }
    huart1.gState = HAL_UART_STATE_READY;
    mock_read_status = BMI270_SPI_ERROR; mock_ms += 10U; IMU_Angle_Test();
    assert(isnan(imu_vofa_data.yaw_angle) && isnan(imu_vofa_data.accel_y));
    reset_test(); IMU_TestSetTextOutput(0U); mock_init_status = BMI270_CONFIG_ERROR;
    IMU_Angle_Test(); mock_ms += 1000U; IMU_Angle_Test();
    assert(init_calls == 1U && read_calls == 0U && tx_calls == 0U);
    assert(isnan(imu_vofa_data.pitch_angle));
    assert(isnan(imu_test_get_yaw_angle()) && isnan(imu_test_get_pitch_angle()));
    puts("PASS IMU VOFA: 10ms task, calibration, snapshot units/order, no init/text mixing,");
    puts("               async lifetime, stale/error NaNs and silent init failure.");
}

int main(void)
{
    test_noise_and_pitch();
    test_yaw_and_wrap();
    test_rejection_and_gaps();
    test_task_and_output();
    test_vofa_integration();
    puts("All IMU angle host tests passed (synthetic input; not hardware acceptance).");
    return 0;
}

