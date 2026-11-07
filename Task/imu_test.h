#ifndef TASK_IMU_TEST_H
#define TASK_IMU_TEST_H

#include <stdint.h>

/* 默认保留独立测试的文本日志；VOFA 模式在进入循环前传0，禁止混流。 */
void IMU_TestSetTextOutput(uint8_t enabled);

/* 外设初始化后调用一次，准备 SPI 通信并启用 DWT。 */
void IMU_TestInit(void);

/* 读取两条驱动接口的共享 CHIP_ID，并通过 usart_Tx 打印。
 * 先调用 IMU_TestInit；单次尝试，串口忙时立即返回。 */
void Task_IMU_Read_ID(void);

/* 首次调用完整初始化，之后仅回读配置并打印；失败需复位后重试。
 * 包含阻塞初始化过程，仅用于当前独立测试任务。 */
void Task_IMU_Init_Test(void);

/* 原始物理量测试，保留供单独切换验证；不要与角度测试同时调度。 */
void Task_IMU_Data_Test(void);

/* Three-axis gravity observer. Stationary gyro calibration takes ~2 s.
 * Actual startup tilt initializes gravity; startup is NOT a level reference.
 * Right handed sensor axes; roll=atan2(ay,az), pitch=atan2(-ax,hypot(ay,az)).
 * Existing Yaw remains the relative integral of gyro projected on vertical,
 * not Roll, absolute heading or general ZYX Euler yaw. Units: degrees.
 * Publishes a snapshot then derives VOFA fields. Installation requires calibration. */
void IMU_Angle_Test(void);

/* 只读最新解算结果，不采样；标定未完成、读取错误或超过100ms返回NaN。
 * 单位deg：Yaw为标定后相对角，Pitch为重力参考角，Yaw在[-180,180)，Pitch在[-90,90]。 */
float imu_test_get_yaw_angle(void);
float imu_test_get_pitch_angle(void);

/* Main-loop shared data: copy once, check valid AND unsigned age <=100 ms.
 * No ISR consumers. Add synchronization if moved to concurrent tasks.
 * up_sensor is unit UP, the negative of physical gravitational acceleration. */
typedef struct
{
    float up_sensor[3];
    float roll_deg, pitch_deg, yaw_deg;
    float accel_roll_deg, accel_pitch_deg, accel_weight;
    uint32_t sample_ms, sensor_time;
    uint8_t valid;
    float gyro_sensor_rads[3]; /* Same sample, stationary bias removed; sensor axes. */
} IMU_AttitudeSnapshot;
extern IMU_AttitudeSnapshot imu_attitude_snapshot;

/* 主循环反复调用，当前调度 IMU_Angle_Test。 */
void IMU_Test(void);

#endif /* TASK_IMU_TEST_H */

