#ifndef TASK_IMU_LEVEL_CONFIG_H
#define TASK_IMU_LEVEL_CONFIG_H
#include "gimbal_level.h"
#include "motor.h"
#include <math.h>

/* Installation/geometry only; motor.h remains the sole travel/PID source.
 * 2026-10-15 coarse physical calibration, NOT a precision-level acceptance:
 * base flat on table, platform visually level and fronts aligned. The two
 * zero_deg values are GM6020 absolute single-turn readings in that pose,
 * NOT the startup logical YAW=0/PITCH=100. Front-down/up Pitch sweeps and
 * fixed-base tilted Yaw sweep checked axis signs and Rz*Ry consistency.
 * The matrix maps BMI270 up in sensor coordinates into top coordinates;
 * +Z was the first marked reference. The later returned visually level pose
 * had Pitch +3.823 deg, so platform_up_top records that separate, coarsely
 * observed horizontal reference (n_T=Ry(+3.823 deg)*+Z). The 0.5 deg solver
 * tolerance and near-vertical cone avoid needless Yaw rotation from smaller
 * residuals. No independent level or mechanical zero mark was available.
 * Recheck this
 * provisional calibration and physical travel before precision use. Task
 * latches the snapshot at Init; never retune it during a run. */
#define IMU_LEVEL_CONFIG_INITIALIZER { \
    .sensor_to_top={{-0.14015827f,0.98873400f,0.05254269f}, \
                    {-0.98987686f,-0.13872733f,-0.02997553f}, \
                    {-0.02234872f,-0.05621212f,0.99816869f}}, \
    .platform_up_top={0.06667866f,0,0.99777450f}, \
    .zero_deg={297.37793f,296.98242f}, \
    .direction={1,-1}, \
    .minimum_deg={MOTOR_YAW_MIN_DEG,MOTOR_PITCH_MIN_DEG}, \
    .maximum_deg={MOTOR_YAW_MAX_DEG,MOTOR_PITCH_MAX_DEG}, \
    .slew_dps={20.0f,20.0f}, .level_tolerance_deg=0.50f, \
    .singular_cone_deg=0.50f, .branch_hysteresis_deg=3.0f, .calibrated=1U }

/* Main-loop configuration object; set before Init, read-only during a run. */
extern GimbalLevelConfig imu_level_config;
#endif
