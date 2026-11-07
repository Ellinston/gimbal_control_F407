#ifndef BSP_ALGORITHM_VELOCITY_FEEDFORWARD_H
#define BSP_ALGORITHM_VELOCITY_FEEDFORWARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* target_velocity: rpm; gain: GM6020 voltage command per rpm.
 * Invalid inputs or an overflowing product produce zero feedforward. */
float calculate_velocity_ff(float target_velocity, float feedforward_gain);

/* Both rates use the same joint-positive coordinates and rpm.
 * Base rate = top IMU rate - encoder joint rate; compensate with its negative.
 * Invalid inputs return NaN so the caller can stop instead of using old data. */
float calculate_base_velocity_ff(float imu_velocity_rpm, float encoder_velocity_rpm);

#ifdef __cplusplus
}
#endif

#endif
