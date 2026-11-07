#ifndef GRAVITY_ESTIMATOR_H
#define GRAVITY_ESTIMATOR_H
#include <stdint.h>

#define GRAVITY_MS2 9.80665f
#define GRAVITY_CALIBRATION_SAMPLES 200U

/* Right handed sensor frame, SI input. up_sensor is opposite physical gravity:
 * a stationary accelerometer measures +g upward. No installation/level offset
 * is estimated here. yaw_deg retains the relative vertical-rate integral,
 * NOT Roll, magnetic heading, or a general ZYX Euler yaw. */
typedef struct
{
    float up_sensor[3], gyro_bias_rads[3];
    float roll_deg, pitch_deg, yaw_deg, accel_roll_deg, accel_pitch_deg;
    float mean[3], m2[3], accel_mean[3];
    float accel_weight, uncorrected_s;
    uint32_t calibration_samples;
    uint8_t ready;
} GravityEstimator;

void Gravity_Reset(GravityEstimator *state);
/* dt is actual new-sample period, (0, .05] seconds. Returns 0 on invalid
 * input or >2 s without usable acceleration correction (requires recalibration).
 * During stationary calibration returns 1, but ready remains false. */
uint8_t Gravity_Update(GravityEstimator *state, const float accel[3],
                       const float gyro[3], float dt_s);
#endif
