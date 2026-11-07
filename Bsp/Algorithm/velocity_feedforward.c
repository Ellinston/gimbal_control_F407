#include "velocity_feedforward.h"
#include <math.h>

float calculate_velocity_ff(float target_velocity, float feedforward_gain)
{
    if (!isfinite(target_velocity) || !isfinite(feedforward_gain)) { return 0.0f; }
    float output = target_velocity * feedforward_gain;
    return isfinite(output) ? output : 0.0f;
}

float calculate_base_velocity_ff(float imu_velocity_rpm, float encoder_velocity_rpm)
{
    if (!isfinite(imu_velocity_rpm) || !isfinite(encoder_velocity_rpm)) { return NAN; }
    float base_velocity_rpm = imu_velocity_rpm - encoder_velocity_rpm;
    return isfinite(base_velocity_rpm) ? -base_velocity_rpm : NAN;
}
