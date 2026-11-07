#include "gravity_estimator.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.2957795131f
#define CAL_GYRO_MAX 0.15f
#define CAL_GYRO_VARIANCE 0.0001f
#define CORRECTION_RATE 2.0f /* 1/s; software initial setting, not hardware tuned */
#define INNOVATION_COS 0.939692621f /* reject >20 degrees */
#define MAX_UNCORRECTED_S 2.0f

static float dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void angles(GravityEstimator *s)
{
    s->roll_deg = atan2f(s->up_sensor[1], s->up_sensor[2]) * RAD_TO_DEG;
    s->pitch_deg = atan2f(-s->up_sensor[0],
        hypotf(s->up_sensor[1], s->up_sensor[2])) * RAD_TO_DEG;
}

void Gravity_Reset(GravityEstimator *s) { memset(s, 0, sizeof(*s)); }

uint8_t Gravity_Update(GravityEstimator *s, const float a[3], const float w[3], float dt)
{
    if (!s || !a || !w) { return 0U; }
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.05f) { Gravity_Reset(s); return 0U; }
    for (unsigned i = 0; i < 3; ++i)
    {
        /* Driver ranges: +/-4g, +/-2000 deg/s. Saturation is not a valid prediction. */
        if (!isfinite(a[i]) || !isfinite(w[i]) || fabsf(a[i]) >= 4.0f*GRAVITY_MS2 ||
            fabsf(w[i]) >= 34.9f) { Gravity_Reset(s); return 0U; }
    }
    float norm = sqrtf(dot(a, a));
    float norm_error = fabsf(norm / GRAVITY_MS2 - 1.0f);
    s->accel_roll_deg = norm > 0.001f ? atan2f(a[1], a[2])*RAD_TO_DEG : NAN;
    s->accel_pitch_deg = norm > 0.001f ? atan2f(-a[0], hypotf(a[1], a[2]))*RAD_TO_DEG : NAN;
    if (!s->ready)
    {
        if (norm_error >= 0.1f || dot(w,w) > CAL_GYRO_MAX*CAL_GYRO_MAX)
        { Gravity_Reset(s); return 1U; }
        uint32_t n = ++s->calibration_samples;
        for (unsigned i = 0; i < 3; ++i)
        {
            float d = w[i] - s->mean[i];
            s->mean[i] += d/(float)n;
            s->m2[i] += d*(w[i] - s->mean[i]);
            s->accel_mean[i] += (a[i]-s->accel_mean[i])/(float)n;
        }
        /* Also reject changing tilt during calibration, even at slow gyro rates. */
        float mean_norm = sqrtf(dot(s->accel_mean, s->accel_mean));
        if (dot(a, s->accel_mean)/(norm*mean_norm) < 0.999390827f) /* 2 deg */
        { Gravity_Reset(s); return 1U; }
        if (n < GRAVITY_CALIBRATION_SAMPLES) { return 1U; }
        for (unsigned i = 0; i < 3; ++i)
        {
            if (s->m2[i]/(float)(n-1U) > CAL_GYRO_VARIANCE)
            { Gravity_Reset(s); return 1U; }
        }
        for (unsigned i = 0; i < 3; ++i)
        {
            s->gyro_bias_rads[i] = s->mean[i];
            s->up_sensor[i] = s->accel_mean[i]/mean_norm;
        }
        s->ready = 1U; /* Initialize actual tilt, never declare startup level. */
        angles(s);
        return 1U;
    }

    float omega[3], old[3], predicted[3];
    for (unsigned i = 0; i < 3; ++i)
    { omega[i] = w[i]-s->gyro_bias_rads[i]; old[i] = s->up_sensor[i]; }
    float rate = sqrtf(dot(omega, omega));
    /* u_dot = -omega x u. Rodrigues rotation avoids Euler singularities. */
    float c = cosf(rate*dt);
    float k1 = rate > 1e-6f ? sinf(rate*dt)/rate : dt;
    float k2 = rate > 1e-6f ? (1.0f-c)/(rate*rate) : 0.5f*dt*dt;
    float projection = dot(omega, old);
    for (unsigned i = 0; i < 3; ++i)
    {
        unsigned j = (i+1U)%3U, k = (i+2U)%3U;
        predicted[i] = c*old[i] - k1*(omega[j]*old[k]-omega[k]*old[j]) +
                       k2*omega[i]*projection;
    }
    s->yaw_deg = remainderf(s->yaw_deg + projection*dt*RAD_TO_DEG, 360.0f);
    if (s->yaw_deg >= 180.0f) { s->yaw_deg -= 360.0f; }
    float measured[3] = {0};
    if (norm > 0.001f)
    { for (unsigned i = 0; i < 3; ++i) { measured[i] = a[i]/norm; } }
    float agreement = dot(measured, predicted);
    /* Norm and innovation gating reduce acceleration contamination; six axes
     * cannot distinguish all sustained translations from actual gravity. */
    s->accel_weight = (norm_error < 0.1f && agreement > INNOVATION_COS) ?
        (1.0f-norm_error/0.1f)*(agreement-INNOVATION_COS)/(1.0f-INNOVATION_COS) : 0.0f;
    s->uncorrected_s = s->accel_weight > 0.05f ? 0.0f : s->uncorrected_s + dt;
    if (s->uncorrected_s > MAX_UNCORRECTED_S) { Gravity_Reset(s); return 0U; }
    float gain = (1.0f-expf(-CORRECTION_RATE*dt))*s->accel_weight;
    for (unsigned i = 0; i < 3; ++i)
    { s->up_sensor[i] = predicted[i] + gain*(measured[i]-predicted[i]); }
    float length = sqrtf(dot(s->up_sensor,s->up_sensor));
    if (!isfinite(length) || length < 0.1f) { Gravity_Reset(s); return 0U; }
    for (unsigned i = 0; i < 3; ++i) { s->up_sensor[i] /= length; }
    angles(s);
    return 1U;
}
