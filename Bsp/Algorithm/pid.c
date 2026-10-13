#include "pid.h"
#include <math.h>
#include <stddef.h>

static float PID_Clamp(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    if (pid == NULL) { return; }
    *pid = (PID_t){.kp = kp, .ki = ki, .kd = kd, .max_output = max_output};
}

float PID_Calculate(PID_t *pid, float error)
{
    if (pid == NULL) { return 0.0f; }
    /* 将一次调用作为一个离散时间单位，复用同一限幅/抗饱和算法。 */
    PID_Position controller = {
        .kp = pid->kp, .ki = pid->ki, .kd = pid->kd,
        .integral_limit = pid->max_output, .output_limit = pid->max_output,
        .integral = pid->integral, .previous_error = pid->last_error,
        .has_previous_error = 1U
    };
    pid->output = PID_Position_Calculate(&controller, error, 1.0f);
    pid->error = controller.has_previous_error ? error : 0.0f;
    pid->last_error = controller.previous_error;
    pid->integral = controller.integral;
    return pid->output;
}

void PID_Position_Reset(PID_Position *pid)
{
    if (pid == NULL) { return; }
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->has_previous_error = 0U;
}

void PID_Position_Init(PID_Position *pid, float kp, float ki, float kd,
                       float integral_limit, float output_limit)
{
    if (pid == NULL) { return; }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    PID_Position_Reset(pid);
}

float PID_Position_Calculate(PID_Position *pid, float error, float dt_s)
{
    if (pid == NULL) { return 0.0f; }
    if (!isfinite(error) || !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(pid->kp) || !isfinite(pid->ki) || !isfinite(pid->kd) ||
        !isfinite(pid->integral_limit) || pid->integral_limit < 0.0f ||
        !isfinite(pid->output_limit) || pid->output_limit <= 0.0f)
    {
        PID_Position_Reset(pid);
        return 0.0f;
    }

    float p = pid->kp * error;
    float d = pid->has_previous_error ?
        pid->kd * ((error - pid->previous_error) / dt_s) : 0.0f;
    float increment = pid->ki * error * dt_s;
    float candidate_i = PID_Clamp(pid->integral + increment, pid->integral_limit);
    float candidate_output = p + candidate_i + d;
    if (!isfinite(p) || !isfinite(d) || !isfinite(increment) ||
        !isfinite(candidate_output))
    {
        PID_Position_Reset(pid);
        return 0.0f;
    }
    /* 饱和时禁止积分继续向饱和方向增长，反向积分仍可释放累积量。 */
    if (!((candidate_output > pid->output_limit && increment > 0.0f) ||
          (candidate_output < -pid->output_limit && increment < 0.0f)))
    {
        pid->integral = candidate_i;
    }
    pid->previous_error = error;
    pid->has_previous_error = 1U;
    pid->output = PID_Clamp(p + pid->integral + d, pid->output_limit);
    return pid->output;
}
