#ifndef BSP_ALGORITHM_PID_H
#define BSP_ALGORITHM_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;   /* 已乘 ki 的 I 项输出，与 output 同单位。 */
    float output;
    float max_output;
} PID_t;

/* 固定周期离散接口：I += ki*error，D = kd*(error-last_error)。
 * ki/kd 已包含采样周期；周期改变后须重新整定。初始化 last_error=0。
 * I 项和总输出均限幅，饱和方向停止积分；非法输入清历史并返回0。 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output);
float PID_Calculate(PID_t *pid, float error);

typedef struct
{
    float kp, ki, kd;
    float integral_limit; /* I 项输出限幅，与最终输出同单位。 */
    float output_limit;
    float integral, previous_error, output;
    uint8_t has_previous_error;
} PID_Position;

/* kp: 输出/误差；ki: 输出/(误差*s)；kd: 输出*s/误差。 */
void PID_Position_Init(PID_Position *pid, float kp, float ki, float kd,
                       float integral_limit, float output_limit);
void PID_Position_Reset(PID_Position *pid);
/* 位置式 PID：u = Kp*e + Ki*积分(e dt) + Kd*de/dt；dt 单位为秒。
 * 包含 I 项/输出限幅与条件积分抗饱和；首次计算不产生微分冲击。
 * 非有限输入、无效 dt/限幅返回 0，并清除历史状态。 */
float PID_Position_Calculate(PID_Position *pid, float error, float dt_s);

#ifdef __cplusplus
}
#endif
#endif
