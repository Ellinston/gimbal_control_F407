#ifndef GIMBAL_LEVEL_H
#define GIMBAL_LEVEL_H
#include <stdint.h>

/* Ideal orthogonal, direct-drive serial joints: R_base_top = Rz(yaw)*Ry(pitch).
 * Right handed rotations; +Z base is the outer Yaw axis, +Y after Yaw is Pitch.
 * top denotes the terminal joint frame, not the sensor frame. */
typedef struct
{
    float sensor_to_top[3][3]; /* proper rotation, vector_top = M * vector_sensor */
    float platform_up_top[3]; /* measured platform normal, unit length */
    float zero_deg[2];       /* geometric zero expressed in caller's joint coordinates */
    float direction[2];      /* +/-1: encoder increasing -> geometric rotation */
    float minimum_deg[2], maximum_deg[2];
    float slew_dps[2];       /* target trajectory only; does not retune motor PID */
    float level_tolerance_deg, singular_cone_deg, branch_hysteresis_deg;
    uint8_t calibrated;
} GimbalLevelConfig;

typedef enum
{
    LEVEL_OK = 0, LEVEL_NEAR_VERTICAL, LEVEL_UNREACHABLE, LEVEL_CONFIG_INVALID,
    LEVEL_INPUT_INVALID, LEVEL_SINGULAR
} GimbalLevelResult;

typedef struct
{
    float command_deg[2], goal_deg[2], up_base[3], residual_deg;
    uint8_t initialized;
} GimbalLevelState;

uint8_t GimbalLevel_ConfigValid(const GimbalLevelConfig *config);
/* All vectors point UP, opposite gravitational acceleration. Returns only
 * exact/tolerance-qualified feasible goals; unreachable is a stop request.
 * command_deg slews toward goal_deg, so transient commands need not be level. */
GimbalLevelResult GimbalLevel_Update(const GimbalLevelConfig *config,
    GimbalLevelState *state, const float up_sensor[3], const float current_deg[2], float dt_s);
#endif
