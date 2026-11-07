#include "gimbal_level.h"
#include <math.h>
#include <stddef.h>

#define D2R 0.0174532925199f
#define R2D 57.2957795131f
static float clamp(float x, float lo, float hi) { return fminf(hi,fmaxf(lo,x)); }
static float dot(const float a[3], const float b[3])
{ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }

uint8_t GimbalLevel_ConfigValid(const GimbalLevelConfig *c)
{
    if (!c || !c->calibrated) { return 0U; }
    for (unsigned i=0; i<3; ++i)
    {
        if (!isfinite(c->platform_up_top[i])) { return 0U; }
        for (unsigned j=0; j<3; ++j)
        { if (!isfinite(c->sensor_to_top[i][j])) { return 0U; } }
        for (unsigned j=0; j<=i; ++j)
        {
            if (fabsf(dot(c->sensor_to_top[i],c->sensor_to_top[j])-(i==j?1.0f:0.0f)) > 0.001f)
            { return 0U; }
        }
    }
    const float (*m)[3]=c->sensor_to_top;
    float determinant=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-
        m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    if (fabsf(determinant-1.0f)>0.002f || fabsf(dot(c->platform_up_top,c->platform_up_top)-1.0f)>0.001f)
    { return 0U; }
    for (unsigned i=0; i<2; ++i)
    {
        if (!isfinite(c->zero_deg[i]) || fabsf(c->zero_deg[i])>10000.0f ||
            !isfinite(c->direction[i]) || fabsf(c->direction[i])!=1.0f ||
            !isfinite(c->minimum_deg[i]) || !isfinite(c->maximum_deg[i]) ||
            c->minimum_deg[i]>=c->maximum_deg[i] ||
            fabsf(c->minimum_deg[i])>10000.0f || fabsf(c->maximum_deg[i])>10000.0f ||
            c->maximum_deg[i]-c->minimum_deg[i]>720.0f ||
            !isfinite(c->slew_dps[i]) || c->slew_dps[i]<=0.0f || c->slew_dps[i]>360.0f)
        { return 0U; }
    }
    return isfinite(c->level_tolerance_deg) && c->level_tolerance_deg>0.0f && c->level_tolerance_deg<=1.0f &&
        isfinite(c->singular_cone_deg) && c->singular_cone_deg>=0.0f &&
        c->singular_cone_deg<=c->level_tolerance_deg &&
        isfinite(c->branch_hysteresis_deg) && c->branch_hysteresis_deg>=0.0f && c->branch_hysteresis_deg<=20.0f;
}

static void rotate(float yaw, float pitch, const float in[3], float out[3])
{
    float cy=cosf(yaw*D2R), sy=sinf(yaw*D2R), cp=cosf(pitch*D2R), sp=sinf(pitch*D2R);
    float x=cp*in[0]+sp*in[2];
    out[0]=cy*x-sy*in[1]; out[1]=sy*x+cy*in[1]; out[2]=-sp*in[0]+cp*in[2];
}

/* Enumerate all 360-degree equivalents in the bounded encoder interval.
 * Never wrap the command across a mechanical stop. */
static void consider(const GimbalLevelConfig *c, const GimbalLevelState *s,
    const float current[2], const float up[3], float y, float p,
    float best[2], float *score, float *residual)
{
    float a=c->zero_deg[0]+y/c->direction[0], b=c->zero_deg[1]+p/c->direction[1];
    /* Quantization/noise can put an otherwise valid endpoint just outside the
     * interval. Admit a small neighborhood, then test the CLAMPED physical
     * normal. Without this test an endpoint can flip to the 180-degree branch. */
    float margin=c->level_tolerance_deg;
    int y0=(int)ceilf((c->minimum_deg[0]-a-margin)/360.0f);
    int y1=(int)floorf((c->maximum_deg[0]-a+margin)/360.0f);
    int p0=(int)ceilf((c->minimum_deg[1]-b-margin)/360.0f);
    int p1=(int)floorf((c->maximum_deg[1]-b+margin)/360.0f);
    for (int i=y0; i<=y1; ++i) for (int j=p0; j<=p1; ++j)
    {
        float q[2]={clamp(a+360.0f*(float)i,c->minimum_deg[0],c->maximum_deg[0]),
                    clamp(b+360.0f*(float)j,c->minimum_deg[1],c->maximum_deg[1])};
        float normal[3];
        rotate(c->direction[0]*(q[0]-c->zero_deg[0]),
               c->direction[1]*(q[1]-c->zero_deg[1]),c->platform_up_top,normal);
        float err=acosf(clamp(dot(normal,up),-1.0f,1.0f))*R2D;
        if (err>c->level_tolerance_deg) { continue; }
        float distance=hypotf(q[0]-current[0],q[1]-current[1]);
        if (s->initialized)
        {
            /* Penalize branch switches, not small tracking movement. */
            if (hypotf(q[0]-s->goal_deg[0],q[1]-s->goal_deg[1])>30.0f)
            { distance+=c->branch_hysteresis_deg; }
        }
        if (distance<*score) { *score=distance; best[0]=q[0]; best[1]=q[1]; *residual=err; }
    }
}

GimbalLevelResult GimbalLevel_Update(const GimbalLevelConfig *c, GimbalLevelState *s,
    const float input[3], const float current[2], float dt)
{
    if (!GimbalLevel_ConfigValid(c)) { return LEVEL_CONFIG_INVALID; }
    if (!s || !input || !current || !isfinite(dt) || dt<=0 || dt>0.1f) { return LEVEL_INPUT_INVALID; }
    for (unsigned i=0; i<3; ++i) { if (!isfinite(input[i])) { return LEVEL_INPUT_INVALID; } }
    float length=sqrtf(dot(input,input));
    if (length<0.9f || length>1.1f) { return LEVEL_INPUT_INVALID; }
    for (unsigned i=0; i<2; ++i)
    {
        /* A single encoder count can cross a configured endpoint between the
         * first feedback and the first solve. Admit only the same small
         * tolerance used for endpoint candidates; commands stay clamped to
         * travel. Task retains its original measured-angle fault guard. */
        if (!isfinite(current[i]) || fabsf(current[i])>10000.0f ||
            (!s->initialized && (current[i]<c->minimum_deg[i]-c->level_tolerance_deg ||
                                 current[i]>c->maximum_deg[i]+c->level_tolerance_deg)))
        { return LEVEL_INPUT_INVALID; }
    }
    float top[3], base[3];
    for (unsigned i=0; i<3; ++i) { top[i]=dot(c->sensor_to_top[i],input)/length; }
    rotate(c->direction[0]*(current[0]-c->zero_deg[0]),
           c->direction[1]*(current[1]-c->zero_deg[1]),top,base);
    for (unsigned i=0; i<3; ++i) { s->up_base[i]=base[i]; }
    const float *n=c->platform_up_top;
    float radius=hypotf(n[0],n[2]);
    if (radius<1e-4f) { return LEVEL_SINGULAR; } /* Pitch cannot change platform normal. */
    if (fabsf(base[2])>radius+1e-6f) { return LEVEL_UNREACHABLE; }
    float alpha=acosf(clamp(base[2]/radius,-1.0f,1.0f))*R2D;
    float offset=atan2f(n[0],n[2])*R2D;
    float score=INFINITY, best[2]={0}, residual=NAN;
    uint8_t near=(hypotf(base[0],base[1])<=sinf(c->singular_cone_deg*D2R));
    /* A pose already level within tolerance is the closest feasible answer. */
    consider(c,s,current,base,c->direction[0]*(current[0]-c->zero_deg[0]),
             c->direction[1]*(current[1]-c->zero_deg[1]),best,&score,&residual);
    if (near)
    {
        /* Azimuth is poorly observed near vertical. First try the pose that
         * points the platform normal straight up while retaining Yaw. Its
         * residual is the small measured base tilt. Solving the exact noisy
         * azimuth here can otherwise demand a large Yaw change, or latch an
         * unreachable fault when the held Yaw misses tolerance by one count. */
        float y=c->direction[0]*((s->initialized?s->command_deg[0]:current[0])-c->zero_deg[0]);
        consider(c,s,current,base,y,-offset,best,&score,&residual);
    }
    if (!isfinite(score)) for (unsigned branch=0; branch<2; ++branch)
    {
        float p=(branch ? -alpha : alpha)-offset;
        float v[3]; rotate(0,p,n,v);
        float y;
        if (near)
        { y=c->direction[0]*((s->initialized?s->command_deg[0]:current[0])-c->zero_deg[0]); }
        else { y=(atan2f(base[1],base[0])-atan2f(v[1],v[0]))*R2D; }
        consider(c,s,current,base,y,p,best,&score,&residual);
    }
    /* Exact azimuth can lie outside a joint stop even when the normal at the
     * stop is within the permitted angular error. Project onto each travel
     * edge before choosing a distant exact branch (especially near level). */
    for (unsigned edge=0; edge<2; ++edge)
    {
        float yaw_q=edge?c->maximum_deg[0]:c->minimum_deg[0];
        float yaw=c->direction[0]*(yaw_q-c->zero_deg[0]);
        float cy=cosf(yaw*D2R), sy=sinf(yaw*D2R);
        float x=cy*base[0]+sy*base[1]; /* Rz(-yaw) * base */
        float pitch=atan2f(x,base[2])*R2D-offset;
        consider(c,s,current,base,yaw,pitch,best,&score,&residual);
    }
    for (unsigned edge=0; edge<2; ++edge)
    {
        float pitch_q=edge?c->maximum_deg[1]:c->minimum_deg[1];
        float pitch=c->direction[1]*(pitch_q-c->zero_deg[1]);
        float v[3]; rotate(0,pitch,n,v);
        float yaw=(hypotf(v[0],v[1])>1e-6f && hypotf(base[0],base[1])>1e-6f)
            ? (atan2f(base[1],base[0])-atan2f(v[1],v[0]))*R2D
            : c->direction[0]*(current[0]-c->zero_deg[0]);
        consider(c,s,current,base,yaw,pitch,best,&score,&residual);
        for (unsigned yaw_edge=0; yaw_edge<2; ++yaw_edge)
        {
            float yaw_q=yaw_edge?c->maximum_deg[0]:c->minimum_deg[0];
            consider(c,s,current,base,c->direction[0]*(yaw_q-c->zero_deg[0]),
                     pitch,best,&score,&residual);
        }
    }
    if (!isfinite(score)) { return LEVEL_UNREACHABLE; }
    for (unsigned i=0; i<2; ++i)
    {
        float previous=s->initialized?s->command_deg[i]:current[i];
        s->goal_deg[i]=best[i];
        s->command_deg[i]=previous+clamp(best[i]-previous,-c->slew_dps[i]*dt,c->slew_dps[i]*dt);
    }
    s->residual_deg=residual; s->initialized=1U;
    return near?LEVEL_NEAR_VERTICAL:LEVEL_OK;
}
