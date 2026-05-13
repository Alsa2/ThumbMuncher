#include "pid.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void pid_init(PID *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->i_term = 0.0f;
    p->prev_error = 0.0f;
    p->initialized = 0;
}

void pid_reset(PID *p)
{
    p->i_term = 0.0f;
    p->prev_error = 0.0f;
    p->initialized = 0;
}

float pid_update(PID *p, float target, float measured, float dt_s)
{
    const float error = target - measured;
    const float deriv = p->initialized ? ((error - p->prev_error) / dt_s) : 0.0f;

    p->i_term += p->ki * error * dt_s;
    p->i_term = clampf(p->i_term, p->out_min, p->out_max);

    float out = p->kp * error + p->i_term + p->kd * deriv;
    out = clampf(out, p->out_min, p->out_max);

    p->prev_error = error;
    p->initialized = 1;
    return out;
}
