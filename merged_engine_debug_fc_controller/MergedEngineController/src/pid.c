#include "pid.h"

static float clampf_pid(float x, float lo, float hi)
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
    p->initialized = 0u;
}

void pid_reset(PID *p)
{
    p->i_term = 0.0f;
    p->prev_error = 0.0f;
    p->initialized = 0u;
}

void pid_set_output_limits(PID *p, float out_min, float out_max)
{
    if (out_min > out_max) {
        const float tmp = out_min;
        out_min = out_max;
        out_max = tmp;
    }
    p->out_min = out_min;
    p->out_max = out_max;
    // Keep the stored integral term inside the currently available actuator
    // correction range. This prevents windup when PWM reaches 1000 or 2000 us.
    p->i_term = clampf_pid(p->i_term, p->out_min, p->out_max);
}

void pid_track_output(PID *p, float desired_output, float error)
{
    // With derivative deliberately initialized to zero, choose the integral
    // term that makes Kp*error + I equal the requested current output.
    // The configured actuator limits remain authoritative.
    desired_output = clampf_pid(desired_output, p->out_min, p->out_max);
    p->i_term = clampf_pid(desired_output - p->kp * error,
                           p->out_min,
                           p->out_max);
    p->prev_error = error;
    p->initialized = 1u;
}

float pid_update(PID *p, float target, float measured, float dt_s)
{
    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    const float error = target - measured;
    const float deriv = p->initialized ? ((error - p->prev_error) / dt_s) : 0.0f;

    p->i_term += p->ki * error * dt_s;
    p->i_term = clampf_pid(p->i_term, p->out_min, p->out_max);

    float out = p->kp * error + p->i_term + p->kd * deriv;
    out = clampf_pid(out, p->out_min, p->out_max);

    p->prev_error = error;
    p->initialized = 1u;
    return out;
}
