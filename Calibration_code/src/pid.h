#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float i_term;
    float prev_error;
    int initialized;
} PID;

void pid_init(PID *p, float kp, float ki, float kd, float out_min, float out_max);
void pid_reset(PID *p);
float pid_update(PID *p, float target, float measured, float dt_s);
