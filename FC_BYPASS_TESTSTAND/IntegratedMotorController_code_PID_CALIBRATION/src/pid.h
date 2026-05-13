#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float i_term;
    float prev_error;
    uint8_t initialized;
} PID;

void pid_init(PID *p, float kp, float ki, float kd, float out_min, float out_max);
void pid_reset(PID *p);
float pid_update(PID *p, float target, float measured, float dt_s);

#endif // PID_H
