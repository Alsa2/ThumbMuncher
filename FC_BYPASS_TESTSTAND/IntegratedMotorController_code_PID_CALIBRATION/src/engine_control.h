#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ENGINE_DISARMED = 0,
    ENGINE_ARMED_WAIT_FOR_SPIN,
    ENGINE_PRIMING_AFTER_SPIN,
    ENGINE_IDLE_WAIT_FOR_ZERO,
    ENGINE_RUNNING,
    ENGINE_SERVO_TEST,
    ENGINE_FAULT,
} EngineState;

typedef struct {
    float idle_rpm;
    float max_rpm;
    uint16_t idle_us;
    uint16_t max_us;
    float kp_us_per_rpm;
    float ki_us_per_rpm_s;
    float kd_us_per_rpm_per_s;
    float correction_limit_us;
} EngineControlRuntimeConfig;

void engine_control_init(void);
void engine_control_set_armed(bool armed);
void engine_control_set_cmd_pct(float pct);
void engine_control_update(uint32_t now_ms);

// Runtime tuning from custom CAN packets.
bool engine_control_set_pid(float kp, float ki, float kd, float correction_limit_us);
bool engine_control_set_feedforward_idle(float rpm, uint16_t throttle_us);
bool engine_control_set_feedforward_max(float rpm, uint16_t throttle_us);
void engine_control_get_runtime_config(EngineControlRuntimeConfig *out);

// Starts a relay-powered servo sweep when safe. Returns false if rejected.
bool engine_control_request_servo_test(uint32_t now_ms);
bool engine_control_servo_test_active(void);

EngineState engine_control_get_state(void);
float engine_control_get_cmd_pct(void);
float engine_control_get_target_rpm(void);
float engine_control_get_feedforward_us(void);
float engine_control_get_pid_correction_us(void);
uint16_t engine_control_get_output_us(void);

#endif // ENGINE_CONTROL_H
