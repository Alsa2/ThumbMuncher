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

void engine_control_init(void);
void engine_control_set_armed(bool armed);
void engine_control_set_cmd_pct(float pct);
void engine_control_update(uint32_t now_ms);

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
