#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define ENGINE_FF_MAX_POINTS 12u
#define ENGINE_FF_MAX_POLY_ORDER 3u

typedef enum {
    ENGINE_FF_MODEL_LINEAR = 0,
    ENGINE_FF_MODEL_POLYNOMIAL = 1,
    ENGINE_FF_MODEL_PIECEWISE_LINEAR = 2,
} EngineFeedforwardModelType;

typedef struct {
    uint8_t model_type;
    uint8_t point_count;
    uint8_t polynomial_order;
    uint8_t reserved;
    // Polynomial models use x = throttle_pct / 100.0 and
    // us = c0 + c1*x + c2*x^2 + c3*x^3.
    float coefficients[ENGINE_FF_MAX_POLY_ORDER + 1u];
    // Linear and piecewise models use sorted throttle points in centi-percent.
    uint16_t point_pct_x100[ENGINE_FF_MAX_POINTS];
    uint16_t point_us[ENGINE_FF_MAX_POINTS];
} EngineFeedforwardModelConfig;

typedef enum {
    ENGINE_DISARMED = 0,
    ENGINE_ARMED_WAIT_FOR_SPIN,
    ENGINE_PRIMING_AFTER_SPIN,
    ENGINE_IDLE_WAIT_FOR_ZERO,
    ENGINE_RUNNING,
    ENGINE_SERVO_TEST,
    ENGINE_FAULT,
    ENGINE_HALL_AUTO_CAL,
    ENGINE_MANUAL_PWM_TEST = 8,
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
    uint16_t start_us;
    uint16_t start_hold_ms;
    EngineFeedforwardModelConfig feedforward_model;
} EngineControlRuntimeConfig;

void engine_control_init(void);
void engine_control_set_armed(bool armed);
void engine_control_set_cmd_pct(float pct);
void engine_control_update(uint32_t now_ms);

// Runtime tuning from custom CAN packets.
bool engine_control_set_pid(float kp, float ki, float kd, float correction_limit_us);
bool engine_control_set_feedforward_idle(float rpm, uint16_t throttle_us);
bool engine_control_set_feedforward_max(float rpm, uint16_t throttle_us);
bool engine_control_set_feedforward_model(const EngineFeedforwardModelConfig *model);
bool engine_control_set_feedforward_fit(float idle_rpm, float max_rpm, const EngineFeedforwardModelConfig *model);
void engine_control_get_feedforward_model(EngineFeedforwardModelConfig *out);
void engine_control_make_default_feedforward_model(EngineFeedforwardModelConfig *out, uint16_t idle_us, uint16_t max_us);
bool engine_control_feedforward_model_is_valid(const EngineFeedforwardModelConfig *model);
void engine_control_get_runtime_config(EngineControlRuntimeConfig *out);
bool engine_control_runtime_config_is_valid(const EngineControlRuntimeConfig *cfg);
bool engine_control_apply_runtime_config(const EngineControlRuntimeConfig *cfg);
bool engine_control_consume_config_dirty(void);

// Debug endpoint auto-tune. endpoint_is_max=false tunes the 0%/idle endpoint,
// endpoint_is_max=true tunes the 100%/max endpoint. The engine must still be
// armed/running through the normal state machine; this only changes the
// feedforward servo opening slowly toward the requested RPM.
bool engine_control_start_endpoint_auto(bool endpoint_is_max,
                                        float target_rpm,
                                        float max_adjust_us_per_s,
                                        float rpm_deadband,
                                        uint16_t start_us);
bool engine_control_stop_endpoint_auto(void);
bool engine_control_endpoint_auto_active(void);
bool engine_control_endpoint_auto_is_max(void);
float engine_control_endpoint_auto_target_rpm(void);
float engine_control_endpoint_auto_error_rpm(void);
uint16_t engine_control_endpoint_auto_us(void);

// Starts a relay-powered servo sweep when safe. Returns false if rejected.
bool engine_control_request_servo_test(uint32_t now_ms);
bool engine_control_servo_test_active(void);

// Debug manual throttle-PWM test. Commands the throttle servo to exactly
// throttle_us for hold_ms while disarmed/stationary; relay on, starter off.
bool engine_control_request_manual_pwm_test(uint16_t throttle_us, uint32_t hold_ms, uint32_t now_ms);
bool engine_control_stop_manual_pwm_test(void);
bool engine_control_manual_pwm_test_active(void);

// Armed/running debug direct-throttle override. When enabled, the selected
// board bypasses the feedforward curve and PID trim and drives throttle_us
// directly while the normal armed state machine is running. Disarm disables it.
bool engine_control_set_manual_pwm_bypass(bool enable, uint16_t throttle_us);
bool engine_control_manual_pwm_bypass_active(void);

// Runtime-configurable startup priming. start_us is used while waiting for RPM
// and held for start_hold_ms after the first believable RPM.
bool engine_control_set_start_config(uint16_t start_us, uint16_t start_hold_ms);

// Debug Hall raw min/max auto-calibration. This powers the same relay used while
// armed so the Hall sensor/spark-plug power rail is live, but it keeps starter
// off and throttle at idle while an external spinner holds target RPM. duration_ms
// may be 0 to run until the sensor signal is clean/stable or the user aborts.
bool engine_control_start_hall_auto_cal(float target_rpm, uint32_t duration_ms, uint32_t now_ms);
bool engine_control_stop_hall_auto_cal(void);
bool engine_control_hall_auto_cal_active(void);

EngineState engine_control_get_state(void);
float engine_control_get_cmd_pct(void);
float engine_control_get_target_rpm(void);
float engine_control_get_feedforward_us(void);
float engine_control_get_pid_correction_us(void);
uint16_t engine_control_get_output_us(void);

#endif // ENGINE_CONTROL_H
