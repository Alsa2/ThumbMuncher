#include "engine_control.h"
#include "board_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>

#include "actuators.h"
#include "sensors.h"
#include "pid.h"

typedef struct {
    EngineState state;
    bool armed;
    float cmd_pct;
    float target_rpm;
    float feedforward_us;
    float pid_correction_us;
    uint16_t output_us;
    uint32_t state_enter_ms;
    uint32_t last_update_ms;
    uint32_t last_spin_ms;
    uint16_t manual_pwm_us;
    uint32_t manual_pwm_hold_ms;
    bool manual_pwm_bypass_active;
    uint16_t manual_pwm_bypass_us;
} EngineControl;

static EngineControl g_ctrl;
static PID g_pid;
static EngineControlRuntimeConfig g_cfg;
static bool g_config_dirty = false;

typedef struct {
    bool active;
    bool endpoint_is_max;
    float target_rpm;
    float max_adjust_us_per_s;
    float rpm_deadband;
    float last_error_rpm;
    float current_us_f;
    uint16_t start_us;
} EndpointAutoTune;

static EndpointAutoTune g_auto;

static float clampf_engine(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t clamp_servo_us_runtime(uint16_t us)
{
    if (us < SERVO_US_HARD_MIN) return SERVO_US_HARD_MIN;
    if (us > SERVO_US_HARD_MAX) return SERVO_US_HARD_MAX;
    return us;
}

static uint16_t round_to_u16(float x)
{
    if (x <= 0.0f) {
        return 0u;
    }
    if (x >= 65535.0f) {
        return 65535u;
    }
    return (uint16_t)(x + 0.5f);
}

static float normalize_pct(float pct)
{
    return clampf_engine(pct, 0.0f, 100.0f);
}

static bool is_finite_local(float x)
{
    return isfinite(x);
}

static bool validate_runtime_config(const EngineControlRuntimeConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    if (!is_finite_local(cfg->idle_rpm) || !is_finite_local(cfg->max_rpm) ||
        !is_finite_local(cfg->kp_us_per_rpm) || !is_finite_local(cfg->ki_us_per_rpm_s) ||
        !is_finite_local(cfg->kd_us_per_rpm_per_s) || !is_finite_local(cfg->correction_limit_us)) {
        return false;
    }
    if (cfg->idle_rpm < 0.0f || cfg->max_rpm <= cfg->idle_rpm || cfg->max_rpm > 50000.0f) {
        return false;
    }
    if (cfg->idle_us < SERVO_US_HARD_MIN || cfg->idle_us > SERVO_US_HARD_MAX ||
        cfg->max_us < SERVO_US_HARD_MIN || cfg->max_us > SERVO_US_HARD_MAX ||
        cfg->start_us < SERVO_US_HARD_MIN || cfg->start_us > SERVO_US_HARD_MAX) {
        return false;
    }
    if (cfg->start_hold_ms > MANUAL_PWM_TEST_MAX_HOLD_MS) {
        return false;
    }
    if (cfg->kp_us_per_rpm < 0.0f || cfg->ki_us_per_rpm_s < 0.0f || cfg->kd_us_per_rpm_per_s < 0.0f ||
        cfg->correction_limit_us <= 0.0f || cfg->correction_limit_us > 2000.0f) {
        return false;
    }
    return true;
}


static float throttle_pct_to_target_rpm(float pct)
{
    pct = normalize_pct(pct);
    return g_cfg.idle_rpm + (pct / 100.0f) * (g_cfg.max_rpm - g_cfg.idle_rpm);
}

static float throttle_pct_to_feedforward_us(float pct)
{
    pct = normalize_pct(pct);
    return (float)g_cfg.idle_us +
           (pct / 100.0f) * ((float)g_cfg.max_us - (float)g_cfg.idle_us);
}

static uint16_t lerp_u16(uint16_t a, uint16_t b, uint32_t elapsed_ms, uint32_t duration_ms)
{
    if (duration_ms == 0u || elapsed_ms >= duration_ms) {
        return b;
    }

    const int32_t delta = (int32_t)b - (int32_t)a;
    const int32_t value = (int32_t)a + (delta * (int32_t)elapsed_ms) / (int32_t)duration_ms;

    if (value < 0) {
        return 0u;
    }
    if (value > 65535) {
        return 65535u;
    }
    return (uint16_t)value;
}

static void reset_pid_with_current_config(void)
{
    pid_init(&g_pid,
             g_cfg.kp_us_per_rpm,
             g_cfg.ki_us_per_rpm_s,
             g_cfg.kd_us_per_rpm_per_s,
             -g_cfg.correction_limit_us,
             g_cfg.correction_limit_us);
}

static void enter_state(EngineState s, uint32_t now_ms)
{
    if (g_ctrl.state != s) {
        pid_reset(&g_pid);
    }
    g_ctrl.state = s;
    g_ctrl.state_enter_ms = now_ms;
}

static void set_throttle_idle(void)
{
    g_ctrl.target_rpm = g_cfg.idle_rpm;
    g_ctrl.feedforward_us = (float)g_cfg.idle_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.idle_us;
    actuators_set_throttle_us(g_ctrl.output_us);
}

static void set_throttle_start(void)
{
    // Use the exact runtime/FRAM-configured startup PWM. Do not fall back to
    // a fixed neutral-ish value here; the whole point is to make startup opening
    // board-configurable and reproducible.
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_cfg.start_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.start_us;
    actuators_set_throttle_us(g_ctrl.output_us);
}

static uint16_t apply_feedforward_pid(float cmd_pct, float measured_rpm, float dt_s)
{
    g_ctrl.target_rpm = throttle_pct_to_target_rpm(cmd_pct);
    g_ctrl.feedforward_us = throttle_pct_to_feedforward_us(cmd_pct);

    // Positive RPM error means the engine is slow. This servo opens as PWM gets
    // smaller, so the PID correction is subtracted from the feedforward pulse.
    g_ctrl.pid_correction_us = pid_update(&g_pid,
                                          g_ctrl.target_rpm,
                                          measured_rpm,
                                          dt_s);

    float out_us = g_ctrl.feedforward_us - g_ctrl.pid_correction_us;

    // Stay inside the feedforward endpoint span, regardless of endpoint ordering.
    const float lo = ((float)g_cfg.idle_us < (float)g_cfg.max_us) ? (float)g_cfg.idle_us : (float)g_cfg.max_us;
    const float hi = ((float)g_cfg.idle_us > (float)g_cfg.max_us) ? (float)g_cfg.idle_us : (float)g_cfg.max_us;
    out_us = clampf_engine(out_us, lo, hi);

    g_ctrl.output_us = round_to_u16(out_us);
    actuators_set_throttle_us(g_ctrl.output_us);
    return g_ctrl.output_us;
}


static uint16_t apply_manual_pwm_bypass(void)
{
    const uint16_t us = clamp_servo_us_runtime(g_ctrl.manual_pwm_bypass_us);
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = us;
    actuators_set_throttle_us(g_ctrl.output_us);
    return g_ctrl.output_us;
}


static uint16_t *auto_endpoint_us_ptr(void)
{
    return g_auto.endpoint_is_max ? &g_cfg.max_us : &g_cfg.idle_us;
}

static float auto_endpoint_nominal_pct(void)
{
    return g_auto.endpoint_is_max ? 100.0f : 0.0f;
}

static uint16_t apply_endpoint_auto(float measured_rpm, float dt_s)
{
    uint16_t *endpoint_us = auto_endpoint_us_ptr();
    const float error = g_auto.target_rpm - measured_rpm;
    g_auto.last_error_rpm = error;

    // Do not infer servo polarity from idle_us/max_us. During auto-tune we may
    // intentionally start both endpoints from the requested start PWM, and the two stored
    // endpoints may temporarily cross. This board opens throttle as PWM gets
    // smaller, so positive RPM error must drive PWM down.
    float delta_us = 0.0f;
    if (fabsf(error) > g_auto.rpm_deadband) {
        delta_us = ENDPOINT_AUTO_TUNE_RPM_UP_US_SIGN *
                   error * ENDPOINT_AUTO_TUNE_GAIN_US_PER_RPM_S * dt_s;
        const float max_step = g_auto.max_adjust_us_per_s * dt_s;
        delta_us = clampf_engine(delta_us, -max_step, max_step);
    }

    // Keep a floating accumulator. The old code rounded each 20 ms step to a
    // uint16_t endpoint, so a safe rate like 8 us/s produced 0.16 us per loop
    // and was rounded away forever. The endpoint field is still saved/reported
    // as integer microseconds, but the search itself accumulates sub-us motion.
    g_auto.current_us_f = clampf_engine(g_auto.current_us_f + delta_us,
                                        (float)SERVO_US_HARD_MIN,
                                        (float)SERVO_US_HARD_MAX);

    const uint16_t endpoint_after = clamp_servo_us_runtime(round_to_u16(g_auto.current_us_f));
    if (endpoint_after != *endpoint_us) {
        *endpoint_us = endpoint_after;
        g_config_dirty = true;
    }

    // During endpoint auto-tune, do NOT use the normal throttle-to-RPM PID map.
    // Command the adjusted endpoint directly so the endpoint itself converges.
    g_ctrl.target_rpm = g_auto.target_rpm;
    g_ctrl.feedforward_us = g_auto.current_us_f;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.cmd_pct = auto_endpoint_nominal_pct();
    g_ctrl.output_us = endpoint_after;
    actuators_set_throttle_us(g_ctrl.output_us);
    return g_ctrl.output_us;
}

static void outputs_safe_disarmed(void)
{
    actuators_set_relay(false);
    actuators_set_starter(false);
    actuators_set_choke_closed(false);
    set_throttle_idle();
}

static void outputs_armed_common(void)
{
    actuators_set_relay(true);
    actuators_set_starter(false);
}

static void abort_servo_test_to_disarmed(uint32_t now_ms)
{
    outputs_safe_disarmed();
    pid_reset(&g_pid);
    g_ctrl.target_rpm = 0.0f;
    enter_state(ENGINE_DISARMED, now_ms);
}

static void run_servo_test(uint32_t now_ms, float rpm)
{
    // Safety interlocks during the test. Any arming message or unexpected spin
    // immediately drops relay power and returns to the normal state machine.
    if (g_ctrl.armed || rpm >= RPM_DETECT_THRESHOLD) {
        abort_servo_test_to_disarmed(now_ms);
        return;
    }

    const uint32_t elapsed = now_ms - g_ctrl.state_enter_ms;
    const uint32_t settle_end = SERVO_TEST_RELAY_SETTLE_MS;
    const uint32_t sweep_out_end = settle_end + SERVO_TEST_SWEEP_MS;
    const uint32_t hold_far_end = sweep_out_end + SERVO_TEST_HOLD_END_MS;
    const uint32_t sweep_back_end = hold_far_end + SERVO_TEST_SWEEP_MS;
    const uint32_t hold_home_end = sweep_back_end + SERVO_TEST_HOLD_END_MS;

    actuators_set_relay(true);
    actuators_set_starter(false);
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = 0.0f;
    g_ctrl.pid_correction_us = 0.0f;

    if (elapsed < settle_end) {
        actuators_set_throttle_us(SERVO_TEST_THROTTLE_A_US);
        actuators_set_choke_us(SERVO_TEST_CHOKE_A_US);
    } else if (elapsed < sweep_out_end) {
        const uint32_t t = elapsed - settle_end;
        const uint16_t throttle = lerp_u16(SERVO_TEST_THROTTLE_A_US,
                                           SERVO_TEST_THROTTLE_B_US,
                                           t,
                                           SERVO_TEST_SWEEP_MS);
        const uint16_t choke = lerp_u16(SERVO_TEST_CHOKE_A_US,
                                        SERVO_TEST_CHOKE_B_US,
                                        t,
                                        SERVO_TEST_SWEEP_MS);
        actuators_set_throttle_us(throttle);
        actuators_set_choke_us(choke);
    } else if (elapsed < hold_far_end) {
        actuators_set_throttle_us(SERVO_TEST_THROTTLE_B_US);
        actuators_set_choke_us(SERVO_TEST_CHOKE_B_US);
    } else if (elapsed < sweep_back_end) {
        const uint32_t t = elapsed - hold_far_end;
        const uint16_t throttle = lerp_u16(SERVO_TEST_THROTTLE_B_US,
                                           SERVO_TEST_THROTTLE_A_US,
                                           t,
                                           SERVO_TEST_SWEEP_MS);
        const uint16_t choke = lerp_u16(SERVO_TEST_CHOKE_B_US,
                                        SERVO_TEST_CHOKE_A_US,
                                        t,
                                        SERVO_TEST_SWEEP_MS);
        actuators_set_throttle_us(throttle);
        actuators_set_choke_us(choke);
    } else if (elapsed < hold_home_end) {
        actuators_set_throttle_us(SERVO_TEST_THROTTLE_A_US);
        actuators_set_choke_us(SERVO_TEST_CHOKE_A_US);
    } else {
        abort_servo_test_to_disarmed(now_ms);
        return;
    }

    g_ctrl.output_us = actuators_get_throttle_us();
}


static void abort_manual_pwm_test_to_disarmed(uint32_t now_ms)
{
    outputs_safe_disarmed();
    pid_reset(&g_pid);
    g_ctrl.target_rpm = 0.0f;
    enter_state(ENGINE_DISARMED, now_ms);
}

static void run_manual_pwm_test(uint32_t now_ms, float rpm)
{
    // This is a bounded, disarmed servo-position test. It powers the relay so
    // the throttle servo rail is live, but it never runs the starter. Any spin
    // or ARM command immediately aborts back to safe-disarmed.
    if (g_ctrl.armed || rpm >= RPM_DETECT_THRESHOLD) {
        abort_manual_pwm_test_to_disarmed(now_ms);
        return;
    }

    const uint32_t hold_ms = (g_ctrl.manual_pwm_hold_ms == 0u)
        ? MANUAL_PWM_TEST_DEFAULT_HOLD_MS
        : g_ctrl.manual_pwm_hold_ms;
    if (hold_ms != UINT32_MAX && (now_ms - g_ctrl.state_enter_ms) >= hold_ms) {
        abort_manual_pwm_test_to_disarmed(now_ms);
        return;
    }

    actuators_set_relay(true);
    actuators_set_starter(false);
    actuators_set_choke_closed(false);
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_ctrl.manual_pwm_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_ctrl.manual_pwm_us;
    actuators_set_throttle_us(g_ctrl.output_us);
}

static void abort_hall_auto_cal_to_disarmed(uint32_t now_ms)
{
    (void)sensors_stop_hall_auto_cal(true);
    outputs_safe_disarmed();
    pid_reset(&g_pid);
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_cfg.idle_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.idle_us;
    enter_state(ENGINE_DISARMED, now_ms);
}

static void run_hall_auto_cal(uint32_t now_ms)
{
    // This mode deliberately powers the relay because the Hall sensor is on the
    // same power rail as the spark plug. It does not run the starter and keeps
    // the throttle at the saved idle endpoint while the external spinner turns
    // the engine at the requested calibration RPM.
    // Hall auto-cal is a debug-owned powered state. Do not abort just because
    // the normal debug command heartbeat is disarmed; HALLCAL itself powers the
    // Hall/spark rail until HALLCAL_STOP/ABORT or successful convergence.
    g_ctrl.armed = true;

    (void)engine_control_stop_endpoint_auto();
    actuators_set_relay(true);
    actuators_set_starter(false);
    actuators_set_choke_closed(false);
    set_throttle_idle();
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_cfg.idle_us;
    g_ctrl.pid_correction_us = 0.0f;

    if (sensors_update_hall_auto_cal(now_ms)) {
        outputs_safe_disarmed();
        pid_reset(&g_pid);
        g_ctrl.target_rpm = 0.0f;
        enter_state(ENGINE_DISARMED, now_ms);
    }
}

void engine_control_init(void)
{
    g_cfg.idle_rpm = THROTTLE_IDLE_RPM;
    g_cfg.max_rpm = THROTTLE_MAX_RPM;
    g_cfg.idle_us = THROTTLE_IDLE_US;
    g_cfg.max_us = THROTTLE_MAX_US;
    g_cfg.kp_us_per_rpm = RPM_PID_KP_US_PER_RPM;
    g_cfg.ki_us_per_rpm_s = RPM_PID_KI_US_PER_RPM_S;
    g_cfg.kd_us_per_rpm_per_s = RPM_PID_KD_US_PER_RPM_PER_S;
    g_cfg.correction_limit_us = RPM_PID_CORRECTION_LIMIT_US;
    g_cfg.start_us = THROTTLE_START_US;
    g_cfg.start_hold_ms = START_HOLD_AFTER_RPM_MS;

    g_ctrl.state = ENGINE_DISARMED;
    g_ctrl.armed = false;
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_cfg.idle_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.idle_us;
    g_ctrl.state_enter_ms = 0u;
    g_ctrl.last_update_ms = 0u;
    g_ctrl.last_spin_ms = 0u;
    g_ctrl.manual_pwm_us = g_cfg.idle_us;
    g_ctrl.manual_pwm_hold_ms = MANUAL_PWM_TEST_DEFAULT_HOLD_MS;
    g_ctrl.manual_pwm_bypass_active = false;
    g_ctrl.manual_pwm_bypass_us = g_cfg.idle_us;

    g_auto.active = false;
    g_auto.endpoint_is_max = false;
    g_auto.target_rpm = 0.0f;
    g_auto.max_adjust_us_per_s = ENDPOINT_AUTO_TUNE_DEFAULT_RATE_US_PER_S;
    g_auto.rpm_deadband = ENDPOINT_AUTO_TUNE_DEADBAND_RPM;
    g_auto.last_error_rpm = 0.0f;
    g_auto.current_us_f = (float)ENDPOINT_AUTO_TUNE_START_US;
    g_auto.start_us = ENDPOINT_AUTO_TUNE_START_US;

    reset_pid_with_current_config();
    g_config_dirty = false;
}

bool engine_control_set_pid(float kp, float ki, float kd, float correction_limit_us)
{
    EngineControlRuntimeConfig cfg = g_cfg;
    cfg.kp_us_per_rpm = kp;
    cfg.ki_us_per_rpm_s = ki;
    cfg.kd_us_per_rpm_per_s = kd;
    cfg.correction_limit_us = correction_limit_us;
    if (!validate_runtime_config(&cfg)) {
        return false;
    }
    g_cfg = cfg;
    reset_pid_with_current_config();
    g_config_dirty = true;
    return true;
}

bool engine_control_set_feedforward_idle(float rpm, uint16_t throttle_us)
{
    if (!is_finite_local(rpm) || rpm < 0.0f || rpm > 50000.0f) {
        return false;
    }
    if (throttle_us < SERVO_US_HARD_MIN || throttle_us > SERVO_US_HARD_MAX) {
        return false;
    }
    g_cfg.idle_rpm = rpm;
    g_cfg.idle_us = clamp_servo_us_runtime(throttle_us);
    g_config_dirty = true;
    if (g_ctrl.state == ENGINE_DISARMED) {
        set_throttle_idle();
    }
    return true;
}

bool engine_control_set_feedforward_max(float rpm, uint16_t throttle_us)
{
    if (!is_finite_local(rpm) || rpm < 0.0f || rpm > 50000.0f) {
        return false;
    }
    if (throttle_us < SERVO_US_HARD_MIN || throttle_us > SERVO_US_HARD_MAX) {
        return false;
    }
    g_cfg.max_rpm = rpm;
    g_cfg.max_us = clamp_servo_us_runtime(throttle_us);
    g_config_dirty = true;
    return true;
}


bool engine_control_set_start_config(uint16_t start_us, uint16_t start_hold_ms)
{
    EngineControlRuntimeConfig cfg = g_cfg;
    cfg.start_us = clamp_servo_us_runtime(start_us);
    cfg.start_hold_ms = start_hold_ms;
    if (!validate_runtime_config(&cfg)) {
        return false;
    }
    g_cfg = cfg;
    g_config_dirty = true;
    if (g_ctrl.state == ENGINE_ARMED_WAIT_FOR_SPIN || g_ctrl.state == ENGINE_PRIMING_AFTER_SPIN) {
        set_throttle_start();
    }
    return true;
}

void engine_control_get_runtime_config(EngineControlRuntimeConfig *out)
{
    if (out != NULL) {
        *out = g_cfg;
    }
}


bool engine_control_runtime_config_is_valid(const EngineControlRuntimeConfig *cfg)
{
    return validate_runtime_config(cfg);
}

bool engine_control_apply_runtime_config(const EngineControlRuntimeConfig *cfg)
{
    if (!validate_runtime_config(cfg)) {
        return false;
    }
    g_cfg = *cfg;
    reset_pid_with_current_config();
    g_config_dirty = false;
    if (g_ctrl.state == ENGINE_DISARMED) {
        set_throttle_idle();
    }
    return true;
}

bool engine_control_consume_config_dirty(void)
{
    const bool dirty = g_config_dirty;
    g_config_dirty = false;
    return dirty;
}


bool engine_control_start_endpoint_auto(bool endpoint_is_max,
                                        float target_rpm,
                                        float max_adjust_us_per_s,
                                        float rpm_deadband,
                                        uint16_t start_us)
{
    if (!is_finite_local(target_rpm) || target_rpm < 0.0f || target_rpm > 50000.0f) {
        return false;
    }
    if (!is_finite_local(max_adjust_us_per_s) || max_adjust_us_per_s <= 0.0f || max_adjust_us_per_s > 200.0f) {
        return false;
    }
    if (!is_finite_local(rpm_deadband) || rpm_deadband < 0.0f || rpm_deadband > 2000.0f) {
        return false;
    }
    if (start_us < SERVO_US_HARD_MIN || start_us > SERVO_US_HARD_MAX) {
        return false;
    }

    // Target RPM is read from the saved/user GUI field and must remain fixed.
    // Auto-tune changes only the endpoint PWM. Start both 0% and 100% searches
    // from a known neutral-ish opening, normally 1700 us, so a bad old endpoint
    // does not poison the search.
    g_auto.active = true;
    g_auto.endpoint_is_max = endpoint_is_max;
    g_auto.target_rpm = target_rpm;
    g_auto.max_adjust_us_per_s = max_adjust_us_per_s;
    g_auto.rpm_deadband = rpm_deadband;
    g_auto.last_error_rpm = 0.0f;
    g_auto.start_us = clamp_servo_us_runtime(start_us);
    g_auto.current_us_f = (float)g_auto.start_us;

    uint16_t *endpoint_us = auto_endpoint_us_ptr();
    if (*endpoint_us != g_auto.start_us) {
        *endpoint_us = g_auto.start_us;
        g_config_dirty = true;
    }

    pid_reset(&g_pid);
    return true;
}

bool engine_control_stop_endpoint_auto(void)
{
    const bool was_active = g_auto.active;
    g_auto.active = false;
    g_auto.last_error_rpm = 0.0f;
    g_auto.current_us_f = (float)(g_auto.endpoint_is_max ? g_cfg.max_us : g_cfg.idle_us);
    pid_reset(&g_pid);
    return was_active;
}

bool engine_control_endpoint_auto_active(void)
{
    return g_auto.active;
}

bool engine_control_endpoint_auto_is_max(void)
{
    return g_auto.endpoint_is_max;
}

float engine_control_endpoint_auto_target_rpm(void)
{
    return g_auto.target_rpm;
}

float engine_control_endpoint_auto_error_rpm(void)
{
    return g_auto.last_error_rpm;
}

uint16_t engine_control_endpoint_auto_us(void)
{
    return g_auto.endpoint_is_max ? g_cfg.max_us : g_cfg.idle_us;
}

void engine_control_set_armed(bool armed)
{
    g_ctrl.armed = armed;
}

void engine_control_set_cmd_pct(float pct)
{
    g_ctrl.cmd_pct = normalize_pct(pct);
}

bool engine_control_request_servo_test(uint32_t now_ms)
{
    EngineSensors s;
    sensors_snapshot(&s);

    if (g_ctrl.armed || g_ctrl.state != ENGINE_DISARMED || s.rpm >= RPM_DETECT_THRESHOLD) {
        return false;
    }

    (void)engine_control_stop_endpoint_auto();
    g_ctrl.manual_pwm_bypass_active = false;
    pid_reset(&g_pid);
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = 0.0f;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.idle_us;

    actuators_set_relay(true);
    actuators_set_starter(false);
    actuators_set_throttle_us(SERVO_TEST_THROTTLE_A_US);
    actuators_set_choke_us(SERVO_TEST_CHOKE_A_US);
    enter_state(ENGINE_SERVO_TEST, now_ms);
    return true;
}

bool engine_control_servo_test_active(void)
{
    return g_ctrl.state == ENGINE_SERVO_TEST;
}


bool engine_control_request_manual_pwm_test(uint16_t throttle_us, uint32_t hold_ms, uint32_t now_ms)
{
    EngineSensors s;
    sensors_snapshot(&s);

    if (g_ctrl.armed || s.rpm >= RPM_DETECT_THRESHOLD) {
        return false;
    }
    if (hold_ms == 0u) {
        hold_ms = MANUAL_PWM_TEST_DEFAULT_HOLD_MS;
    }
    if (hold_ms > MANUAL_PWM_TEST_MAX_HOLD_MS) {
        hold_ms = MANUAL_PWM_TEST_MAX_HOLD_MS;
    }

    (void)engine_control_stop_endpoint_auto();
    g_ctrl.manual_pwm_bypass_active = false;
    pid_reset(&g_pid);
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)clamp_servo_us_runtime(throttle_us);
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.manual_pwm_us = clamp_servo_us_runtime(throttle_us);
    g_ctrl.manual_pwm_hold_ms = hold_ms;
    g_ctrl.output_us = g_ctrl.manual_pwm_us;

    actuators_set_relay(true);
    actuators_set_starter(false);
    actuators_set_choke_closed(false);
    actuators_set_throttle_us(g_ctrl.output_us);
    enter_state(ENGINE_MANUAL_PWM_TEST, now_ms);
    return true;
}

bool engine_control_stop_manual_pwm_test(void)
{
    if (g_ctrl.state != ENGINE_MANUAL_PWM_TEST) {
        return false;
    }

    outputs_safe_disarmed();
    pid_reset(&g_pid);
    g_ctrl.armed = false;
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_cfg.idle_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.idle_us;
    g_ctrl.manual_pwm_us = g_cfg.idle_us;
    g_ctrl.manual_pwm_hold_ms = MANUAL_PWM_TEST_DEFAULT_HOLD_MS;
    g_ctrl.manual_pwm_bypass_active = false;
    g_ctrl.manual_pwm_bypass_us = g_cfg.idle_us;
    enter_state(ENGINE_DISARMED, g_ctrl.last_update_ms);
    return true;
}

bool engine_control_manual_pwm_test_active(void)
{
    return g_ctrl.state == ENGINE_MANUAL_PWM_TEST;
}

bool engine_control_set_manual_pwm_bypass(bool enable, uint16_t throttle_us)
{
    if (!enable) {
        const bool was_active = g_ctrl.manual_pwm_bypass_active;
        g_ctrl.manual_pwm_bypass_active = false;
        g_ctrl.manual_pwm_bypass_us = g_cfg.idle_us;
        pid_reset(&g_pid);
        return was_active;
    }

    if (throttle_us < SERVO_US_HARD_MIN || throttle_us > SERVO_US_HARD_MAX) {
        return false;
    }

    g_ctrl.manual_pwm_bypass_active = true;
    g_ctrl.manual_pwm_bypass_us = clamp_servo_us_runtime(throttle_us);
    (void)engine_control_stop_endpoint_auto();
    pid_reset(&g_pid);
    return true;
}

bool engine_control_manual_pwm_bypass_active(void)
{
    return g_ctrl.manual_pwm_bypass_active;
}

bool engine_control_start_hall_auto_cal(float target_rpm, uint32_t duration_ms, uint32_t now_ms)
{
    if (g_ctrl.state == ENGINE_SERVO_TEST) {
        return false;
    }
    if (!sensors_start_hall_auto_cal(target_rpm, duration_ms, now_ms)) {
        return false;
    }

    (void)engine_control_stop_endpoint_auto();
    g_ctrl.manual_pwm_bypass_active = false;
    pid_reset(&g_pid);
    g_ctrl.armed = true;
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)g_cfg.idle_us;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = g_cfg.idle_us;
    actuators_set_relay(true);
    actuators_set_starter(false);
    actuators_set_choke_closed(false);
    actuators_set_throttle_us(g_ctrl.output_us);
    enter_state(ENGINE_HALL_AUTO_CAL, now_ms);
    return true;
}

bool engine_control_stop_hall_auto_cal(void)
{
    const bool was_active = (g_ctrl.state == ENGINE_HALL_AUTO_CAL) || sensors_hall_auto_cal_active();

    // Do not call sensors_stop_hall_auto_cal(aborted=true) when Hall auto-cal is
    // not actually running. The normal ARM path calls this as a defensive
    // cleanup on its rising edge; marking an inactive calibration as FAILED
    // leaves stale HALLCAL_FAILED telemetry on the bus, and the GUI can interpret
    // that old status as a reason to disarm the fresh ARM command.
    if (!was_active) {
        return false;
    }

    (void)sensors_stop_hall_auto_cal(true);
    outputs_safe_disarmed();
    pid_reset(&g_pid);
    g_ctrl.target_rpm = 0.0f;
    enter_state(ENGINE_DISARMED, g_ctrl.last_update_ms);
    return true;
}

bool engine_control_hall_auto_cal_active(void)
{
    return (g_ctrl.state == ENGINE_HALL_AUTO_CAL) || sensors_hall_auto_cal_active();
}

EngineState engine_control_get_state(void)
{
    return g_ctrl.state;
}

float engine_control_get_cmd_pct(void)
{
    return g_ctrl.cmd_pct;
}

float engine_control_get_target_rpm(void)
{
    return g_ctrl.target_rpm;
}

float engine_control_get_feedforward_us(void)
{
    return g_ctrl.feedforward_us;
}

float engine_control_get_pid_correction_us(void)
{
    return g_ctrl.pid_correction_us;
}

uint16_t engine_control_get_output_us(void)
{
    return g_ctrl.output_us;
}

void engine_control_update(uint32_t now_ms)
{
    // This update loop is intentionally source-agnostic. Debug CAN and
    // DroneCAN/FC commands both enter through engine_control_set_armed() and
    // engine_control_set_cmd_pct(), so every saved runtime setting applies to
    // both modes: throttle endpoints, PID gains, Hall thresholds/calibration,
    // and configured startup PWM/hold timing.
    EngineSensors s;
    sensors_snapshot(&s);

    float dt_s = (float)CONTROL_DT_MS / 1000.0f;
    if (g_ctrl.last_update_ms != 0u && now_ms >= g_ctrl.last_update_ms) {
        const uint32_t dt_ms = now_ms - g_ctrl.last_update_ms;
        if (dt_ms > 0u && dt_ms < 250u) {
            dt_s = (float)dt_ms / 1000.0f;
        }
    }
    g_ctrl.last_update_ms = now_ms;

    if (s.rpm >= RPM_DETECT_THRESHOLD) {
        g_ctrl.last_spin_ms = now_ms;
    }

    if (g_ctrl.state == ENGINE_SERVO_TEST) {
        run_servo_test(now_ms, s.rpm);
        return;
    }

    if (g_ctrl.state == ENGINE_MANUAL_PWM_TEST) {
        run_manual_pwm_test(now_ms, s.rpm);
        return;
    }

    if (g_ctrl.state == ENGINE_HALL_AUTO_CAL) {
        run_hall_auto_cal(now_ms);
        return;
    }

    // Hard safety gate: disarm or command-link timeout from the outer loop always
    // cuts relay power and returns the throttle to idle.
    if (!g_ctrl.armed) {
        (void)engine_control_stop_endpoint_auto();
        outputs_safe_disarmed();
        pid_reset(&g_pid);
        g_ctrl.target_rpm = 0.0f;
        g_ctrl.last_spin_ms = 0u;
        g_ctrl.manual_pwm_us = g_cfg.idle_us;
        g_ctrl.manual_pwm_hold_ms = MANUAL_PWM_TEST_DEFAULT_HOLD_MS;
        g_ctrl.manual_pwm_bypass_active = false;
        g_ctrl.manual_pwm_bypass_us = g_cfg.idle_us;
        enter_state(ENGINE_DISARMED, now_ms);
        return;
    }

    switch (g_ctrl.state) {
    case ENGINE_DISARMED:
        outputs_armed_common();
        actuators_set_choke_closed(true);
        set_throttle_idle();
        enter_state(ENGINE_ARMED_WAIT_FOR_SPIN, now_ms);
        break;

    case ENGINE_ARMED_WAIT_FOR_SPIN:
        outputs_armed_common();
        actuators_set_choke_closed(true);
        set_throttle_start();

        if (s.rpm >= RPM_DETECT_THRESHOLD) {
            g_ctrl.last_spin_ms = now_ms;
            actuators_set_choke_closed(false);
            set_throttle_start();
            enter_state(ENGINE_PRIMING_AFTER_SPIN, now_ms);
        }
        break;

    case ENGINE_PRIMING_AFTER_SPIN:
        outputs_armed_common();
        actuators_set_choke_closed(false);
        set_throttle_start();

        if (g_ctrl.last_spin_ms != 0u && (now_ms - g_ctrl.last_spin_ms) > RPM_ZERO_TIMEOUT_MS) {
            set_throttle_idle();
            enter_state(ENGINE_ARMED_WAIT_FOR_SPIN, now_ms);
            break;
        }

        if ((now_ms - g_ctrl.state_enter_ms) >= g_cfg.start_hold_ms) {
            set_throttle_idle();
            enter_state(ENGINE_IDLE_WAIT_FOR_ZERO, now_ms);
        }
        break;

    case ENGINE_IDLE_WAIT_FOR_ZERO:
        outputs_armed_common();
        actuators_set_choke_closed(false);

        if (g_ctrl.last_spin_ms != 0u && (now_ms - g_ctrl.last_spin_ms) > RPM_ZERO_TIMEOUT_MS) {
            set_throttle_idle();
            enter_state(ENGINE_ARMED_WAIT_FOR_SPIN, now_ms);
            break;
        }

        // Hold idle after priming until the incoming CAN throttle command is actually zero.
        if (s.rpm >= RPM_DETECT_THRESHOLD) {
            if (g_ctrl.manual_pwm_bypass_active) {
                (void)apply_manual_pwm_bypass();
            } else if (g_auto.active) {
                (void)apply_endpoint_auto(s.rpm, dt_s);
            } else {
                (void)apply_feedforward_pid(0.0f, s.rpm, dt_s);
            }
        } else {
            set_throttle_idle();
        }

        if (g_ctrl.cmd_pct <= THROTTLE_ZERO_DEADBAND_PCT) {
            pid_reset(&g_pid);
            enter_state(ENGINE_RUNNING, now_ms);
        }
        break;

    case ENGINE_RUNNING:
        outputs_armed_common();
        actuators_set_choke_closed(false);

        if (g_ctrl.last_spin_ms != 0u && (now_ms - g_ctrl.last_spin_ms) > RPM_ZERO_TIMEOUT_MS) {
            set_throttle_idle();
            enter_state(ENGINE_ARMED_WAIT_FOR_SPIN, now_ms);
            break;
        }

        if (s.rpm >= RPM_DETECT_THRESHOLD) {
            if (g_ctrl.manual_pwm_bypass_active) {
                (void)apply_manual_pwm_bypass();
            } else if (g_auto.active) {
                (void)apply_endpoint_auto(s.rpm, dt_s);
            } else {
                (void)apply_feedforward_pid(g_ctrl.cmd_pct, s.rpm, dt_s);
            }
        } else {
            // If RPM is not believable, do not open the throttle based on CAN command.
            set_throttle_idle();
        }
        break;

    case ENGINE_SERVO_TEST:
        // Handled before the disarm gate.
        break;

    case ENGINE_MANUAL_PWM_TEST:
        // Handled before the disarm gate.
        break;

    case ENGINE_HALL_AUTO_CAL:
        // Handled before the disarm gate.
        break;

    case ENGINE_FAULT:
    default:
        actuators_set_relay(false);
        actuators_set_starter(false);
        actuators_set_choke_closed(false);
        set_throttle_idle();
        pid_reset(&g_pid);
        g_ctrl.target_rpm = 0.0f;
        break;
    }
}
