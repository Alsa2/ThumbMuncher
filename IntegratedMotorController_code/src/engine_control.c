#include "engine_control.h"
#include "board_config.h"

#include <stdbool.h>
#include <stdint.h>

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
} EngineControl;

static EngineControl g_ctrl;
static PID g_pid;

static float clampf_engine(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t round_to_u16(float x)
{
    if (x <= 0.0f) {
        return 0u;
    }
    return (uint16_t)(x + 0.5f);
}

static float normalize_pct(float pct)
{
    return clampf_engine(pct, 0.0f, 100.0f);
}

static float throttle_pct_to_target_rpm(float pct)
{
    pct = normalize_pct(pct);
    return THROTTLE_IDLE_RPM + (pct / 100.0f) * (THROTTLE_MAX_RPM - THROTTLE_IDLE_RPM);
}

static float throttle_pct_to_feedforward_us(float pct)
{
    pct = normalize_pct(pct);
    return (float)THROTTLE_IDLE_US +
           (pct / 100.0f) * ((float)THROTTLE_MAX_US - (float)THROTTLE_IDLE_US);
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
    g_ctrl.target_rpm = THROTTLE_IDLE_RPM;
    g_ctrl.feedforward_us = (float)THROTTLE_IDLE_US;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = THROTTLE_IDLE_US;
    actuators_set_throttle_us(g_ctrl.output_us);
}

static void set_throttle_prime(void)
{
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)THROTTLE_PRIME_US;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = THROTTLE_PRIME_US;
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

    // Do not command outside the requested 0..100% throttle curve endpoints.
    out_us = clampf_engine(out_us, (float)THROTTLE_MAX_US, (float)THROTTLE_IDLE_US);

    g_ctrl.output_us = round_to_u16(out_us);
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
        // Relay is on, but servos stay at safe/home positions while power rails settle.
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

void engine_control_init(void)
{
    g_ctrl.state = ENGINE_DISARMED;
    g_ctrl.armed = false;
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = (float)THROTTLE_IDLE_US;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = THROTTLE_IDLE_US;
    g_ctrl.state_enter_ms = 0u;
    g_ctrl.last_update_ms = 0u;
    g_ctrl.last_spin_ms = 0u;

    pid_init(&g_pid,
             RPM_PID_KP_US_PER_RPM,
             RPM_PID_KI_US_PER_RPM_S,
             RPM_PID_KD_US_PER_RPM_PER_S,
             -RPM_PID_CORRECTION_LIMIT_US,
             RPM_PID_CORRECTION_LIMIT_US);
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

    pid_reset(&g_pid);
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.feedforward_us = 0.0f;
    g_ctrl.pid_correction_us = 0.0f;
    g_ctrl.output_us = THROTTLE_IDLE_US;

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

    // Hard safety gate: disarm or FC timeout from the outer loop always cuts relay
    // and returns the throttle to idle.
    if (!g_ctrl.armed) {
        outputs_safe_disarmed();
        pid_reset(&g_pid);
        g_ctrl.target_rpm = 0.0f;
        g_ctrl.last_spin_ms = 0u;
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
        set_throttle_prime();

        if (s.rpm >= RPM_DETECT_THRESHOLD) {
            g_ctrl.last_spin_ms = now_ms;
            actuators_set_choke_closed(false);
            set_throttle_prime();
            enter_state(ENGINE_PRIMING_AFTER_SPIN, now_ms);
        }
        break;

    case ENGINE_PRIMING_AFTER_SPIN:
        outputs_armed_common();
        actuators_set_choke_closed(false);
        set_throttle_prime();

        if (g_ctrl.last_spin_ms != 0u && (now_ms - g_ctrl.last_spin_ms) > RPM_ZERO_TIMEOUT_MS) {
            set_throttle_idle();
            enter_state(ENGINE_ARMED_WAIT_FOR_SPIN, now_ms);
            break;
        }

        if ((now_ms - g_ctrl.state_enter_ms) >= PRIME_AFTER_SPIN_MS) {
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

        // Hold idle after priming until the CAN throttle command is actually zero.
        if (s.rpm >= RPM_DETECT_THRESHOLD) {
            (void)apply_feedforward_pid(0.0f, s.rpm, dt_s);
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
            (void)apply_feedforward_pid(g_ctrl.cmd_pct, s.rpm, dt_s);
        } else {
            // If RPM is not believable, do not open the throttle based on CAN command.
            set_throttle_idle();
        }
        break;

    case ENGINE_SERVO_TEST:
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
