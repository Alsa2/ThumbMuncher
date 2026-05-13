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
    uint32_t state_enter_ms;
    uint32_t runtime_ms;
} EngineControl;

static EngineControl g_ctrl;
static PID g_pid;

static float cmd_pct_to_target_rpm(float pct)
{
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return RPM_IDLE + (pct / 100.0f) * (RPM_MAX - RPM_IDLE);
}

static void enter_state(EngineState s, uint32_t now_ms)
{
    g_ctrl.state = s;
    g_ctrl.state_enter_ms = now_ms;
}

void engine_control_init(void)
{
    g_ctrl.state = ENGINE_DISARMED;
    g_ctrl.armed = false;
    g_ctrl.cmd_pct = 0.0f;
    g_ctrl.target_rpm = 0.0f;
    g_ctrl.state_enter_ms = 0;
    g_ctrl.runtime_ms = 0;

    pid_init(&g_pid,
             0.08f,    // kp
             0.02f,    // ki
             0.00f,    // kd
             (float)SERVO_US_MIN,
             (float)SERVO_US_MAX);
}

void engine_control_set_armed(bool armed)
{
    g_ctrl.armed = armed;
}

void engine_control_set_cmd_pct(float pct)
{
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    g_ctrl.cmd_pct = pct;
}

EngineState engine_control_get_state(void)
{
    return g_ctrl.state;
}

float engine_control_get_target_rpm(void)
{
    return g_ctrl.target_rpm;
}

void engine_control_update(uint32_t now_ms)
{
    EngineSensors s;
    sensors_snapshot(&s);

    // Hard safety gate: disarm or FC timeout from outer loop must always cut relay
    if (!g_ctrl.armed) {
        actuators_set_relay(false);
        actuators_set_starter(false);
        actuators_set_choke_closed(false);
        actuators_set_throttle_us(SERVO_US_MIN);
        pid_reset(&g_pid);

        g_ctrl.target_rpm = 0.0f;
        enter_state(ENGINE_DISARMED, now_ms);
        return;
    }

    // Runtime accumulator
    if (s.rpm > RPM_START_STABLE) {
        g_ctrl.runtime_ms += CONTROL_DT_MS;
    }

    switch (g_ctrl.state) {
    case ENGINE_DISARMED:
        actuators_set_relay(true);
        actuators_set_starter(false);
        actuators_set_choke_closed(true);     // arm => close choke
        actuators_set_throttle_us(SERVO_US_MIN);
        pid_reset(&g_pid);

        g_ctrl.target_rpm = 0.0f;
        enter_state(ENGINE_ARMED_WAIT, now_ms);
        break;

    case ENGINE_ARMED_WAIT:
        actuators_set_relay(true);
        actuators_set_starter(false);
        actuators_set_choke_closed(true);     // all chokes closed while armed
        actuators_set_throttle_us(SERVO_US_MIN);
        g_ctrl.target_rpm = 0.0f;

        if (g_ctrl.cmd_pct > START_REQUEST_PCT && s.stationary) {
            sensors_reset_start_counter();
            enter_state(ENGINE_CRANKING, now_ms);
        }
        break;

    case ENGINE_CRANKING:
        actuators_set_relay(true);
        actuators_set_starter(true);
        actuators_set_choke_closed(true);
        actuators_set_throttle_us(SERVO_US_MIN);
        g_ctrl.target_rpm = 0.0f;

        if (s.turns_since_start >= CHOKE_TURNS_TO_OPEN) {
            actuators_set_starter(false);
            actuators_set_choke_closed(false);    // open choke individually after 15 turns
            pid_reset(&g_pid);
            enter_state(ENGINE_IDLE_HOLD, now_ms);
        }
        break;

    case ENGINE_IDLE_HOLD: {
        actuators_set_relay(true);
        actuators_set_starter(false);
        actuators_set_choke_closed(false);

        g_ctrl.target_rpm = RPM_IDLE;
        float servo_us = pid_update(&g_pid,
                                    g_ctrl.target_rpm,
                                    s.rpm,
                                    CONTROL_DT_MS / 1000.0f);
        actuators_set_throttle_us((uint16_t)servo_us);

        if ((now_ms - g_ctrl.state_enter_ms) > 1000u) {
            enter_state(ENGINE_RUNNING, now_ms);
        }
        break;
    }

    case ENGINE_RUNNING: {
        actuators_set_relay(true);
        actuators_set_starter(false);
        actuators_set_choke_closed(false);

        g_ctrl.target_rpm = cmd_pct_to_target_rpm(g_ctrl.cmd_pct);

        float servo_us = pid_update(&g_pid,
                                    g_ctrl.target_rpm,
                                    s.rpm,
                                    CONTROL_DT_MS / 1000.0f);
        actuators_set_throttle_us((uint16_t)servo_us);
        break;
    }

    case ENGINE_FAULT:
    default:
        actuators_set_relay(false);
        actuators_set_starter(false);
        actuators_set_choke_closed(false);
        actuators_set_throttle_us(SERVO_US_MIN);
        g_ctrl.target_rpm = 0.0f;
        break;
    }
}