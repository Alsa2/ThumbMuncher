#include "custom_can_node.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "custom_can_protocol.h"
#include "engine_control.h"
#include "mcp2518fd.h"
#include "sensors.h"
#include "persistent_config.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#ifndef CUSTOM_CAN_DEBUG_TIMEOUT_MS
#define CUSTOM_CAN_DEBUG_TIMEOUT_MS 1000u
#endif
#ifndef CUSTOM_CAN_COMMAND_TIMEOUT_MS
#define CUSTOM_CAN_COMMAND_TIMEOUT_MS 1000u
#endif
#ifndef CUSTOM_CAN_IDENTIFY_MS
#define CUSTOM_CAN_IDENTIFY_MS 3000u
#endif

static bool g_selected = false;
static bool g_armed = false;
static float g_cmd_pct = 0.0f;
static uint64_t g_last_debug_us = 0u;
static uint64_t g_last_command_us = 0u;
static uint32_t g_config_reject_count = 0u;
static uint16_t g_telem_a_ms = CUSTOM_CAN_TELEM_A_PERIOD_MS;
static uint16_t g_telem_b_ms = CUSTOM_CAN_TELEM_B_PERIOD_MS;
static uint16_t g_telem_c_ms = CUSTOM_CAN_TELEM_C_PERIOD_MS;
static uint32_t g_board_id = 1u;
static uint32_t g_identify_until_ms = 0u;
static bool g_identify_active = false;
static bool g_servo_test_requested = false;
static bool g_start_beep_requested = false;
static bool g_stop_auto_requested = false;
static bool g_config_dirty = false;

static float g_pid_kp = RPM_PID_KP_US_PER_RPM;
static float g_pid_ki = RPM_PID_KI_US_PER_RPM_S;
static float g_pid_kd = RPM_PID_KD_US_PER_RPM_PER_S;
static float g_pid_limit = RPM_PID_CORRECTION_LIMIT_US;

static float clamp_pct(float pct)
{
    if (pct < 0.0f) return 0.0f;
    if (pct > 100.0f) return 100.0f;
    return pct;
}

static bool frame_id_is(const CanardCANFrame *frame, uint32_t id)
{
    return frame != NULL &&
           ((frame->id & CANARD_CAN_FRAME_EFF) != 0u) &&
           ((frame->id & CANARD_CAN_EXT_ID_MASK) == id);
}

static void mark_debug_seen(uint64_t timestamp_usec)
{
    g_last_debug_us = timestamp_usec;
}

static bool this_board_or_broadcast(uint32_t board_id)
{
    return board_id == g_board_id || board_id == CUSTOM_CAN_BROADCAST_BOARD_ID;
}

static bool accepts_selected_command(void)
{
    return g_selected;
}

static void transmit_frame(uint32_t id, const uint8_t data[8], uint8_t len)
{
    CanardCANFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = CANARD_CAN_FRAME_EFF | (id & CANARD_CAN_EXT_ID_MASK);
    frame.data_len = (len <= 8u) ? len : 8u;
    memcpy(frame.data, data, frame.data_len);
    frame.iface_id = 0u;
    (void)mcp2518fd_transmit(&frame);
}

static uint32_t fnv1a32(const uint8_t *data, size_t len)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= (uint32_t)data[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t make_default_board_id(void)
{
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    uint32_t id = fnv1a32(uid.id, sizeof(uid.id));
    id ^= (uint32_t)time_us_64();
    id &= 0x7FFFFFFFu;  // keep it easy to type in the GUI.
    if (id == 0u || id == CUSTOM_CAN_BROADCAST_BOARD_ID) {
        id = 1u;
    }
    return id;
}

static void reset_to_safe_unselected(void)
{
    g_selected = false;
    g_armed = false;
    g_cmd_pct = 0.0f;
    g_last_command_us = 0u;
}

static void identify_until_stopped(void)
{
    g_identify_active = true;
    g_identify_until_ms = UINT32_MAX;
}

static void stop_identify(void)
{
    g_identify_active = false;
    g_identify_until_ms = 0u;
    g_start_beep_requested = false;
}


static void publish_config_snapshot(void)
{
    if (!g_selected) {
        return;
    }

    EngineControlRuntimeConfig cfg;
    engine_control_get_runtime_config(&cfg);
    uint16_t high_raw = 0u;
    uint16_t low_raw = 0u;
    sensors_get_hall_thresholds_raw(&high_raw, &low_raw);

    uint8_t data[8] = {0};
    custom_can_float_store(&data[0], cfg.kp_us_per_rpm);
    custom_can_float_store(&data[4], cfg.ki_us_per_rpm_s);
    transmit_frame(CUSTOM_CAN_ID_CONFIG_PID_A, data, 8u);

    memset(data, 0, sizeof(data));
    custom_can_float_store(&data[0], cfg.kd_us_per_rpm_per_s);
    custom_can_float_store(&data[4], cfg.correction_limit_us);
    transmit_frame(CUSTOM_CAN_ID_CONFIG_PID_B, data, 8u);

    memset(data, 0, sizeof(data));
    custom_can_float_store(&data[0], cfg.idle_rpm);
    custom_can_le16_store(&data[4], cfg.idle_us);
    transmit_frame(CUSTOM_CAN_ID_CONFIG_FF_IDLE, data, 8u);

    memset(data, 0, sizeof(data));
    custom_can_float_store(&data[0], cfg.max_rpm);
    custom_can_le16_store(&data[4], cfg.max_us);
    transmit_frame(CUSTOM_CAN_ID_CONFIG_FF_MAX, data, 8u);

    memset(data, 0, sizeof(data));
    custom_can_le16_store(&data[0], cfg.start_us);
    custom_can_le16_store(&data[2], cfg.start_hold_ms);
    custom_can_le16_store(&data[4], high_raw);
    custom_can_le16_store(&data[6], low_raw);
    transmit_frame(CUSTOM_CAN_ID_CONFIG_MISC, data, 8u);
}

void custom_can_node_init(void)
{
    EngineControlRuntimeConfig cfg;
    engine_control_get_runtime_config(&cfg);

    uint32_t saved_board_id = 0u;
    if (persistent_config_get_saved_board_id(&saved_board_id)) {
        g_board_id = saved_board_id;
    } else {
        g_board_id = make_default_board_id();
    }
    reset_to_safe_unselected();
    g_last_debug_us = 0u;
    g_config_reject_count = 0u;
    g_telem_a_ms = CUSTOM_CAN_TELEM_A_PERIOD_MS;
    g_telem_b_ms = CUSTOM_CAN_TELEM_B_PERIOD_MS;
    g_telem_c_ms = CUSTOM_CAN_TELEM_C_PERIOD_MS;
    g_identify_until_ms = 0u;
    g_identify_active = false;
    g_servo_test_requested = false;
    g_start_beep_requested = false;
    g_stop_auto_requested = false;
    g_config_dirty = false;
    g_pid_kp = cfg.kp_us_per_rpm;
    g_pid_ki = cfg.ki_us_per_rpm_s;
    g_pid_kd = cfg.kd_us_per_rpm_per_s;
    g_pid_limit = cfg.correction_limit_us;

    printf("debug board_id=%lu\r\n", (unsigned long)g_board_id);
}

void custom_can_node_handle_frame(const CanardCANFrame *frame, uint64_t timestamp_usec)
{
    if (frame == NULL) {
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_PROBE)) {
        mark_debug_seen(timestamp_usec);

        // The USB-CAN bridge includes its currently selected board ID in every
        // PROBE heartbeat. Treat that as a redundant selection heartbeat so a
        // single missed SELECT frame cannot make TEL A/B/C disappear after the
        // GUI switches engines. This also cleanly unselects the previous board
        // when the bridge moves to a different engine.
        if (frame->data_len >= 4u) {
            const uint32_t selected_id = custom_can_le32_load(&frame->data[0]);
            if (selected_id == 0u) {
                if (g_selected) {
                    g_selected = false;
                    g_armed = false;
                    g_cmd_pct = 0.0f;
                    g_last_command_us = 0u;
                }
            } else {
                const bool now_selected = this_board_or_broadcast(selected_id);
                if (g_selected && !now_selected) {
                    g_armed = false;
                    g_cmd_pct = 0.0f;
                    g_last_command_us = 0u;
                }
                g_selected = now_selected;
            }
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_SELECT) && frame->data_len >= 5u) {
        mark_debug_seen(timestamp_usec);
        const uint32_t id = custom_can_le32_load(&frame->data[0]);
        const uint8_t flags = frame->data[4];
        if ((flags & CUSTOM_CAN_SELECT_FLAG_CLEAR) != 0u) {
            reset_to_safe_unselected();
        } else if ((flags & CUSTOM_CAN_SELECT_FLAG_SELECT) != 0u) {
            g_selected = this_board_or_broadcast(id);
            if (!g_selected) {
                g_armed = false;
                g_cmd_pct = 0.0f;
                g_last_command_us = 0u;
            }
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_COMMAND) && frame->data_len >= 4u) {
        mark_debug_seen(timestamp_usec);
        if (!accepts_selected_command()) {
            return;
        }
        const bool new_armed = (frame->data[0] & CUSTOM_CAN_COMMAND_FLAG_ARMED) != 0u;

        // A normal ARM command is the clean owner of the engine after debug
        // special modes. Do this on the rising edge only, so endpoint auto-tune
        // is not cancelled by every armed heartbeat. This also makes the GUI ARM
        // button behave like the Hall-calibration path: the board-side state
        // machine exits debug calibration/powered-test mode and then arms.
        if (new_armed && !g_armed) {
            (void)engine_control_stop_hall_auto_cal();
            (void)engine_control_stop_endpoint_auto();
        }

        g_armed = new_armed;
        const uint16_t centi_pct = custom_can_le16_load(&frame->data[2]);
        g_cmd_pct = clamp_pct((float)centi_pct / 100.0f);
        g_last_command_us = timestamp_usec;
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_ACTION) && frame->data_len >= 2u) {
        mark_debug_seen(timestamp_usec);
        const uint8_t action = frame->data[0];
        const uint8_t flags = frame->data[1];
        const bool global = (flags & CUSTOM_CAN_ACTION_FLAG_GLOBAL) != 0u;
        if (!global && !accepts_selected_command()) {
            return;
        }
        switch (action) {
        case CUSTOM_CAN_ACTION_IDENTIFY:
            identify_until_stopped();
            break;
        case CUSTOM_CAN_ACTION_SERVO_TEST:
            g_servo_test_requested = true;
            break;
        case CUSTOM_CAN_ACTION_STOP_AUTO:
            g_stop_auto_requested = true;
            break;
        case CUSTOM_CAN_ACTION_START_BEEP:
            g_start_beep_requested = true;
            identify_until_stopped();
            break;
        case CUSTOM_CAN_ACTION_GLOBAL_DISARM:
            g_armed = false;
            g_cmd_pct = 0.0f;
            g_last_command_us = timestamp_usec;
            g_stop_auto_requested = true;
            stop_identify();
            break;
        case CUSTOM_CAN_ACTION_STOP_IDENTIFY:
            stop_identify();
            break;
        case CUSTOM_CAN_ACTION_GET_CONFIG:
            publish_config_snapshot();
            break;
        default:
            break;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_SET_BOARD_ID) && frame->data_len >= 8u) {
        mark_debug_seen(timestamp_usec);
        const uint32_t old_id = custom_can_le32_load(&frame->data[0]);
        const uint32_t new_id = custom_can_le32_load(&frame->data[4]);
        if ((old_id == CUSTOM_CAN_BROADCAST_BOARD_ID && accepts_selected_command()) || old_id == g_board_id) {
            if (new_id != 0u && new_id != CUSTOM_CAN_BROADCAST_BOARD_ID) {
                if (g_board_id != new_id) {
                    g_board_id = new_id;
                    // Save the ID immediately so a quick power cycle after pressing
                    // Set ID in the GUI does not fall back to a random ID. Keep the
                    // dirty flag set only if the immediate FRAM save fails.
                    const bool id_saved_now = persistent_config_save_board_id(g_board_id);
                    const bool settings_saved_now = persistent_config_save_from_runtime();
                    g_config_dirty = !settings_saved_now;
                    printf("debug SETID board_id=%lu FRAM_id=%s FRAM_settings=%s\r\n",
                           (unsigned long)g_board_id,
                           id_saved_now ? "OK" : "FAILED",
                           settings_saved_now ? "OK" : "FAILED");
                }
                g_selected = true;
            }
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_AUTO_ENDPOINT_EX) && frame->data_len >= 8u) {
        mark_debug_seen(timestamp_usec);
        const uint8_t flags = frame->data[1];
        const bool global = (flags & CUSTOM_CAN_AUTO_ENDPOINT_GLOBAL) != 0u;
        if (!global && !accepts_selected_command()) {
            return;
        }
        const bool enable = (flags & CUSTOM_CAN_AUTO_ENDPOINT_ENABLE) != 0u;
        if (!enable) {
            (void)engine_control_stop_endpoint_auto();
            return;
        }
        const uint8_t endpoint = frame->data[0];
        const uint16_t rate = custom_can_le16_load(&frame->data[2]);
        const uint16_t target_rpm_u16 = custom_can_le16_load(&frame->data[4]);
        const uint16_t start_us = custom_can_le16_load(&frame->data[6]);
        if (!engine_control_start_endpoint_auto(endpoint == CUSTOM_CAN_AUTO_ENDPOINT_MAX,
                                                (float)target_rpm_u16,
                                                (float)rate,
                                                ENDPOINT_AUTO_TUNE_DEADBAND_RPM,
                                                start_us)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_AUTO_ENDPOINT) && frame->data_len >= 8u) {
        mark_debug_seen(timestamp_usec);
        const uint8_t flags = frame->data[1];
        const bool global = (flags & CUSTOM_CAN_AUTO_ENDPOINT_GLOBAL) != 0u;
        if (!global && !accepts_selected_command()) {
            return;
        }
        const bool enable = (flags & CUSTOM_CAN_AUTO_ENDPOINT_ENABLE) != 0u;
        if (!enable) {
            (void)engine_control_stop_endpoint_auto();
            return;
        }
        const uint8_t endpoint = frame->data[0];
        const uint16_t rate = custom_can_le16_load(&frame->data[2]);
        const float target_rpm = custom_can_float_load(&frame->data[4]);
        if (!engine_control_start_endpoint_auto(endpoint == CUSTOM_CAN_AUTO_ENDPOINT_MAX,
                                                target_rpm,
                                                (float)rate,
                                                ENDPOINT_AUTO_TUNE_DEADBAND_RPM,
                                                ENDPOINT_AUTO_TUNE_START_US)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_HALL_AUTO_CAL) && frame->data_len >= 8u) {
        mark_debug_seen(timestamp_usec);
        const uint8_t flags = frame->data[0];
        const bool global = (flags & CUSTOM_CAN_HALL_CAL_GLOBAL) != 0u;
        if (!global && !accepts_selected_command()) {
            return;
        }
        const bool enable = (flags & CUSTOM_CAN_HALL_CAL_ENABLE) != 0u;
        if (!enable) {
            (void)engine_control_stop_hall_auto_cal();
            return;
        }
        const uint16_t duration_ms = custom_can_le16_load(&frame->data[2]);
        const float target_rpm = custom_can_float_load(&frame->data[4]);
        if (!engine_control_start_hall_auto_cal(target_rpm, (uint32_t)duration_ms,
                                                to_ms_since_boot(get_absolute_time()))) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_MANUAL_PWM_BYPASS) && frame->data_len >= 4u) {
        mark_debug_seen(timestamp_usec);
        if (!accepts_selected_command()) {
            return;
        }
        const bool enable = (frame->data[0] & CUSTOM_CAN_MANUAL_PWM_BYPASS_ENABLE) != 0u;
        const uint16_t throttle_us = custom_can_le16_load(&frame->data[2]);
        if (!engine_control_set_manual_pwm_bypass(enable, throttle_us)) {
            g_config_reject_count++;
        }
        return;
    }

    if (!accepts_selected_command()) {
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_PID_KP_KI) && frame->data_len >= 8u) {
        mark_debug_seen(timestamp_usec);
        const float kp = custom_can_float_load(&frame->data[0]);
        const float ki = custom_can_float_load(&frame->data[4]);
        g_pid_kp = kp;
        g_pid_ki = ki;
        if (!engine_control_set_pid(g_pid_kp, g_pid_ki, g_pid_kd, g_pid_limit)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_PID_KD_LIMIT) && frame->data_len >= 8u) {
        mark_debug_seen(timestamp_usec);
        const float kd = custom_can_float_load(&frame->data[0]);
        const float limit = custom_can_float_load(&frame->data[4]);
        g_pid_kd = kd;
        g_pid_limit = limit;
        if (!engine_control_set_pid(g_pid_kp, g_pid_ki, g_pid_kd, g_pid_limit)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_FF_IDLE) && frame->data_len >= 6u) {
        mark_debug_seen(timestamp_usec);
        const float rpm = custom_can_float_load(&frame->data[0]);
        const uint16_t us = custom_can_le16_load(&frame->data[4]);
        if (!engine_control_set_feedforward_idle(rpm, us)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_FF_MAX) && frame->data_len >= 6u) {
        mark_debug_seen(timestamp_usec);
        const float rpm = custom_can_float_load(&frame->data[0]);
        const uint16_t us = custom_can_le16_load(&frame->data[4]);
        if (!engine_control_set_feedforward_max(rpm, us)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_START_CONFIG) && frame->data_len >= 4u) {
        mark_debug_seen(timestamp_usec);
        const uint16_t start_us = custom_can_le16_load(&frame->data[0]);
        const uint16_t start_hold_ms = custom_can_le16_load(&frame->data[2]);
        if (!engine_control_set_start_config(start_us, start_hold_ms)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_MANUAL_PWM_TEST) && frame->data_len >= 4u) {
        mark_debug_seen(timestamp_usec);
        const uint16_t throttle_us = custom_can_le16_load(&frame->data[0]);
        const uint16_t hold_ms = custom_can_le16_load(&frame->data[2]);

        // throttle_us=0 and hold_ms=0 is an explicit immediate stop/safe-off
        // for the manual direct-PWM popup. This avoids waiting for the hold
        // timeout if the user presses Stop.
        if (throttle_us == 0u && hold_ms == 0u) {
            (void)engine_control_stop_manual_pwm_test();
            return;
        }

        if (!engine_control_request_manual_pwm_test(throttle_us, (uint32_t)hold_ms,
                                                    to_ms_since_boot(get_absolute_time()))) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_RPM_THRESH) && frame->data_len >= 4u) {
        mark_debug_seen(timestamp_usec);
        if (!accepts_selected_command()) {
            return;
        }
        const uint16_t high_raw = custom_can_le16_load(&frame->data[0]);
        const uint16_t low_raw = custom_can_le16_load(&frame->data[2]);
        if (!sensors_set_hall_thresholds_raw(high_raw, low_raw)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_TELEM_RATE) && frame->data_len >= 6u) {
        mark_debug_seen(timestamp_usec);
        if (!accepts_selected_command()) {
            return;
        }
        const uint16_t a = custom_can_le16_load(&frame->data[0]);
        const uint16_t b = custom_can_le16_load(&frame->data[2]);
        const uint16_t c = custom_can_le16_load(&frame->data[4]);
        if (a >= 5u && a <= 5000u) g_telem_a_ms = a;
        if (b >= 5u && b <= 5000u) g_telem_b_ms = b;
        if (c >= 5u && c <= 5000u) g_telem_c_ms = c;
        return;
    }
}

uint32_t custom_can_node_get_board_id(void)
{
    return g_board_id;
}

uint32_t custom_can_node_command_age_ms(uint64_t now_us)
{
    if (g_last_command_us == 0u) {
        return UINT32_MAX;
    }
    if (now_us <= g_last_command_us) {
        return 0u;
    }
    return (uint32_t)((now_us - g_last_command_us) / 1000u);
}

bool custom_can_node_command_alive(uint64_t now_us)
{
    return custom_can_node_command_age_ms(now_us) < CUSTOM_CAN_COMMAND_TIMEOUT_MS;
}

bool custom_can_node_debug_alive(uint64_t now_us)
{
    if (g_last_debug_us == 0u) {
        return false;
    }
    if (now_us <= g_last_debug_us) {
        return true;
    }
    return ((now_us - g_last_debug_us) / 1000u) < CUSTOM_CAN_DEBUG_TIMEOUT_MS;
}

bool custom_can_node_selected(void)
{
    return g_selected;
}

bool custom_can_node_get_armed(void)
{
    return g_armed;
}

float custom_can_node_get_cmd_pct(void)
{
    return g_cmd_pct;
}

uint32_t custom_can_node_get_config_reject_count(void)
{
    return g_config_reject_count;
}

void custom_can_node_get_telem_periods(uint16_t *a_ms, uint16_t *b_ms, uint16_t *c_ms)
{
    if (a_ms != NULL) *a_ms = g_telem_a_ms;
    if (b_ms != NULL) *b_ms = g_telem_b_ms;
    if (c_ms != NULL) *c_ms = g_telem_c_ms;
}



void custom_can_node_get_state(CustomCanDebugState *out, uint64_t now_us)
{
    if (out == NULL) {
        return;
    }
    out->board_id = g_board_id;
    out->selected = g_selected;
    out->debug_alive = custom_can_node_debug_alive(now_us);
    out->command_alive = custom_can_node_command_alive(now_us);
    out->armed = g_armed;
    out->cmd_pct = g_cmd_pct;
    out->config_reject_count = g_config_reject_count;
}

bool custom_can_node_consume_config_dirty(void)
{
    const bool dirty = g_config_dirty;
    g_config_dirty = false;
    return dirty;
}

bool custom_can_node_identify_active(uint32_t now_ms)
{
    if (!g_identify_active) {
        return false;
    }
    if (g_identify_until_ms == UINT32_MAX) {
        return true;
    }
    if ((int32_t)(g_identify_until_ms - now_ms) > 0) {
        return true;
    }
    stop_identify();
    return false;
}

bool custom_can_node_consume_servo_test_request(void)
{
    const bool v = g_servo_test_requested;
    g_servo_test_requested = false;
    return v;
}

bool custom_can_node_consume_start_beep_request(void)
{
    const bool v = g_start_beep_requested;
    g_start_beep_requested = false;
    return v;
}

bool custom_can_node_consume_stop_auto_request(void)
{
    const bool v = g_stop_auto_requested;
    g_stop_auto_requested = false;
    return v;
}

void custom_can_node_publish_board_announce(float rpm, uint8_t engine_state, bool armed, bool fc_alive)
{
    uint8_t data[8] = {0};
    custom_can_le32_store(&data[0], g_board_id);
    float rpm_x1 = rpm;
    if (rpm_x1 < 0.0f) rpm_x1 = 0.0f;
    if (rpm_x1 > 65535.0f) rpm_x1 = 65535.0f;
    custom_can_le16_store(&data[4], (uint16_t)(rpm_x1 + 0.5f));
    data[6] = engine_state;
    data[7] = (g_selected ? 0x01u : 0u) |
              (armed ? 0x02u : 0u) |
              (fc_alive ? 0x04u : 0u) |
              (custom_can_node_debug_alive(to_us_since_boot(get_absolute_time())) ? 0x08u : 0u);
    transmit_frame(CUSTOM_CAN_ID_BOARD_ANNOUNCE, data, 8u);
}

void custom_can_node_publish_telem_a(float rpm,
                                     uint16_t throttle_output_us,
                                     uint8_t engine_state,
                                     bool command_alive,
                                     bool armed,
                                     bool stationary)
{
    if (!g_selected) return;
    uint8_t data[8] = {0};
    custom_can_float_store(&data[0], rpm);
    custom_can_le16_store(&data[4], throttle_output_us);
    data[6] = engine_state;
    data[7] = (armed ? 0x01u : 0u) |
              (command_alive ? 0x02u : 0u) |
              (stationary ? 0x04u : 0u) |
              (g_selected ? 0x08u : 0u);
    transmit_frame(CUSTOM_CAN_ID_TELEM_A, data, 8u);
}

void custom_can_node_publish_telem_b(float target_rpm,
                                     uint16_t feedforward_us,
                                     float pid_correction_us)
{
    if (!g_selected) return;
    uint8_t data[8] = {0};
    custom_can_float_store(&data[0], target_rpm);
    custom_can_le16_store(&data[4], feedforward_us);
    float scaled = pid_correction_us * 10.0f;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    custom_can_le16_store_i(&data[6], (int16_t)scaled);
    transmit_frame(CUSTOM_CAN_ID_TELEM_B, data, 8u);
}

void custom_can_node_publish_telem_c(float temperature_c,
                                     float current_a,
                                     float bus_voltage_v,
                                     uint32_t runtime_ms)
{
    if (!g_selected) return;
    uint8_t data[8] = {0};

    float temp_scaled = temperature_c * 10.0f;
    if (temp_scaled > 32767.0f) temp_scaled = 32767.0f;
    if (temp_scaled < -32768.0f) temp_scaled = -32768.0f;

    float current_scaled = current_a * 1000.0f;
    if (current_scaled > 32767.0f) current_scaled = 32767.0f;
    if (current_scaled < -32768.0f) current_scaled = -32768.0f;

    float bus_scaled = bus_voltage_v * 1000.0f;
    if (bus_scaled < 0.0f) bus_scaled = 0.0f;
    if (bus_scaled > 65535.0f) bus_scaled = 65535.0f;

    uint32_t runtime_s = runtime_ms / 1000u;
    if (runtime_s > 65535u) runtime_s = 65535u;

    custom_can_le16_store_i(&data[0], (int16_t)temp_scaled);
    custom_can_le16_store_i(&data[2], (int16_t)current_scaled);
    custom_can_le16_store(&data[4], (uint16_t)bus_scaled);
    custom_can_le16_store(&data[6], (uint16_t)runtime_s);
    transmit_frame(CUSTOM_CAN_ID_TELEM_C, data, 8u);
}

void custom_can_node_publish_auto_status(uint8_t endpoint,
                                         bool active,
                                         float target_rpm,
                                         uint16_t endpoint_us,
                                         float rpm_error)
{
    if (!g_selected) return;
    uint8_t data[8] = {0};
    data[0] = endpoint;
    data[1] = active ? 1u : 0u;
    custom_can_le16_store(&data[2], endpoint_us);
    float err_scaled = rpm_error * 10.0f;
    if (err_scaled > 32767.0f) err_scaled = 32767.0f;
    if (err_scaled < -32768.0f) err_scaled = -32768.0f;
    custom_can_le16_store_i(&data[4], (int16_t)err_scaled);
    float target_scaled = target_rpm;
    if (target_scaled < 0.0f) target_scaled = 0.0f;
    if (target_scaled > 65535.0f) target_scaled = 65535.0f;
    custom_can_le16_store(&data[6], (uint16_t)(target_scaled + 0.5f));
    transmit_frame(CUSTOM_CAN_ID_AUTO_STATUS, data, 8u);
}
void custom_can_node_publish_hall_cal_status(const HallAutoCalStatus *status)
{
    if (!g_selected || status == NULL) return;
    uint8_t data[8] = {0};
    data[0] = status->status_code;
    data[1] = status->progress_pct;
    custom_can_le16_store(&data[2], status->min_raw);
    custom_can_le16_store(&data[4], status->max_raw);
    float target_scaled = status->target_rpm;
    if (target_scaled < 0.0f) target_scaled = 0.0f;
    if (target_scaled > 65535.0f) target_scaled = 65535.0f;
    custom_can_le16_store(&data[6], (uint16_t)(target_scaled + 0.5f));
    transmit_frame(CUSTOM_CAN_ID_HALL_CAL_STATUS, data, 8u);
}
