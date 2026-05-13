#include "custom_can_node.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_config.h"
#include "custom_can_protocol.h"
#include "engine_control.h"
#include "mcp2518fd.h"

static bool g_armed = false;
static float g_cmd_pct = 0.0f;
static uint64_t g_last_command_us = 0u;
static uint32_t g_config_reject_count = 0u;

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

static bool frame_id_is(const CanFrame *frame, uint32_t id)
{
    return frame != NULL &&
           ((frame->id & CAN_FRAME_EFF) != 0u) &&
           ((frame->id & CAN_FRAME_ID_MASK) == id);
}

static void transmit_frame(uint32_t id, const uint8_t data[8], uint8_t len)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = CAN_FRAME_EFF | (id & CAN_FRAME_ID_MASK);
    frame.data_len = (len <= 8u) ? len : 8u;
    memcpy(frame.data, data, frame.data_len);
    (void)mcp2518fd_transmit(&frame);
}

void custom_can_node_init(void)
{
    EngineControlRuntimeConfig cfg;
    engine_control_get_runtime_config(&cfg);

    g_armed = false;
    g_cmd_pct = 0.0f;
    g_last_command_us = 0u;
    g_config_reject_count = 0u;
    g_pid_kp = cfg.kp_us_per_rpm;
    g_pid_ki = cfg.ki_us_per_rpm_s;
    g_pid_kd = cfg.kd_us_per_rpm_per_s;
    g_pid_limit = cfg.correction_limit_us;
}

void custom_can_node_handle_frame(const CanFrame *frame, uint64_t timestamp_usec)
{
    if (frame == NULL) {
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_COMMAND) && frame->data_len >= 4u) {
        g_armed = (frame->data[0] & CUSTOM_CAN_COMMAND_FLAG_ARMED) != 0u;
        const uint16_t centi_pct = custom_can_le16_load(&frame->data[2]);
        g_cmd_pct = clamp_pct((float)centi_pct / 100.0f);
        g_last_command_us = timestamp_usec;
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_PID_KP_KI) && frame->data_len >= 8u) {
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
        const float rpm = custom_can_float_load(&frame->data[0]);
        const uint16_t us = custom_can_le16_load(&frame->data[4]);
        if (!engine_control_set_feedforward_idle(rpm, us)) {
            g_config_reject_count++;
        }
        return;
    }

    if (frame_id_is(frame, CUSTOM_CAN_ID_FF_MAX) && frame->data_len >= 6u) {
        const float rpm = custom_can_float_load(&frame->data[0]);
        const uint16_t us = custom_can_le16_load(&frame->data[4]);
        if (!engine_control_set_feedforward_max(rpm, us)) {
            g_config_reject_count++;
        }
        return;
    }
}

bool custom_can_node_get_armed(void)
{
    return g_armed;
}

float custom_can_node_get_cmd_pct(void)
{
    return g_cmd_pct;
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

uint32_t custom_can_node_get_config_reject_count(void)
{
    return g_config_reject_count;
}

void custom_can_node_publish_telem_a(float rpm,
                                     uint16_t throttle_output_us,
                                     uint8_t engine_state,
                                     bool command_alive,
                                     bool armed,
                                     bool stationary)
{
    uint8_t data[8] = {0};
    custom_can_float_store(&data[0], rpm);
    custom_can_le16_store(&data[4], throttle_output_us);
    data[6] = engine_state;
    data[7] = (armed ? 0x01u : 0u) |
              (command_alive ? 0x02u : 0u) |
              (stationary ? 0x04u : 0u);
    transmit_frame(CUSTOM_CAN_ID_TELEM_A, data, 8u);
}

void custom_can_node_publish_telem_b(float target_rpm,
                                     uint16_t feedforward_us,
                                     float pid_correction_us)
{
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
