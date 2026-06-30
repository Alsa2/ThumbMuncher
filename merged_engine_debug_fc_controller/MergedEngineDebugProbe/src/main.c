#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "board_config.h"
#include "can_frame.h"
#include "custom_can_protocol.h"
#include "mcp2518fd.h"

#define SERIAL_LINE_MAX 192
#define DEBUG_PROBE_PERIOD_MS 250u
#define DEBUG_COMMAND_HEARTBEAT_MS 100u

static bool g_armed = false;
static float g_throttle_pct = 0.0f;
static uint8_t g_command_seq = 0u;
static uint32_t g_selected_id = 0u;
static char g_line[SERIAL_LINE_MAX];
static size_t g_line_len = 0u;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t clamp_throttle_centi(float pct)
{
    pct = clampf_local(pct, 0.0f, 100.0f);
    float v = pct * 100.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 10000.0f) v = 10000.0f;
    return (uint16_t)(v + 0.5f);
}

static bool tx_frame(uint32_t id, const uint8_t data[8], uint8_t len)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = CAN_FRAME_EFF | (id & CAN_FRAME_ID_MASK);
    frame.data_len = (len <= 8u) ? len : 8u;
    memcpy(frame.data, data, frame.data_len);
    return mcp2518fd_transmit(&frame);
}

static bool send_probe(void)
{
    uint8_t data[8] = {0};
    custom_can_le32_store(&data[0], g_selected_id);
    data[4] = 1u;  // protocol minor marker
    return tx_frame(CUSTOM_CAN_ID_PROBE, data, 8u);
}

static bool send_select(uint32_t board_id)
{
    uint8_t data[8] = {0};
    custom_can_le32_store(&data[0], board_id);
    data[4] = CUSTOM_CAN_SELECT_FLAG_SELECT;
    g_selected_id = board_id;
    return tx_frame(CUSTOM_CAN_ID_SELECT, data, 8u);
}

static bool send_select_before_targeted_command(void)
{
    if (g_selected_id == 0u) {
        return true;
    }
    const bool ok = send_select(g_selected_id);
    sleep_us(300);
    return ok;
}

static bool send_clear_select(void)
{
    uint8_t data[8] = {0};
    data[4] = CUSTOM_CAN_SELECT_FLAG_CLEAR;
    g_selected_id = 0u;
    g_armed = false;
    g_throttle_pct = 0.0f;
    return tx_frame(CUSTOM_CAN_ID_SELECT, data, 8u);
}

static bool send_command_frame(void)
{
    uint8_t data[8] = {0};
    data[0] = g_armed ? CUSTOM_CAN_COMMAND_FLAG_ARMED : 0u;
    data[1] = g_command_seq++;
    custom_can_le16_store(&data[2], clamp_throttle_centi(g_throttle_pct));
    return tx_frame(CUSTOM_CAN_ID_COMMAND, data, 8u);
}

static bool send_selected_command_frame(void)
{
    const bool ok_select = send_select_before_targeted_command();
    return send_command_frame() && ok_select;
}

static bool send_pid_frames(float kp, float ki, float kd, float limit)
{
    uint8_t a[8] = {0};
    uint8_t b[8] = {0};
    custom_can_float_store(&a[0], kp);
    custom_can_float_store(&a[4], ki);
    custom_can_float_store(&b[0], kd);
    custom_can_float_store(&b[4], limit);
    const bool ok_a = tx_frame(CUSTOM_CAN_ID_PID_KP_KI, a, 8u);
    sleep_us(500);
    const bool ok_b = tx_frame(CUSTOM_CAN_ID_PID_KD_LIMIT, b, 8u);
    return ok_a && ok_b;
}

static bool send_feedforward_idle(float rpm, uint16_t us)
{
    uint8_t data[8] = {0};
    custom_can_float_store(&data[0], rpm);
    custom_can_le16_store(&data[4], us);
    return tx_frame(CUSTOM_CAN_ID_FF_IDLE, data, 8u);
}

static bool send_feedforward_max(float rpm, uint16_t us)
{
    uint8_t data[8] = {0};
    custom_can_float_store(&data[0], rpm);
    custom_can_le16_store(&data[4], us);
    return tx_frame(CUSTOM_CAN_ID_FF_MAX, data, 8u);
}

static bool send_start_config(uint16_t start_us, uint16_t hold_ms)
{
    uint8_t data[8] = {0};
    custom_can_le16_store(&data[0], start_us);
    custom_can_le16_store(&data[2], hold_ms);
    return tx_frame(CUSTOM_CAN_ID_START_CONFIG, data, 8u);
}

static bool send_manual_pwm_test(uint16_t throttle_us, uint16_t hold_ms)
{
    uint8_t data[8] = {0};
    custom_can_le16_store(&data[0], throttle_us);
    custom_can_le16_store(&data[2], hold_ms);
    return tx_frame(CUSTOM_CAN_ID_MANUAL_PWM_TEST, data, 8u);
}

static bool send_manual_pwm_stop(void)
{
    uint8_t data[8] = {0};
    return tx_frame(CUSTOM_CAN_ID_MANUAL_PWM_TEST, data, 8u);
}

static bool send_telemetry_rate(uint16_t telem_a_ms, uint16_t telem_b_ms, uint16_t telem_c_ms)
{
    uint8_t data[8] = {0};
    custom_can_le16_store(&data[0], telem_a_ms);
    custom_can_le16_store(&data[2], telem_b_ms);
    custom_can_le16_store(&data[4], telem_c_ms);
    return tx_frame(CUSTOM_CAN_ID_TELEM_RATE, data, 8u);
}

static bool send_rpm_thresholds(uint16_t high_raw, uint16_t low_raw)
{
    uint8_t data[8] = {0};
    custom_can_le16_store(&data[0], high_raw);
    custom_can_le16_store(&data[2], low_raw);
    return tx_frame(CUSTOM_CAN_ID_RPM_THRESH, data, 8u);
}

static bool send_action(uint8_t action, bool global)
{
    uint8_t data[8] = {0};
    data[0] = action;
    data[1] = global ? CUSTOM_CAN_ACTION_FLAG_GLOBAL : 0u;
    return tx_frame(CUSTOM_CAN_ID_ACTION, data, 8u);
}

static bool send_set_board_id(uint32_t old_id, uint32_t new_id)
{
    uint8_t data[8] = {0};
    custom_can_le32_store(&data[0], old_id);
    custom_can_le32_store(&data[4], new_id);
    if (old_id == g_selected_id || old_id == CUSTOM_CAN_BROADCAST_BOARD_ID) {
        g_selected_id = new_id;
    }
    return tx_frame(CUSTOM_CAN_ID_SET_BOARD_ID, data, 8u);
}

static bool send_auto_endpoint(bool endpoint_is_max, bool enable, float target_rpm, uint16_t rate_us_per_s, uint16_t start_us, bool global)
{
    uint8_t data[8] = {0};
    if (target_rpm < 0.0f) target_rpm = 0.0f;
    if (target_rpm > 65535.0f) target_rpm = 65535.0f;
    data[0] = endpoint_is_max ? CUSTOM_CAN_AUTO_ENDPOINT_MAX : CUSTOM_CAN_AUTO_ENDPOINT_IDLE;
    data[1] = (enable ? CUSTOM_CAN_AUTO_ENDPOINT_ENABLE : 0u) |
              (global ? CUSTOM_CAN_AUTO_ENDPOINT_GLOBAL : 0u);
    custom_can_le16_store(&data[2], rate_us_per_s);
    custom_can_le16_store(&data[4], (uint16_t)(target_rpm + 0.5f));
    custom_can_le16_store(&data[6], start_us);
    return tx_frame(CUSTOM_CAN_ID_AUTO_ENDPOINT_EX, data, 8u);
}

static bool send_hall_auto_cal(bool enable, float target_rpm, uint16_t duration_ms, bool global)
{
    uint8_t data[8] = {0};
    data[0] = (enable ? CUSTOM_CAN_HALL_CAL_ENABLE : 0u) |
              (global ? CUSTOM_CAN_HALL_CAL_GLOBAL : 0u);
    custom_can_le16_store(&data[2], duration_ms);
    custom_can_float_store(&data[4], target_rpm);
    return tx_frame(CUSTOM_CAN_ID_HALL_AUTO_CAL, data, 8u);
}

static void print_ok_or_busy(const char *what, bool ok)
{
    printf("%s %s\r\n", ok ? "OK" : "ERR TX_BUSY", what);
}

static char *skip_spaces(char *s)
{
    while (*s != '\0' && isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}

static bool parse_u32_token(const char *s, uint32_t *out)
{
    if (s == NULL || out == NULL) return false;
    if (strcmp(s, "ALL") == 0 || strcmp(s, "all") == 0) {
        *out = CUSTOM_CAN_BROADCAST_BOARD_ID;
        return true;
    }
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s || *end != '\0' || v > 0xFFFFFFFFul) {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static void handle_line(char *line)
{
    char cmd[32] = {0};
    int consumed = 0;
    if (sscanf(line, "%31s%n", cmd, &consumed) != 1) {
        return;
    }
    char *args = skip_spaces(line + consumed);

    if (strcmp(cmd, "PING") == 0) {
        printf("PONG selected=%lu\r\n", (unsigned long)g_selected_id);
        return;
    }

    if (strcmp(cmd, "INFO") == 0) {
        printf("INFO bridge=rp2040_usb_can_debug_probe protocol=custom_can_v2 bitrate=%u selected=%lu commands=PING,INFO,PROBE,SELECT,CLEAR_SELECT,CMD,ARM,THROTTLE,PID,FF0,FF100,STARTCFG,PWMTEST,PWMTEST_STOP,THRESH,HALLCAL,HALLCAL_STOP,RATE,IDENTIFY,STOP_IDENTIFY,SERVO_TEST,AUTO0,AUTO100,AUTO_STOP,SETID,PANIC_ALL\r\n",
               CAN_BITRATE_HZ,
               (unsigned long)g_selected_id);
        return;
    }

    if (strcmp(cmd, "PROBE") == 0 || strcmp(cmd, "SCAN") == 0) {
        print_ok_or_busy("PROBE", send_probe());
        return;
    }

    if (strcmp(cmd, "SELECT") == 0) {
        char token[32] = {0};
        uint32_t id = 0u;
        if (sscanf(args, "%31s", token) == 1 && parse_u32_token(token, &id)) {
            print_ok_or_busy("SELECT", send_select(id));
        } else {
            printf("ERR BAD_SELECT usage=SELECT <board_id|ALL>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "CLEAR_SELECT") == 0) {
        print_ok_or_busy("CLEAR_SELECT", send_clear_select());
        return;
    }

    if (strcmp(cmd, "CMD") == 0) {
        int armed = 0;
        float throttle = 0.0f;
        if (sscanf(args, "%d %f", &armed, &throttle) == 2) {
            g_armed = (armed != 0);
            g_throttle_pct = clampf_local(throttle, 0.0f, 100.0f);
            print_ok_or_busy("CMD", send_selected_command_frame());
        } else {
            printf("ERR BAD_CMD usage=CMD <armed0or1> <throttle_pct>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "ARM") == 0) {
        int armed = 0;
        if (sscanf(args, "%d", &armed) == 1) {
            g_armed = (armed != 0);
            if (!g_armed) {
                g_throttle_pct = 0.0f;
                (void)send_auto_endpoint(false, false, 0.0f, 0u, CUSTOM_CAN_AUTO_ENDPOINT_DEFAULT_START_US, false);
            }
            print_ok_or_busy("ARM", send_selected_command_frame());
        } else {
            printf("ERR BAD_ARM usage=ARM <0or1>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "THROTTLE") == 0) {
        float throttle = 0.0f;
        if (sscanf(args, "%f", &throttle) == 1) {
            g_throttle_pct = clampf_local(throttle, 0.0f, 100.0f);
            print_ok_or_busy("THROTTLE", send_selected_command_frame());
        } else {
            printf("ERR BAD_THROTTLE usage=THROTTLE <pct>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "PID") == 0) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, limit = 0.0f;
        if (sscanf(args, "%f %f %f %f", &kp, &ki, &kd, &limit) == 4) {
            print_ok_or_busy("PID", send_pid_frames(kp, ki, kd, limit));
        } else {
            printf("ERR BAD_PID usage=PID <kp> <ki> <kd> <limit_us>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "FF0") == 0) {
        float rpm = 0.0f;
        unsigned int us = 0u;
        if (sscanf(args, "%f %u", &rpm, &us) == 2 && us <= 65535u) {
            print_ok_or_busy("FF0", send_feedforward_idle(rpm, (uint16_t)us));
        } else {
            printf("ERR BAD_FF0 usage=FF0 <rpm> <throttle_us>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "FF100") == 0) {
        float rpm = 0.0f;
        unsigned int us = 0u;
        if (sscanf(args, "%f %u", &rpm, &us) == 2 && us <= 65535u) {
            print_ok_or_busy("FF100", send_feedforward_max(rpm, (uint16_t)us));
        } else {
            printf("ERR BAD_FF100 usage=FF100 <rpm> <throttle_us>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "STARTCFG") == 0 || strcmp(cmd, "START_CONFIG") == 0) {
        unsigned int start_us = 0u;
        unsigned int hold_ms = 0u;
        if (sscanf(args, "%u %u", &start_us, &hold_ms) == 2 &&
            start_us <= 65535u && hold_ms <= 65535u) {
            print_ok_or_busy("STARTCFG", send_start_config((uint16_t)start_us, (uint16_t)hold_ms));
        } else {
            printf("ERR BAD_STARTCFG usage=STARTCFG <start_throttle_us> <hold_after_rpm_ms>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "PWMTEST") == 0 || strcmp(cmd, "PWM_TEST") == 0 || strcmp(cmd, "SERVO_PWM") == 0) {
        unsigned int throttle_us = 0u;
        unsigned int hold_ms = 0u;
        if (sscanf(args, "%u %u", &throttle_us, &hold_ms) >= 1 &&
            throttle_us <= 65535u && hold_ms <= 65535u) {
            if (hold_ms == 0u) {
                hold_ms = 10000u;
            }
            print_ok_or_busy("PWMTEST", send_manual_pwm_test((uint16_t)throttle_us, (uint16_t)hold_ms));
        } else {
            printf("ERR BAD_PWMTEST usage=PWMTEST <throttle_us> [hold_ms]\r\n");
        }
        return;
    }

    if (strcmp(cmd, "PWMTEST_STOP") == 0 || strcmp(cmd, "PWM_STOP") == 0 || strcmp(cmd, "SERVO_PWM_STOP") == 0) {
        print_ok_or_busy("PWMTEST_STOP", send_manual_pwm_stop());
        return;
    }

    if (strcmp(cmd, "THRESH") == 0 || strcmp(cmd, "RPM_THRESH") == 0) {
        unsigned int high = 0u, low = 0u;
        if (sscanf(args, "%u %u", &high, &low) == 2 && high <= 4095u && low <= 4095u && high > low) {
            print_ok_or_busy("THRESH", send_rpm_thresholds((uint16_t)high, (uint16_t)low));
        } else {
            printf("ERR BAD_THRESH usage=THRESH <high_raw_0_4095> <low_raw_0_4095>, high must be > low\r\n");
        }
        return;
    }

    if (strcmp(cmd, "HALLCAL") == 0 || strcmp(cmd, "HALL_CAL") == 0) {
        float rpm = 0.0f;
        float timeout_s = 0.0f;
        const int n = sscanf(args, "%f %f", &rpm, &timeout_s);
        if (n >= 1 && rpm > 0.0f && rpm <= 50000.0f &&
            timeout_s >= 0.0f && timeout_s <= 60.0f) {
            uint32_t timeout_ms_u32 = (uint32_t)(timeout_s * 1000.0f + 0.5f);
            if (timeout_ms_u32 > 65535u) timeout_ms_u32 = 65535u;
            // timeout=0 means run until clean or until HALLCAL_STOP/abort.
            print_ok_or_busy("HALLCAL", send_hall_auto_cal(true, rpm, (uint16_t)timeout_ms_u32, false));
        } else {
            printf("ERR BAD_HALLCAL usage=HALLCAL <external_spinner_target_rpm> [optional_timeout_s_0_to_60]\r\n");
        }
        return;
    }

    if (strcmp(cmd, "HALLCAL_STOP") == 0 || strcmp(cmd, "HALL_CAL_STOP") == 0) {
        print_ok_or_busy("HALLCAL_STOP", send_hall_auto_cal(false, 0.0f, 0u, true));
        return;
    }

    if (strcmp(cmd, "RATE") == 0) {
        unsigned int a_ms = 0u, b_ms = 0u, c_ms = 0u;
        if (sscanf(args, "%u %u %u", &a_ms, &b_ms, &c_ms) == 3 &&
            a_ms <= 65535u && b_ms <= 65535u && c_ms <= 65535u) {
            print_ok_or_busy("RATE", send_telemetry_rate((uint16_t)a_ms, (uint16_t)b_ms, (uint16_t)c_ms));
        } else {
            printf("ERR BAD_RATE usage=RATE <telem_a_ms> <telem_b_ms> <telem_c_ms>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "IDENTIFY") == 0 || strcmp(cmd, "BLINK") == 0) {
        print_ok_or_busy("IDENTIFY", send_action(CUSTOM_CAN_ACTION_IDENTIFY, false));
        return;
    }

    if (strcmp(cmd, "STOP_IDENTIFY") == 0 || strcmp(cmd, "STOP_BLINK") == 0 ||
        strcmp(cmd, "IDENTIFY_STOP") == 0 || strcmp(cmd, "FOUND") == 0) {
        // Stop is intentionally broadcast/global so the operator can silence any
        // board that is currently locating itself even if the GUI selection changed.
        print_ok_or_busy("STOP_IDENTIFY", send_action(CUSTOM_CAN_ACTION_STOP_IDENTIFY, true));
        return;
    }

    // Kept as a serial backdoor for compatibility, but the GUI no longer exposes
    // a separate beep-only button; IDENTIFY is the normal audible/visual locator.
    if (strcmp(cmd, "BEEP") == 0 || strcmp(cmd, "START_BEEP") == 0) {
        print_ok_or_busy("BEEP", send_action(CUSTOM_CAN_ACTION_START_BEEP, false));
        return;
    }

    if (strcmp(cmd, "SERVO_TEST") == 0) {
        print_ok_or_busy("SERVO_TEST", send_action(CUSTOM_CAN_ACTION_SERVO_TEST, false));
        return;
    }

    if (strcmp(cmd, "AUTO0") == 0 || strcmp(cmd, "AUTO100") == 0) {
        float rpm = 0.0f;
        unsigned int rate = 25u;
        unsigned int start_us = CUSTOM_CAN_AUTO_ENDPOINT_DEFAULT_START_US;
        if (sscanf(args, "%f %u %u", &rpm, &rate, &start_us) >= 1 &&
            rate <= 65535u && start_us >= 500u && start_us <= 2500u) {
            const bool is_max = strcmp(cmd, "AUTO100") == 0;
            g_throttle_pct = is_max ? 100.0f : 0.0f;
            print_ok_or_busy(cmd, send_auto_endpoint(is_max, true, rpm, (uint16_t)rate, (uint16_t)start_us, false));
            (void)send_selected_command_frame();
        } else {
            printf("ERR BAD_AUTO usage=%s <target_rpm> [rate_us_per_s] [start_us]\r\n", cmd);
        }
        return;
    }

    if (strcmp(cmd, "AUTO_STOP") == 0) {
        // Stop auto-tune/calibration, but do not clear ARM. The bridge-owned
        // command heartbeat should immediately move the selected engine to 0%
        // throttle while preserving whether it was armed.
        g_throttle_pct = 0.0f;
        const bool ok_action = send_action(CUSTOM_CAN_ACTION_STOP_AUTO, false);
        const bool ok_hall = send_hall_auto_cal(false, 0.0f, 0u, true);
        const bool ok_cmd = send_selected_command_frame();
        print_ok_or_busy("AUTO_STOP", ok_action && ok_hall && ok_cmd);
        return;
    }

    if (strcmp(cmd, "SETID") == 0) {
        char old_tok[32] = {0};
        unsigned long new_ul = 0;
        uint32_t old_id = CUSTOM_CAN_BROADCAST_BOARD_ID;
        uint32_t new_id = 0u;
        int n = sscanf(args, "%31s %lu", old_tok, &new_ul);
        if (n == 1) {
            new_ul = strtoul(old_tok, NULL, 0);
            old_id = CUSTOM_CAN_BROADCAST_BOARD_ID;
        } else if (n == 2) {
            if (!parse_u32_token(old_tok, &old_id)) n = 0;
        }
        new_id = (uint32_t)new_ul;
        if (n >= 1 && new_id != 0u && new_id != CUSTOM_CAN_BROADCAST_BOARD_ID) {
            print_ok_or_busy("SETID", send_set_board_id(old_id, new_id));
        } else {
            printf("ERR BAD_SETID usage=SETID <new_id> OR SETID <old_id|ALL> <new_id>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "PANIC_ALL") == 0) {
        g_armed = false;
        g_throttle_pct = 0.0f;
        (void)send_select(CUSTOM_CAN_BROADCAST_BOARD_ID);
        (void)send_action(CUSTOM_CAN_ACTION_GLOBAL_DISARM, true);
        print_ok_or_busy("PANIC_ALL", send_command_frame());
        return;
    }

    printf("ERR UNKNOWN_COMMAND cmd=%s\r\n", cmd);
}

static void poll_usb_serial(void)
{
    for (;;) {
        const int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            break;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            g_line[g_line_len] = '\0';
            if (g_line_len > 0u) {
                handle_line(g_line);
            }
            g_line_len = 0u;
            continue;
        }
        if (g_line_len + 1u < SERIAL_LINE_MAX) {
            g_line[g_line_len++] = (char)ch;
        } else {
            g_line_len = 0u;
            printf("ERR SERIAL_LINE_TOO_LONG\r\n");
        }
    }
}

static bool frame_id_is(const CanFrame *frame, uint32_t id)
{
    return frame != NULL &&
           ((frame->id & CAN_FRAME_EFF) != 0u) &&
           ((frame->id & CAN_FRAME_ID_MASK) == id);
}

static void print_telem_a(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const float rpm = custom_can_float_load(&frame->data[0]);
    const uint16_t out_us = custom_can_le16_load(&frame->data[4]);
    const uint8_t state = frame->data[6];
    const uint8_t flags = frame->data[7];
    printf("TEL A rpm=%.3f out_us=%u state=%u flags=%u selected=%lu\r\n",
           (double)rpm, out_us, state, flags, (unsigned long)g_selected_id);
}

static void print_telem_b(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const float target_rpm = custom_can_float_load(&frame->data[0]);
    const uint16_t ff_us = custom_can_le16_load(&frame->data[4]);
    const int16_t pid_x10 = custom_can_le16_load_i(&frame->data[6]);
    printf("TEL B target_rpm=%.3f ff_us=%u pid_us=%.1f selected=%lu\r\n",
           (double)target_rpm, ff_us, (double)pid_x10 / 10.0, (unsigned long)g_selected_id);
}

static void print_telem_c(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const int16_t temp_x10 = custom_can_le16_load_i(&frame->data[0]);
    const int16_t current_ma = custom_can_le16_load_i(&frame->data[2]);
    const uint16_t bus_mv = custom_can_le16_load(&frame->data[4]);
    const uint16_t runtime_s = custom_can_le16_load(&frame->data[6]);
    printf("TEL C temp_c=%.1f current_a=%.3f vbus_v=%.3f runtime_s=%u selected=%lu\r\n",
           (double)temp_x10 / 10.0,
           (double)current_ma / 1000.0,
           (double)bus_mv / 1000.0,
           runtime_s,
           (unsigned long)g_selected_id);
}

static void print_board_announce(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const uint32_t id = custom_can_le32_load(&frame->data[0]);
    const uint16_t rpm = custom_can_le16_load(&frame->data[4]);
    const uint8_t state = frame->data[6];
    const uint8_t flags = frame->data[7];
    printf("BOARD id=%lu rpm=%u state=%u flags=%u selected=%u armed=%u fc=%u debug=%u\r\n",
           (unsigned long)id,
           rpm,
           state,
           flags,
           (flags & 0x01u) ? 1u : 0u,
           (flags & 0x02u) ? 1u : 0u,
           (flags & 0x04u) ? 1u : 0u,
           (flags & 0x08u) ? 1u : 0u);
}

static void print_auto_status(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const uint8_t endpoint = frame->data[0];
    const bool active = frame->data[1] != 0u;
    const uint16_t us = custom_can_le16_load(&frame->data[2]);
    const int16_t err_x10 = custom_can_le16_load_i(&frame->data[4]);
    const uint16_t target = custom_can_le16_load(&frame->data[6]);
    printf("AUTO endpoint=%u active=%u us=%u error_rpm=%.1f target_rpm=%u selected=%lu\r\n",
           endpoint,
           active ? 1u : 0u,
           us,
           (double)err_x10 / 10.0,
           target,
           (unsigned long)g_selected_id);
}

static const char *hall_status_name(uint8_t status)
{
    switch (status) {
    case CUSTOM_CAN_HALL_CAL_STATUS_IDLE: return "IDLE";
    case CUSTOM_CAN_HALL_CAL_STATUS_RUNNING: return "RUNNING";
    case CUSTOM_CAN_HALL_CAL_STATUS_DONE_OK: return "DONE_OK";
    case CUSTOM_CAN_HALL_CAL_STATUS_FAILED: return "FAILED";
    default: return "UNKNOWN";
    }
}

static void print_hall_cal_status(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const uint8_t status = frame->data[0];
    const uint8_t progress = frame->data[1];
    const uint16_t min_raw = custom_can_le16_load(&frame->data[2]);
    const uint16_t max_raw = custom_can_le16_load(&frame->data[4]);
    const uint16_t target = custom_can_le16_load(&frame->data[6]);
    const uint16_t span = (max_raw >= min_raw) ? (uint16_t)(max_raw - min_raw) : 0u;
    printf("HALLCAL status=%s code=%u active=%u quality=%u progress=%u min=%u max=%u span=%u target_rpm=%u selected=%lu\r\n",
           hall_status_name(status),
           status,
           status == CUSTOM_CAN_HALL_CAL_STATUS_RUNNING ? 1u : 0u,
           progress,
           progress,
           min_raw,
           max_raw,
           span,
           target,
           (unsigned long)g_selected_id);
}

static void poll_can(void)
{
    CanFrame frame;
    int budget = 96;
    while ((budget-- > 0) && mcp2518fd_receive(&frame)) {
        if (frame_id_is(&frame, CUSTOM_CAN_ID_TELEM_A)) {
            print_telem_a(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_TELEM_B)) {
            print_telem_b(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_TELEM_C)) {
            print_telem_c(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_BOARD_ANNOUNCE)) {
            print_board_announce(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_AUTO_STATUS)) {
            print_auto_status(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_HALL_CAL_STATUS)) {
            print_hall_cal_status(&frame);
        }
#if BRIDGE_PRINT_RX_RAW
        else {
            printf("RAW id=0x%08lx dlc=%u\r\n", (unsigned long)(frame.id & CAN_FRAME_ID_MASK), frame.data_len);
        }
#endif
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1200);

    printf("BRIDGE READY custom_can_v2\r\n");
    printf("INFO commands=PING,INFO,PROBE,SCAN,SELECT,CLEAR_SELECT,CMD,ARM,THROTTLE,PID,FF0,FF100,STARTCFG,PWMTEST,PWMTEST_STOP,THRESH,HALLCAL,HALLCAL_STOP,RATE,IDENTIFY,BLINK,STOP_IDENTIFY,SERVO_TEST,AUTO0,AUTO100,AUTO_STOP,SETID,PANIC_ALL\r\n");

    if (!mcp2518fd_init()) {
        printf("ERR MCP2518FD_INIT_FAILED\r\n");
        while (true) {
            sleep_ms(100);
        }
    }

    uint32_t next_probe_ms = now_ms() + 50u;
    uint32_t next_command_ms = now_ms() + 80u;

    while (true) {
        poll_usb_serial();
        poll_can();

        const uint32_t ms = now_ms();
        if ((int32_t)(ms - next_probe_ms) >= 0) {
            (void)send_probe();
            if (g_selected_id != 0u) {
                (void)send_select(g_selected_id);
            }
            next_probe_ms += DEBUG_PROBE_PERIOD_MS;
        }

        // The USB GUI also sends CMD heartbeats, but the bridge should own the
        // safety heartbeat once a board is selected. This prevents a short GUI
        // event-loop stall or serial hiccup from looking like an immediate
        // command timeout on the motor controller.
        if (g_selected_id != 0u && (int32_t)(ms - next_command_ms) >= 0) {
            // Keep the selected-board latch alive together with the command
            // heartbeat. This prevents a controller-side selection timeout or
            // missed SELECT frame from making TEL A/B/C disappear while the GUI
            // still shows an engine selected.
            (void)send_selected_command_frame();
            next_command_ms += DEBUG_COMMAND_HEARTBEAT_MS;
        }

        sleep_ms(1);
    }
}
