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

#define SERIAL_LINE_MAX 160

static bool g_armed = false;
static float g_throttle_pct = 0.0f;
static uint8_t g_command_seq = 0u;
static char g_line[SERIAL_LINE_MAX];
static size_t g_line_len = 0u;

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

static bool send_command_frame(void)
{
    uint8_t data[8] = {0};
    data[0] = g_armed ? CUSTOM_CAN_COMMAND_FLAG_ARMED : 0u;
    data[1] = g_command_seq++;
    custom_can_le16_store(&data[2], clamp_throttle_centi(g_throttle_pct));
    return tx_frame(CUSTOM_CAN_ID_COMMAND, data, 8u);
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

static bool send_telemetry_rate(uint16_t telem_a_ms, uint16_t telem_b_ms, uint16_t telem_c_ms)
{
    uint8_t data[8] = {0};
    custom_can_le16_store(&data[0], telem_a_ms);
    custom_can_le16_store(&data[2], telem_b_ms);
    custom_can_le16_store(&data[4], telem_c_ms);
    return tx_frame(CUSTOM_CAN_ID_TELEM_RATE, data, 8u);
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

static void handle_line(char *line)
{
    char cmd[24] = {0};
    int consumed = 0;
    if (sscanf(line, "%23s%n", cmd, &consumed) != 1) {
        return;
    }
    char *args = skip_spaces(line + consumed);

    if (strcmp(cmd, "PING") == 0) {
        printf("PONG\r\n");
        return;
    }

    if (strcmp(cmd, "INFO") == 0) {
        printf("INFO bridge=rp2040_usb_can_bridge protocol=custom_can_v1 bitrate=%u\r\n", CAN_BITRATE_HZ);
        return;
    }

    if (strcmp(cmd, "CMD") == 0) {
        int armed = 0;
        float throttle = 0.0f;
        if (sscanf(args, "%d %f", &armed, &throttle) == 2) {
            g_armed = (armed != 0);
            g_throttle_pct = clampf_local(throttle, 0.0f, 100.0f);
            print_ok_or_busy("CMD", send_command_frame());
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
            }
            print_ok_or_busy("ARM", send_command_frame());
        } else {
            printf("ERR BAD_ARM usage=ARM <0or1>\r\n");
        }
        return;
    }

    if (strcmp(cmd, "THROTTLE") == 0) {
        float throttle = 0.0f;
        if (sscanf(args, "%f", &throttle) == 1) {
            g_throttle_pct = clampf_local(throttle, 0.0f, 100.0f);
            print_ok_or_busy("THROTTLE", send_command_frame());
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
    printf("TEL A rpm=%.3f out_us=%u state=%u flags=%u\r\n",
           (double)rpm, out_us, state, flags);
}

static void print_telem_b(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const float target_rpm = custom_can_float_load(&frame->data[0]);
    const uint16_t ff_us = custom_can_le16_load(&frame->data[4]);
    const int16_t pid_x10 = custom_can_le16_load_i(&frame->data[6]);
    printf("TEL B target_rpm=%.3f ff_us=%u pid_us=%.1f\r\n",
           (double)target_rpm, ff_us, (double)pid_x10 / 10.0);
}

static void print_telem_c(const CanFrame *frame)
{
    if (frame->data_len < 8u) return;
    const int16_t temp_x10 = custom_can_le16_load_i(&frame->data[0]);
    const int16_t current_ma = custom_can_le16_load_i(&frame->data[2]);
    const uint16_t bus_mv = custom_can_le16_load(&frame->data[4]);
    const uint16_t runtime_s = custom_can_le16_load(&frame->data[6]);
    printf("TEL C temp_c=%.1f current_a=%.3f vbus_v=%.3f runtime_s=%u\r\n",
           (double)temp_x10 / 10.0,
           (double)current_ma / 1000.0,
           (double)bus_mv / 1000.0,
           runtime_s);
}

static void poll_can(void)
{
    CanFrame frame;
    int budget = 64;
    while ((budget-- > 0) && mcp2518fd_receive(&frame)) {
        if (frame_id_is(&frame, CUSTOM_CAN_ID_TELEM_A)) {
            print_telem_a(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_TELEM_B)) {
            print_telem_b(&frame);
        } else if (frame_id_is(&frame, CUSTOM_CAN_ID_TELEM_C)) {
            print_telem_c(&frame);
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

    printf("BRIDGE READY custom_can_v1\r\n");
    printf("INFO commands=PING,INFO,CMD,ARM,THROTTLE,PID,FF0,FF100,RATE\r\n");

    if (!mcp2518fd_init()) {
        printf("ERR MCP2518FD_INIT_FAILED\r\n");
        while (true) {
            sleep_ms(100);
        }
    }

    while (true) {
        poll_usb_serial();
        poll_can();
        sleep_ms(1);
    }
}
