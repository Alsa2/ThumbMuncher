#pragma once
#include <stdint.h>
#include <string.h>

// Custom extended classic-CAN IDs used only by the debug probe/GUI path.
// The FC path remains DroneCAN. Receipt of PROBE/SELECT/COMMAND places every
// motor controller into debug override; only the selected board accepts commands.
#define CUSTOM_CAN_ID_COMMAND          0x1CEB0001u
#define CUSTOM_CAN_ID_SELECT           0x1CEB0002u
#define CUSTOM_CAN_ID_PROBE            0x1CEB0003u
#define CUSTOM_CAN_ID_ACTION           0x1CEB0004u
#define CUSTOM_CAN_ID_SET_BOARD_ID     0x1CEB0005u
#define CUSTOM_CAN_ID_AUTO_ENDPOINT    0x1CEB0006u
#define CUSTOM_CAN_ID_HALL_AUTO_CAL    0x1CEB0007u
#define CUSTOM_CAN_ID_AUTO_ENDPOINT_EX 0x1CEB0008u

#define CUSTOM_CAN_ID_PID_KP_KI        0x1CEB0010u
#define CUSTOM_CAN_ID_PID_KD_LIMIT     0x1CEB0011u
#define CUSTOM_CAN_ID_FF_IDLE          0x1CEB0020u
#define CUSTOM_CAN_ID_FF_MAX           0x1CEB0021u
#define CUSTOM_CAN_ID_RPM_THRESH       0x1CEB0022u
#define CUSTOM_CAN_ID_START_CONFIG     0x1CEB0023u
#define CUSTOM_CAN_ID_MANUAL_PWM_TEST  0x1CEB0024u
#define CUSTOM_CAN_ID_TELEM_RATE       0x1CEB0030u

#define CUSTOM_CAN_ID_TELEM_A          0x1CEB0100u
#define CUSTOM_CAN_ID_TELEM_B          0x1CEB0101u
#define CUSTOM_CAN_ID_TELEM_C          0x1CEB0102u
#define CUSTOM_CAN_ID_BOARD_ANNOUNCE   0x1CEB0110u
#define CUSTOM_CAN_ID_AUTO_STATUS      0x1CEB0111u
#define CUSTOM_CAN_ID_HALL_CAL_STATUS  0x1CEB0112u

#define CUSTOM_CAN_BROADCAST_BOARD_ID  0xFFFFFFFFu

// COMMAND payload, 8 bytes:
//   byte 0: flags bit0 = armed
//   byte 1: sequence / heartbeat counter
//   byte 2..3: throttle in centi-percent, 0..10000 = 0.00..100.00 %
//   byte 4..7: reserved for v1 compatibility
#define CUSTOM_CAN_COMMAND_FLAG_ARMED  0x01u

// SELECT payload, 8 bytes:
//   byte 0..3: selected board_id, or 0xFFFFFFFF for broadcast/all boards
//   byte 4: flags bit0 = select, bit1 = clear selection
//   byte 5..7: reserved
#define CUSTOM_CAN_SELECT_FLAG_SELECT  0x01u
#define CUSTOM_CAN_SELECT_FLAG_CLEAR   0x02u

// ACTION payload, 8 bytes. Accepted by selected boards, unless GLOBAL flag set.
//   byte 0: action ID
//   byte 1: flags bit0 = global/all boards
//   byte 2..7: action-specific/reserved
#define CUSTOM_CAN_ACTION_IDENTIFY      1u  // blink/beep selected engine
#define CUSTOM_CAN_ACTION_SERVO_TEST    2u  // run existing safe servo sweep
#define CUSTOM_CAN_ACTION_STOP_AUTO     3u  // stop endpoint auto-adjust
#define CUSTOM_CAN_ACTION_START_BEEP    4u  // pre-startup audible identify
#define CUSTOM_CAN_ACTION_GLOBAL_DISARM 5u  // force debug command to disarmed/0%
#define CUSTOM_CAN_ACTION_STOP_IDENTIFY 6u // stop temporary blink/beep identify pattern
#define CUSTOM_CAN_ACTION_FLAG_GLOBAL   0x01u

// SET_BOARD_ID payload, 8 bytes:
//   byte 0..3: old/current board_id, or 0xFFFFFFFF for selected board
//   byte 4..7: new board_id. Controller persists it to FRAM when FRAM is present.

// RPM_THRESH payload, 8 bytes:
//   byte 0..1: Hall/RPM sensor high threshold ADC raw count, 0..4095
//   byte 2..3: Hall/RPM sensor low threshold ADC raw count, 0..4095
//   byte 4..7: reserved

// START_CONFIG payload, 8 bytes:
//   byte 0..1: startup/prime throttle PWM in microseconds. This exact value is
//              commanded while armed and waiting for the first believable RPM.
//   byte 2..3: hold time after first RPM, in milliseconds.
//   byte 4..7: reserved. Controller persists this to FRAM with PID/feedforward.

// MANUAL_PWM_TEST payload, 8 bytes:
//   byte 0..1: exact throttle PWM in microseconds
//   byte 2..3: bounded hold time in milliseconds. 0 uses firmware default.
//   byte 4..7: reserved. Selected board only.
//   Special stop: throttle_us=0 and hold_ms=0 immediately returns safe-off.

// AUTO_ENDPOINT payload, 8 bytes, legacy format:
//   byte 0: endpoint 0 = 0% / idle, 1 = 100% / max
//   byte 1: flags bit0 = enable, bit1 = selected-only bypass/global
//   byte 2..3: maximum adjustment rate in servo microseconds/second
//   byte 4..7: target RPM float32
//
// AUTO_ENDPOINT_EX payload, 8 bytes, preferred format from the GUI/probe:
//   byte 0: endpoint 0 = 0% / idle, 1 = 100% / max
//   byte 1: flags bit0 = enable, bit1 = selected-only bypass/global
//   byte 2..3: maximum adjustment rate in servo microseconds/second
//   byte 4..5: target RPM as uint16. This is the fixed desired RPM; auto-tune
//              must never replace it with measured RPM.
//   byte 6..7: starting servo PWM in microseconds, usually 1500.
#define CUSTOM_CAN_AUTO_ENDPOINT_IDLE   0u
#define CUSTOM_CAN_AUTO_ENDPOINT_MAX    1u
#define CUSTOM_CAN_AUTO_ENDPOINT_ENABLE 0x01u
#define CUSTOM_CAN_AUTO_ENDPOINT_GLOBAL 0x02u
#define CUSTOM_CAN_AUTO_ENDPOINT_DEFAULT_START_US 1500u

// HALL_AUTO_CAL payload, 8 bytes:
//   byte 0: flags bit0 = enable, bit1 = selected-only bypass/global
//   byte 1: reserved
//   byte 2..3: optional timeout in milliseconds. 0 means run until clean/abort.
//   byte 4..7: external-spinner target RPM float32. The target is used to
//              validate clean edge timing while thresholds are learned from raw ADC.
#define CUSTOM_CAN_HALL_CAL_ENABLE     0x01u
#define CUSTOM_CAN_HALL_CAL_GLOBAL     0x02u

// HALL_CAL_STATUS payload, 8 bytes:
//   byte 0: status 0=idle, 1=running, 2=done_ok, 3=failed/aborted
//   byte 1: signal-quality/convergence percent 0..100
//   byte 2..3: observed raw ADC minimum
//   byte 4..5: observed raw ADC maximum
//   byte 6..7: target RPM rounded/clamped to uint16
#define CUSTOM_CAN_HALL_CAL_STATUS_IDLE     0u
#define CUSTOM_CAN_HALL_CAL_STATUS_RUNNING  1u
#define CUSTOM_CAN_HALL_CAL_STATUS_DONE_OK  2u
#define CUSTOM_CAN_HALL_CAL_STATUS_FAILED   3u

static inline uint16_t custom_can_le16_load(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline int16_t custom_can_le16_load_i(const uint8_t *p)
{
    return (int16_t)custom_can_le16_load(p);
}

static inline uint32_t custom_can_le32_load(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline void custom_can_le16_store(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void custom_can_le16_store_i(uint8_t *p, int16_t v)
{
    custom_can_le16_store(p, (uint16_t)v);
}

static inline void custom_can_le32_store(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline float custom_can_float_load(const uint8_t *p)
{
    float f = 0.0f;
    uint32_t raw = custom_can_le32_load(p);
    memcpy(&f, &raw, sizeof(f));
    return f;
}

static inline void custom_can_float_store(uint8_t *p, float f)
{
    uint32_t raw = 0u;
    memcpy(&raw, &f, sizeof(raw));
    custom_can_le32_store(p, raw);
}
