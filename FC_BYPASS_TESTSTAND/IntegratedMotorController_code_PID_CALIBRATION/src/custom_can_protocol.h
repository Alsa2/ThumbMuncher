#pragma once
#include <stdint.h>
#include <string.h>

// Custom extended classic-CAN IDs. These are dedicated bench-control identifiers.
#define CUSTOM_CAN_ID_COMMAND       0x1CEB0001u
#define CUSTOM_CAN_ID_PID_KP_KI     0x1CEB0010u
#define CUSTOM_CAN_ID_PID_KD_LIMIT  0x1CEB0011u
#define CUSTOM_CAN_ID_FF_IDLE       0x1CEB0020u
#define CUSTOM_CAN_ID_FF_MAX        0x1CEB0021u
#define CUSTOM_CAN_ID_TELEM_A       0x1CEB0100u
#define CUSTOM_CAN_ID_TELEM_B       0x1CEB0101u
#define CUSTOM_CAN_ID_TELEM_C       0x1CEB0102u

// COMMAND payload, 8 bytes:
//   byte 0: flags bit0 = armed
//   byte 1: sequence / heartbeat counter
//   byte 2..3: throttle in centi-percent, 0..10000 = 0.00..100.00 %
//   byte 4..7: reserved
#define CUSTOM_CAN_COMMAND_FLAG_ARMED 0x01u

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
