#pragma once
#include <stdint.h>

// Keep the same flag layout commonly used by SocketCAN/libcanard-style frame IDs.
#define CAN_FRAME_EFF 0x80000000u
#define CAN_FRAME_RTR 0x40000000u
#define CAN_FRAME_ERR 0x20000000u
#define CAN_FRAME_ID_MASK 0x1FFFFFFFu

typedef struct {
    uint32_t id;       // 29-bit CAN ID plus CAN_FRAME_* flags.
    uint8_t data_len;  // Classic CAN payload length: 0..8.
    uint8_t data[8];
    uint8_t iface_id;
} CanFrame;
