#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "canard.h"

bool mcp2518fd_init(void);
bool mcp2518fd_receive(CanardCANFrame *out_frame);
bool mcp2518fd_transmit(const CanardCANFrame *in_frame);
uint32_t mcp2518fd_read_devid(void);

// Debug helpers
uint32_t mcp2518fd_debug_read_osc(void);
uint32_t mcp2518fd_debug_read_c1con(void);
uint32_t mcp2518fd_debug_read_c1int(void);
uint32_t mcp2518fd_debug_read_fifo1sta(void);
uint32_t mcp2518fd_debug_read_fifo1ua(void);
void mcp2518fd_debug_get_modes(uint8_t *reqop, uint8_t *opmod);
bool mcp2518fd_debug_peek_fifo1(uint8_t raw20[20]);
