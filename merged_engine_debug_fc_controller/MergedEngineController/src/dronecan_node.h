#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "canard.h"

void dronecan_node_init(void);
void dronecan_node_init_with_node_id(uint8_t node_id);
void dronecan_node_init_with_ids(uint8_t node_id, uint8_t esc_index);
uint8_t dronecan_node_resolve_node_id_from_board_id(uint32_t board_id);
uint8_t dronecan_node_resolve_esc_index_from_board_id(uint32_t board_id);
uint8_t dronecan_node_get_local_node_id(void);
uint8_t dronecan_node_get_esc_index(void);
void dronecan_node_handle_frame(CanardCANFrame *frame, uint64_t timestamp_usec);

// RX-side state
bool dronecan_node_get_armed(void);
float dronecan_node_get_cmd_pct(void);
uint32_t dronecan_node_fc_age_ms(uint64_t now_us);
bool dronecan_node_fc_alive(uint64_t now_us);

// TX-side telemetry
void dronecan_node_publish_node_status(uint32_t uptime_s, uint8_t health, uint8_t mode, uint16_t vendor_status);
void dronecan_node_publish_esc_status(float rpm, float current_a, float temperature_c, float throttle_pct);
void dronecan_node_process_tx(void);
