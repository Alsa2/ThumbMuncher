#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

void custom_can_node_init(void);
void custom_can_node_handle_frame(const CanFrame *frame, uint64_t timestamp_usec);

bool custom_can_node_get_armed(void);
float custom_can_node_get_cmd_pct(void);
uint32_t custom_can_node_command_age_ms(uint64_t now_us);
bool custom_can_node_command_alive(uint64_t now_us);
uint32_t custom_can_node_get_config_reject_count(void);

void custom_can_node_publish_telem_a(float rpm,
                                     uint16_t throttle_output_us,
                                     uint8_t engine_state,
                                     bool command_alive,
                                     bool armed,
                                     bool stationary);
void custom_can_node_publish_telem_b(float target_rpm,
                                     uint16_t feedforward_us,
                                     float pid_correction_us);
void custom_can_node_publish_telem_c(float temperature_c,
                                     float current_a,
                                     float bus_voltage_v,
                                     uint32_t runtime_ms);
