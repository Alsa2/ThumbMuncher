#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "canard.h"
#include "sensors.h"

typedef enum {
    CUSTOM_CAN_AUTO_NONE = 0,
    CUSTOM_CAN_AUTO_IDLE = 1,
    CUSTOM_CAN_AUTO_MAX = 2,
} CustomCanAutoEndpoint;

typedef struct {
    uint32_t board_id;
    bool selected;
    bool debug_alive;
    bool command_alive;
    bool armed;
    float cmd_pct;
    uint32_t config_reject_count;
} CustomCanDebugState;

void custom_can_node_init(void);
void custom_can_node_handle_frame(const CanardCANFrame *frame, uint64_t timestamp_usec);

uint32_t custom_can_node_get_board_id(void);
bool custom_can_node_debug_alive(uint64_t now_us);
bool custom_can_node_selected(void);
bool custom_can_node_get_armed(void);
float custom_can_node_get_cmd_pct(void);
uint32_t custom_can_node_command_age_ms(uint64_t now_us);
bool custom_can_node_command_alive(uint64_t now_us);
uint32_t custom_can_node_get_config_reject_count(void);
void custom_can_node_get_telem_periods(uint16_t *a_ms, uint16_t *b_ms, uint16_t *c_ms);
void custom_can_node_get_state(CustomCanDebugState *out, uint64_t now_us);

bool custom_can_node_identify_active(uint32_t now_ms);
bool custom_can_node_consume_servo_test_request(void);
bool custom_can_node_consume_start_beep_request(void);
bool custom_can_node_consume_stop_auto_request(void);
bool custom_can_node_consume_config_dirty(void);

void custom_can_node_publish_board_announce(float rpm, uint8_t engine_state, bool armed, bool fc_alive);
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
void custom_can_node_publish_auto_status(uint8_t endpoint,
                                         bool active,
                                         float target_rpm,
                                         uint16_t endpoint_us,
                                         float rpm_error);
void custom_can_node_publish_hall_cal_status(const HallAutoCalStatus *status);
