#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ENGINE_DISARMED = 0,
    ENGINE_ARMED_WAIT = 1,
    ENGINE_CRANKING = 2,
    ENGINE_IDLE_HOLD = 3,
    ENGINE_RUNNING = 4,
    ENGINE_FAULT = 5
} EngineState;

void engine_control_init(void);
void engine_control_set_armed(bool armed);
void engine_control_set_cmd_pct(float pct);
void engine_control_update(uint32_t now_ms);

EngineState engine_control_get_state(void);
float engine_control_get_target_rpm(void);