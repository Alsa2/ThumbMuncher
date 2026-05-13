#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float rpm;
    float temperature_c;
    float current_a;
    float bus_voltage_v;
    uint32_t turns_since_start;
    bool stationary;
    bool hall_high;
    bool current_sensor_present;
} EngineSensors;

void sensors_init(void);
void sensors_update(uint32_t now_ms);
void sensors_snapshot(EngineSensors *out);
void sensors_reset_start_counter(void);
uint32_t sensors_get_runtime_ms(void);

#endif // SENSORS_H
