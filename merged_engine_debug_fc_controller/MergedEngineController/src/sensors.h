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


typedef struct {
    bool active;
    bool done;
    bool ok;
    uint8_t status_code;
    uint8_t progress_pct;
    uint16_t min_raw;
    uint16_t max_raw;
    uint16_t threshold_high_raw;
    uint16_t threshold_low_raw;
    uint32_t sample_count;
    uint32_t elapsed_ms;
    uint32_t duration_ms;
    float target_rpm;
} HallAutoCalStatus;

void sensors_init(void);
void sensors_update(uint32_t now_ms);
void sensors_snapshot(EngineSensors *out);
void sensors_reset_start_counter(void);
uint32_t sensors_get_runtime_ms(void);

bool sensors_hall_thresholds_are_valid(uint16_t high_raw, uint16_t low_raw);
bool sensors_set_hall_thresholds_raw(uint16_t high_raw, uint16_t low_raw);
bool sensors_apply_hall_thresholds_raw(uint16_t high_raw, uint16_t low_raw);
void sensors_get_hall_thresholds_raw(uint16_t *high_raw, uint16_t *low_raw);
bool sensors_consume_hall_thresholds_dirty(void);

bool sensors_hall_calibration_is_valid(uint16_t min_raw, uint16_t max_raw);
bool sensors_apply_hall_calibration_raw(uint16_t min_raw, uint16_t max_raw);
void sensors_get_hall_calibration_raw(uint16_t *min_raw, uint16_t *max_raw);
bool sensors_start_hall_auto_cal(float target_rpm, uint32_t duration_ms, uint32_t now_ms);
bool sensors_update_hall_auto_cal(uint32_t now_ms);
bool sensors_stop_hall_auto_cal(bool aborted);
bool sensors_hall_auto_cal_active(void);
void sensors_get_hall_auto_cal_status(HallAutoCalStatus *out);

#endif // SENSORS_H
