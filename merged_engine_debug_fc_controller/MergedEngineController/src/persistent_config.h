#ifndef PERSISTENT_CONFIG_H
#define PERSISTENT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

// Initializes the FM24CL64B-compatible FRAM on the board I2C1 lane.
bool persistent_config_init(void);

// Loads validated settings from FRAM and applies them to the engine/sensor runtime.
// If no valid record is present, firmware defaults remain active.
bool persistent_config_load_into_runtime(void);

// Saves the current engine throttle/PID settings, RPM sensor thresholds, and board ID to FRAM.
bool persistent_config_save_from_runtime(void);

// Saves only the debug board ID to a small backup FRAM record. This is used
// immediately after SETID so the ID survives a power cycle even if the full
// tuning/settings record later fails validation.
bool persistent_config_save_board_id(uint32_t board_id);

// Returns the last valid board ID loaded from FRAM, if one exists.
bool persistent_config_get_saved_board_id(uint32_t *out_board_id);

bool persistent_config_present(void);
bool persistent_config_last_load_valid(void);

#endif // PERSISTENT_CONFIG_H
