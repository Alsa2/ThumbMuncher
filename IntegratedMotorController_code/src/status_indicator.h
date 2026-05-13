#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "engine_control.h"

void status_indicator_init(void);
void status_indicator_update(uint32_t now_ms,
                             bool fc_alive,
                             bool armed,
                             EngineState state);

#endif // STATUS_INDICATOR_H
