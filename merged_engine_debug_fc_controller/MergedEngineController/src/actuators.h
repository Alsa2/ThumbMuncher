#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdbool.h>
#include <stdint.h>

void actuators_init(void);
void actuators_set_relay(bool on);
void actuators_set_starter(bool on);
void actuators_set_choke_closed(bool closed);
void actuators_set_choke_us(uint16_t us);
void actuators_set_throttle_us(uint16_t us);
uint16_t actuators_get_choke_us(void);
uint16_t actuators_get_throttle_us(void);

#endif // ACTUATORS_H
