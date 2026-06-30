#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include "hardware/spi.h"

#define MCP_SPI_PORT                    spi0
#define PIN_CAN_SCK                     2u
#define PIN_CAN_MOSI                    3u
#define PIN_CAN_MISO                    4u
#define PIN_CAN_CS                      5u
#define PIN_CAN_INT                     6u

#define MCP_OSC_HZ                      20000000u
#define CAN_BITRATE_HZ                  1000000u

#define BRIDGE_PRINT_RX_RAW             0
#define BRIDGE_SERIAL_BAUD_PLACEHOLDER 115200u

#endif // BOARD_CONFIG_H
