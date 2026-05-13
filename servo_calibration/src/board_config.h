#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include "hardware/i2c.h"
#include "hardware/spi.h"

// -----------------------------------------------------------------------------
// RP2040 pin map
// Adjust here if your board wiring differs.
// -----------------------------------------------------------------------------
#define PIN_LED                         18u
#define PIN_BUZZER                      19u
#define PIN_RELAY                       8u
#define PIN_STARTER                     9u      // kept safe/off by this firmware
#define PIN_CHOKE_SERVO                 16u
#define PIN_THROTTLE_SERVO              17u

#define PIN_TEMP_ADC_GPIO               26u     // ADC0
#define TEMP_ADC_INPUT                  0u
#define PIN_HALL_ADC_GPIO               27u     // ADC1
#define HALL_ADC_INPUT                  1u

// -----------------------------------------------------------------------------
// I2C lanes
// -----------------------------------------------------------------------------
// Keep the original I2C1 lane for FRAM / any existing board peripherals.
#define PIN_I2C_SDA                     10u
#define PIN_I2C_SCL                     11u
#define SENSOR_I2C_PORT                 i2c1
#define SENSOR_I2C_BAUD_HZ              400000u

#define PIN_FRAM_I2C_SDA                PIN_I2C_SDA
#define PIN_FRAM_I2C_SCL                PIN_I2C_SCL
#define FRAM_I2C_PORT                   SENSOR_I2C_PORT
#define FRAM_I2C_BAUD_HZ                SENSOR_I2C_BAUD_HZ

// Current/voltage sensor is on the separate I2C0 lane.
#define PIN_CURRENT_I2C_SDA             0u
#define PIN_CURRENT_I2C_SCL             1u
#define CURRENT_I2C_PORT                i2c0
#define CURRENT_I2C_BAUD_HZ             100000u
#define CURRENT_I2C_TIMEOUT_US          5000u

#define MCP_SPI_PORT                    spi0
#define PIN_CAN_SCK                     2u
#define PIN_CAN_MOSI                    3u
#define PIN_CAN_MISO                    4u
#define PIN_CAN_CS                      5u
#define PIN_CAN_INT                     6u

// -----------------------------------------------------------------------------
// CAN / DroneCAN
// -----------------------------------------------------------------------------
#define MCP_OSC_HZ                      20000000u
#define CAN_BITRATE_HZ                  1000000u
#define DRONECAN_NODE_ID                42u
#define DRONECAN_ESC_INDEX              0u
#define DRONECAN_FC_TIMEOUT_MS          1000u
#define NODE_STATUS_PERIOD_MS           1000u
#define ESC_STATUS_PERIOD_MS            100u

// -----------------------------------------------------------------------------
// Servo outputs
// Note: this throttle servo/ESC opens as the PWM pulse gets smaller.
// -----------------------------------------------------------------------------
#define SERVO_FREQ_HZ                   50u
#define SERVO_US_HARD_MIN               1000u
#define SERVO_US_HARD_MAX               2000u

#define THROTTLE_IDLE_US                1900u   // 0% throttle / idle point
#define THROTTLE_PRIME_US               1500u   // post-spin prime pulse
#define THROTTLE_MAX_US                 1450u   // 100% throttle feedforward point

#define CHOKE_US_CLOSED                 1900u
#define CHOKE_US_OPEN                   1100u

// -----------------------------------------------------------------------------
// Requested throttle/RPM feedforward curve
//   0%   -> 1900 us -> 1800 RPM
//   100% -> 1450 us -> 4250 RPM
// PID adds a bounded correction around the feedforward PWM.
// -----------------------------------------------------------------------------
#define THROTTLE_IDLE_RPM               1800.0f
#define THROTTLE_MAX_RPM                4250.0f
#define RPM_PID_CORRECTION_LIMIT_US     175.0f
#define RPM_PID_KP_US_PER_RPM           0.035f
#define RPM_PID_KI_US_PER_RPM_S         0.012f
#define RPM_PID_KD_US_PER_RPM_PER_S     0.000f

// -----------------------------------------------------------------------------
// State-machine timing / thresholds
// -----------------------------------------------------------------------------
#define CONTROL_DT_MS                   10u
#define PRINT_LOOP_MS                   500u
#define PRIME_AFTER_SPIN_MS             1000u
#define RPM_DETECT_THRESHOLD            50.0f
#define RPM_ZERO_TIMEOUT_MS             2000u
#define RPM_STATIONARY_THRESH           120.0f
#define RPM_START_STABLE                900.0f
#define THROTTLE_ZERO_DEADBAND_PCT      1.0f

// -----------------------------------------------------------------------------
// Hall sensor calibration.
// Defaults match the uploaded monolithic code: hall output enters a divider:
//   hall output -> 2.35k equivalent top -> ADC -> 4.7k bottom -> GND
// HALL_SENSOR_*_V are the hall voltages before the divider.
// -----------------------------------------------------------------------------
#define ADC_VREF                        3.3f
#define ADC_COUNTS_MAX                  4095.0f
#define HALL_DIVIDER_R_TOP_OHM          2350.0f
#define HALL_DIVIDER_R_BOTTOM_OHM       4700.0f
#define HALL_SENSOR_HIGH_V              0.26f
#define HALL_SENSOR_LOW_V               0.10f
#define HALL_PULSES_PER_REV             1.0f

// -----------------------------------------------------------------------------
// Thermistor defaults: 3.3V -- 10k fixed -- ADC -- thermistor -- GND
// Set TEMP_THERMISTOR_TO_GND to 0 if your divider is reversed.
// -----------------------------------------------------------------------------
#define TEMP_FIXED_RES_OHM              10000.0f
#define TEMP_R0_OHM                     10000.0f
#define TEMP_BETA                       3977.0f
#define TEMP_T0_K                       298.15f
#define TEMP_THERMISTOR_TO_GND          1

// -----------------------------------------------------------------------------
// Current/voltage sensor defaults.
// The board currently behaves as INA219-compatible:
//   bus register:   (raw >> 3) * 4 mV
//   shunt register: raw * 10 uV
// Current is computed directly from shunt voltage / resistor, so the INA219
// calibration register is not required for telemetry current.
// -----------------------------------------------------------------------------
#define CURRENT_SENSOR_ENABLED          1
#define INA219_I2C_ADDR                 0x40u
#define CURRENT_SHUNT_RES_OHM           0.050f
#define CURRENT_FILTER_ALPHA            0.20f

// Backward-compatible aliases for older files still using the previous names.
#define INA226_I2C_ADDR                 INA219_I2C_ADDR
#define INA226_SHUNT_RES_OHM            CURRENT_SHUNT_RES_OHM



// -----------------------------------------------------------------------------
// LED / buzzer status indication
// -----------------------------------------------------------------------------
// No FC link: quiet visual-only heartbeat.
#define INDICATOR_NO_FC_PERIOD_MS         1000u
#define INDICATOR_NO_FC_ON_MS             80u

// FC link present but disarmed/before arming: LED and buzzer pulse together.
#define INDICATOR_DISARMED_PERIOD_MS      1000u
#define INDICATOR_DISARMED_ON_MS          80u

// Kept for optional future pre-prime blink patterns. Current behavior in
// ENGINE_ARMED_WAIT_FOR_SPIN is continuous LED + continuous buzzer.
#define INDICATOR_PREPRIME_PERIOD_MS      500u
#define INDICATOR_PREPRIME_ON_MS          90u

// Active priming = spin detected and throttle held at THROTTLE_PRIME_US.
#define INDICATOR_PRIMING_PERIOD_MS       200u
#define INDICATOR_PRIMING_ON_MS           70u

// Servo test / fault patterns.
#define INDICATOR_SERVO_TEST_PERIOD_MS    100u
#define INDICATOR_SERVO_TEST_ON_MS        50u
#define INDICATOR_FAULT_PERIOD_MS         150u
#define INDICATOR_FAULT_ON_MS             75u

// -----------------------------------------------------------------------------
// Servo sweep test
// -----------------------------------------------------------------------------
#define SERVO_TEST_ENABLE_CAN_PARAM       1
#define SERVO_TEST_RAWCOMMAND_VALUE       (-8192) // optional fallback trigger on ESC RawCommand
#define SERVO_TEST_RELAY_SETTLE_MS        350u
#define SERVO_TEST_SWEEP_MS               2500u
#define SERVO_TEST_HOLD_END_MS            500u

// Throttle sweep uses the requested control range.
#define SERVO_TEST_THROTTLE_A_US          THROTTLE_IDLE_US
#define SERVO_TEST_THROTTLE_B_US          THROTTLE_MAX_US

// Choke sweep uses the configured mechanical range.
#define SERVO_TEST_CHOKE_A_US             CHOKE_US_OPEN
#define SERVO_TEST_CHOKE_B_US             CHOKE_US_CLOSED
#endif // BOARD_CONFIG_H
