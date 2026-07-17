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
#define FRAM_I2C_ADDR                   0x50u
#define FRAM_CONFIG_ADDR                0x0000u

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
#define DRONECAN_NODE_ID                42u   // legacy/fallback node ID from the original FC-only code
#define DRONECAN_NODE_ID_BASE           40u   // runtime node ID = base + saved small board identifier
#define DRONECAN_NODE_ID_MAX_OFFSET     80u   // only use saved board IDs 1..80 as node-ID offsets
#define DRONECAN_ESC_INDEX              0u    // legacy/fallback ESC index; runtime ESC index = saved board identifier - 1
#define DRONECAN_ESC_INDEX_MAX          31u   // uavcan.equipment.esc.Status.esc_index is 5 bits: 0..31
#define DRONECAN_FC_TIMEOUT_MS          1000u
#define NODE_STATUS_PERIOD_MS           1000u
#define ESC_STATUS_PERIOD_MS            100u

// Custom debug probe override. When a probe heartbeat is present, the
// controller ignores FC commands and only obeys the selected debug board.
#define CUSTOM_CAN_DEBUG_TIMEOUT_MS       1000u
#define CUSTOM_CAN_COMMAND_TIMEOUT_MS     1000u
#define CUSTOM_CAN_BOARD_ANNOUNCE_MS      500u
#define CUSTOM_CAN_TELEM_A_PERIOD_MS        50u
#define CUSTOM_CAN_TELEM_B_PERIOD_MS       100u
#define CUSTOM_CAN_TELEM_C_PERIOD_MS       250u
#define CUSTOM_CAN_AUTO_STATUS_PERIOD_MS   100u


// -----------------------------------------------------------------------------
// Servo outputs
// Note: this throttle servo/ESC opens as the PWM pulse gets smaller.
// -----------------------------------------------------------------------------
#define SERVO_FREQ_HZ                   50u
#define SERVO_US_HARD_MIN               1000u
#define SERVO_US_HARD_MAX               2000u

#define THROTTLE_IDLE_US                1850u   // 0% throttle / idle point
#define THROTTLE_START_US               1400u   // startup/prime throttle PWM; runtime-configurable and saved in FRAM
#define THROTTLE_MAX_US                 1450u   // 100% throttle feedforward point
#define THROTTLE_PRIME_US               THROTTLE_START_US // legacy alias

#define CHOKE_US_CLOSED                 1900u
#define CHOKE_US_OPEN                   1100u

// -----------------------------------------------------------------------------
// Requested throttle/RPM feedforward curve
//   0%   -> 1900 us -> 1800 RPM
//   100% -> 1450 us -> 4250 RPM
// PID starts from feedforward but may correct across the full 1000..2000 us servo range.
// -----------------------------------------------------------------------------
#define THROTTLE_IDLE_RPM               2200.0f
#define THROTTLE_MAX_RPM                4250.0f
#define RPM_PID_CORRECTION_LIMIT_US     1000.0f
#define RPM_PID_KP_US_PER_RPM           0.035f
#define RPM_PID_KI_US_PER_RPM_S         0.012f
#define RPM_PID_KD_US_PER_RPM_PER_S     0.000f

// Slow endpoint auto-tune used only from the debug GUI. The algorithm adjusts
// the feedforward servo opening rather than mapping throttle directly to RPM.
#define ENDPOINT_AUTO_TUNE_DEFAULT_RATE_US_PER_S  25.0f
#define ENDPOINT_AUTO_TUNE_DEADBAND_RPM           35.0f
#define ENDPOINT_AUTO_TUNE_GAIN_US_PER_RPM_S      0.020f
#define ENDPOINT_AUTO_TUNE_START_US               1700u
// This board's throttle opens as the servo PWM pulse gets smaller. Keep this
// explicit instead of inferring direction from the current endpoints; auto-tune
// deliberately starts both 0% and 100% searches near 1700 us, where endpoint
// ordering is not a reliable polarity signal. Use +1.0f on boards where a
// larger PWM pulse increases RPM.
#define ENDPOINT_AUTO_TUNE_RPM_UP_US_SIGN         (-1.0f)


// -----------------------------------------------------------------------------
// State-machine timing / thresholds
// -----------------------------------------------------------------------------
#define CONTROL_DT_MS                   10u
#define PRINT_LOOP_MS                   500u
#define START_HOLD_AFTER_RPM_MS         1000u   // hold startup PWM this long after first believable RPM; runtime-configurable
#define PRIME_AFTER_SPIN_MS             START_HOLD_AFTER_RPM_MS // legacy alias
#define RPM_DETECT_THRESHOLD            50.0f
#define RPM_ZERO_TIMEOUT_MS             2000u
#define RPM_STATIONARY_THRESH           120.0f
#define RPM_START_STABLE                900.0f
#define THROTTLE_ZERO_DEADBAND_PCT      1.0f

// -----------------------------------------------------------------------------
// Hall sensor calibration and sampling.
//
// The Hall pickup remains on ADC1. We do NOT treat it as a digital GPIO because
// the signal is analogue/small. Instead, sensors.c samples it periodically,
// detects threshold crossings with hysteresis, and timestamps accepted rising
// edges in microseconds.
//
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

// Hall ADC sampler period.
// 100 us = 10 kHz sampling. This makes RPM period timing far less quantized
// than the old ~1 ms main-loop polling.
#define HALL_SAMPLE_PERIOD_US           100u

// Ignore impossible double-triggers/bounce. 2000 us still permits up to
// 30,000 RPM at one pulse/rev, far above this engine's range.
#define HALL_MIN_EDGE_SPACING_US        2000u

// Raw Hall auto-calibration. The debug GUI starts this while an external
// spinner holds the engine at a known calibration RPM. The board powers the
// Hall circuit by arming the relay, samples raw ADC min/max, derives hysteresis
// thresholds, and persists them to FRAM.
// 0 duration means "run until clean/abort". Nonzero durations are treated as
// an optional safety timeout for CLI/backwards compatibility only.
#define HALL_AUTO_CAL_DEFAULT_DURATION_MS     0u
#define HALL_AUTO_CAL_MIN_DURATION_MS         0u
#define HALL_AUTO_CAL_MAX_DURATION_MS     60000u
#define HALL_AUTO_CAL_MIN_SPAN_RAW          20u

// Use wider hysteresis than the old 35/65 split. On your engines the raw span
// can be only ~100 counts; 25/75 leaves much more noise margin around center.
#define HALL_AUTO_CAL_LOW_FRACTION          0.35f
#define HALL_AUTO_CAL_HIGH_FRACTION         0.65f

// Windowed Hall auto-cal: the routine keeps running until it sees multiple
// consecutive clean windows at the requested spinner RPM.
#define HALL_AUTO_CAL_WINDOW_MS             1000u
#define HALL_AUTO_CAL_REQUIRED_CLEAN_WINDOWS   3u
#define HALL_AUTO_CAL_RPM_TOLERANCE_PCT       25.0f
#define HALL_AUTO_CAL_EDGE_RATIO_MIN           0.55f
#define HALL_AUTO_CAL_EDGE_RATIO_MAX           1.60f
#define HALL_AUTO_CAL_GOOD_EDGE_RATIO_MIN      0.80f
#define HALL_AUTO_CAL_STABILITY_MIN_RAW        8u
#define HALL_AUTO_CAL_STABILITY_FRACTION       0.18f
#define HALL_AUTO_CAL_MIN_SAMPLES_PER_WINDOW  500u

// Light RPM telemetry/control filtering. The Hall edge capture still uses raw
// timings, but the exported RPM is filtered to remove single-period jitter.
#define HALL_RPM_FILTER_ALPHA                 0.85f
#define HALL_RPM_OUTLIER_MAX_RATIO            0.45f


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
// Direct manual throttle-PWM test used by the debug GUI. This replaces the old
// servo sweep button; it powers the servo rail for a bounded time and commands
// exactly the requested microsecond value. It never substitutes a hard-coded fallback value.
#define MANUAL_PWM_TEST_DEFAULT_HOLD_MS   10000u
#define MANUAL_PWM_TEST_MAX_HOLD_MS       60000u

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