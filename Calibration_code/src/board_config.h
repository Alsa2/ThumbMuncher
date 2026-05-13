#pragma once

#include <stdint.h>

// -------------------------------
// DroneCAN node identity
// -------------------------------
#define DRONECAN_NODE_ID              42
#define DRONECAN_ESC_INDEX            0

// -------------------------------
// MCP2518FD / CAN timing
// -------------------------------
#define MCP_OSC_HZ                    20000000u
#define CAN_BITRATE_HZ                1000000u

// -------------------------------
// Board pins
// -------------------------------
#define PIN_CAN_SCK                   2
#define PIN_CAN_MOSI                  3
#define PIN_CAN_MISO                  4
#define PIN_CAN_CS                    5
#define PIN_CAN_INT                   6

#define PIN_RELAY                     8
#define PIN_I2C_SDA                   0
#define PIN_I2C_SCL                   1
#define PIN_CHOKE_SERVO               16
#define PIN_THROTTLE_SERVO            17
#define PIN_LED                       18
#define PIN_BUZZER                    19

#define PIN_TEMP_ADC                  26
#define PIN_HALL_ADC                  27
#define PIN_STARTER                   28   // Assumed starter/cranker output; change if different on your board

// -------------------------------
// INA219 current sensor
// -------------------------------
#define INA219_ADDR                   0x40
#define INA219_SHUNT_OHMS             0.05f

// -------------------------------
// Engine behaviour
// -------------------------------
#define RPM_IDLE                      500.0f
#define RPM_MAX                       6800.0f
#define RPM_START_STABLE              900.0f
#define RPM_STATIONARY_THRESH         120.0f

#define START_REQUEST_PCT             1.0f
#define CHOKE_TURNS_TO_OPEN           15u
#define CRANK_TIMEOUT_MS              8000u
#define STALL_TIMEOUT_MS              1500u

// -------------------------------
// Servo calibration
// -------------------------------
#define SERVO_FREQ_HZ                 50u
#define SERVO_US_MIN                  1000u
#define SERVO_US_MAX                  2000u
#define SERVO_US_IDLE_GUESS           1100u
#define SERVO_US_CHOKE_OPEN           1100u
#define SERVO_US_CHOKE_CLOSED         1900u

// -------------------------------
// Hall ADC thresholds (hysteresis)
// -------------------------------
#define HALL_HIGH_V                   1.80f
#define HALL_LOW_V                    1.40f
#define HALL_STATIONARY_TIMEOUT_MS    250u

// -------------------------------
// NTC model (10k pull-up, 10k beta 3977)
// -------------------------------
#define ADC_REF_V                     3.3f
#define THERM_PULLUP_OHMS             10000.0f
#define THERM_BETA                    3977.0f
#define THERM_R0                      10000.0f
#define THERM_T0_K                    298.15f

// -------------------------------
// PID gains (delta around SERVO_US_IDLE_GUESS)
// -------------------------------
#define PID_KP                        0.020f
#define PID_KI                        0.004f
#define PID_KD                        0.000f
#define PID_OUT_MIN_US                (-80.0f)
#define PID_OUT_MAX_US                (850.0f)

// -------------------------------
// Scheduler periods
// -------------------------------
#define CONTROL_DT_MS                 10u
#define TELEMETRY_DT_MS               100u
#define NODE_STATUS_DT_MS             1000u
#define HEARTBEAT_PRINT_DT_MS         1000u
#define FC_TIMEOUT_MS                 10000u


// TEMPERATURE PROBE
#define PIN_TEMP_ADC_GPIO          26
#define TEMP_ADC_INPUT             0

#define ADC_VREF                   3.3f
#define TEMP_FIXED_RES_OHM         10000.0f
#define TEMP_R0_OHM                10000.0f
#define TEMP_BETA                  3977.0f
#define TEMP_T0_K                  298.15f

// 1 = thermistor to GND, fixed 10k to 3V3
// 0 = thermistor to 3V3, fixed 10k to GND
#define TEMP_THERMISTOR_TO_GND     1
