#include "sensors.h"
#include "board_config.h"

#include <math.h>
#include <string.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"

// -----------------------------------------------------------------------------
// Assumptions for the thermistor divider
//
// Default here:
//   3.3V --- 10k fixed --- ADC --- thermistor --- GND
//
// If your wiring is instead:
//   3.3V --- thermistor --- ADC --- 10k fixed --- GND
//
// set TEMP_THERMISTOR_TO_GND to 0.
// -----------------------------------------------------------------------------
#ifndef PIN_TEMP_ADC_GPIO
#define PIN_TEMP_ADC_GPIO            26      // ADC0
#endif

#ifndef TEMP_ADC_INPUT
#define TEMP_ADC_INPUT               0
#endif

#ifndef PIN_HALL_ADC_GPIO
#define PIN_HALL_ADC_GPIO            27      // ADC1
#endif

#ifndef HALL_ADC_INPUT
#define HALL_ADC_INPUT               1
#endif

#ifndef ADC_VREF
#define ADC_VREF                     3.3f
#endif

#ifndef TEMP_FIXED_RES_OHM
#define TEMP_FIXED_RES_OHM           10000.0f
#endif

#ifndef TEMP_R0_OHM
#define TEMP_R0_OHM                  10000.0f
#endif

#ifndef TEMP_BETA
#define TEMP_BETA                    3977.0f
#endif

#ifndef TEMP_T0_K
#define TEMP_T0_K                    298.15f
#endif

#ifndef TEMP_THERMISTOR_TO_GND
#define TEMP_THERMISTOR_TO_GND       1
#endif

#ifndef HALL_THRESHOLD_HIGH_RAW
#define HALL_THRESHOLD_HIGH_RAW      30000u
#endif

#ifndef HALL_THRESHOLD_LOW_RAW
#define HALL_THRESHOLD_LOW_RAW       22000u
#endif

#ifndef RPM_ZERO_TIMEOUT_MS
#define RPM_ZERO_TIMEOUT_MS          2000u
#endif

#ifndef RPM_STATIONARY_THRESH
#define RPM_STATIONARY_THRESH        120.0f
#endif

#ifndef RPM_START_STABLE
#define RPM_START_STABLE             900.0f
#endif

typedef struct {
    EngineSensors s;
    uint32_t last_hall_edge_ms;
    uint32_t last_update_ms;
    uint32_t runtime_ms;
    uint32_t last_temp_sample_ms;
} SensorState;

static SensorState g;

static float thermistor_raw_to_c(uint16_t raw)
{
    if (raw == 0) {
        raw = 1;
    }
    if (raw >= 4095) {
        raw = 4094;
    }

    const float v = ((float)raw / 4095.0f) * ADC_VREF;

    float r_therm = 0.0f;

#if TEMP_THERMISTOR_TO_GND
    // 3V3 -- Rfixed -- ADC -- Rtherm -- GND
    // V = Vref * Rtherm / (Rfixed + Rtherm)
    // Rtherm = Rfixed * V / (Vref - V)
    if ((ADC_VREF - v) < 1e-6f) {
        return -273.15f;
    }
    r_therm = TEMP_FIXED_RES_OHM * v / (ADC_VREF - v);
#else
    // 3V3 -- Rtherm -- ADC -- Rfixed -- GND
    // V = Vref * Rfixed / (Rfixed + Rtherm)
    // Rtherm = Rfixed * (Vref - V) / V
    if (v < 1e-6f) {
        return -273.15f;
    }
    r_therm = TEMP_FIXED_RES_OHM * (ADC_VREF - v) / v;
#endif

    const float inv_t =
        (1.0f / TEMP_T0_K) +
        (1.0f / TEMP_BETA) * logf(r_therm / TEMP_R0_OHM);

    const float t_k = 1.0f / inv_t;
    return t_k - 273.15f;
}

static uint16_t adc_read_avg(uint input, int n)
{
    uint32_t acc = 0;
    adc_select_input(input);
    for (int i = 0; i < n; i++) {
        acc += adc_read();
    }
    return (uint16_t)(acc / (uint32_t)n);
}

static void update_hall(uint32_t now_ms)
{
    const uint16_t raw = adc_read_avg(HALL_ADC_INPUT, 1);

    if (!g.s.hall_high && raw >= HALL_THRESHOLD_HIGH_RAW) {
        g.s.hall_high = true;
        g.s.turns_since_start++;

        if (g.last_hall_edge_ms != 0) {
            const uint32_t dt_ms = now_ms - g.last_hall_edge_ms;
            if (dt_ms > 0) {
                // assumes 1 pulse per revolution
                g.s.rpm = 60000.0f / (float)dt_ms;
            }
        }

        g.last_hall_edge_ms = now_ms;
    } else if (g.s.hall_high && raw <= HALL_THRESHOLD_LOW_RAW) {
        g.s.hall_high = false;
    }

    if (g.last_hall_edge_ms == 0 || (now_ms - g.last_hall_edge_ms) > RPM_ZERO_TIMEOUT_MS) {
        g.s.rpm = 0.0f;
    }

    g.s.stationary = (g.s.rpm < RPM_STATIONARY_THRESH);
}

static void update_temperature(uint32_t now_ms)
{
    if (g.last_temp_sample_ms != 0 && (now_ms - g.last_temp_sample_ms) < 20u) {
        return;
    }
    g.last_temp_sample_ms = now_ms;

    const uint16_t raw = adc_read_avg(TEMP_ADC_INPUT, 8);
    const float tc = thermistor_raw_to_c(raw);

    // light IIR filter so CAN telemetry is stable
    g.s.temperature_c = 0.85f * g.s.temperature_c + 0.15f * tc;
}

void sensors_init(void)
{
    memset(&g, 0, sizeof(g));

    adc_init();
    adc_gpio_init(PIN_TEMP_ADC_GPIO);
    adc_gpio_init(PIN_HALL_ADC_GPIO);

    g.s.rpm = 0.0f;
    g.s.temperature_c = 25.0f;
    g.s.current_a = 0.0f;
    g.s.turns_since_start = 0;
    g.s.stationary = true;
    g.s.hall_high = false;
}

void sensors_update(uint32_t now_ms)
{
    update_hall(now_ms);
    update_temperature(now_ms);

    // keep placeholder current unless you add a current sensor path
    g.s.current_a = 0.0f;

    if (g.last_update_ms != 0) {
        const uint32_t dt = now_ms - g.last_update_ms;
        if (g.s.rpm > RPM_START_STABLE) {
            g.runtime_ms += dt;
        }
    }
    g.last_update_ms = now_ms;
}

void sensors_snapshot(EngineSensors *out)
{
    *out = g.s;
}

void sensors_reset_start_counter(void)
{
    g.s.turns_since_start = 0;
}

uint32_t sensors_get_runtime_ms(void)
{
    return g.runtime_ms;
}