#include "sensors.h"
#include "board_config.h"

#include <math.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#define INA219_REG_CONFIG          0x00u
#define INA219_REG_SHUNT_VOLTAGE   0x01u
#define INA219_REG_BUS_VOLTAGE     0x02u

#define INA219_CONFIG_VALUE        0x3FFFu
#define INA219_SHUNT_LSB_V         0.000010f
#define INA219_BUS_LSB_V           0.004f

typedef struct {
    EngineSensors s;

    // Hall sampler state written from the repeating-timer IRQ callback.
    volatile uint64_t hall_last_edge_us;
    volatile uint32_t hall_last_period_us;
    volatile uint32_t hall_turns_total;
    volatile bool hall_high_irq;

    // Main-context bookkeeping.
    uint32_t hall_turns_reset_offset;
    uint16_t hall_threshold_high_raw;
    uint16_t hall_threshold_low_raw;
    repeating_timer_t hall_timer;
    bool hall_timer_running;

    uint32_t last_update_ms;
    uint32_t runtime_ms;
    uint32_t last_temp_sample_ms;
    uint32_t last_current_sample_ms;
    bool current_checked;
    bool current_present;
} SensorState;

static SensorState g;

static float clampf_sensor(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t hall_sensor_voltage_to_raw(float hall_v)
{
    const float vadc =
        hall_v * (HALL_DIVIDER_R_BOTTOM_OHM /
                 (HALL_DIVIDER_R_TOP_OHM + HALL_DIVIDER_R_BOTTOM_OHM));

    const float vadc_clamped = clampf_sensor(vadc, 0.0f, ADC_VREF);
    const float rawf = (vadc_clamped / ADC_VREF) * ADC_COUNTS_MAX;
    return (uint16_t)(rawf + 0.5f);
}

static uint16_t hall_threshold_high_raw(void)
{
#ifdef HALL_THRESHOLD_HIGH_RAW
    return HALL_THRESHOLD_HIGH_RAW;
#else
    return hall_sensor_voltage_to_raw(HALL_SENSOR_HIGH_V);
#endif
}

static uint16_t hall_threshold_low_raw(void)
{
#ifdef HALL_THRESHOLD_LOW_RAW
    return HALL_THRESHOLD_LOW_RAW;
#else
    return hall_sensor_voltage_to_raw(HALL_SENSOR_LOW_V);
#endif
}

static float thermistor_raw_to_c(uint16_t raw)
{
    if (raw == 0u) {
        raw = 1u;
    }
    if (raw >= 4095u) {
        raw = 4094u;
    }

    const float v = ((float)raw / 4095.0f) * ADC_VREF;
    float r_therm = 0.0f;

#if TEMP_THERMISTOR_TO_GND
    // 3V3 -- Rfixed -- ADC -- Rtherm -- GND
    if ((ADC_VREF - v) < 1e-6f) {
        return -273.15f;
    }
    r_therm = TEMP_FIXED_RES_OHM * v / (ADC_VREF - v);
#else
    // 3V3 -- Rtherm -- ADC -- Rfixed -- GND
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

// ADC helper used from normal/main context. Interrupts are briefly disabled so
// the Hall repeating-timer callback cannot switch ADC channels mid-conversion.
static uint16_t adc_read_avg_main_context(uint input, int n)
{
    uint32_t acc = 0u;
    const uint32_t irq_state = save_and_disable_interrupts();

    adc_select_input(input);
    for (int i = 0; i < n; i++) {
        acc += adc_read();
    }

    restore_interrupts(irq_state);
    return (uint16_t)(acc / (uint32_t)n);
}

static void hall_accept_or_rearm_from_raw(uint16_t raw, uint64_t now_us)
{
    if (!g.hall_high_irq && raw >= g.hall_threshold_high_raw) {
        g.hall_high_irq = true;

        const uint64_t previous_edge_us = g.hall_last_edge_us;
        const uint64_t zero_timeout_us = (uint64_t)RPM_ZERO_TIMEOUT_MS * 1000ull;

        bool accept_edge = true;
        bool period_is_valid = false;
        uint32_t period_us = 0u;

        if (previous_edge_us != 0u) {
            const uint64_t dt_us_64 = now_us - previous_edge_us;

            // If the engine had been stopped long enough to time out, this is
            // the first edge of a new spin-up. Do not convert the stale gap into
            // a fake very-low RPM period.
            if (dt_us_64 > zero_timeout_us) {
                period_is_valid = false;
            }
            // Reject impossible fast double-triggers/bounce/noise.
            else if (dt_us_64 < (uint64_t)HALL_MIN_EDGE_SPACING_US) {
                accept_edge = false;
            }
            else if (dt_us_64 <= 0xFFFFFFFFull) {
                period_us = (uint32_t)dt_us_64;
                period_is_valid = true;
            }
        }

        if (accept_edge) {
            g.hall_turns_total++;
            g.hall_last_edge_us = now_us;
            g.hall_last_period_us = period_is_valid ? period_us : 0u;
        }
    } else if (g.hall_high_irq && raw <= g.hall_threshold_low_raw) {
        g.hall_high_irq = false;
    }
}

static bool hall_sample_timer_callback(repeating_timer_t *rt)
{
    (void)rt;

    // This callback runs in IRQ context on the default alarm pool's core.
    // It owns ADC channel switching whenever it runs; main-context ADC reads
    // briefly disable interrupts to avoid collisions.
    adc_select_input(HALL_ADC_INPUT);
    const uint16_t raw = adc_read();
    const uint64_t now_us = time_us_64();

    hall_accept_or_rearm_from_raw(raw, now_us);
    return true;
}

// Fallback used only if the repeating timer could not be created. It keeps the
// firmware functional, but it will not be as timing-clean as the timer sampler.
static void hall_sample_once_main_context(void)
{
    const uint16_t raw = adc_read_avg_main_context(HALL_ADC_INPUT, 1);
    const uint64_t now_us = time_us_64();

    const uint32_t irq_state = save_and_disable_interrupts();
    hall_accept_or_rearm_from_raw(raw, now_us);
    restore_interrupts(irq_state);
}

static void update_hall_from_captured_period(void)
{
    uint64_t last_edge_us = 0u;
    uint32_t last_period_us = 0u;
    uint32_t turns_total = 0u;
    bool hall_high = false;

    const uint32_t irq_state = save_and_disable_interrupts();
    last_edge_us = g.hall_last_edge_us;
    last_period_us = g.hall_last_period_us;
    turns_total = g.hall_turns_total;
    hall_high = g.hall_high_irq;
    restore_interrupts(irq_state);

    const uint64_t now_us = time_us_64();
    const uint64_t zero_timeout_us = (uint64_t)RPM_ZERO_TIMEOUT_MS * 1000ull;

    if (last_edge_us == 0u ||
        last_period_us == 0u ||
        (now_us - last_edge_us) > zero_timeout_us) {
        g.s.rpm = 0.0f;
    } else {
        g.s.rpm = 60000000.0f /
                  ((float)last_period_us * HALL_PULSES_PER_REV);
    }

    g.s.turns_since_start = turns_total - g.hall_turns_reset_offset;
    g.s.hall_high = hall_high;
    g.s.stationary = (g.s.rpm < RPM_STATIONARY_THRESH);
}

static void update_temperature(uint32_t now_ms)
{
    if (g.last_temp_sample_ms != 0u && (now_ms - g.last_temp_sample_ms) < 20u) {
        return;
    }
    g.last_temp_sample_ms = now_ms;

    const uint16_t raw = adc_read_avg_main_context(TEMP_ADC_INPUT, 8);
    const float tc = thermistor_raw_to_c(raw);

    // Light IIR filter so CAN telemetry is stable.
    g.s.temperature_c = 0.85f * g.s.temperature_c + 0.15f * tc;
}

#if CURRENT_SENSOR_ENABLED
static void current_i2c_bus_recover(void)
{
    i2c_deinit(CURRENT_I2C_PORT);

    gpio_set_function(PIN_CURRENT_I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_function(PIN_CURRENT_I2C_SCL, GPIO_FUNC_SIO);
    gpio_pull_up(PIN_CURRENT_I2C_SDA);
    gpio_pull_up(PIN_CURRENT_I2C_SCL);

    gpio_set_dir(PIN_CURRENT_I2C_SDA, GPIO_IN);
    gpio_set_dir(PIN_CURRENT_I2C_SCL, GPIO_OUT);
    gpio_put(PIN_CURRENT_I2C_SCL, 1);
    sleep_us(5);

    for (int i = 0; i < 9; i++) {
        gpio_put(PIN_CURRENT_I2C_SCL, 0);
        sleep_us(5);
        gpio_put(PIN_CURRENT_I2C_SCL, 1);
        sleep_us(5);
    }

    gpio_set_dir(PIN_CURRENT_I2C_SDA, GPIO_OUT);
    gpio_put(PIN_CURRENT_I2C_SDA, 0);
    sleep_us(5);
    gpio_put(PIN_CURRENT_I2C_SCL, 1);
    sleep_us(5);
    gpio_set_dir(PIN_CURRENT_I2C_SDA, GPIO_IN);
    sleep_us(5);
}

static bool current_read_reg(uint8_t reg, uint16_t *out)
{
    int wr = i2c_write_timeout_us(CURRENT_I2C_PORT,
                                  INA219_I2C_ADDR,
                                  &reg,
                                  1,
                                  true,
                                  CURRENT_I2C_TIMEOUT_US);
    if (wr != 1) {
        return false;
    }

    uint8_t rx[2] = {0u, 0u};
    int rd = i2c_read_timeout_us(CURRENT_I2C_PORT,
                                 INA219_I2C_ADDR,
                                 rx,
                                 sizeof(rx),
                                 false,
                                 CURRENT_I2C_TIMEOUT_US);
    if (rd != (int)sizeof(rx)) {
        return false;
    }

    *out = ((uint16_t)rx[0] << 8) | rx[1];
    return true;
}

static bool current_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFFu),
    };

    int wr = i2c_write_timeout_us(CURRENT_I2C_PORT,
                                  INA219_I2C_ADDR,
                                  buf,
                                  sizeof(buf),
                                  false,
                                  CURRENT_I2C_TIMEOUT_US);
    return wr == (int)sizeof(buf);
}

static void current_sensor_init_if_needed(void)
{
    if (g.current_checked) {
        return;
    }

    g.current_checked = true;

    if (current_write_reg(INA219_REG_CONFIG, INA219_CONFIG_VALUE)) {
        uint16_t cfg = 0u;
        g.current_present = current_read_reg(INA219_REG_CONFIG, &cfg);
    } else {
        g.current_present = false;
    }

    g.s.current_sensor_present = g.current_present;
}

static void update_current(uint32_t now_ms)
{
    if (g.last_current_sample_ms != 0u && (now_ms - g.last_current_sample_ms) < 20u) {
        return;
    }
    g.last_current_sample_ms = now_ms;

    current_sensor_init_if_needed();
    if (!g.current_present) {
        g.s.current_a = 0.0f;
        g.s.bus_voltage_v = 0.0f;
        g.s.current_sensor_present = false;
        return;
    }

    uint16_t shunt_raw_u = 0u;
    uint16_t bus_raw = 0u;

    if (!current_read_reg(INA219_REG_SHUNT_VOLTAGE, &shunt_raw_u) ||
        !current_read_reg(INA219_REG_BUS_VOLTAGE, &bus_raw)) {
        g.current_present = false;
        g.s.current_sensor_present = false;
        g.s.current_a = 0.0f;
        g.s.bus_voltage_v = 0.0f;
        return;
    }

    const int16_t shunt_raw = (int16_t)shunt_raw_u;
    const float shunt_v = (float)shunt_raw * INA219_SHUNT_LSB_V;
    const float current_a = shunt_v / CURRENT_SHUNT_RES_OHM;

    const uint16_t bus_code = bus_raw >> 3;
    const float bus_voltage_v = (float)bus_code * INA219_BUS_LSB_V;

    g.s.current_a = (1.0f - CURRENT_FILTER_ALPHA) * g.s.current_a +
                    CURRENT_FILTER_ALPHA * current_a;
    g.s.bus_voltage_v = (1.0f - CURRENT_FILTER_ALPHA) * g.s.bus_voltage_v +
                        CURRENT_FILTER_ALPHA * bus_voltage_v;
    g.s.current_sensor_present = true;
}
#else
static void update_current(uint32_t now_ms)
{
    (void)now_ms;
    g.s.current_a = 0.0f;
    g.s.bus_voltage_v = 0.0f;
    g.s.current_sensor_present = false;
}
#endif

void sensors_init(void)
{
    memset(&g, 0, sizeof(g));

    adc_init();
    adc_gpio_init(PIN_TEMP_ADC_GPIO);
    adc_gpio_init(PIN_HALL_ADC_GPIO);

    g.hall_threshold_high_raw = hall_threshold_high_raw();
    g.hall_threshold_low_raw = hall_threshold_low_raw();

#if CURRENT_SENSOR_ENABLED
    // Preserve the original I2C1 lane for FRAM / existing peripherals.
    i2c_init(FRAM_I2C_PORT, FRAM_I2C_BAUD_HZ);
    gpio_set_function(PIN_FRAM_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_FRAM_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_FRAM_I2C_SDA);
    gpio_pull_up(PIN_FRAM_I2C_SCL);

    // Current/voltage sensor is on the separate I2C0 lane.
    current_i2c_bus_recover();
    i2c_init(CURRENT_I2C_PORT, CURRENT_I2C_BAUD_HZ);
    gpio_set_function(PIN_CURRENT_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_CURRENT_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_CURRENT_I2C_SDA);
    gpio_pull_up(PIN_CURRENT_I2C_SCL);
#endif

    g.s.rpm = 0.0f;
    g.s.temperature_c = 25.0f;
    g.s.current_a = 0.0f;
    g.s.bus_voltage_v = 0.0f;
    g.s.turns_since_start = 0u;
    g.s.stationary = true;
    g.s.hall_high = false;
    g.s.current_sensor_present = false;

    // Negative delay = fixed time between callback starts, which gives a more
    // stable ADC sampling cadence than waiting HALL_SAMPLE_PERIOD_US after the
    // previous callback finishes.
    g.hall_timer_running = add_repeating_timer_us(
        -((int64_t)HALL_SAMPLE_PERIOD_US),
        hall_sample_timer_callback,
        NULL,
        &g.hall_timer);
}

void sensors_update(uint32_t now_ms)
{
    if (!g.hall_timer_running) {
        hall_sample_once_main_context();
    }

    update_hall_from_captured_period();
    update_temperature(now_ms);
    update_current(now_ms);

    if (g.last_update_ms != 0u) {
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
    const uint32_t irq_state = save_and_disable_interrupts();
    g.hall_turns_reset_offset = g.hall_turns_total;
    restore_interrupts(irq_state);

    g.s.turns_since_start = 0u;
}

uint32_t sensors_get_runtime_ms(void)
{
    return g.runtime_ms;
}