#include "sensors.h"
#include "board_config.h"

#include <math.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "custom_can_protocol.h"

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
    volatile uint32_t hall_period_filtered_us_irq;

    // Main-context bookkeeping.
    uint32_t hall_turns_reset_offset;
    uint16_t hall_threshold_high_raw;
    uint16_t hall_threshold_low_raw;
    uint16_t hall_cal_min_raw;
    uint16_t hall_cal_max_raw;
    repeating_timer_t hall_timer;
    bool hall_timer_running;

    volatile bool hall_auto_cal_active_irq;
    volatile uint16_t hall_auto_min_raw_irq;
    volatile uint16_t hall_auto_max_raw_irq;
    volatile uint32_t hall_auto_sample_count_irq;

    // Windowed Hall auto-cal state, written in the sampler IRQ and evaluated in
    // main context. The dynamic thresholds are updated once per clean-up window
    // so the calibration does not depend on the previously saved Hall settings.
    volatile uint16_t hall_auto_dyn_high_irq;
    volatile uint16_t hall_auto_dyn_low_irq;
    volatile bool hall_auto_dyn_high_state_irq;
    volatile uint64_t hall_auto_dyn_last_edge_us_irq;
    volatile uint32_t hall_auto_edge_count_irq;
    volatile uint32_t hall_auto_good_edge_count_irq;
    volatile uint32_t hall_auto_bad_edge_count_irq;
    volatile uint32_t hall_auto_period_min_us_irq;
    volatile uint32_t hall_auto_period_max_us_irq;
    volatile uint64_t hall_auto_period_sum_us_irq;

    // Robust raw endpoint estimation for Hall auto-calibration. Each window is
    // split around a moving midpoint; low/high plateau averages are used instead
    // of absolute min/max so one ADC spike does not poison the thresholds.
    volatile uint16_t hall_auto_classify_mid_irq;
    volatile uint32_t hall_auto_low_count_irq;
    volatile uint32_t hall_auto_high_count_irq;
    volatile uint64_t hall_auto_low_sum_irq;
    volatile uint64_t hall_auto_high_sum_irq;

    bool hall_auto_cal_done;
    bool hall_auto_cal_ok;
    uint8_t hall_auto_cal_status_code;
    uint8_t hall_auto_quality_pct;
    uint8_t hall_auto_clean_windows;
    bool hall_auto_have_candidate;
    uint32_t hall_auto_cal_start_ms;
    uint32_t hall_auto_cal_duration_ms;
    uint32_t hall_auto_last_eval_ms;
    float hall_auto_cal_target_rpm;
    volatile uint32_t hall_auto_expected_period_min_us;
    volatile uint32_t hall_auto_expected_period_max_us;
    uint16_t hall_auto_candidate_min_raw;
    uint16_t hall_auto_candidate_max_raw;
    uint32_t hall_auto_clean_min_sum;
    uint32_t hall_auto_clean_max_sum;
    uint16_t hall_auto_result_min_raw;
    uint16_t hall_auto_result_max_raw;
    uint16_t hall_auto_result_high_raw;
    uint16_t hall_auto_result_low_raw;

    float hall_rpm_filtered;

    uint32_t last_update_ms;
    uint32_t runtime_ms;
    uint32_t last_temp_sample_ms;
    uint32_t last_current_sample_ms;
    bool current_checked;
    bool current_present;
    bool thresholds_dirty;
} SensorState;

static SensorState g;

typedef struct {
    uint16_t min_raw;
    uint16_t max_raw;
    uint32_t sample_count;
    uint32_t edge_count;
    uint32_t good_edge_count;
    uint32_t bad_edge_count;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint64_t period_sum_us;
    uint32_t low_count;
    uint32_t high_count;
    uint64_t low_sum;
    uint64_t high_sum;
} HallAutoCalWindowSnapshot;


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

static void hall_auto_cal_track_raw_irq(uint16_t raw, uint64_t now_us)
{
    if (!g.hall_auto_cal_active_irq) {
        return;
    }

    if (raw < g.hall_auto_min_raw_irq) {
        g.hall_auto_min_raw_irq = raw;
    }
    if (raw > g.hall_auto_max_raw_irq) {
        g.hall_auto_max_raw_irq = raw;
    }
    g.hall_auto_sample_count_irq++;

    const uint16_t mid = g.hall_auto_classify_mid_irq;
    if (raw <= mid) {
        g.hall_auto_low_count_irq++;
        g.hall_auto_low_sum_irq += raw;
    } else {
        g.hall_auto_high_count_irq++;
        g.hall_auto_high_sum_irq += raw;
    }

    const uint16_t high = g.hall_auto_dyn_high_irq;
    const uint16_t low = g.hall_auto_dyn_low_irq;
    if (high <= low || high > 4095u) {
        return;
    }

    if (!g.hall_auto_dyn_high_state_irq && raw >= high) {
        g.hall_auto_dyn_high_state_irq = true;

        const uint64_t prev = g.hall_auto_dyn_last_edge_us_irq;
        if (prev != 0u) {
            const uint64_t period64 = now_us - prev;
            if (period64 >= (uint64_t)HALL_MIN_EDGE_SPACING_US && period64 <= 0xFFFFFFFFull) {
                const uint32_t period = (uint32_t)period64;
                g.hall_auto_edge_count_irq++;
                g.hall_auto_period_sum_us_irq += period;
                if (period < g.hall_auto_period_min_us_irq) {
                    g.hall_auto_period_min_us_irq = period;
                }
                if (period > g.hall_auto_period_max_us_irq) {
                    g.hall_auto_period_max_us_irq = period;
                }
                if (period >= g.hall_auto_expected_period_min_us &&
                    period <= g.hall_auto_expected_period_max_us) {
                    g.hall_auto_good_edge_count_irq++;
                } else {
                    g.hall_auto_bad_edge_count_irq++;
                }
            }
        }
        g.hall_auto_dyn_last_edge_us_irq = now_us;
    } else if (g.hall_auto_dyn_high_state_irq && raw <= low) {
        g.hall_auto_dyn_high_state_irq = false;
    }
}

static void hall_auto_cal_reset_window_irq(uint16_t dyn_high, uint16_t dyn_low, uint16_t classify_mid, bool reset_edge_state)
{
    g.hall_auto_min_raw_irq = 4095u;
    g.hall_auto_max_raw_irq = 0u;
    g.hall_auto_sample_count_irq = 0u;
    g.hall_auto_edge_count_irq = 0u;
    g.hall_auto_good_edge_count_irq = 0u;
    g.hall_auto_bad_edge_count_irq = 0u;
    g.hall_auto_period_min_us_irq = UINT32_MAX;
    g.hall_auto_period_max_us_irq = 0u;
    g.hall_auto_period_sum_us_irq = 0u;
    g.hall_auto_low_count_irq = 0u;
    g.hall_auto_high_count_irq = 0u;
    g.hall_auto_low_sum_irq = 0u;
    g.hall_auto_high_sum_irq = 0u;
    g.hall_auto_dyn_high_irq = dyn_high;
    g.hall_auto_dyn_low_irq = dyn_low;
    g.hall_auto_classify_mid_irq = classify_mid;
    if (reset_edge_state) {
        g.hall_auto_dyn_high_state_irq = false;
        g.hall_auto_dyn_last_edge_us_irq = 0u;
    }
}

static HallAutoCalWindowSnapshot hall_auto_cal_snapshot_and_reset(uint16_t next_high, uint16_t next_low, uint16_t next_mid, bool reset_edge_state)
{
    HallAutoCalWindowSnapshot w;
    const uint32_t irq_state = save_and_disable_interrupts();
    w.min_raw = g.hall_auto_min_raw_irq;
    w.max_raw = g.hall_auto_max_raw_irq;
    w.sample_count = g.hall_auto_sample_count_irq;
    w.edge_count = g.hall_auto_edge_count_irq;
    w.good_edge_count = g.hall_auto_good_edge_count_irq;
    w.bad_edge_count = g.hall_auto_bad_edge_count_irq;
    w.period_min_us = (g.hall_auto_period_min_us_irq == UINT32_MAX) ? 0u : g.hall_auto_period_min_us_irq;
    w.period_max_us = g.hall_auto_period_max_us_irq;
    w.period_sum_us = g.hall_auto_period_sum_us_irq;
    w.low_count = g.hall_auto_low_count_irq;
    w.high_count = g.hall_auto_high_count_irq;
    w.low_sum = g.hall_auto_low_sum_irq;
    w.high_sum = g.hall_auto_high_sum_irq;
    hall_auto_cal_reset_window_irq(next_high, next_low, next_mid, reset_edge_state);
    restore_interrupts(irq_state);
    return w;
}

static bool hall_period_plausible_for_filter(uint32_t period_us, uint32_t filtered_us)
{
    if (filtered_us == 0u) {
        return true;
    }

    // Reject one-sample spikes and missed-pulse outliers. Keep the window wide:
    // real RPM changes still get through, but contact/noise double-triggers do not.
    const uint64_t p100 = (uint64_t)period_us * 100ull;
    const uint64_t f = (uint64_t)filtered_us;
    return p100 >= (f * 55ull) && p100 <= (f * 180ull);
}

static void hall_accept_or_rearm_from_raw(uint16_t raw, uint64_t now_us)
{
    if (!g.hall_high_irq && raw >= g.hall_threshold_high_raw) {
        const uint64_t previous_edge_us = g.hall_last_edge_us;
        const uint64_t zero_timeout_us = (uint64_t)RPM_ZERO_TIMEOUT_MS * 1000ull;

        bool accept_edge = true;
        bool period_is_valid = false;
        uint32_t period_us = 0u;

        if (previous_edge_us != 0u) {
            const uint64_t dt_us_64 = now_us - previous_edge_us;

            if (dt_us_64 > zero_timeout_us) {
                // First edge after a stop. Accept the edge but reset the period filter.
                g.hall_period_filtered_us_irq = 0u;
                period_is_valid = false;
            } else if (dt_us_64 < (uint64_t)HALL_MIN_EDGE_SPACING_US) {
                accept_edge = false;
            } else if (dt_us_64 <= 0xFFFFFFFFull) {
                period_us = (uint32_t)dt_us_64;
                const uint32_t filtered = g.hall_period_filtered_us_irq;
                if (!hall_period_plausible_for_filter(period_us, filtered)) {
                    accept_edge = false;
                } else {
                    period_is_valid = true;
                }
            }
        }

        if (accept_edge) {
            g.hall_high_irq = true;
            g.hall_turns_total++;
            g.hall_last_edge_us = now_us;

            if (period_is_valid) {
                uint32_t filtered = g.hall_period_filtered_us_irq;
                if (filtered == 0u) {
                    filtered = period_us;
                } else {
                    filtered = (uint32_t)(((uint64_t)filtered * 7ull + (uint64_t)period_us) / 8ull);
                }
                g.hall_period_filtered_us_irq = filtered;
                g.hall_last_period_us = filtered;
            } else {
                g.hall_last_period_us = 0u;
            }
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

    hall_auto_cal_track_raw_irq(raw, now_us);
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
    hall_auto_cal_track_raw_irq(raw, now_us);
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
        g.hall_rpm_filtered = 0.0f;
    g.hall_period_filtered_us_irq = 0u;
        g.s.rpm = 0.0f;
    } else {
        const float instant_rpm = 60000000.0f /
                                  ((float)last_period_us * HALL_PULSES_PER_REV);
        if (g.hall_rpm_filtered <= 0.0f) {
            g.hall_rpm_filtered = instant_rpm;
        } else {
            const float diff = fabsf(instant_rpm - g.hall_rpm_filtered);
            const float ratio = diff / fmaxf(g.hall_rpm_filtered, 1.0f);
            const float alpha = (ratio > HALL_RPM_OUTLIER_MAX_RATIO) ? 0.90f : HALL_RPM_FILTER_ALPHA;
            g.hall_rpm_filtered = alpha * g.hall_rpm_filtered + (1.0f - alpha) * instant_rpm;
        }
        g.s.rpm = g.hall_rpm_filtered;
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


bool sensors_hall_thresholds_are_valid(uint16_t high_raw, uint16_t low_raw)
{
    return high_raw <= 4095u && low_raw <= 4095u && high_raw > low_raw;
}

bool sensors_apply_hall_thresholds_raw(uint16_t high_raw, uint16_t low_raw)
{
    if (!sensors_hall_thresholds_are_valid(high_raw, low_raw)) {
        return false;
    }
    const uint32_t irq_state = save_and_disable_interrupts();
    g.hall_threshold_high_raw = high_raw;
    g.hall_threshold_low_raw = low_raw;
    g.hall_high_irq = false;
    g.hall_period_filtered_us_irq = 0u;
    g.hall_last_period_us = 0u;
    restore_interrupts(irq_state);
    g.thresholds_dirty = false;
    return true;
}

bool sensors_set_hall_thresholds_raw(uint16_t high_raw, uint16_t low_raw)
{
    if (!sensors_apply_hall_thresholds_raw(high_raw, low_raw)) {
        return false;
    }
    g.thresholds_dirty = true;
    return true;
}

void sensors_get_hall_thresholds_raw(uint16_t *high_raw, uint16_t *low_raw)
{
    const uint32_t irq_state = save_and_disable_interrupts();
    const uint16_t high = g.hall_threshold_high_raw;
    const uint16_t low = g.hall_threshold_low_raw;
    restore_interrupts(irq_state);
    if (high_raw != NULL) *high_raw = high;
    if (low_raw != NULL) *low_raw = low;
}

bool sensors_consume_hall_thresholds_dirty(void)
{
    const bool dirty = g.thresholds_dirty;
    g.thresholds_dirty = false;
    return dirty;
}

bool sensors_hall_calibration_is_valid(uint16_t min_raw, uint16_t max_raw)
{
    return min_raw <= 4095u &&
           max_raw <= 4095u &&
           max_raw > min_raw &&
           (uint16_t)(max_raw - min_raw) >= (uint16_t)HALL_AUTO_CAL_MIN_SPAN_RAW;
}

static void hall_thresholds_from_min_max(uint16_t min_raw, uint16_t max_raw, uint16_t *high_raw, uint16_t *low_raw)
{
    const float span = (float)(max_raw - min_raw);
    uint16_t low = (uint16_t)((float)min_raw + span * HALL_AUTO_CAL_LOW_FRACTION + 0.5f);
    uint16_t high = (uint16_t)((float)min_raw + span * HALL_AUTO_CAL_HIGH_FRACTION + 0.5f);

    if (high <= low) {
        high = (low < 4095u) ? (uint16_t)(low + 1u) : 4095u;
        if (high <= low && low > 0u) {
            low--;
        }
    }
    if (high_raw != NULL) *high_raw = high;
    if (low_raw != NULL) *low_raw = low;
}

static uint16_t hall_midpoint_raw(uint16_t low_raw, uint16_t high_raw)
{
    return (uint16_t)(((uint32_t)low_raw + (uint32_t)high_raw) / 2u);
}

static bool hall_auto_window_plateaus(const HallAutoCalWindowSnapshot *w,
                                      uint16_t *low_mean_raw,
                                      uint16_t *high_mean_raw)
{
    if (w == NULL || w->sample_count == 0u) {
        return false;
    }

    // Require both sides of the waveform to appear. A very unbalanced duty cycle
    // is still allowed, but not a window made entirely of noise on one side.
    const uint32_t min_side = w->sample_count / 20u;  // 5% of the window.
    if (w->low_count < min_side || w->high_count < min_side ||
        w->low_count == 0u || w->high_count == 0u) {
        return false;
    }

    const uint16_t low = (uint16_t)((w->low_sum + (w->low_count / 2u)) / w->low_count);
    const uint16_t high = (uint16_t)((w->high_sum + (w->high_count / 2u)) / w->high_count);
    if (!sensors_hall_calibration_is_valid(low, high)) {
        return false;
    }

    if (low_mean_raw != NULL) *low_mean_raw = low;
    if (high_mean_raw != NULL) *high_mean_raw = high;
    return true;
}

bool sensors_apply_hall_calibration_raw(uint16_t min_raw, uint16_t max_raw)
{
    if (!sensors_hall_calibration_is_valid(min_raw, max_raw)) {
        return false;
    }
    const uint32_t irq_state = save_and_disable_interrupts();
    g.hall_cal_min_raw = min_raw;
    g.hall_cal_max_raw = max_raw;
    restore_interrupts(irq_state);
    return true;
}

void sensors_get_hall_calibration_raw(uint16_t *min_raw, uint16_t *max_raw)
{
    const uint32_t irq_state = save_and_disable_interrupts();
    const uint16_t minv = g.hall_cal_min_raw;
    const uint16_t maxv = g.hall_cal_max_raw;
    restore_interrupts(irq_state);
    if (min_raw != NULL) *min_raw = minv;
    if (max_raw != NULL) *max_raw = maxv;
}

static uint32_t hall_auto_expected_edges_per_window(void)
{
    const float edges = (g.hall_auto_cal_target_rpm * HALL_PULSES_PER_REV *
                         (float)HALL_AUTO_CAL_WINDOW_MS) / 60000.0f;
    uint32_t rounded = (uint32_t)(edges + 0.5f);
    return (rounded == 0u) ? 1u : rounded;
}

static uint16_t hall_auto_stability_limit_raw(uint16_t span)
{
    uint16_t by_fraction = (uint16_t)((float)span * HALL_AUTO_CAL_STABILITY_FRACTION + 0.5f);
    if (by_fraction < (uint16_t)HALL_AUTO_CAL_STABILITY_MIN_RAW) {
        by_fraction = (uint16_t)HALL_AUTO_CAL_STABILITY_MIN_RAW;
    }
    return by_fraction;
}

static bool hall_auto_window_rpm_is_reasonable(const HallAutoCalWindowSnapshot *w)
{
    if (w == NULL || w->edge_count < 2u || w->period_sum_us == 0u) {
        return false;
    }

    const float avg_period_us = (float)w->period_sum_us / (float)w->edge_count;
    if (avg_period_us <= 0.0f) {
        return false;
    }

    const float rpm = 60000000.0f / (avg_period_us * HALL_PULSES_PER_REV);
    const float tol = HALL_AUTO_CAL_RPM_TOLERANCE_PCT / 100.0f;
    return rpm >= g.hall_auto_cal_target_rpm * (1.0f - tol) &&
           rpm <= g.hall_auto_cal_target_rpm * (1.0f + tol);
}

static bool hall_auto_window_jitter_is_clean(const HallAutoCalWindowSnapshot *w)
{
    if (w == NULL || w->edge_count < 3u || w->period_sum_us == 0u || w->period_min_us == 0u) {
        return false;
    }

    const float avg = (float)w->period_sum_us / (float)w->edge_count;
    if (avg <= 0.0f) {
        return false;
    }

    // A steady external spinner should produce very similar periods. This is
    // intentionally looser than target-RPM matching; it mainly rejects noisy
    // extra edges caused by bad thresholds.
    return (float)w->period_min_us >= avg * 0.55f &&
           (float)w->period_max_us <= avg * 1.80f;
}

static uint8_t hall_auto_window_quality_pct(const HallAutoCalWindowSnapshot *w,
                                            bool span_ok,
                                            bool edge_seen,
                                            bool stable_ok,
                                            bool jitter_ok,
                                            bool rpm_ok)
{
    if (w == NULL || w->sample_count < HALL_AUTO_CAL_MIN_SAMPLES_PER_WINDOW) {
        return 0u;
    }
    uint8_t q = 15u;
    if (span_ok) q = 40u;
    if (span_ok && edge_seen) q = 60u;
    if (span_ok && edge_seen && stable_ok) q = 75u;
    if (span_ok && edge_seen && stable_ok && jitter_ok) q = 90u;
    if (span_ok && edge_seen && stable_ok && jitter_ok && rpm_ok) q = 100u;
    return q;
}

static void hall_auto_finish_success(uint16_t min_raw, uint16_t max_raw)
{
    uint16_t high_raw = 0u;
    uint16_t low_raw = 0u;
    hall_thresholds_from_min_max(min_raw, max_raw, &high_raw, &low_raw);

    const uint32_t irq_state = save_and_disable_interrupts();
    g.hall_auto_cal_active_irq = false;
    restore_interrupts(irq_state);

    g.hall_auto_cal_done = true;
    g.hall_auto_cal_ok = true;
    g.hall_auto_cal_status_code = CUSTOM_CAN_HALL_CAL_STATUS_DONE_OK;
    g.hall_auto_quality_pct = 100u;
    g.hall_auto_result_min_raw = min_raw;
    g.hall_auto_result_max_raw = max_raw;
    g.hall_auto_result_high_raw = high_raw;
    g.hall_auto_result_low_raw = low_raw;

    if (sensors_apply_hall_thresholds_raw(high_raw, low_raw)) {
        (void)sensors_apply_hall_calibration_raw(min_raw, max_raw);
        g.thresholds_dirty = true;
    } else {
        g.hall_auto_cal_ok = false;
        g.hall_auto_cal_status_code = CUSTOM_CAN_HALL_CAL_STATUS_FAILED;
    }
}

bool sensors_start_hall_auto_cal(float target_rpm, uint32_t duration_ms, uint32_t now_ms)
{
    if (!isfinite(target_rpm) || target_rpm <= 0.0f || target_rpm > 50000.0f) {
        return false;
    }
    if (duration_ms > HALL_AUTO_CAL_MAX_DURATION_MS) {
        return false;
    }

    const float expected_period_f = 60000000.0f / (target_rpm * HALL_PULSES_PER_REV);
    const float tol = HALL_AUTO_CAL_RPM_TOLERANCE_PCT / 100.0f;
    uint32_t min_period = (uint32_t)(expected_period_f * (1.0f - tol) + 0.5f);
    uint32_t max_period = (uint32_t)(expected_period_f * (1.0f + tol) + 0.5f);
    if (min_period < HALL_MIN_EDGE_SPACING_US) {
        min_period = HALL_MIN_EDGE_SPACING_US;
    }
    if (max_period <= min_period) {
        max_period = min_period + 1u;
    }

    g.hall_auto_cal_done = false;
    g.hall_auto_cal_ok = false;
    g.hall_auto_cal_status_code = CUSTOM_CAN_HALL_CAL_STATUS_RUNNING;
    g.hall_auto_quality_pct = 0u;
    g.hall_auto_clean_windows = 0u;
    g.hall_auto_have_candidate = false;
    g.hall_auto_cal_start_ms = now_ms;
    g.hall_auto_cal_duration_ms = duration_ms;  // 0 = run until clean or abort.
    g.hall_auto_last_eval_ms = now_ms;
    g.hall_auto_cal_target_rpm = target_rpm;
    g.hall_auto_expected_period_min_us = min_period;
    g.hall_auto_expected_period_max_us = max_period;
    g.hall_auto_candidate_min_raw = 0u;
    g.hall_auto_candidate_max_raw = 0u;
    g.hall_auto_clean_min_sum = 0u;
    g.hall_auto_clean_max_sum = 0u;
    g.hall_auto_result_min_raw = 4095u;
    g.hall_auto_result_max_raw = 0u;
    g.hall_auto_result_high_raw = g.hall_threshold_high_raw;
    g.hall_auto_result_low_raw = g.hall_threshold_low_raw;

    const uint16_t start_mid = hall_midpoint_raw(g.hall_threshold_low_raw, g.hall_threshold_high_raw);
    const uint32_t irq_state = save_and_disable_interrupts();
    g.hall_auto_cal_active_irq = true;
    g.hall_auto_expected_period_min_us = min_period;
    g.hall_auto_expected_period_max_us = max_period;
    hall_auto_cal_reset_window_irq(g.hall_threshold_high_raw, g.hall_threshold_low_raw, start_mid, true);
    restore_interrupts(irq_state);
    return true;
}

bool sensors_update_hall_auto_cal(uint32_t now_ms)
{
    if (!g.hall_auto_cal_active_irq) {
        return false;
    }

    const uint32_t elapsed_window = now_ms - g.hall_auto_last_eval_ms;
    if (elapsed_window < HALL_AUTO_CAL_WINDOW_MS) {
        return false;
    }
    g.hall_auto_last_eval_ms = now_ms;

    // Evaluate one raw-data window, then immediately update the next window's
    // thresholds from robust low/high plateau averages. This avoids using one
    // noisy ADC spike as the Hall "min" or "max".
    const uint16_t current_mid = hall_midpoint_raw(g.hall_auto_result_low_raw,
                                                   g.hall_auto_result_high_raw);
    HallAutoCalWindowSnapshot w = hall_auto_cal_snapshot_and_reset(
        g.hall_auto_result_high_raw,
        g.hall_auto_result_low_raw,
        current_mid,
        false);

    if (w.sample_count < HALL_AUTO_CAL_MIN_SAMPLES_PER_WINDOW || w.max_raw <= w.min_raw) {
        g.hall_auto_clean_windows = 0u;
        g.hall_auto_quality_pct = 0u;
        return false;
    }

    const bool raw_span_ok = sensors_hall_calibration_is_valid(w.min_raw, w.max_raw);

    uint16_t low_endpoint = 0u;
    uint16_t high_endpoint = 0u;
    const bool plateaus_ok = hall_auto_window_plateaus(&w, &low_endpoint, &high_endpoint);

    // First clean-ish window may not classify correctly if the old thresholds
    // were bad. Use raw extrema to move the midpoint once, then use plateau
    // means on later windows.
    if (!plateaus_ok && raw_span_ok) {
        low_endpoint = w.min_raw;
        high_endpoint = w.max_raw;
    }

    const bool span_ok = sensors_hall_calibration_is_valid(low_endpoint, high_endpoint);
    uint16_t next_high = g.hall_auto_result_high_raw;
    uint16_t next_low = g.hall_auto_result_low_raw;
    uint16_t next_mid = current_mid;
    if (span_ok) {
        hall_thresholds_from_min_max(low_endpoint, high_endpoint, &next_high, &next_low);
        next_mid = hall_midpoint_raw(low_endpoint, high_endpoint);
        const uint32_t irq_state = save_and_disable_interrupts();
        g.hall_auto_dyn_high_irq = next_high;
        g.hall_auto_dyn_low_irq = next_low;
        g.hall_auto_classify_mid_irq = next_mid;
        restore_interrupts(irq_state);
        g.hall_auto_result_high_raw = next_high;
        g.hall_auto_result_low_raw = next_low;
    }

    bool stable_ok = false;
    if (span_ok && g.hall_auto_have_candidate) {
        const uint16_t lim = hall_auto_stability_limit_raw((uint16_t)(high_endpoint - low_endpoint));
        const uint16_t dmin = (low_endpoint > g.hall_auto_candidate_min_raw) ?
            (uint16_t)(low_endpoint - g.hall_auto_candidate_min_raw) :
            (uint16_t)(g.hall_auto_candidate_min_raw - low_endpoint);
        const uint16_t dmax = (high_endpoint > g.hall_auto_candidate_max_raw) ?
            (uint16_t)(high_endpoint - g.hall_auto_candidate_max_raw) :
            (uint16_t)(g.hall_auto_candidate_max_raw - high_endpoint);
        stable_ok = dmin <= lim && dmax <= lim;
    }

    const uint32_t expected_edges = hall_auto_expected_edges_per_window();
    uint32_t min_edges = expected_edges / 5u;
    if (min_edges < 2u) {
        min_edges = 2u;
    }
    const bool edge_seen = w.edge_count >= min_edges;
    const bool jitter_ok = hall_auto_window_jitter_is_clean(&w);
    const bool rpm_ok = hall_auto_window_rpm_is_reasonable(&w);

    g.hall_auto_result_min_raw = low_endpoint;
    g.hall_auto_result_max_raw = high_endpoint;
    g.hall_auto_quality_pct = hall_auto_window_quality_pct(&w, span_ok, edge_seen, stable_ok, jitter_ok, rpm_ok);

    // Acceptance is raw-signal-first. RPM target affects the quality display,
    // but does not block saving when the external spinner is a bit off target.
    const bool clean = span_ok && stable_ok && edge_seen && (jitter_ok || plateaus_ok);
    if (clean) {
        if (g.hall_auto_clean_windows == 0u) {
            g.hall_auto_clean_min_sum = 0u;
            g.hall_auto_clean_max_sum = 0u;
        }
        g.hall_auto_clean_windows++;
        g.hall_auto_clean_min_sum += low_endpoint;
        g.hall_auto_clean_max_sum += high_endpoint;
    } else {
        g.hall_auto_clean_windows = 0u;
        g.hall_auto_clean_min_sum = 0u;
        g.hall_auto_clean_max_sum = 0u;
    }

    g.hall_auto_candidate_min_raw = low_endpoint;
    g.hall_auto_candidate_max_raw = high_endpoint;
    g.hall_auto_have_candidate = span_ok;

    if (g.hall_auto_clean_windows >= HALL_AUTO_CAL_REQUIRED_CLEAN_WINDOWS) {
        const uint16_t final_min = (uint16_t)(g.hall_auto_clean_min_sum / g.hall_auto_clean_windows);
        const uint16_t final_max = (uint16_t)(g.hall_auto_clean_max_sum / g.hall_auto_clean_windows);
        hall_auto_finish_success(final_min, final_max);
        return true;
    }

    return false;
}

bool sensors_stop_hall_auto_cal(bool aborted)
{
    const bool was_active = g.hall_auto_cal_active_irq;
    uint16_t min_raw = 4095u;
    uint16_t max_raw = 0u;
    uint32_t irq_state = save_and_disable_interrupts();
    min_raw = g.hall_auto_min_raw_irq;
    max_raw = g.hall_auto_max_raw_irq;
    g.hall_auto_cal_active_irq = false;
    restore_interrupts(irq_state);

    if (was_active) {
        g.hall_auto_cal_done = true;
        g.hall_auto_cal_ok = false;
        g.hall_auto_cal_status_code = aborted ? CUSTOM_CAN_HALL_CAL_STATUS_FAILED
                                               : CUSTOM_CAN_HALL_CAL_STATUS_IDLE;
        g.hall_auto_quality_pct = aborted ? 100u : 0u;
        g.hall_auto_result_min_raw = min_raw;
        g.hall_auto_result_max_raw = max_raw;
    }
    return was_active;
}

bool sensors_hall_auto_cal_active(void)
{
    return g.hall_auto_cal_active_irq;
}

void sensors_get_hall_auto_cal_status(HallAutoCalStatus *out)
{
    if (out == NULL) {
        return;
    }

    uint16_t min_raw = 0u;
    uint16_t max_raw = 0u;
    uint32_t sample_count = 0u;
    const uint32_t irq_state = save_and_disable_interrupts();
    const bool active = g.hall_auto_cal_active_irq;
    min_raw = active ? g.hall_auto_min_raw_irq : g.hall_auto_result_min_raw;
    max_raw = active ? g.hall_auto_max_raw_irq : g.hall_auto_result_max_raw;
    sample_count = g.hall_auto_sample_count_irq;
    restore_interrupts(irq_state);

    memset(out, 0, sizeof(*out));
    out->active = active;
    out->done = g.hall_auto_cal_done;
    out->ok = g.hall_auto_cal_ok;
    out->status_code = active ? CUSTOM_CAN_HALL_CAL_STATUS_RUNNING : g.hall_auto_cal_status_code;
    out->min_raw = min_raw;
    out->max_raw = max_raw;
    out->threshold_high_raw = g.hall_auto_result_high_raw;
    out->threshold_low_raw = g.hall_auto_result_low_raw;
    out->sample_count = sample_count;
    out->duration_ms = g.hall_auto_cal_duration_ms;
    out->target_rpm = g.hall_auto_cal_target_rpm;
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    out->elapsed_ms = (g.hall_auto_cal_start_ms == 0u) ? 0u : now - g.hall_auto_cal_start_ms;
    out->progress_pct = active ? g.hall_auto_quality_pct : (g.hall_auto_cal_ok ? 100u : g.hall_auto_quality_pct);
}

void sensors_init(void)
{
    memset(&g, 0, sizeof(g));

    adc_init();
    adc_gpio_init(PIN_TEMP_ADC_GPIO);
    adc_gpio_init(PIN_HALL_ADC_GPIO);

    g.hall_threshold_high_raw = hall_threshold_high_raw();
    g.hall_threshold_low_raw = hall_threshold_low_raw();
    g.hall_cal_min_raw = 0u;
    g.hall_cal_max_raw = 0u;
    g.hall_auto_cal_active_irq = false;
    g.hall_auto_cal_done = false;
    g.hall_auto_cal_ok = false;
    g.hall_auto_cal_status_code = CUSTOM_CAN_HALL_CAL_STATUS_IDLE;
    g.hall_auto_quality_pct = 0u;
    g.hall_auto_clean_windows = 0u;
    g.hall_auto_have_candidate = false;
    g.hall_auto_cal_duration_ms = HALL_AUTO_CAL_DEFAULT_DURATION_MS;
    g.hall_auto_last_eval_ms = 0u;
    g.hall_auto_expected_period_min_us = HALL_MIN_EDGE_SPACING_US;
    g.hall_auto_expected_period_max_us = 1000000u;
    g.hall_auto_candidate_min_raw = 0u;
    g.hall_auto_candidate_max_raw = 0u;
    g.hall_auto_clean_min_sum = 0u;
    g.hall_auto_clean_max_sum = 0u;
    g.hall_auto_result_min_raw = 0u;
    g.hall_auto_result_max_raw = 0u;
    g.hall_auto_result_high_raw = g.hall_threshold_high_raw;
    g.hall_auto_result_low_raw = g.hall_threshold_low_raw;
    g.hall_rpm_filtered = 0.0f;
    g.hall_period_filtered_us_irq = 0u;
    const uint32_t hall_init_irq = save_and_disable_interrupts();
    hall_auto_cal_reset_window_irq(g.hall_threshold_high_raw,
                                   g.hall_threshold_low_raw,
                                   hall_midpoint_raw(g.hall_threshold_low_raw, g.hall_threshold_high_raw),
                                   true);
    restore_interrupts(hall_init_irq);
    g.thresholds_dirty = false;

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