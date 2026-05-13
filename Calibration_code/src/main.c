#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

#include "mcp2518fd.h"
#include "dronecan_node.h"

// ============================================================================
// MANUAL CALIBRATION SETTINGS - EDIT THESE FIRST
// ============================================================================
#define PIN_LED                18
#define PIN_BUZZER             19
#define PIN_RELAY              8
#define PIN_CHOKE_SERVO        16
#define PIN_THROTTLE_SERVO     17
#define PIN_HALL_ADC_GPIO      27      // ADC1 on RP2040
#define HALL_ADC_INPUT         1

#define PIN_I2C_SDA            10
#define PIN_I2C_SCL            11
#define FRAM_I2C_PORT          i2c1
#define FRAM_I2C_ADDR          0x50

#define PREARM_STEP_INTERVAL_MS    1000u
#define DISARM_RELAY_OFF_MS        5000u
#define POST_START_WAIT_MS         1000u
#define SWEEP_STEP_HOLD_MS         3700u
#define CONTROL_LOOP_MS            2u
#define PRINT_LOOP_MS              500u
#define BEEP_MS                    120u

// NEW: CAN telemetry periods
#define NODE_STATUS_PERIOD_MS      1000u
#define ESC_STATUS_PERIOD_MS       100u

// Servo positions in microseconds
#define THROTTLE_US_CLOSED         1910u // iddle speed
#define THROTTLE_US_MID            1400u
#define THROTTLE_US_OPEN           1100u // 1100 MAX
#define CHOKE_US_CLOSED            1800u
#define CHOKE_US_OPEN              1000u

// ============================================================================
// HALL SENSOR CONFIGURATION
//
// Enter the HALL SENSOR OUTPUT voltages here, BEFORE the divider.
// Example:
//   high = 4.0 V
//   low  = 2.0 V
//
// Divider:
//   hall output -> (4.7k || 4.7k) -> ADC pin -> 4.7k -> GND
//
// So:
//   R_top    = 2.35k
//   R_bottom = 4.7k
//   Vadc = Vhall * R_bottom / (R_top + R_bottom)
// ==============================================================================
#define ADC_VREF                    3.3f
#define ADC_COUNTS_MAX              4095.0f

#define HALL_DIVIDER_R_TOP_OHM      2350.0f
#define HALL_DIVIDER_R_BOTTOM_OHM   4700.0f

// Change these two values:
#define HALL_SENSOR_HIGH_V          0.26f
#define HALL_SENSOR_LOW_V           0.1f

#define RPM_ZERO_TIMEOUT_MS         2000u
#define RPM_DETECT_THRESHOLD        50.0f

// ============================================================================
// FRAM LOG LAYOUT
// ============================================================================
#define FRAM_MAGIC                 0x45534C47u  // 'ESLG'
#define FRAM_TOTAL_BYTES           8192u
#define FRAM_HEADER_ADDR           0u
#define FRAM_DATA_ADDR             32u
#define FRAM_END_ADDR              FRAM_TOTAL_BYTES

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t next_write_addr;
    uint16_t run_counter;
    uint16_t reserved0;
    uint16_t reserved1;
    uint32_t reserved2;
} FramHeader;

typedef struct __attribute__((packed)) {
    uint8_t tag;          // 'R'
    uint16_t run_number;
    uint32_t timestamp_ms;
} RunStartRecord;

typedef struct __attribute__((packed)) {
    uint8_t tag;          // 'E'
    uint16_t run_number;
    uint8_t code;         // 1 = RPM detected -> opening choke
    uint8_t reserved;
    uint32_t timestamp_ms;
    float rpm;
} EventRecord;

typedef struct __attribute__((packed)) {
    uint8_t tag;          // 'D'
    uint16_t run_number;
    uint8_t throttle_pct;
    uint16_t pwm_us;     // PWM in microseconds
    float rpm;
} SweepPointRecord;

// ============================================================================
// UI / state machine
// ============================================================================
typedef enum {
    APP_PREARM_SWEEP = 0,
    APP_DISARM_RELAY_OFF,
    APP_ARMED_WAIT_FOR_SPIN,
    APP_POST_SPIN_WAIT,
    APP_SWEEPING
} AppState;

// ============================================================================
// Local state
// ============================================================================
static AppState g_state = APP_PREARM_SWEEP;

static uint g_throttle_slice, g_throttle_chan;
static uint g_choke_slice, g_choke_chan;
static uint g_buzzer_slice, g_buzzer_chan;
static uint16_t g_pwm_wrap = 20000u - 1u; // 20 ms period at 1 MHz

static bool g_beep_active = false;
static bool g_buzzer_continuous = false;
static uint32_t g_beep_off_ms = 0;

static uint32_t g_last_prearm_step_ms = 0;
static uint8_t g_prearm_step = 0;

static bool g_hall_high = false;
static uint32_t g_last_hall_edge_ms = 0;
static uint32_t g_total_turns = 0;
static float g_rpm = 0.0f;

static uint16_t g_run_number = 0;
static int g_sweep_pct = 100;
static uint32_t g_last_sweep_step_ms = 0;
static uint32_t g_state_enter_ms = 0;
static bool g_prev_armed = false;

// ============================================================================
// Helpers
// ============================================================================
static inline uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static float clampf_local(float x, float lo, float hi)
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

    const float vadc_clamped = clampf_local(vadc, 0.0f, ADC_VREF);
    const float rawf = (vadc_clamped / ADC_VREF) * ADC_COUNTS_MAX;
    return (uint16_t)(rawf + 0.5f);
}

static uint16_t hall_threshold_high_raw(void)
{
    return hall_sensor_voltage_to_raw(HALL_SENSOR_HIGH_V);
}

static uint16_t hall_threshold_low_raw(void)
{
    return hall_sensor_voltage_to_raw(HALL_SENSOR_LOW_V);
}

static void enter_state(AppState s, uint32_t ms)
{
    g_state = s;
    g_state_enter_ms = ms;
}

static void pwm_servo_init(uint pin, uint *slice_out, uint *chan_out)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan = pwm_gpio_to_channel(pin);

    pwm_set_clkdiv(slice, 125.0f);     // 125 MHz / 125 = 1 MHz => 1 us ticks
    pwm_set_wrap(slice, g_pwm_wrap);
    pwm_set_chan_level(slice, chan, 1500u);
    pwm_set_enabled(slice, true);

    *slice_out = slice;
    *chan_out = chan;
}

static void servo_write_us(uint slice, uint chan, uint16_t us)
{
    if (us > g_pwm_wrap) us = g_pwm_wrap;
    pwm_set_chan_level(slice, chan, us);
}

static void throttle_write_us(uint16_t us)
{
    servo_write_us(g_throttle_slice, g_throttle_chan, us);
}

static void choke_write_us(uint16_t us)
{
    servo_write_us(g_choke_slice, g_choke_chan, us);
}

static uint16_t throttle_percent_to_us(float pct)
{
    pct = clampf_local(pct, 0.0f, 100.0f);
    const float t = pct / 100.0f;
    return (uint16_t)lrintf((1.0f - t) * (float)THROTTLE_US_CLOSED + t * (float)THROTTLE_US_OPEN);
}

static void buzzer_init_local(void)
{
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    g_buzzer_slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    g_buzzer_chan  = pwm_gpio_to_channel(PIN_BUZZER);

    pwm_set_clkdiv(g_buzzer_slice, 1.0f);
    pwm_set_wrap(g_buzzer_slice, 62499u); // 2 kHz
    pwm_set_chan_level(g_buzzer_slice, g_buzzer_chan, 0);
    pwm_set_enabled(g_buzzer_slice, true);
}

static void buzzer_set_hw(bool on)
{
    pwm_set_chan_level(g_buzzer_slice, g_buzzer_chan, on ? 31250u : 0u);
}

static void buzzer_set_continuous(bool on)
{
    g_buzzer_continuous = on;
    if (on) {
        g_beep_active = false;
        buzzer_set_hw(true);
    } else if (!g_beep_active) {
        buzzer_set_hw(false);
    }
}

static void beep_once(uint32_t duration_ms)
{
    g_buzzer_continuous = false;
    g_beep_active = true;
    g_beep_off_ms = now_ms() + duration_ms;
    buzzer_set_hw(true);
}

static void beep_service(uint32_t ms)
{
    if (g_buzzer_continuous) {
        buzzer_set_hw(true);
        return;
    }

    if (g_beep_active && ((int32_t)(ms - g_beep_off_ms) >= 0)) {
        g_beep_active = false;
        buzzer_set_hw(false);
    }
}

static void relay_set(bool on)
{
    gpio_put(PIN_RELAY, on ? 1 : 0);
}

static void led_set(bool on)
{
    gpio_put(PIN_LED, on ? 1 : 0);
}

static void hall_init(void)
{
    adc_init();
    adc_gpio_init(PIN_HALL_ADC_GPIO);
    adc_select_input(HALL_ADC_INPUT);
}

static void hall_update(uint32_t ms)
{
    adc_select_input(HALL_ADC_INPUT);
    const uint16_t raw = adc_read();

    const uint16_t high_raw = hall_threshold_high_raw();
    const uint16_t low_raw  = hall_threshold_low_raw();

    if (!g_hall_high && raw >= high_raw) {
        g_hall_high = true;

        if (g_last_hall_edge_ms != 0) {
            const uint32_t dt = ms - g_last_hall_edge_ms;
            if (dt > 0) {
                g_rpm = 60000.0f / (float)dt;   // assumes 1 pulse per revolution
            }
        }

        g_last_hall_edge_ms = ms;
        g_total_turns++;
    } else if (g_hall_high && raw <= low_raw) {
        g_hall_high = false;
    }

    if (g_last_hall_edge_ms == 0 || (ms - g_last_hall_edge_ms) > RPM_ZERO_TIMEOUT_MS) {
        g_rpm = 0.0f;
    }
}

// ============================================================================
// FRAM
// ============================================================================
static void fram_init(void)
{
    i2c_init(FRAM_I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

static void fram_read(uint16_t addr, uint8_t *dst, size_t len)
{
    uint8_t reg[2] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFFu)};
    i2c_write_blocking(FRAM_I2C_PORT, FRAM_I2C_ADDR, reg, 2, true);
    i2c_read_blocking(FRAM_I2C_PORT, FRAM_I2C_ADDR, dst, len, false);
}

static void fram_write(uint16_t addr, const uint8_t *src, size_t len)
{
    uint8_t buf[34];

    while (len > 0) {
        size_t chunk = len > 32 ? 32 : len;
        buf[0] = (uint8_t)(addr >> 8);
        buf[1] = (uint8_t)(addr & 0xFFu);
        memcpy(&buf[2], src, chunk);
        i2c_write_blocking(FRAM_I2C_PORT, FRAM_I2C_ADDR, buf, chunk + 2, false);
        addr += (uint16_t)chunk;
        src += chunk;
        len -= chunk;
    }
}

static FramHeader fram_header_load(void)
{
    FramHeader h;
    fram_read(FRAM_HEADER_ADDR, (uint8_t *)&h, sizeof(h));

    if (h.magic != FRAM_MAGIC || h.next_write_addr < FRAM_DATA_ADDR || h.next_write_addr > FRAM_END_ADDR) {
        memset(&h, 0, sizeof(h));
        h.magic = FRAM_MAGIC;
        h.next_write_addr = FRAM_DATA_ADDR;
        h.run_counter = 0;
        fram_write(FRAM_HEADER_ADDR, (const uint8_t *)&h, sizeof(h));
    }

    return h;
}

static void fram_header_store(const FramHeader *h)
{
    fram_write(FRAM_HEADER_ADDR, (const uint8_t *)h, sizeof(*h));
}

static void fram_append_bytes(FramHeader *h, const void *src, uint16_t len)
{
    if ((uint32_t)h->next_write_addr + len > FRAM_END_ADDR) {
        h->next_write_addr = FRAM_DATA_ADDR;
    }

    fram_write(h->next_write_addr, (const uint8_t *)src, len);
    h->next_write_addr = (uint16_t)(h->next_write_addr + len);
    fram_header_store(h);
}

static uint16_t fram_start_new_run(uint32_t ms)
{
    FramHeader h = fram_header_load();
    h.run_counter = (uint16_t)(h.run_counter + 1u);
    fram_header_store(&h);

    RunStartRecord r = {
        .tag = 'R',
        .run_number = h.run_counter,
        .timestamp_ms = ms,
    };
    fram_append_bytes(&h, &r, sizeof(r));
    return h.run_counter;
}

static void fram_append_event(uint16_t run_number, uint8_t code, uint32_t ms, float rpm)
{
    FramHeader h = fram_header_load();
    EventRecord e = {
        .tag = 'E',
        .run_number = run_number,
        .code = code,
        .reserved = 0,
        .timestamp_ms = ms,
        .rpm = rpm,
    };
    fram_append_bytes(&h, &e, sizeof(e));
}

static void fram_append_point(uint16_t run_number, uint8_t throttle_pct, uint16_t pwm_us, float rpm)
{
    FramHeader h = fram_header_load();
    SweepPointRecord p = {
        .tag = 'D',
        .run_number = run_number,
        .throttle_pct = throttle_pct,
        .pwm_us = pwm_us,
        .rpm = rpm,
    };
    fram_append_bytes(&h, &p, sizeof(p));
}

static void fram_dump_all_if_usb(void)
{
    sleep_ms(300);
    if (!stdio_usb_connected()) {
        return;
    }

    FramHeader h = fram_header_load();
    printf("\r\n--- FRAM dump (all currently stored records) ---\r\n");
    printf("next_write=%u run_counter=%u\r\n",
           h.next_write_addr, h.run_counter);

    uint16_t addr = FRAM_DATA_ADDR;
    while (addr < h.next_write_addr) {
        uint8_t tag = 0;
        fram_read(addr, &tag, 1);

        if (tag == 'R') {
            RunStartRecord r;
            fram_read(addr, (uint8_t *)&r, sizeof(r));
            printf("RUN %u start_ms=%lu\r\n",
                   r.run_number,
                   (unsigned long)r.timestamp_ms);
            addr = (uint16_t)(addr + sizeof(r));

        } else if (tag == 'E') {
            EventRecord e;
            fram_read(addr, (uint8_t *)&e, sizeof(e));
            printf("  EVENT run=%u code=%u ms=%lu rpm=%.1f\r\n",
                   e.run_number,
                   e.code,
                   (unsigned long)e.timestamp_ms,
                   (double)e.rpm);
            addr = (uint16_t)(addr + sizeof(e));

        } else if (tag == 'D') {
            SweepPointRecord p;
            fram_read(addr, (uint8_t *)&p, sizeof(p));
            printf("  DATA  run=%u pct=%u pwm=%u rpm=%.1f\r\n",
                   p.run_number,
                   p.throttle_pct,
                   p.pwm_us,
                   (double)p.rpm);
            addr = (uint16_t)(addr + sizeof(p));

        } else {
            printf("Unknown record at %u tag=0x%02X\r\n", addr, tag);
            break;
        }
    }

    printf("--- end FRAM dump ---\r\n\r\n");
}

// static void fram_format(void)
// {
//     FramHeader h;
//     memset(&h, 0, sizeof(h));
//     h.magic = FRAM_MAGIC;
//     h.next_write_addr = FRAM_DATA_ADDR;
//     h.run_counter = 0;

//     fram_header_store(&h);

//     uint8_t zero[32];
//     memset(zero, 0, sizeof(zero));

//     for (uint16_t addr = FRAM_DATA_ADDR; addr < FRAM_END_ADDR; addr += sizeof(zero)) {
//         uint16_t remaining = FRAM_END_ADDR - addr;
//         uint16_t chunk = remaining > sizeof(zero) ? sizeof(zero) : remaining;
//         fram_write(addr, zero, chunk);
//     }

//     printf("FRAM formatted.\r\n");
// }

// ============================================================================
// App helpers
// ============================================================================
static void force_all_outputs_safe(void)
{
    relay_set(false);
    buzzer_set_continuous(false);
    buzzer_set_hw(false);
    throttle_write_us(THROTTLE_US_CLOSED);
    choke_write_us(CHOKE_US_OPEN);
}

static void apply_prearm_step(uint8_t step)
{
    relay_set(true);   // relay ON during the 3-position sweep

    switch (step % 3u) {
    case 0:
        throttle_write_us(THROTTLE_US_CLOSED);
        choke_write_us(CHOKE_US_CLOSED);
        break;

    case 1:
        throttle_write_us(THROTTLE_US_MID);
        choke_write_us(CHOKE_US_CLOSED);
        break;

    default:
        throttle_write_us(THROTTLE_US_OPEN);
        choke_write_us(CHOKE_US_OPEN);
        break;
    }

    beep_once(BEEP_MS);
}

static void start_prearm_sequence(uint32_t ms)
{
    g_total_turns = 0;
    g_last_hall_edge_ms = 0;
    g_rpm = 0.0f;

    g_run_number = 0;
    g_sweep_pct = 100;

    g_prearm_step = 0;
    g_last_prearm_step_ms = ms;

    enter_state(APP_PREARM_SWEEP, ms);
    apply_prearm_step(g_prearm_step);
    g_prearm_step = 1;
}

static void ui_update_prearm(uint32_t ms)
{
    led_set(((ms / 500u) & 1u) != 0u);
    buzzer_set_continuous(false);
}

static void ui_update_armed(void)
{
    led_set(true);
    buzzer_set_continuous(true);
}

static void ui_update_disarm_off(void)
{
    led_set(false);
    buzzer_set_continuous(false);
}

// ============================================================================
// main
// ============================================================================
int main(void)
{
    stdio_init_all();
    sleep_ms(1200);


    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    led_set(false);

    gpio_init(PIN_RELAY);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);
    relay_set(false);

    buzzer_init_local();
    pwm_servo_init(PIN_THROTTLE_SERVO, &g_throttle_slice, &g_throttle_chan);
    pwm_servo_init(PIN_CHOKE_SERVO, &g_choke_slice, &g_choke_chan);

    hall_init();

    fram_init();
    //fram_format();
    fram_dump_all_if_usb();

    printf("Hall thresholds: sensor_high=%.2fV sensor_low=%.2fV adc_high_raw=%u adc_low_raw=%u\r\n",
           (double)HALL_SENSOR_HIGH_V,
           (double)HALL_SENSOR_LOW_V,
           hall_threshold_high_raw(),
           hall_threshold_low_raw());

    if (!mcp2518fd_init()) {
        while (true) {
            led_set(!gpio_get(PIN_LED));
            sleep_ms(150);
        }
    }

    dronecan_node_init();
    //watchdog_enable(1000, 1);

    uint32_t next_loop_ms = now_ms();
    uint32_t next_print_ms = now_ms() + PRINT_LOOP_MS;
    uint32_t next_node_status_ms = now_ms() + NODE_STATUS_PERIOD_MS;
    uint32_t next_esc_status_ms = now_ms() + ESC_STATUS_PERIOD_MS;

    g_prev_armed = false;
    start_prearm_sequence(now_ms());

    while (true) {
        CanardCANFrame frame;
        int budget = 32;
        while ((budget-- > 0) && mcp2518fd_receive(&frame)) {
            dronecan_node_handle_frame(&frame, to_us_since_boot(get_absolute_time()));
        }

        const uint32_t ms = now_ms();
        const uint64_t us = to_us_since_boot(get_absolute_time());
        const bool fc_alive = dronecan_node_fc_alive(us);
        const bool armed = fc_alive && dronecan_node_get_armed();

        hall_update(ms);
        beep_service(ms);

        if (g_prev_armed && !armed) {
            force_all_outputs_safe();
            enter_state(APP_DISARM_RELAY_OFF, ms);
        }
        g_prev_armed = armed;

        if ((int32_t)(ms - next_loop_ms) >= 0) {
            next_loop_ms += CONTROL_LOOP_MS;

            switch (g_state) {
            case APP_PREARM_SWEEP:
                ui_update_prearm(ms);

                if ((ms - g_last_prearm_step_ms) >= PREARM_STEP_INTERVAL_MS) {
                    apply_prearm_step(g_prearm_step);
                    g_last_prearm_step_ms = ms;
                    g_prearm_step = (uint8_t)((g_prearm_step + 1u) % 3u);
                }

                if (armed) {
                    relay_set(true);
                    ui_update_armed();
                    choke_write_us(CHOKE_US_CLOSED);
                    throttle_write_us(THROTTLE_US_MID);
                    g_total_turns = 0;
                    g_last_hall_edge_ms = 0;
                    g_rpm = 0.0f;
                    enter_state(APP_ARMED_WAIT_FOR_SPIN, ms);
                }
                break;

            case APP_DISARM_RELAY_OFF:
                ui_update_disarm_off();
                force_all_outputs_safe();

                if ((ms - g_state_enter_ms) >= DISARM_RELAY_OFF_MS) {
                    start_prearm_sequence(ms);
                }
                break;

            case APP_ARMED_WAIT_FOR_SPIN:
                relay_set(true);
                ui_update_armed();

                choke_write_us(CHOKE_US_CLOSED);
                throttle_write_us(THROTTLE_US_MID);

                if (!armed) {
                    force_all_outputs_safe();
                    enter_state(APP_DISARM_RELAY_OFF, ms);
                    break;
                }

                if (g_rpm >= RPM_DETECT_THRESHOLD) {
                    g_run_number = fram_start_new_run(ms);
                    fram_append_event(g_run_number, 1u, ms, g_rpm);   // RPM detected -> opening choke
                    choke_write_us(CHOKE_US_OPEN);
                    enter_state(APP_POST_SPIN_WAIT, ms);
                }
                break;

            case APP_POST_SPIN_WAIT:
                relay_set(true);
                ui_update_armed();

                choke_write_us(CHOKE_US_OPEN);
                throttle_write_us(THROTTLE_US_MID);

                if (!armed || g_rpm <= 0.1f) {
                    force_all_outputs_safe();
                    enter_state(APP_DISARM_RELAY_OFF, ms);
                    break;
                }

                if ((ms - g_state_enter_ms) >= POST_START_WAIT_MS) {
                    g_sweep_pct = 0;
                    g_last_sweep_step_ms = 0;
                    enter_state(APP_SWEEPING, ms);
                }
                break;

            case APP_SWEEPING:
                relay_set(true);
                ui_update_armed();

                choke_write_us(CHOKE_US_OPEN);

                if (!armed || g_rpm <= 0.1f) {
                    force_all_outputs_safe();
                    enter_state(APP_DISARM_RELAY_OFF, ms);
                    break;
                }

                if (g_last_sweep_step_ms == 0 || (ms - g_last_sweep_step_ms) >= SWEEP_STEP_HOLD_MS) {
                    g_last_sweep_step_ms = ms;

                    const uint16_t pwm_us = throttle_percent_to_us((float)g_sweep_pct);
                    throttle_write_us(pwm_us);
                    fram_append_point(g_run_number, (uint8_t)g_sweep_pct, pwm_us, g_rpm);

                    printf("run=%u pct=%d pwm=%u rpm=%.1f turns=%lu\r\n",
                           g_run_number,
                           g_sweep_pct,
                           pwm_us,
                           (double)g_rpm,
                           (unsigned long)g_total_turns);

                    g_sweep_pct += 1;
                    if (g_sweep_pct > 100) {
                        force_all_outputs_safe();
                        enter_state(APP_DISARM_RELAY_OFF, ms);
                    }
                }
                break;
            }
        }

        // --------------------------------------------------------------------
        // NEW: publish this Pico as a DroneCAN node so PX4 can see it
        // --------------------------------------------------------------------
        if ((int32_t)(ms - next_node_status_ms) >= 0) {
            dronecan_node_publish_node_status(ms / 1000u,
                                              0,                        // health OK
                                              0,                        // mode operational
                                              (uint16_t)g_state);       // vendor status = local state
            next_node_status_ms += NODE_STATUS_PERIOD_MS;
        }

        // --------------------------------------------------------------------
        // NEW: publish RPM back to PX4 in ESC status
        // current and temp are placeholders here because this main.c
        // does not measure them yet
        // --------------------------------------------------------------------
        if ((int32_t)(ms - next_esc_status_ms) >= 0) {
            const float throttle_pct_for_telem =
                (g_state == APP_SWEEPING) ? (float)g_sweep_pct : 0.0f;

            dronecan_node_publish_esc_status(g_rpm,
                                             0.0f,                      // current_a placeholder
                                             0.0f,                      // temperature_c placeholder
                                             throttle_pct_for_telem);
            next_esc_status_ms += ESC_STATUS_PERIOD_MS;
        }

        // --------------------------------------------------------------------
        // NEW: actually flush queued CAN TX frames out through MCP2518FD
        // --------------------------------------------------------------------
        dronecan_node_process_tx();

        if ((int32_t)(ms - next_print_ms) >= 0) {
            next_print_ms += PRINT_LOOP_MS;
            printf("state=%d fc_alive=%d armed=%d rpm=%.1f turns=%lu run=%u step=%d age_ms=%lu hall_high_raw=%u hall_low_raw=%u\r\n",
                   (int)g_state,
                   (int)fc_alive,
                   (int)armed,
                   (double)g_rpm,
                   (unsigned long)g_total_turns,
                   g_run_number,
                   g_sweep_pct,
                   (unsigned long)dronecan_node_fc_age_ms(us),
                   hall_threshold_high_raw(),
                   hall_threshold_low_raw());
        }

        watchdog_update();
        sleep_ms(1);
    }
}