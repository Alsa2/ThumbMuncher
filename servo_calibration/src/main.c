#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ===================== USER CONFIG =====================
// Change these only if your board uses different pins.
#define PIN_RELAY        8u
#define PIN_CHOKE_SERVO  16u

// Servo pulse limits from your project config.
#define CHOKE_US_OPEN    1000u
#define CHOKE_US_CLOSED  1800u

// Sweep behavior.
#define STEP_US          25u
#define STEP_DELAY_MS    200u
#define POWER_WAIT_MS    1500u

// Standard 50 Hz servo PWM.
#define SERVO_FRAME_US   20000u
// =======================================================

static uint servo_slice;
static uint servo_channel;
static uint16_t current_us = CHOKE_US_OPEN;
static bool sweeping = true;
static int sweep_dir = +1;

static void relay_set(bool on)
{
    gpio_put(PIN_RELAY, on ? 1 : 0);
}

static void choke_servo_init(void)
{
    gpio_set_function(PIN_CHOKE_SERVO, GPIO_FUNC_PWM);

    servo_slice = pwm_gpio_to_slice_num(PIN_CHOKE_SERVO);
    servo_channel = pwm_gpio_to_channel(PIN_CHOKE_SERVO);

    // Make PWM counter tick at 1 MHz, so 1 count = 1 us.
    // Default clk_sys is normally 125 MHz, so divider = 125.
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, SERVO_FRAME_US - 1u);
    pwm_init(servo_slice, &cfg, true);
}

static void choke_write_us(uint16_t us)
{
    if (us < CHOKE_US_OPEN) us = CHOKE_US_OPEN;
    if (us > CHOKE_US_CLOSED) us = CHOKE_US_CLOSED;

    current_us = us;
    pwm_set_chan_level(servo_slice, servo_channel, current_us);
}

static float closed_percent_from_us(uint16_t us)
{
    return 100.0f * ((float)us - (float)CHOKE_US_OPEN) /
           ((float)CHOKE_US_CLOSED - (float)CHOKE_US_OPEN);
}

static void print_status(void)
{
    float closed_pct = closed_percent_from_us(current_us);
    float open_pct = 100.0f - closed_pct;

    printf("t=%lu ms | relay=ON | sweep=%d | choke=%u us | closed=%.1f %% | open=%.1f %%\n",
           to_ms_since_boot(get_absolute_time()),
           sweeping ? 1 : 0,
           current_us,
           closed_pct,
           open_pct);
}

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  s  start/stop automatic sweep\n");
    printf("  o  force choke open   (%u us)\n", CHOKE_US_OPEN);
    printf("  c  force choke closed (%u us)\n", CHOKE_US_CLOSED);
    printf("  +  increase pulse by %u us\n", STEP_US);
    printf("  -  decrease pulse by %u us\n", STEP_US);
    printf("  r  relay ON again\n");
    printf("  h  print this help\n\n");
}

static void handle_serial(void)
{
    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) return;

    if (ch == 's') {
        sweeping = !sweeping;
        printf("Command: sweep %s\n", sweeping ? "ON" : "OFF");
    } else if (ch == 'o') {
        sweeping = false;
        choke_write_us(CHOKE_US_OPEN);
        printf("Command: choke OPEN\n");
        print_status();
    } else if (ch == 'c') {
        sweeping = false;
        choke_write_us(CHOKE_US_CLOSED);
        printf("Command: choke CLOSED\n");
        print_status();
    } else if (ch == '+') {
        sweeping = false;
        choke_write_us(current_us + STEP_US);
        printf("Command: manual +\n");
        print_status();
    } else if (ch == '-') {
        sweeping = false;
        if (current_us > STEP_US) choke_write_us(current_us - STEP_US);
        else choke_write_us(CHOKE_US_OPEN);
        printf("Command: manual -\n");
        print_status();
    } else if (ch == 'r') {
        relay_set(true);
        printf("Command: relay ON\n");
        print_status();
    } else if (ch == 'h') {
        print_help();
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== Choke servo sweep with relay power ON ===\n");
    printf("Relay pin: GPIO %u\n", PIN_RELAY);
    printf("Choke servo pin: GPIO %u\n", PIN_CHOKE_SERVO);
    printf("Open pulse: %u us | Closed pulse: %u us\n", CHOKE_US_OPEN, CHOKE_US_CLOSED);

    gpio_init(PIN_RELAY);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);

    // Turn relay ON before initializing/sweeping the servo supply.
    relay_set(true);
    printf("Relay ON. Waiting %u ms for servo power supply...\n", POWER_WAIT_MS);
    sleep_ms(POWER_WAIT_MS);

    choke_servo_init();
    choke_write_us(CHOKE_US_OPEN);

    printf("Servo initialized. Starting sweep.\n");
    print_help();
    print_status();

    absolute_time_t next_step = make_timeout_time_ms(STEP_DELAY_MS);

    while (true) {
        handle_serial();

        // Keep relay forced ON during the whole test.
        relay_set(true);

        if (sweeping && absolute_time_diff_us(get_absolute_time(), next_step) <= 0) {
            int next = (int)current_us + sweep_dir * (int)STEP_US;

            if (next >= (int)CHOKE_US_CLOSED) {
                next = CHOKE_US_CLOSED;
                sweep_dir = -1;
            } else if (next <= (int)CHOKE_US_OPEN) {
                next = CHOKE_US_OPEN;
                sweep_dir = +1;
            }

            choke_write_us((uint16_t)next);
            print_status();
            next_step = make_timeout_time_ms(STEP_DELAY_MS);
        }

        sleep_ms(5);
    }

    return 0;
}