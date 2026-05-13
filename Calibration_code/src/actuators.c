#include "actuators.h"
#include "board_config.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

static uint g_choke_slice;
static uint g_choke_chan;
static uint g_thr_slice;
static uint g_thr_chan;
static uint16_t g_wrap;

static void set_pwm_us(uint slice, uint chan, uint16_t us)
{
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    const uint32_t period_us = 1000000u / SERVO_FREQ_HZ;
    const uint16_t level = (uint16_t)(((uint32_t)us * g_wrap) / period_us);
    pwm_set_chan_level(slice, chan, level);
}

void actuators_init(void)
{
    gpio_init(PIN_RELAY);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);
    gpio_put(PIN_RELAY, 0);

    gpio_init(PIN_STARTER);
    gpio_set_dir(PIN_STARTER, GPIO_OUT);
    gpio_put(PIN_STARTER, 0);

    gpio_set_function(PIN_CHOKE_SERVO, GPIO_FUNC_PWM);
    gpio_set_function(PIN_THROTTLE_SERVO, GPIO_FUNC_PWM);

    g_choke_slice = pwm_gpio_to_slice_num(PIN_CHOKE_SERVO);
    g_choke_chan  = pwm_gpio_to_channel(PIN_CHOKE_SERVO);
    g_thr_slice   = pwm_gpio_to_slice_num(PIN_THROTTLE_SERVO);
    g_thr_chan    = pwm_gpio_to_channel(PIN_THROTTLE_SERVO);

    // 1 MHz PWM clock for straightforward microsecond conversion.
    pwm_set_clkdiv_int_frac(g_choke_slice, 125, 0);
    pwm_set_clkdiv_int_frac(g_thr_slice,   125, 0);
    g_wrap = (1000000u / SERVO_FREQ_HZ);
    pwm_set_wrap(g_choke_slice, g_wrap);
    pwm_set_wrap(g_thr_slice, g_wrap);
    pwm_set_enabled(g_choke_slice, true);
    pwm_set_enabled(g_thr_slice, true);

    actuators_set_choke_closed(false);
    actuators_set_throttle_us(SERVO_US_MIN);
}

void actuators_set_relay(bool on)
{
    gpio_put(PIN_RELAY, on ? 1 : 0);
}

void actuators_set_starter(bool on)
{
    gpio_put(PIN_STARTER, on ? 1 : 0);
}

void actuators_set_choke_closed(bool closed)
{
    set_pwm_us(g_choke_slice, g_choke_chan,
               closed ? SERVO_US_CHOKE_CLOSED : SERVO_US_CHOKE_OPEN);
}

void actuators_set_throttle_us(uint16_t us)
{
    set_pwm_us(g_thr_slice, g_thr_chan, us);
}
