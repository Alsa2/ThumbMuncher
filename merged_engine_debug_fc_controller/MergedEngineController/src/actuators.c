#include "actuators.h"
#include "board_config.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

static uint g_choke_slice;
static uint g_choke_chan;
static uint g_thr_slice;
static uint g_thr_chan;
static uint16_t g_wrap;
static uint16_t g_last_throttle_us = THROTTLE_IDLE_US;
static uint16_t g_last_choke_us = CHOKE_US_OPEN;

static uint16_t clamp_servo_us(uint16_t us)
{
    if (us < SERVO_US_HARD_MIN) return SERVO_US_HARD_MIN;
    if (us > SERVO_US_HARD_MAX) return SERVO_US_HARD_MAX;
    return us;
}

static void set_pwm_us(uint slice, uint chan, uint16_t us)
{
    us = clamp_servo_us(us);
    // PWM clock is configured to 1 MHz, so one PWM count equals one microsecond.
    pwm_set_chan_level(slice, chan, us);
}

static void servo_pwm_init(uint pin, uint *slice_out, uint *chan_out)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);

    const uint slice = pwm_gpio_to_slice_num(pin);
    const uint chan = pwm_gpio_to_channel(pin);

    pwm_set_clkdiv_int_frac(slice, 125, 0);   // 125 MHz / 125 = 1 MHz
    pwm_set_wrap(slice, g_wrap);              // 20 ms period for 50 Hz
    pwm_set_enabled(slice, true);

    *slice_out = slice;
    *chan_out = chan;
}

void actuators_init(void)
{
    gpio_init(PIN_RELAY);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);
    gpio_put(PIN_RELAY, 0);

    gpio_init(PIN_STARTER);
    gpio_set_dir(PIN_STARTER, GPIO_OUT);
    gpio_put(PIN_STARTER, 0);

    const uint32_t period_us = 1000000u / SERVO_FREQ_HZ;
    g_wrap = (uint16_t)(period_us - 1u);

    servo_pwm_init(PIN_CHOKE_SERVO, &g_choke_slice, &g_choke_chan);
    servo_pwm_init(PIN_THROTTLE_SERVO, &g_thr_slice, &g_thr_chan);

    actuators_set_starter(false);
    actuators_set_choke_closed(false);
    actuators_set_throttle_us(THROTTLE_IDLE_US);
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
    actuators_set_choke_us(closed ? CHOKE_US_CLOSED : CHOKE_US_OPEN);
}

void actuators_set_choke_us(uint16_t us)
{
    g_last_choke_us = clamp_servo_us(us);
    set_pwm_us(g_choke_slice, g_choke_chan, g_last_choke_us);
}

void actuators_set_throttle_us(uint16_t us)
{
    g_last_throttle_us = clamp_servo_us(us);
    set_pwm_us(g_thr_slice, g_thr_chan, g_last_throttle_us);
}

uint16_t actuators_get_choke_us(void)
{
    return g_last_choke_us;
}

uint16_t actuators_get_throttle_us(void)
{
    return g_last_throttle_us;
}
