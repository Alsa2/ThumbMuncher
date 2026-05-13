#include "buzzer.h"
#include "board_config.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

static uint g_slice;
static uint g_chan;
static uint16_t g_wrap;

void buzzer_init(void)
{
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    g_slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    g_chan  = pwm_gpio_to_channel(PIN_BUZZER);

    // 2 kHz tone
    g_wrap = 62499;
    pwm_set_wrap(g_slice, g_wrap);
    pwm_set_chan_level(g_slice, g_chan, 0);
    pwm_set_enabled(g_slice, true);
}

void buzzer_set_enabled(bool enabled)
{
    pwm_set_chan_level(g_slice, g_chan, enabled ? (g_wrap / 2u) : 0u);
}