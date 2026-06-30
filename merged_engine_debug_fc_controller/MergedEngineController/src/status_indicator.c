#include "status_indicator.h"

#include "board_config.h"
#include "buzzer.h"

#include "pico/stdlib.h"

static void led_set(bool on)
{
    gpio_put(PIN_LED, on ? 1 : 0);
}

static bool pulse_on(uint32_t now_ms, uint32_t period_ms, uint32_t on_ms)
{
    if (period_ms == 0u || on_ms >= period_ms) {
        return true;
    }

    return (now_ms % period_ms) < on_ms;
}

void status_indicator_init(void)
{
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    led_set(false);
    buzzer_set_enabled(false);
}

void status_indicator_update(uint32_t now_ms,
                             bool fc_alive,
                             bool armed,
                             EngineState state)
{
    bool led = false;
    bool buzzer = false;

    switch (state) {
    case ENGINE_DISARMED:
        if (fc_alive) {
            // Flight controller link is present but the system is not armed yet:
            // beep exactly with the LED heartbeat so the operator can verify both.
            led = pulse_on(now_ms, INDICATOR_DISARMED_PERIOD_MS, INDICATOR_DISARMED_ON_MS);
            buzzer = led;
        } else {
            // No recent FC command/status: show a visual heartbeat only. Keep the
            // buzzer quiet so the pre-arm beep specifically means FC is connected.
            led = pulse_on(now_ms, INDICATOR_NO_FC_PERIOD_MS, INDICATOR_NO_FC_ON_MS);
            buzzer = false;
        }
        break;

    case ENGINE_ARMED_WAIT_FOR_SPIN:
        // Armed and ready to be primed: relay is on, throttle is idle,
        // choke is closed, and the controller is waiting for spin detection.
        // User requested continuous indication here.
        led = true;
        buzzer = true;
        break;

    case ENGINE_PRIMING_AFTER_SPIN:
        // Active configured startup PWM/hold-time priming pulse.
        led = pulse_on(now_ms, INDICATOR_PRIMING_PERIOD_MS, INDICATOR_PRIMING_ON_MS);
        buzzer = led;
        break;

    case ENGINE_IDLE_WAIT_FOR_ZERO:
        // Primed and waiting for sticks to return to zero: continuous indication.
        led = true;
        buzzer = true;
        break;

    case ENGINE_RUNNING:
        // Running: solid LED, no continuous buzzer so normal operation is not noisy.
        led = true;
        buzzer = false;
        break;

    case ENGINE_SERVO_TEST:
        // Servo test: fast synchronized blink/beep while relay is on.
        led = pulse_on(now_ms, INDICATOR_SERVO_TEST_PERIOD_MS, INDICATOR_SERVO_TEST_ON_MS);
        buzzer = led;
        break;

    case ENGINE_HALL_AUTO_CAL:
        // Hall calibration: relay/Hall power is on, starter off, external spinner expected.
        led = pulse_on(now_ms, INDICATOR_SERVO_TEST_PERIOD_MS, INDICATOR_SERVO_TEST_ON_MS);
        buzzer = false;
        break;

    case ENGINE_MANUAL_PWM_TEST:
        // Manual PWM test: relay/servo rail on for a bounded direct-position command.
        led = pulse_on(now_ms, INDICATOR_SERVO_TEST_PERIOD_MS, INDICATOR_SERVO_TEST_ON_MS);
        buzzer = false;
        break;

    case ENGINE_FAULT:
    default:
        // Fault/no-link style pattern: fast LED blink and fast beep.
        led = pulse_on(now_ms, INDICATOR_FAULT_PERIOD_MS, INDICATOR_FAULT_ON_MS);
        buzzer = led;
        break;
    }

    // If the flight controller disappears while armed, make that obvious even
    // before the state machine has dropped back through the disarm gate.
    if (armed && !fc_alive && state != ENGINE_HALL_AUTO_CAL && state != ENGINE_MANUAL_PWM_TEST) {
        led = pulse_on(now_ms, INDICATOR_FAULT_PERIOD_MS, INDICATOR_FAULT_ON_MS);
        buzzer = led;
    }

    led_set(led);
    buzzer_set_enabled(buzzer);
}
