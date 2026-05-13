#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "actuators.h"
#include "board_config.h"
#include "buzzer.h"
#include "can_frame.h"
#include "custom_can_node.h"
#include "engine_control.h"
#include "mcp2518fd.h"
#include "sensors.h"
#include "status_indicator.h"

static inline uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1200);

    buzzer_init();
    status_indicator_init();
    actuators_init();
    sensors_init();
    engine_control_init();

    printf("RP2040 custom-CAN motor controller starting\r\n");
    printf("Command timeout: %u ms\r\n", CUSTOM_CAN_COMMAND_TIMEOUT_MS);
    printf("Default curve: 0%%=%uus/%.0frpm  100%%=%uus/%.0frpm  prime=%uus/%ums\r\n",
           THROTTLE_IDLE_US,
           (double)THROTTLE_IDLE_RPM,
           THROTTLE_MAX_US,
           (double)THROTTLE_MAX_RPM,
           THROTTLE_PRIME_US,
           PRIME_AFTER_SPIN_MS);

    if (!mcp2518fd_init()) {
        printf("MCP2518FD init failed\r\n");
        while (true) {
            const uint32_t ms = now_ms();
            status_indicator_update(ms, false, false, ENGINE_FAULT);
            sleep_ms(10);
        }
    }

    custom_can_node_init();

    uint32_t next_control_ms = now_ms();
    uint32_t next_print_ms = now_ms() + PRINT_LOOP_MS;
    uint32_t next_telem_a_ms = now_ms() + CUSTOM_CAN_TELEM_A_PERIOD_MS;
    uint32_t next_telem_b_ms = now_ms() + CUSTOM_CAN_TELEM_B_PERIOD_MS;
    uint32_t next_telem_c_ms = now_ms() + CUSTOM_CAN_TELEM_C_PERIOD_MS;

    while (true) {
        CanFrame frame;
        int rx_budget = 32;
        while ((rx_budget-- > 0) && mcp2518fd_receive(&frame)) {
            custom_can_node_handle_frame(&frame, to_us_since_boot(get_absolute_time()));
        }

        const uint32_t ms = now_ms();
        const uint64_t us = to_us_since_boot(get_absolute_time());
        const bool command_alive = custom_can_node_command_alive(us);
        const bool armed = command_alive && custom_can_node_get_armed();
        const float cmd_pct = command_alive ? custom_can_node_get_cmd_pct() : 0.0f;

        sensors_update(ms);
        engine_control_set_armed(armed);
        engine_control_set_cmd_pct(cmd_pct);

        if ((int32_t)(ms - next_control_ms) >= 0) {
            next_control_ms += CONTROL_DT_MS;
            engine_control_update(ms);
        }

        const EngineState state = engine_control_get_state();
        status_indicator_update(ms, command_alive, armed, state);

        if ((int32_t)(ms - next_telem_a_ms) >= 0) {
            EngineSensors snap;
            sensors_snapshot(&snap);
            custom_can_node_publish_telem_a(snap.rpm,
                                             engine_control_get_output_us(),
                                             (uint8_t)state,
                                             command_alive,
                                             armed,
                                             snap.stationary);
            next_telem_a_ms += CUSTOM_CAN_TELEM_A_PERIOD_MS;
        }

        if ((int32_t)(ms - next_telem_b_ms) >= 0) {
            custom_can_node_publish_telem_b(engine_control_get_target_rpm(),
                                             (uint16_t)engine_control_get_feedforward_us(),
                                             engine_control_get_pid_correction_us());
            next_telem_b_ms += CUSTOM_CAN_TELEM_B_PERIOD_MS;
        }

        if ((int32_t)(ms - next_telem_c_ms) >= 0) {
            EngineSensors snap;
            sensors_snapshot(&snap);
            custom_can_node_publish_telem_c(snap.temperature_c,
                                             snap.current_a,
                                             snap.bus_voltage_v,
                                             sensors_get_runtime_ms());
            next_telem_c_ms += CUSTOM_CAN_TELEM_C_PERIOD_MS;
        }

        if ((int32_t)(ms - next_print_ms) >= 0) {
            EngineSensors snap;
            sensors_snapshot(&snap);
            printf("state=%d link=%d armed=%d cmd=%.2f rpm=%.1f target=%.1f ff=%.1f pid=%.1f out=%u temp=%.1fC vbus=%.3fV current=%.3fA cfg_rejects=%lu age_ms=%lu\r\n",
                   (int)state,
                   (int)command_alive,
                   (int)armed,
                   (double)cmd_pct,
                   (double)snap.rpm,
                   (double)engine_control_get_target_rpm(),
                   (double)engine_control_get_feedforward_us(),
                   (double)engine_control_get_pid_correction_us(),
                   engine_control_get_output_us(),
                   (double)snap.temperature_c,
                   (double)snap.bus_voltage_v,
                   (double)snap.current_a,
                   (unsigned long)custom_can_node_get_config_reject_count(),
                   (unsigned long)custom_can_node_command_age_ms(us));
            next_print_ms += PRINT_LOOP_MS;
        }

        sleep_ms(1);
    }
}
