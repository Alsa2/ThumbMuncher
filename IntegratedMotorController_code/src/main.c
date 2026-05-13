#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "board_config.h"
#include "actuators.h"
#include "buzzer.h"
#include "dronecan_node.h"
#include "engine_control.h"
#include "mcp2518fd.h"
#include "sensors.h"
#include "status_indicator.h"

static inline uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static float applied_throttle_pct_for_telem(void)
{
    const EngineState st = engine_control_get_state();

    if (st == ENGINE_RUNNING) {
        return engine_control_get_cmd_pct();
    }

    // During priming and idle-wait, the controller is intentionally not following
    // the CAN throttle stick yet, so report 0% commanded power.
    return 0.0f;
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

    printf("RP2040 DroneCAN motor controller starting\r\n");
    printf("Servo test: QGC param SERVO_TEST=1, fallback RawCommand=%d; relay settles %ums before sweep\r\n",
           SERVO_TEST_RAWCOMMAND_VALUE,
           SERVO_TEST_RELAY_SETTLE_MS);
    printf("Throttle curve: 0%%=%uus/%.0frpm  100%%=%uus/%.0frpm  prime=%uus/%ums\r\n",
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

    dronecan_node_init();

    uint32_t next_control_ms = now_ms();
    uint32_t next_print_ms = now_ms() + PRINT_LOOP_MS;
    uint32_t next_node_status_ms = now_ms() + NODE_STATUS_PERIOD_MS;
    uint32_t next_esc_status_ms = now_ms() + ESC_STATUS_PERIOD_MS;

    while (true) {
        CanardCANFrame frame;
        int rx_budget = 32;
        while ((rx_budget-- > 0) && mcp2518fd_receive(&frame)) {
            dronecan_node_handle_frame(&frame, to_us_since_boot(get_absolute_time()));
        }

        const uint32_t ms = now_ms();
        const uint64_t us = to_us_since_boot(get_absolute_time());
        const bool fc_alive = dronecan_node_fc_alive(us);
        const bool armed = fc_alive && dronecan_node_get_armed();
        const float cmd_pct = fc_alive ? dronecan_node_get_cmd_pct() : 0.0f;

        sensors_update(ms);
        engine_control_set_armed(armed);
        engine_control_set_cmd_pct(cmd_pct);

        // if (dronecan_node_consume_servo_test_request()) {
        //     const bool accepted = engine_control_request_servo_test(ms);
        //     printf("SERVO_TEST command %s\r\n", accepted ? "accepted" : "rejected");
        // }

        if ((int32_t)(ms - next_control_ms) >= 0) {
            next_control_ms += CONTROL_DT_MS;
            engine_control_update(ms);
        }

        const EngineState state = engine_control_get_state();
        status_indicator_update(ms, fc_alive, armed, state);

        if ((int32_t)(ms - next_node_status_ms) >= 0) {
            dronecan_node_publish_node_status(ms / 1000u,
                                              0u,                  // health OK
                                              0u,                  // mode operational
                                              (uint16_t)state);    // vendor status = EngineState
            next_node_status_ms += NODE_STATUS_PERIOD_MS;
        }

        if ((int32_t)(ms - next_esc_status_ms) >= 0) {
            EngineSensors snap;
            sensors_snapshot(&snap);

            dronecan_node_publish_esc_status(snap.rpm,
                                             snap.current_a,
                                             snap.temperature_c,
                                             applied_throttle_pct_for_telem());
            next_esc_status_ms += ESC_STATUS_PERIOD_MS;
        }

        dronecan_node_process_tx();

        if ((int32_t)(ms - next_print_ms) >= 0) {
            EngineSensors snap;
            sensors_snapshot(&snap);

            printf("state=%d fc_alive=%d armed=%d cmd=%.1f rpm=%.1f target=%.1f ff=%.1f pid=%.1f out=%u temp=%.1fC vbus=%.3fV current=%.3fA current_ok=%d age_ms=%lu\r\n",
                   (int)state,
                   (int)fc_alive,
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
                   (int)snap.current_sensor_present,
                   (unsigned long)dronecan_node_fc_age_ms(us));

            next_print_ms += PRINT_LOOP_MS;
        }

        sleep_ms(1);
    }
}
