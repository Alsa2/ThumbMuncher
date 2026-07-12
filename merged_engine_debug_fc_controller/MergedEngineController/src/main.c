#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "board_config.h"
#include "actuators.h"
#include "buzzer.h"
#include "custom_can_node.h"
#include "dronecan_node.h"
#include "engine_control.h"
#include "mcp2518fd.h"
#include "persistent_config.h"
#include "sensors.h"
#include "status_indicator.h"
#include "custom_can_protocol.h"

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

    return 0.0f;
}

static void apply_identify_override(uint32_t ms)
{
    static bool was_identifying = false;

    if (!custom_can_node_identify_active(ms)) {
        if (was_identifying) {
            // Make the Stop blink/beep command immediate even if the last identify
            // phase left the buzzer PWM duty cycle high. The normal status
            // indicator will redraw the LED/buzzer state on the next loop.
            buzzer_set_enabled(false);
        }
        was_identifying = false;
        return;
    }

    const bool on = ((ms / 120u) & 1u) == 0u;
    gpio_put(PIN_LED, on ? 1 : 0);
    buzzer_set_enabled(on);
    was_identifying = true;
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
    const bool fram_present = persistent_config_init();
    const bool fram_loaded = persistent_config_load_into_runtime();

    printf("RP2040 merged FC/debug motor controller starting\r\n");
    printf("FRAM @0x%02X present=%d settings_loaded=%d\r\n", FRAM_I2C_ADDR, (int)fram_present, (int)fram_loaded);
    printf("FC default: DroneCAN fallback_node=%u fallback_esc_index=%u timeout=%ums\r\n",
           DRONECAN_NODE_ID,
           DRONECAN_ESC_INDEX,
           DRONECAN_FC_TIMEOUT_MS);
    printf("Debug override: custom CAN timeout=%ums command_timeout=%ums\r\n",
           CUSTOM_CAN_DEBUG_TIMEOUT_MS,
           CUSTOM_CAN_COMMAND_TIMEOUT_MS);
    EngineControlRuntimeConfig startup_cfg;
    engine_control_get_runtime_config(&startup_cfg);
    printf("Throttle curve: 0%%=%uus/%.0frpm  100%%=%uus/%.0frpm  start=%uus/%ums\r\n",
           startup_cfg.idle_us,
           (double)startup_cfg.idle_rpm,
           startup_cfg.max_us,
           (double)startup_cfg.max_rpm,
           startup_cfg.start_us,
           startup_cfg.start_hold_ms);
    printf("Runtime/FRAM tuning is shared: debug GUI writes it, FC/DroneCAN and debug both consume it.\r\n");

    if (!mcp2518fd_init()) {
        printf("MCP2518FD init failed\r\n");
        while (true) {
            const uint32_t ms = now_ms();
            status_indicator_update(ms, false, false, ENGINE_FAULT);
            sleep_ms(10);
        }
    }

    custom_can_node_init();
    uint32_t debug_board_id = custom_can_node_get_board_id();
    uint8_t dronecan_node_id = dronecan_node_resolve_node_id_from_board_id(debug_board_id);
    uint8_t dronecan_esc_index = dronecan_node_resolve_esc_index_from_board_id(debug_board_id);
    dronecan_node_init_with_ids(dronecan_node_id, dronecan_esc_index);
    printf("DroneCAN local node_id=%u esc_index=%u motor=%u (node=base %u + board_id %lu; esc=board_id-1; fallback node=%u esc=%u)\r\n",
           dronecan_node_get_local_node_id(),
           dronecan_node_get_esc_index(),
           (unsigned)(dronecan_node_get_esc_index() + 1u),
           DRONECAN_NODE_ID_BASE,
           (unsigned long)debug_board_id,
           DRONECAN_NODE_ID,
           DRONECAN_ESC_INDEX);

    uint32_t next_control_ms = now_ms();
    uint32_t next_print_ms = now_ms() + PRINT_LOOP_MS;
    uint32_t next_node_status_ms = now_ms() + NODE_STATUS_PERIOD_MS;
    uint32_t next_esc_status_ms = now_ms() + ESC_STATUS_PERIOD_MS;
    uint32_t next_board_announce_ms = now_ms() + 100u;
    uint32_t next_custom_telem_a_ms = now_ms() + CUSTOM_CAN_TELEM_A_PERIOD_MS;
    uint32_t next_custom_telem_b_ms = now_ms() + CUSTOM_CAN_TELEM_B_PERIOD_MS;
    uint32_t next_custom_telem_c_ms = now_ms() + CUSTOM_CAN_TELEM_C_PERIOD_MS;
    uint32_t next_auto_status_ms = now_ms() + CUSTOM_CAN_AUTO_STATUS_PERIOD_MS;
    bool config_save_pending = false;
    uint32_t next_config_save_ms = now_ms() + 1000u;
    bool debug_was_alive = false;

    while (true) {
        CanardCANFrame frame;
        int rx_budget = 48;
        while ((rx_budget-- > 0) && mcp2518fd_receive(&frame)) {
            const uint64_t rx_us = to_us_since_boot(get_absolute_time());
            custom_can_node_handle_frame(&frame, rx_us);
            dronecan_node_handle_frame(&frame, rx_us);
        }

        const uint32_t active_board_id = custom_can_node_get_board_id();
        if (active_board_id != debug_board_id) {
            debug_board_id = active_board_id;
            dronecan_node_id = dronecan_node_resolve_node_id_from_board_id(debug_board_id);
            dronecan_esc_index = dronecan_node_resolve_esc_index_from_board_id(debug_board_id);
            dronecan_node_init_with_ids(dronecan_node_id, dronecan_esc_index);
            printf("DroneCAN reconfigured: board_id=%lu node_id=%u esc_index=%u motor=%u\r\n",
                   (unsigned long)debug_board_id,
                   dronecan_node_get_local_node_id(),
                   dronecan_node_get_esc_index(),
                   (unsigned)(dronecan_node_get_esc_index() + 1u));
        }

        const uint32_t ms = now_ms();
        const uint64_t us = to_us_since_boot(get_absolute_time());
        const bool fc_alive = dronecan_node_fc_alive(us);
        const bool debug_alive = custom_can_node_debug_alive(us);
        const bool debug_selected = custom_can_node_selected();
        const bool debug_cmd_alive = debug_alive && debug_selected && custom_can_node_command_alive(us);

        if (debug_was_alive && !debug_alive) {
            // Debug-only powered/test states must not survive probe removal and
            // block normal FC control. The saved FRAM/runtime tuning remains in
            // effect; only active debug procedures are cancelled.
            (void)engine_control_stop_endpoint_auto();
            (void)engine_control_stop_hall_auto_cal();
            (void)engine_control_stop_manual_pwm_test();
            (void)engine_control_set_manual_pwm_bypass(false, 0u);
        }
        debug_was_alive = debug_alive;

        // Default path is FC/DroneCAN. Any live debug probe overrides the FC path;
        // unselected boards stay safely disarmed while the probe is present. Both
        // paths feed the same EngineControl state machine, so FRAM/runtime tuning
        // values such as throttle endpoints, PID, Hall thresholds and startup PWM
        // apply identically once the FC is in control.
        const bool armed = debug_alive
            ? (debug_cmd_alive && custom_can_node_get_armed())
            : (fc_alive && dronecan_node_get_armed());
        const float cmd_pct = debug_alive
            ? (debug_cmd_alive ? custom_can_node_get_cmd_pct() : 0.0f)
            : (fc_alive ? dronecan_node_get_cmd_pct() : 0.0f);

        sensors_update(ms);
        engine_control_set_armed(armed);
        engine_control_set_cmd_pct(cmd_pct);

        if (custom_can_node_consume_stop_auto_request()) {
            (void)engine_control_stop_endpoint_auto();
            (void)engine_control_stop_hall_auto_cal();
            (void)engine_control_stop_manual_pwm_test();
            (void)engine_control_set_manual_pwm_bypass(false, 0u);
        }

        if (custom_can_node_consume_servo_test_request()) {
            const bool accepted = engine_control_request_servo_test(ms);
            printf("debug SERVO_TEST command %s\r\n", accepted ? "accepted" : "rejected");
        }

        if (custom_can_node_consume_start_beep_request()) {
            printf("debug START_BEEP for board_id=%lu\r\n",
                   (unsigned long)custom_can_node_get_board_id());
        }

        if ((int32_t)(ms - next_control_ms) >= 0) {
            next_control_ms += CONTROL_DT_MS;
            engine_control_update(ms);
        }

        const EngineState state = engine_control_get_state();
        // In debug mode, do not use the normal selected-board heartbeat as an
        // audible locator. The selected board should only blink/beep while an
        // explicit IDENTIFY is active or while the engine state itself requires
        // a safety indication. This makes STOP_IDENTIFY actually quiet the board.
        const bool indicator_link_alive = debug_alive ? false : fc_alive;
        status_indicator_update(ms, indicator_link_alive, armed, state);
        apply_identify_override(ms);


        if ((int32_t)(ms - next_node_status_ms) >= 0) {
            dronecan_node_publish_node_status(ms / 1000u,
                                              0u,
                                              0u,
                                              (uint16_t)state);
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

        // Important FC-bus rule: when no debug probe is present, transmit exactly
        // the same traffic family as the original FC-only firmware: DroneCAN node
        // status + ESC status only. The custom 0x1CEB.... debug frames are not
        // DroneCAN transfers; leaving them on the bus makes PX4/UAVCAN report CAN
        // transfer/status errors. A debug probe heartbeat enables these frames.
        if (debug_alive) {
            if ((int32_t)(ms - next_board_announce_ms) >= 0) {
                EngineSensors snap;
                sensors_snapshot(&snap);
                custom_can_node_publish_board_announce(snap.rpm,
                                                       (uint8_t)state,
                                                       armed,
                                                       fc_alive);
                next_board_announce_ms += CUSTOM_CAN_BOARD_ANNOUNCE_MS;
            }

            uint16_t custom_a_ms = CUSTOM_CAN_TELEM_A_PERIOD_MS;
            uint16_t custom_b_ms = CUSTOM_CAN_TELEM_B_PERIOD_MS;
            uint16_t custom_c_ms = CUSTOM_CAN_TELEM_C_PERIOD_MS;
            custom_can_node_get_telem_periods(&custom_a_ms, &custom_b_ms, &custom_c_ms);

            if ((int32_t)(ms - next_custom_telem_a_ms) >= 0) {
                EngineSensors snap;
                sensors_snapshot(&snap);
                custom_can_node_publish_telem_a(snap.rpm,
                                                engine_control_get_output_us(),
                                                (uint8_t)state,
                                                debug_cmd_alive,
                                                armed,
                                                snap.stationary);
                next_custom_telem_a_ms += custom_a_ms;
            }

            if ((int32_t)(ms - next_custom_telem_b_ms) >= 0) {
                custom_can_node_publish_telem_b(engine_control_get_target_rpm(),
                                                (uint16_t)engine_control_get_feedforward_us(),
                                                engine_control_get_pid_correction_us());
                next_custom_telem_b_ms += custom_b_ms;
            }

            if ((int32_t)(ms - next_custom_telem_c_ms) >= 0) {
                EngineSensors snap;
                sensors_snapshot(&snap);
                custom_can_node_publish_telem_c(snap.temperature_c,
                                                snap.current_a,
                                                snap.bus_voltage_v,
                                                sensors_get_runtime_ms());
                next_custom_telem_c_ms += custom_c_ms;
            }

            if ((int32_t)(ms - next_auto_status_ms) >= 0) {
                const bool active = engine_control_endpoint_auto_active();
                custom_can_node_publish_auto_status(engine_control_endpoint_auto_is_max()
                                                        ? CUSTOM_CAN_AUTO_ENDPOINT_MAX
                                                        : CUSTOM_CAN_AUTO_ENDPOINT_IDLE,
                                                    active,
                                                    engine_control_endpoint_auto_target_rpm(),
                                                    engine_control_endpoint_auto_us(),
                                                    engine_control_endpoint_auto_error_rpm());
                HallAutoCalStatus hall_cal_status;
                sensors_get_hall_auto_cal_status(&hall_cal_status);
                custom_can_node_publish_hall_cal_status(&hall_cal_status);
                next_auto_status_ms += CUSTOM_CAN_AUTO_STATUS_PERIOD_MS;
            }
        } else {
            // Keep first debug packets snappy when a probe is plugged in later.
            next_board_announce_ms = ms;
            next_custom_telem_a_ms = ms;
            next_custom_telem_b_ms = ms;
            next_custom_telem_c_ms = ms;
            next_auto_status_ms = ms;
        }

        if (engine_control_consume_config_dirty() ||
            sensors_consume_hall_thresholds_dirty() ||
            custom_can_node_consume_config_dirty()) {
            config_save_pending = true;
            next_config_save_ms = ms + 500u;
        }

        if (config_save_pending && (int32_t)(ms - next_config_save_ms) >= 0) {
            const bool saved = persistent_config_save_from_runtime();
            printf("FRAM settings save %s\r\n", saved ? "OK" : "FAILED");
            config_save_pending = false;
        }

        dronecan_node_process_tx();

        if ((int32_t)(ms - next_print_ms) >= 0) {
            EngineSensors snap;
            sensors_snapshot(&snap);

            printf("mode=%s state=%d fc_alive=%d debug_alive=%d selected=%d armed=%d cmd=%.1f rpm=%.1f target=%.1f ff=%.1f pid=%.1f out=%u auto=%d temp=%.1fC vbus=%.3fV current=%.3fA cfg_rejects=%lu board_id=%lu\r\n",
                   debug_alive ? "DEBUG" : "FC",
                   (int)state,
                   (int)fc_alive,
                   (int)debug_alive,
                   (int)debug_selected,
                   (int)armed,
                   (double)cmd_pct,
                   (double)snap.rpm,
                   (double)engine_control_get_target_rpm(),
                   (double)engine_control_get_feedforward_us(),
                   (double)engine_control_get_pid_correction_us(),
                   engine_control_get_output_us(),
                   (int)engine_control_endpoint_auto_active(),
                   (double)snap.temperature_c,
                   (double)snap.bus_voltage_v,
                   (double)snap.current_a,
                   (unsigned long)custom_can_node_get_config_reject_count(),
                   (unsigned long)custom_can_node_get_board_id());

            next_print_ms += PRINT_LOOP_MS;
        }

        sleep_ms(1);
    }
}
