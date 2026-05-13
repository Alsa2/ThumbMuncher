# Custom CAN protocol v1

This is the byte-level protocol shared by the motor-controller firmware, the RP2040 USB↔CAN bridge, and the Python GUI.

All frames are **29-bit extended classic CAN**.

| ID | Name | Payload |
|---|---|---|
| `0x1CEB0001` | `COMMAND` | `u8 flags`, `u8 seq`, `u16 throttle_centi_pct`, four reserved bytes |
| `0x1CEB0010` | `PID_KP_KI` | `float32 kp`, `float32 ki` |
| `0x1CEB0011` | `PID_KD_LIMIT` | `float32 kd`, `float32 correction_limit_us` |
| `0x1CEB0020` | `FF_IDLE` | `float32 rpm_at_0_pct`, `u16 throttle_us_at_0_pct`, two reserved bytes |
| `0x1CEB0021` | `FF_MAX` | `float32 rpm_at_100_pct`, `u16 throttle_us_at_100_pct`, two reserved bytes |
| `0x1CEB0030` | `TELEMETRY_RATE` | `u16 telem_a_period_ms`, `u16 telem_b_period_ms`, `u16 telem_c_period_ms`, `u16 reserved` |
| `0x1CEB0100` | `TELEM_A` | `float32 rpm`, `u16 output_us`, `u8 engine_state`, `u8 flags` |
| `0x1CEB0101` | `TELEM_B` | `float32 target_rpm`, `u16 feedforward_us`, `i16 pid_correction_us_x10` |
| `0x1CEB0102` | `TELEM_C` | `i16 temp_c_x10`, `i16 current_mA`, `u16 bus_mV`, `u16 runtime_s` |

Numeric multibyte values are little-endian. Floats are IEEE-754 `float32`.

The bridge serial command `RATE A B C` generates the `TELEMETRY_RATE` frame. The current motor firmware accepts 5..5000 ms for each telemetry period.
