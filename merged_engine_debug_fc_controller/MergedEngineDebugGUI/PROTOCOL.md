# Custom CAN protocol v2

This is the byte-level protocol shared by the merged motor-controller firmware, the RP2040 USB↔CAN debug probe, and the Python GUI.

All frames are **29-bit extended classic CAN**.

## IDs

| ID | Name | Payload |
|---|---|---|
| `0x1CEB0001` | `COMMAND` | `u8 flags`, `u8 seq`, `u16 throttle_centi_pct`, four reserved bytes |
| `0x1CEB0002` | `SELECT` | `u32 board_id`, `u8 flags`, three reserved bytes |
| `0x1CEB0003` | `PROBE` | Optional selected/debug info; currently not required by controller |
| `0x1CEB0004` | `ACTION` | `u8 action`, `u8 flags`, six reserved/action bytes |
| `0x1CEB0005` | `SET_BOARD_ID` | `u32 old_id_or_broadcast`, `u32 new_id` |
| `0x1CEB0006` | `AUTO_ENDPOINT` | `u8 endpoint`, `u8 flags`, `u16 rate_us_per_s`, `float32 target_rpm` |
| `0x1CEB0007` | `HALL_AUTO_CAL` | `u8 flags`, `u8 reserved`, `u16 timeout_ms`, `float32 external_spinner_target_rpm` |
| `0x1CEB0010` | `PID_KP_KI` | `float32 kp`, `float32 ki` |
| `0x1CEB0011` | `PID_KD_LIMIT` | `float32 kd`, `float32 correction_limit_us` |
| `0x1CEB0020` | `FF_IDLE` | `float32 rpm_at_0_pct`, `u16 throttle_us_at_0_pct`, two reserved bytes |
| `0x1CEB0021` | `FF_MAX` | `float32 rpm_at_100_pct`, `u16 throttle_us_at_100_pct`, two reserved bytes |
| `0x1CEB0023` | `START_CONFIG` | `u16 start_us`, `u16 hold_after_rpm_ms`, four reserved bytes |
| `0x1CEB0024` | `MANUAL_PWM_TEST` | `u16 pwm_us`, `u16 hold_ms`, four reserved bytes |
| `0x1CEB0025` | `FF_MODEL_META` | `u8 model_type`, `u8 point_count`, `u8 polynomial_order`, five reserved bytes |
| `0x1CEB0026` | `FF_MODEL_COEFF` | `u8 coefficient_index`, reserved byte, `float32 coefficient`, two reserved bytes |
| `0x1CEB0027` | `FF_MODEL_POINT` | `u8 point_index`, reserved byte, `u16 throttle_pct_x100`, `u16 pwm_us`, two reserved bytes |
| `0x1CEB0028` | `FF_MODEL_COMMIT` | `u8 action`, seven reserved bytes |
| `0x1CEB0029` | `FF_MODEL_RPM` | `float32 rpm_at_0_pct`, `float32 rpm_at_100_pct` |
| `0x1CEB0022` | `RPM_THRESH` | `u16 hall_high_raw`, `u16 hall_low_raw`, four reserved bytes |
| `0x1CEB0030` | `TELEMETRY_RATE` | `u16 telem_a_ms`, `u16 telem_b_ms`, `u16 telem_c_ms`, `u16 reserved` |
| `0x1CEB0100` | `TELEM_A` | `float32 rpm`, `u16 output_us`, `u8 engine_state`, `u8 flags` |
| `0x1CEB0101` | `TELEM_B` | `float32 target_rpm`, `u16 feedforward_us`, `i16 pid_correction_us_x10` |
| `0x1CEB0102` | `TELEM_C` | `i16 temp_c_x10`, `i16 current_mA`, `u16 bus_mV`, `u16 runtime_s` |
| `0x1CEB0110` | `BOARD_ANNOUNCE` | `u32 board_id`, `u16 rpm`, `u8 state`, `u8 flags` |
| `0x1CEB0111` | `AUTO_STATUS` | `u8 endpoint`, `u8 active`, `u16 endpoint_us`, `i16 rpm_error_x10`, `u16 target_rpm` |
| `0x1CEB0112` | `HALL_CAL_STATUS` | `u8 status`, `u8 quality_pct`, `u16 min_raw`, `u16 max_raw`, `u16 target_rpm` |
| `0x1CEB0113` | `CONFIG_PID_A` | Pulled `kp`, `ki` |
| `0x1CEB0114` | `CONFIG_PID_B` | Pulled `kd`, correction limit |
| `0x1CEB0115` | `CONFIG_FF_IDLE` | Pulled 0% RPM/PWM endpoint |
| `0x1CEB0116` | `CONFIG_FF_MAX` | Pulled 100% RPM/PWM endpoint |
| `0x1CEB0117` | `CONFIG_MISC` | Pulled startup and Hall threshold values |
| `0x1CEB0118` | `CONFIG_FF_MODEL_META` | Pulled model type/count/order |
| `0x1CEB0119` | `CONFIG_FF_MODEL_COEFF` | Pulled polynomial coefficient |
| `0x1CEB011A` | `CONFIG_FF_MODEL_POINT` | Pulled linear/piecewise knot |

Numeric multibyte values are little-endian. Floats are IEEE-754 `float32`.

## Selection/override behavior

The merged motor firmware normally obeys DroneCAN from the flight controller. Receipt of custom debug-probe traffic marks debug override alive. While debug override is alive, FC throttle is ignored. Only the currently selected engine accepts normal debug commands; unselected engines remain disarmed.

`SELECT` flags:

- bit0 `SELECT`: board accepts debug commands if `board_id` matches its startup ID or is `0xFFFFFFFF`.
- bit1 `CLEAR`: clear selected state and disarm debug command state.

## Actions

`ACTION` IDs:

- `1`: identify selected engine with LED/buzzer blink until `STOP_IDENTIFY`.
- `2`: run selected engine's existing safe servo sweep.
- `3`: stop endpoint auto-adjust.
- `4`: reserved compatibility beep/identify action.
- `5`: global disarm/zero/stop-auto.
- `6`: stop identify blink/beep. The probe broadcasts this globally from the GUI stop button for reliable silence.

`ACTION` flag bit0 makes supported actions global.

## Endpoint auto-adjust

`AUTO_ENDPOINT` endpoint:

- `0`: tune the 0% / idle endpoint servo opening.
- `1`: tune the 100% / max endpoint servo opening.

Flags:

- bit0: enable.
- bit1: global/all boards.

When active, the controller slowly adjusts the chosen endpoint's servo microsecond value toward the requested RPM. This is different from simply commanding throttle-to-RPM through the normal PID map.


## Hall sensor raw min/max auto-calibration

The debug probe serial command is:

```text
HALLCAL <external_spinner_target_rpm> [optional_timeout_s_0_to_60]
HALLCAL_STOP
```

`HALLCAL` arms the selected board only so the shared spark/Hall power rail is energized. During the capture the starter stays off, throttle is held at the idle endpoint, and an external autospinner should hold the engine at the target RPM. The controller samples the raw Hall ADC directly in one-second windows, derives temporary hysteresis thresholds from the raw span, and keeps running until it sees several consecutive clean windows whose edge timing matches the requested spinner RPM. It then applies wider 25/75 hysteresis thresholds and saves raw min/max plus high/low thresholds to FRAM.

The GUI button **Auto-cal Hall min/max at spinner RPM** starts this routine and shows the normal abort popup. Abort sends `HALLCAL_STOP`, zero throttle, and disarm.


## Atomic fitted feedforward model

The serial commands accepted by the debug probe are:

```text
FFMODEL <type> <point_count> <polynomial_order>
FFRPM <rpm_at_0_pct> <rpm_at_100_pct>
FFCOEFF <index_0_to_3> <value>
FFPOINT <index_0_to_11> <throttle_pct_0_to_100> <pwm_us_1000_to_2000>
FFCOMMIT
FFRESET
FFCANCEL
```

Model types are `0=linear`, `1=polynomial`, and `2=piecewise linear`. Polynomial coefficients are low-order first and use `x = throttle_percent / 100`:

```text
pwm_us = c0 + c1*x + c2*x^2 + c3*x^3
```

`FFMODEL` starts a fresh controller-side staging transaction. `FFRPM` plus all required coefficient or knot frames fill that transaction. No active runtime or FRAM value changes until `FFCOMMIT`. The commit is accepted only when:

- the complete set of required frames was received,
- RPM endpoints are finite and increasing,
- every evaluated PWM remains inside 1000–2000 µs,
- the curve spans a nonzero PWM range and remains monotonic,
- point models have 2–12 strictly ordered knots starting at 0% and ending at 100%,
- polynomial order is 1–3,
- the complete staged model passes endpoint, range, and monotonic validation.

Commit/reset may occur while running. The controller preloads PID state so the actuator PWM is unchanged at the model-swap instant. `FFCANCEL` discards staging without changing the active model. An incomplete staging transaction also expires after five seconds before any later commit is processed. `FFRESET` replaces the active model with a two-point linear curve using the selected engine's current endpoint PWM values.

The GUI performs an explicit `GETCFG` pull after commit and compares the returned selected-board RPM endpoints and coefficients/knots against the sent model. A bridge `OK` line alone is not treated as proof that the controller accepted the model.

## FRAM-persisted settings

The controller saves these settings in the onboard FM24CL64B-compatible FRAM at I2C address `0x50` on the board I2C1 lane:

- 0% and 100% RPM endpoints plus the validated linear, polynomial, or piecewise feedforward model.
- PID gains and correction limit.
- RPM/Hall sensor high/low ADC raw thresholds.
- Hall raw min/max values captured by `HALLCAL`.
- Persistent debug board ID set from the GUI.

The record is CRC-protected. If the record is missing or invalid, firmware defaults from `board_config.h` stay active.

## RPM sensor thresholds

The debug probe serial command is:

```text
THRESH <high_raw_0_4095> <low_raw_0_4095>
```

`high_raw` must be greater than `low_raw`. The merged controller applies the thresholds immediately and saves them to FRAM through the same deferred-save path as PID/feedforward settings.


## v3 patch notes
- `SETID` is now persisted in FRAM with the rest of the settings.
- `STOP_IDENTIFY` stops the temporary selected-engine blink/beep pattern once the engine is found.

## v4 patch notes

- `IDENTIFY` is now latched: selected engine blink/beep continues until `STOP_IDENTIFY`/`FOUND` is sent.
- The debug probe sends `STOP_IDENTIFY` as a global stop action so any actively identifying engine is silenced even if the GUI selection changed.
- In debug mode, merely selecting a board no longer produces the normal audible link heartbeat. The buzzer/LED locator is controlled by explicit identify actions or safety/engine-state indications.

## v4 GUI / identify behavior notes

- `STOP_IDENTIFY`, `STOP_BLINK`, `IDENTIFY_STOP`, and `FOUND` now send a global stop-identify action so any board that is currently blinking/beeping as a locator is silenced.
- `IDENTIFY` is now latched until one of the stop-identify commands is sent. This makes the GUI workflow: press identify, find the engine, press **Stop blink/beep**.
- Automated GUI routines now open a modal popup that says the process is running and includes a large **ABORT PROCESS** button. Abort sends `AUTO_STOP`, `STOP_IDENTIFY`, `PANIC_ALL`, and `CMD 0 0`.
- The GUI left canvas was widened to `tk.Canvas(left_container, width=700, ...)`.

## v12 additions

### STARTCFG

```
STARTCFG <start_throttle_us> <hold_after_rpm_ms>
```

Sends `CUSTOM_CAN_ID_START_CONFIG` to the selected board. The controller saves these fields in the same FRAM tuning record as PID, feedforward, board ID, and Hall thresholds.

`start_throttle_us` is commanded exactly while the armed controller is waiting for the first believable RPM. It is **not** replaced by 1500 us. After RPM is detected, the same PWM is held for `hold_after_rpm_ms`, then the controller returns to idle/PID behavior.

### PWMTEST

```
PWMTEST <throttle_us> [hold_ms]
PWMTEST_STOP
```

Sends `CUSTOM_CAN_ID_MANUAL_PWM_TEST` to the selected board. It replaces the old GUI servo-sweep button. The board powers the relay/servo rail, keeps the starter off, commands the exact PWM for a bounded hold time, and aborts if RPM is detected. `PWMTEST_STOP` sends the special zero/zero payload and immediately returns the controller to safe-off instead of waiting for the hold timeout.

## FC/debug shared configuration

All configuration packets sent by the GUI are stored in the controller runtime and then saved to the onboard FRAM. These values are not debug-only. After the debug probe is removed or times out, the normal DroneCAN/flight-controller path uses the same saved settings:

- 0% and 100% RPM/feedforward PWM endpoints
- PID gains and correction limit
- Hall/RPM sensor thresholds and learned min/max
- startup throttle PWM and hold-after-RPM time
- board ID

A live debug probe still intentionally overrides FC commands for safety and bench calibration. Disconnect or stop the debug probe before flight-controller operation.

## v17 GUI telemetry freeze fix

Python/Tk 3.14 can return a transient ttk Combobox popdown focus path while the GUI is processing serial lines.  In v16 this could raise `KeyError: 'popdown'` inside the edit guard, which killed the Tk `after()` serial pump.  The visible symptom was that the graph and live values worked briefly after selecting an engine, then froze and showed stale even though the bridge was still receiving data.

v17 catches that focus-path case, treats it as "not actively editing", and also wraps individual bridge-line parsing so one malformed/stale line can never stop the telemetry pump.  If telemetry still goes stale after an engine switch, the GUI reasserts `SELECT <id>` and `PROBE` about once per second until data resumes.

Endpoint auto-adjust Abort/Stop behavior remains non-disarming: it sends `AUTO_STOP`, commands 0% throttle, and preserves the current ARM state.  Use Panic/Disarm for an actual disarm.

## Maintained PWMBYPASS behavior

```text
PWMBYPASS <1000..2000>
PWMBYPASS_STOP
```

The bridge accepts bypass only when a board is selected and the bridge command state is armed. It refreshes the armed command first, then transmits the exact PWM bypass. While active, the bridge retransmits the bypass on its 100 ms command heartbeat. `CMD 0 ...`, `ARM 0`, `CLEAR_SELECT`, `PANIC_ALL`, or `PWMBYPASS_STOP` clears the maintained bypass state.
