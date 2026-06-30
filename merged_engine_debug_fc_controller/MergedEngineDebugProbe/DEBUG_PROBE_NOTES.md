# RP2040 USB-CAN Debug Probe

This is the updated bridge/probe firmware for the merged engine controller.

Serial commands include:

- `PROBE` / `SCAN` — ask engines to announce their board IDs.
- `SELECT <board_id|ALL>` — select one engine, or all engines for emergency broadcast actions.
- `CLEAR_SELECT` — clear selection and disarm debug command state.
- `IDENTIFY` / `BLINK` — blink/beep selected engine.
- `STOP_IDENTIFY` / `STOP_BLINK` / `FOUND` — stop the selected engine blink/beep after you found it.
- `CMD <0|1> <throttle_pct>` — armed flag plus throttle command heartbeat.
- `PID <kp> <ki> <kd> <limit_us>` — runtime PID update.
- `FF0 <rpm> <us>` / `FF100 <rpm> <us>` — feedforward endpoint update.
- `THRESH <high_raw> <low_raw>` — update RPM/Hall sensor ADC thresholds; high must be greater than low and both must be 0..4095.
- `AUTO0 <target_rpm> [rate_us_per_s] [start_us]` / `AUTO100 <target_rpm> [rate_us_per_s] [start_us]` — slowly auto-adjust the selected endpoint servo opening.
- `AUTO_STOP` — stop endpoint auto-adjust.
- `SERVO_TEST` — run the existing safe servo sweep on the selected controller.
- `SETID <new_id>` or `SETID <old_id|ALL> <new_id>` — persistent board ID override saved by the controller in FRAM.
- `PANIC_ALL` — broadcast disarm/zero/stop-auto to every board.

The probe emits `BOARD`, `TEL A/B/C`, and `AUTO` text lines for the GUI.


## v3 patch notes
- `SETID` is now persisted in FRAM with the rest of the settings.
- `STOP_IDENTIFY` stops the temporary selected-engine blink/beep pattern once the engine is found.


## v6 Hall auto-calibration commands

- `HALLCAL <target_rpm> [duration_s]`: ask the selected board to arm the shared spark/Hall rail, keep starter off, and capture raw Hall ADC min/max while an external autospinner holds the requested RPM.
- `HALLCAL_STOP`: abort/stop Hall auto-calibration.
- `AUTO_STOP` also sends the Hall-cal stop frame for safety.

The bridge prints `HALLCAL status=... progress=... min=... max=... span=...` status lines from the selected controller.

## v7 HALLCAL command

`HALLCAL <target_rpm>` now sends timeout `0`, meaning the motor controller runs Hall calibration until the signal is clean/stable or until `HALLCAL_STOP`/abort. An optional second argument can still be used as a safety timeout in seconds for manual CLI testing.

## v15 selected telemetry keepalive

The bridge now sends `SELECT <board_id>` together with its periodic `CMD` heartbeat while a board is selected. This keeps the controller's selected latch alive and prevents `TEL A/B/C` telemetry from disappearing after the GUI selects an engine.

## v16 AUTO_STOP behavior

`AUTO_STOP` now clears the bridge-owned throttle percentage to 0% and sends a
selected command heartbeat immediately, but it does not clear the bridge ARM
state. This matches the GUI behavior: stopping endpoint auto-adjust returns the
engine to 0% throttle without disarming it.
