# Merged FC + Debug Motor Controller

This project merges the original DroneCAN flight-controller firmware with the custom CAN calibration firmware.

## Runtime behavior

- **Normal mode:** listens to the flight controller over DroneCAN/libcanard and behaves like the original FC-controlled board.
- **Debug override mode:** when the RP2040 USB↔CAN debug probe is connected and sending custom `PROBE` / `SELECT` / `COMMAND` heartbeats, the board stops obeying FC throttle commands.
- In debug override, only the selected board accepts throttle, tuning, servo test, and endpoint auto-adjust commands. Unselected boards stay disarmed at 0% command.

## Board ID and selection

At startup the controller first tries to load a saved board ID from FRAM. If no valid ID has been saved yet, it computes a 31-bit fallback ID from the Pico unique board ID plus startup timing. The GUI can change the ID with `SETID`, and the controller writes the new ID immediately to FRAM.

## Debug features added

- Probe/discovery: periodic `BOARD id=...` announcements.
- GUI board picker by ID.
- Blink/beep selected engine.
- Start/stop blink-beep identify controls for the selected engine.
- Panic all engines.
- Existing PID/feedforward tuning and telemetry.
- RPM/Hall sensor threshold tuning with `THRESH <high_raw> <low_raw>`.
- FRAM persistence for board ID, throttle endpoints, endpoint RPMs, PID values, and RPM sensor thresholds using the onboard FM24CL64B-compatible chip at I2C `0x50`.
- Existing servo sweep test.
- New endpoint servo-opening auto-adjust:
  - `AUTO0 <target_rpm> [rate_us_per_s]` slowly adjusts the 0% throttle servo opening until measured RPM approaches the saved 0% RPM target. The search starts at the supplied start PWM, normally 1500 us, and only the PWM endpoint is changed.
  - `AUTO100 <target_rpm> [rate_us_per_s]` does the same for the 100% throttle servo opening.
  - This auto-adjust directly changes the feedforward endpoint servo microseconds; it does not simply remap throttle to target RPM.

## Safety notes

Use the debug mode on a secured test stand with propellers removed or restrained as appropriate. Connecting the debug probe intentionally overrides FC input, so do not leave the probe connected during flight operation.

## Persistent settings

`src/persistent_config.c` owns the FRAM record. It initializes I2C1, validates the record with magic/version/CRC, and applies it after `sensors_init()` and `engine_control_init()`. Runtime changes mark the config dirty; `main.c` debounces writes so endpoint auto-adjust does not hammer FRAM during each control tick.

Persisted fields:

- `idle_rpm`, `max_rpm`, `idle_us`, `max_us`
- `kp_us_per_rpm`, `ki_us_per_rpm_s`, `kd_us_per_rpm_per_s`, `correction_limit_us`
- `hall_threshold_high_raw`, `hall_threshold_low_raw`
- `board_id`



## v3 patch notes
- `SETID` is now persisted in FRAM with the rest of the settings.
- `STOP_IDENTIFY` stops the temporary selected-engine blink/beep pattern once the engine is found.

- `SETID` writes the persistent board ID immediately to FRAM and also keeps a deferred retry pending if that write fails.
- The GUI exposes Start blink/beep and Stop blink/beep, with no separate beep-only button.

## v4 patch notes

- `IDENTIFY` now stays active until `STOP_IDENTIFY`, which matches the GUI flow of starting blink/beep and stopping once the engine is found.
- `STOP_IDENTIFY` now immediately clears the PWM buzzer output and the probe sends it globally for reliable silence.
- Debug selection no longer creates a disarmed heartbeat beep by itself, so stopping identify actually quiets the selected board unless the engine state is actively requesting a safety beep.
- GUI left scroll canvas width is now `700`.
- Long-running automated GUI tasks now show a process-running popup with a large abort button. Abort stops sweeps, endpoint averaging, endpoint auto-adjust, all-board calibration, identify blink/beep, and sends zero throttle/disarm.

## v4 fixes

- Stop identify now clears the controller-side identify latch and disables the buzzer immediately after a stop command.
- While a debug probe is connected, the normal FC-link heartbeat is not treated as an audible locator. This prevents a selected board from continuing to beep only because it is selected in debug mode.
- `SETID` now writes a small verified board-ID backup record at FRAM address `0x0100` in addition to the full settings record at `0x0000`. On boot, this backup record can recover the saved ID even if the full settings record is invalid or stale.
- Full FRAM settings writes are still read back and CRC-verified before reporting success.


## v5 identify stop behavior

`STOP_IDENTIFY` / `FOUND` now latches a debug-indicator mute in the motor controller.
This is stronger than just clearing the identify timer: while the debug probe is alive,
the controller forces the LED and buzzer off after the normal status-indicator update,
so engine-state or selected-board heartbeat logic cannot restart the beeper one loop later.
The mute is cleared by `IDENTIFY`, `SELECT`, `SETID`, or a controller reboot.


## v6 Hall sensor auto-calibration

Added a debug-only Hall raw min/max auto-calibration flow for use with an external autospinner.

- GUI adds **Auto-cal Hall min/max at spinner RPM** with target RPM and duration fields.
- The probe accepts `HALLCAL <target_rpm> [optional_timeout_s]` and `HALLCAL_STOP`; the GUI sends no timeout, so calibration runs until clean or abort.
- The selected motor controller arms only to power the shared spark/Hall rail, keeps the starter off, keeps throttle at idle, captures raw Hall ADC min/max, derives high/low hysteresis thresholds, applies them immediately, and stores min/max plus thresholds in FRAM.
- The controller publishes `HALL_CAL_STATUS` signal-quality progress so the GUI popup can show convergence and close on success/failure.
- Abort stops Hall capture, sends zero throttle/disarm, and leaves the board safe.

## v7 Hall auto-calibration and identify mute fixes

- `SELECT` and `SETID` no longer trigger an automatic beep/blink pulse. Only `IDENTIFY` starts the locator; `STOP_IDENTIFY`/`FOUND` keeps the selected debug indicator muted until `IDENTIFY` is explicitly pressed again.
- Hall auto-calibration now sends `HALLCAL <target_rpm>` from the GUI with no fixed duration. The controller powers the Hall/spark rail, keeps starter off and throttle at idle, and runs until the signal converges or the user aborts.
- The Hall auto-cal algorithm is now one-second-window based. It learns raw min/max, updates temporary thresholds, validates edge timing against the target spinner RPM, requires consecutive stable clean windows, then saves 25/75 hysteresis thresholds and raw min/max to FRAM.
- RPM telemetry is lightly filtered to reduce single-period Hall edge jitter after calibration.

## v8 cleanup: locator beep, ARM, and Hall calibration

- `IDENTIFY` / GUI `Beep + flash selected` is again the only manual locator beeper.
- `STOP_IDENTIFY` only stops that manual locator pattern. It no longer globally mutes the normal armed/safety/status beeper, so ARM feedback is not hidden.
- The GUI ARM button first sends `HALLCAL_STOP` and `AUTO_STOP` so a stuck/aborted calibration state cannot make normal arming look dead.
- Hall auto-calibration was simplified to a raw-signal-first routine:
  - Runs until clean or until aborted.
  - Captures raw Hall min/max windows using the powered Hall rail.
  - Updates temporary hysteresis thresholds from raw min/max every window, so old bad thresholds do not poison the calibration.
  - Accepts only after consecutive stable raw windows with believable repeating edges.
  - Uses the target RPM for quality/validation reporting, but no longer hard-fails only because the spinner is slightly off target.
- Runtime RPM measurement now filters Hall periods and rejects single-sample outlier periods before converting to RPM.

## v9 notes

- Manual debug ARM is now selection-safe. The GUI re-sends `SELECT <board_id>` before arming and before every armed command heartbeat, and the probe also re-sends its selected board before transmitting a targeted `CMD`/`ARM` frame. This fixes the board falling back to disarmed after probe reset, reconnect, or `PANIC_ALL` leaving the bridge in broadcast/empty selection.
- `IDENTIFY` / `Beep + flash selected` also re-sends `SELECT` first, so the locator beep works again after reconnect or probe reset.
- Hall auto-calibration was simplified around robust raw ADC endpoints: it now estimates low/high Hall plateaus from each window using moving-midpoint classification and plateau averages, instead of accepting absolute raw min/max spikes. The target RPM is still shown and used for quality, but it is not a hard gate.


## v10 ARM fix

- Normal debug ARM no longer sends `HALLCAL_STOP`/`AUTO_STOP` before the first armed `CMD`; those cleanup frames could race the command heartbeat and make the board look like it disarmed immediately.
- The controller now handles the rising edge of a normal debug ARM command as the owner transition out of Hall calibration/endpoint auto mode. It stops those special modes once, then arms from the same board-side state machine.
- Endpoint auto-tune is not cancelled by later armed heartbeats because the cleanup only happens on the false→true ARM transition.

## v11 ARM-drop fix

The normal debug ARM path no longer creates a stale Hall-calibration FAILED status when it defensively stops Hall auto-cal on the rising ARM edge. That stale HALLCAL_FAILED telemetry could make the GUI set `command_armed = False` one telemetry tick after pressing ARM, which looked like the board armed for roughly 0.1 s and then shut off.

Changes:
- `engine_control_stop_hall_auto_cal()` is now a no-op if Hall auto-cal is not actually active.
- `sensors_stop_hall_auto_cal()` only publishes FAILED/aborted status if a calibration was active.
- The debug probe now sends the current CMD frame periodically once a board is selected, so command heartbeat is owned by the bridge as well as the GUI.

## v12 startup PWM + direct PWM popup

- Added runtime/FRAM startup settings: `start_us` and `start_hold_ms`.
- `ENGINE_ARMED_WAIT_FOR_SPIN` now commands the saved `start_us` while waiting for RPM.
- `ENGINE_PRIMING_AFTER_SPIN` holds the same saved `start_us` for `start_hold_ms` after first RPM, then returns to idle/PID behavior.
- Removed the old GUI servo sweep button and replaced it with a direct throttle PWM popup using `PWMTEST`.
- `PWMTEST` commands the exact requested microseconds; it never substitutes 1500 us. `PWMTEST_STOP` immediately stops the manual direct-PWM state and returns safe-off.
- FRAM config record bumped to v4 with v3 fallback loading so old PID/feedforward/Hall settings are still read and then upgraded on the next save.

## v13 FC/debug shared settings guarantee

The motor controller now treats the FRAM/runtime configuration as shared control data, not debug-only data. The debug GUI/probe can write tuning values, but the FC/DroneCAN path consumes the same `EngineControlRuntimeConfig` in `engine_control_update()`. This includes throttle endpoints, endpoint RPMs, PID values, Hall thresholds/calibration, and the startup PWM/hold timing.

When the debug probe disappears, active debug-only procedures such as endpoint auto-tune, Hall auto-calibration, and direct manual PWM test are cancelled so they cannot continue blocking flight-controller control. The saved settings remain active.

## v14 GUI edit guard

No controller firmware protocol change was required for v14. The GUI now guards
FRAM/live-status writes into editable fields so focused entries are not
overwritten while the user is typing. The FC/debug shared runtime configuration
from v13 is unchanged.

## v16 selection heartbeat and auto-stop behavior

- The motor controller now treats the selected board ID inside each debug `PROBE`
  heartbeat as a redundant selection heartbeat. This means switching engines in
  the GUI no longer depends on one single `SELECT` frame; if a `SELECT` is lost,
  the next probe heartbeat re-latches the intended selected engine and full
  telemetry resumes.
- Endpoint auto-adjust stop is intended to stop tuning and return the throttle
  command to 0%, not disarm the board. Normal disarm and panic remain separate
  actions.

## v19 FC CAN-bus cleanup

The FC-only firmware only transmits DroneCAN `uavcan.protocol.NodeStatus` and
`uavcan.equipment.esc.Status`. Earlier merged versions also transmitted custom
`0x1CEB....` debug announce/telemetry frames all the time. Those frames are not
DroneCAN transfers, so PX4/UAVCAN can report CAN/transfer errors when the flight
controller is connected.

v19 gates every custom debug TX frame behind a live debug-probe heartbeat. With
no probe connected, the motor controller transmits the same frame family as the
original FC-only code. The MCP2518FD transmit path is still the original padded
16-byte TX object write, so short final DroneCAN frames do not reuse stale bytes.

DroneCAN local node ID is now derived from the saved board identifier when that
identifier is small and human-assigned:

```
node_id = DRONECAN_NODE_ID_BASE + board_id
```

With the default base of 40, board ID 6 publishes as DroneCAN node 46. If the
saved board ID is still a large random discovery ID, the firmware falls back to
the original node ID 42 until you assign a small board ID from the GUI.
