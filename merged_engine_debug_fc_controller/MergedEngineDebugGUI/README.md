# Engine CAN Tuning GUI

Desktop Python GUI for the RP2040 USB↔CAN bridge.

## Features

- Connect/disconnect to the bridge over USB CDC serial.
- Settings panel with live telemetry sampling-period control (`RATE A B C`) for CAN streams.
- Persistent `engine_gui_settings.json` file that reloads serial, tuning, sweep, square-wave, and telemetry-rate fields on the next launch.
- ARM / DISARM / panic zero+disarm.
- Continuous command heartbeat, matching the motor firmware watchdog.
- Manual throttle slider.
- One-shot throttle ramp sweep with configurable start, end, and duration.
- Separate feedforward calibration sweep window that captures RPM vs commanded throttle and exports a dedicated calibration CSV.
- Two endpoint auto-calibration buttons: command 0% or 100% throttle, average measured RPM for 10 s, write that average into the matching feedforward RPM endpoint, and resend tuning.
- Square-wave PID tuning mode with configurable minimum, maximum, and full-cycle period.
- Separate period-snapshot window that overlays RPM responses with the centered throttle pulse; older captured periods fade for easier comparison.
- Live PID and feedforward tuning packets.
- RPM time graph.
- Live target/feedforward/PID/temperature/current/bus/runtime display.
- CSV telemetry export.

## Install

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 engine_tuning_gui.py
```

On some Linux systems, install Tk support if it is missing:

```bash
sudo dnf install python3-tkinter
# or on Debian/Ubuntu:
sudo apt install python3-tk
```

## Expected bridge text protocol

The bridge accepts lines such as:

```text
CMD 1 25.0
PID 0.035 0.012 0.000 175
FF0 2200 1850
FF100 4250 1450
RATE 10 20 200
```

It emits telemetry lines such as:

```text
TEL A rpm=2198.392 out_us=1848 state=4 flags=3
TEL B target_rpm=2200.000 ff_us=1850 pid_us=1.2
TEL C temp_c=25.3 current_a=0.114 vbus_v=11.983 runtime_s=72
```

## Square-wave tuning window

The square-wave mode uses a 50% high-duty pulse centered inside each full period:

- first quarter-period at the minimum throttle,
- middle half-period at the maximum throttle,
- final quarter-period back at the minimum throttle.

That phase choice makes each captured period line up cleanly in the snapshot window, with the high-throttle pulse visually centered at `t = 0`. The GUI stores the 12 most recent completed periods and fades older RPM traces automatically.

## Telemetry sampling settings

The Settings window sends `RATE A B C` to the bridge, which forwards a custom CAN rate frame to the motor controller. The three values are telemetry periods in milliseconds:

- `A`: RPM, output pulse, engine state, flags.
- `B`: target RPM, feedforward pulse, PID correction.
- `C`: temperature, current, bus voltage, runtime.

Lower period values mean a higher CAN/serial sample rate. For PID tuning, reducing `A` first is usually the most valuable.

## Feedforward calibration sweep

The calibration window runs an independent start→end throttle sweep and captures `command_throttle_pct` against measured RPM, plus controller-side output/target/feedforward/PID columns. Save that CSV to fit or manually choose better 0% and 100% feedforward endpoints.

## Automatic feedforward endpoint RPM capture

The two auto-set buttons in the PID/feedforward panel hold the existing throttle command endpoint and average incoming `TEL A` RPM samples for 10 seconds:

- **0% button:** commands the existing 0%/idle throttle opening, averages RPM for 10 s, writes the result into `0% RPM`, then reapplies tuning.
- **100% button:** commands the existing 100% throttle opening, averages RPM for 10 s, writes the result into `100% RPM`, then reapplies tuning.

These buttons update the **RPM endpoints only**. The throttle-servo microsecond openings remain whatever is currently entered in `0% us` and `100% us`. A cancel button stops the capture and forces commanded throttle back to 0%.

## v2 multi-engine/debug-probe additions

The GUI now understands the merged controller/probe protocol:

- Board discovery through `BOARD id=...` announcements.
- Board selector by startup board ID.
- Blink/beep selected engine and startup beep before all-engine calibration steps.
- Panic all engines.
- Existing PID, feedforward, sweep, square-wave, telemetry, CSV, and endpoint-RPM averaging features are kept.
- New endpoint servo-opening auto-adjust buttons:
  - **Auto-adjust selected 0% us to 0% RPM** sends `AUTO0` using the `0% RPM` target.
  - **Auto-adjust selected 100% us to 100% RPM** sends `AUTO100` using the `100% RPM` target.
  - The controller slowly changes the endpoint servo microsecond value itself, and the GUI updates the visible `0% us` / `100% us` field from `AUTO` status frames.
- Clockwise all-board buttons run the discovered board list in sorted-ID order, beeping/blinking each selected engine before starting its endpoint auto-adjust dwell.

Do not use the debug probe in flight; the merged firmware treats a live probe as an intentional FC override.

## v8 GUI notes

- `Beep + flash selected` starts the manual locator pattern on the selected engine.
- `Stop blink/beep` sends the stop command several times; it only stops the manual locator, not normal ARM/status beeps.
- `ARM` now clears any active Hall/endpoint auto mode before sending the arm heartbeat, which prevents a previous calibration routine from trapping the board in calibration mode.
- Hall auto-calibration runs until the board reports stable raw Hall min/max windows. Keep the external spinner steady at the target RPM and abort from the popup if needed.

## v9 behavior

Before arming, identifying, servo testing, or Hall auto-calibrating, the GUI explicitly re-selects the current board ID. This avoids a common failure mode where the GUI still shows a board, but the USB-CAN probe was reset or left in broadcast mode, so the controller rejects `CMD 1 ...` and immediately reports disarmed.

## v11 ARM-drop fix

The GUI now ignores stale inactive `HALLCAL DONE_OK/FAILED` telemetry for manual ARM/disarm state. Only a Hall-calibration process that this GUI started is allowed to clear `command_armed` on completion/failure. This prevents old Hall-cal status packets from disarming the board immediately after pressing ARM.

## v14 edit-protection behavior

The GUI now protects configuration fields while the user is editing them. If a
CAN/FRAM refresh, endpoint auto-tune status, Hall calibration result, or saved
settings refresh arrives while the cursor is inside a bound entry field, the GUI
keeps the user's typed value and skips that live overwrite. The user can then
press the relevant Apply button to send the edited value to the board/FRAM.

This protection applies to the main tuning fields and the direct PWM popup
because both use the same Tk text variables.

## v15 GUI telemetry fallback

If selecting a board temporarily stops full `TEL A/B/C` telemetry, the GUI now uses the selected board's `BOARD` announcement as a fallback source for RPM/state/live-link updates. The status link shows `LIVE TEL` when full telemetry is fresh and `LIVE BOARD` when the fallback is feeding the graph.

## v16 GUI fixes

- Selecting a board from the drop-down now immediately sends several `SELECT`
  frames plus a `PROBE`, and clears stale TEL-A freshness so BOARD announcements
  can keep the live values and graph moving while TEL-A resumes.
- `Stop endpoint auto-adjust` now sends `AUTO_STOP` and commands 0% throttle
  while preserving the current ARM state. Use Disarm/Panic if you want to remove
  arm state.
