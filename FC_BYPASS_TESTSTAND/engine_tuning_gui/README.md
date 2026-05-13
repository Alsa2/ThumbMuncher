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
