# Engine CAN Tuning GUI

Desktop Python GUI for the RP2040 USB↔CAN bridge and merged engine controller.

## Current features

- Select, identify, arm, disarm, and panic-stop one engine board at a time.
- Continuous selected-board command heartbeat and maintained exact-PWM bypass.
- Live RPM, output PWM, target RPM, feedforward PWM, PID correction, temperature, current, voltage, and runtime.
- PID/startup/Hall-threshold configuration with FRAM pull and save.
- Main-page **0% throttle PWM** quick adjustment plus a dedicated **Feedforward calibration + fitting** window with explicit per-engine Pull and atomic live Send/Commit.
- Linear, polynomial order 1–3, and piecewise-linear feedforward models.
- Indefinite 0% and 100% endpoint searches: start at a chosen PWM, slowly approach the target RPM, then capture only when the operator presses the capture button.
- Automatic intermediate-point search with configurable range, search rate, deadband, stable time, sample time, and per-point timeout.
- Timed direct-PWM sweep that records measured RPM/PWM points without writing the engine model.
- Editable point table, include/exclude controls, manual point entry, CSV import/export, point erase, fit RMSE/max error, and graph overlay.
- Unsaved-field highlighting, selected-engine identity checks, staged-transfer cancellation, and controller pull-back verification after commit.
- Square-wave PID snapshots synchronized to the actual transmitted `CMD` timestamps.
- RPM-triggered single-shot oscilloscope with synchronized RPM, command, and measured PWM traces.
- CSV telemetry export and configurable telemetry periods.

## Install and run

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python3 engine_tuning_gui.py
```

Tk must also be installed with Python. On Debian/Ubuntu, use `sudo apt install python3-tk` when needed.

## Safe feedforward workflow

1. Connect the probe and select exactly one engine.
2. Open **Feedforward calibration + fitting**.
3. Press **Pull model from selected engine**. Pulled points appear as excluded reference points so old model data is not silently mixed into a new fit.
4. Arm the selected engine from the main window.
5. Capture endpoints, run intermediate calibration, run a sweep, load CSV data, or enter points manually.
6. Enable only the points that should influence the fit.
7. Choose the model and parameters, then press **Fit / preview local points**.
8. Review the graph, PWM range, monotonic direction, RMSE, and maximum point error.
9. Press **SEND + LIVE COMMIT to selected engine**. The GUI names the board again before confirmation. The engine may remain running; the controller preserves the current PWM while transferring the model.
10. Wait for **COMMIT VERIFIED**. The GUI pulls the active model back and compares the RPM endpoints and coefficients/knots with what was sent.

Calibration and fitting are local until the final commit. Incomplete transactions do not alter the active model. A board change, serial-write failure, explicit cancel, or five-second controller staging timeout clears an incomplete transfer.

## Feedforward models

The independent variable is normalized throttle command, `x = throttle_percent / 100`.

- **Linear:** two endpoint knots with interpolation.
- **Polynomial:** `PWM_us = c0 + c1*x + c2*x^2 + c3*x^3`, limited to order 1–3.
- **Piecewise linear:** 2–12 ordered knots spanning exactly 0% through 100%.

The current engine installation opens throttle as PWM decreases. The GUI anchors every fit exactly to the enabled 0% and 100% calibration points, then rejects a curve unless it remains within 1000–2000 µs and decreases monotonically from 0% to 100%. The controller independently repeats range and monotonic validation before applying a commit.

## Calibration modes

### Endpoint calibration

**Start 0% calibration** or **Start 100% calibration** begins from the configured starting PWM and slowly searches toward the corresponding target RPM. It runs for an unlimited time. Press **Capture current endpoint + stop** when the result is stable, or abort without capturing.

### Intermediate calibration

The GUI generates targets between the selected minimum and maximum percentages, converts each percentage to a target RPM using the 0%/100% RPM endpoints, slowly searches PWM, waits inside the RPM deadband, averages for the configured sample time, and advances to the next point. A per-point timeout aborts a point that cannot settle.

### Direct-PWM sweep

The sweep moves from the configured start PWM to end PWM over the total time. Incoming RPM samples are normalized using the target-RPM endpoints and plotted as local sweep points. Sweep data is never written automatically.

## Square-wave synchronization

The square-wave snapshot timestamps each successful serial `CMD` write instead of drawing ideal scheduled edges. It overlays measured RPM, actual transmitted command timing, and controller-reported output PWM. The PWM axis is fixed to 1000–2000 µs.

## RPM-triggered oscilloscope

Enter a rising-edge RPM trigger and a total window width. The scope captures half the width before the crossing and half after it, with synchronized RPM, transmitted throttle command, and controller output PWM.

## Maintained PWM bypass

Bypass requires the selected engine to be armed. The GUI and probe maintain the exact PWM periodically. Disarm, panic, selection change, calibration abort, popup close, or `PWMBYPASS_STOP` clears bypass.

## Firmware requirement

This fitting release changes the GUI, USB↔CAN debug probe protocol, controller runtime model, and FRAM record version. Rebuild and flash both:

- `MergedEngineController`
- `MergedEngineDebugProbe`

Old FRAM v3/v4 records are migrated to a two-point linear model using their saved endpoints. Disconnect the debug probe before flight-controller operation because a live probe intentionally overrides DroneCAN commands.

## Endpoint sweep behavior

The direct-PWM fitting sweep always uses the selected engine's exact 0% and 100% PWM endpoints. It holds 0%, ramps without extrapolation or overshoot, holds 100%, then returns to 0%. During the ramp, measured RPM is normalized between the exact configured 0% and 100% RPM targets and used as the throttle-position x-axis. This recovers the nonlinear throttle curve instead of plotting the intentionally linear PWM ramp. Exact endpoint anchors are inserted separately; the source text retains the actual measured endpoint RPM.

## Priming reset diagnostics

The GUI warns when selected-board uptime moves backward. Controller USB diagnostics report whether the RP2040 watchdog caused the last reset. A Hall-period interrupt race that could make RPM disappear and trigger the two-second zero-RPM state fallback has been removed. FRAM writes are deferred during starter/priming states.


## Main-page field protection

Main-page configuration entries are now transactional. Typing marks a field yellow and prevents stale CFG/AUTO telemetry from replacing it after focus moves to the Send button. Send commands are automatically retried and verified with GETCFG; the pending highlight clears only when the selected controller reports the same value. An explicit Pull can replace current fields, but typing after Pull immediately protects that field again.
