# PX4 Motor Dashboard - Read Only + ESC Listener Polling

This version stays radio-safe and does **not** arm/disarm, run motor-test commands, override actuators, or request MAVLink streams.

## What changed in v15

- Kept only the PX4 ESC listener control.
- Removed the UAVCAN status button.
- The button now toggles polling:
  - `Start ESC polling` repeatedly sends `listener esc_status`.
  - `Stop ESC polling` stops it.
- Added `Poll Hz` in the GUI.
  - Default is `0.5 Hz`.
  - It is clamped to `0.05..2.0 Hz` to avoid hammering a telemetry radio.
- The dashboard is fixed to Motors 1–6.
- ESC slots 7 and 8 are ignored.
- Default CAN ID mapping remains:
  - CAN ID 41 -> Motor 1
  - CAN ID 42 -> Motor 2
  - CAN ID 43 -> Motor 3
  - CAN ID 44 -> Motor 4
  - CAN ID 45 -> Motor 5
  - CAN ID 46 -> Motor 6

## Run

```bash
python3 -m pip install -r px4_motor_dashboard_requirements_readonly.txt
./launch_px4_motor_dashboard_readonly.sh --connection /dev/ttyUSB0,57600
```

For UDP:

```bash
./launch_px4_motor_dashboard_readonly.sh --connection udpin:0.0.0.0:14560
```

## Manual NSH command used by the button

```sh
listener esc_status
```

The parser reads fields like `esc_rpm`, `esc_temperature`, `esc_voltage`, `esc_current`, and `esc_address` from the PX4 shell output.
