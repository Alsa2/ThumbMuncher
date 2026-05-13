# PX4 Six Motor Dashboard v9

## What was fixed from the raw dump

The raw dump proved that `listener esc_status` arrives in 70-byte MAVLink shell chunks. v8 was parsing too early: it saw the ESC timestamp first, marked Motor 1 connected, and then ignored the later complete block with rpm/current/temperature because the counter/timestamp key was unchanged.

v9 waits until the full shell listener block has arrived and ignores incomplete ESC reports. It now parses:

- `esc_rpm`
- `esc_voltage`
- `esc_current`
- `esc_temperature`
- `esc_address`
- `actuator_function`

It also treats PX4 `SYS_STATUS.voltage_battery = 65535` as unknown instead of displaying it as 65.535 V.

## Run

MAVProxy:

```bash
mavproxy.py --master=/dev/ttyACM0,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560
```

Dashboard:

```bash
chmod +x launch_px4_motor_dashboard_v9.sh
./launch_px4_motor_dashboard_v9.sh
```

The launcher connects to:

```text
udpin:0.0.0.0:14560
```

## Expected Motor 1 values from your dump

Motor 1 should show roughly:

- RPM: `0`
- ESC voltage: `0.0 V`
- ESC current: about `-0.00056 A`
- Temperature: about `16.35 C`
- Address: `42`
- Actuator function: `101`

## Safety

Remove props before using arm, force-arm, overrides, motor tests, sweeps, or SERVO_TEST.
