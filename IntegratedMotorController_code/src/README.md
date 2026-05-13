# RP2040 DroneCAN Motor Controller Source

This `src` folder is organized around the modular controller path and replaces the older monolithic sweep/logger logic.

## Main behavior

1. **Disarmed / CAN timeout**: relay off, starter off, choke open, throttle at idle (`1900 us`).
2. **Armed, waiting for spin**: relay on, choke closed, throttle at 1500us
3. **Spin detected**: when RPM crosses `RPM_DETECT_THRESHOLD`, choke opens and throttle goes to `1500 us` for `1000 ms`.
4. **Idle wait**: throttle returns to idle and the controller waits until the CAN throttle command is zero before following stick input.
5. **Running**: command percentage follows the requested feedforward curve:
   - `0% -> 1900 us -> 1800 RPM`
   - `100% -> 1450 us -> 4250 RPM`

The RPM PID trims the feedforward pulse but clamps the output within the requested throttle curve endpoints: `1450..1900 us`.

## CAN telemetry

The node publishes DroneCAN node status and ESC status. ESC status includes:

- RPM
- current in amps
- engine temperature converted to kelvin for DroneCAN
- commanded/applied throttle percentage

## Current sensor

Current is implemented for an INA226 on I2C, default address `0x40`, using a `0.05 ohm` shunt. If the INA226 is absent, current publishes as `0 A` and `current_ok=0` prints on USB serial. Set `CURRENT_SENSOR_ENABLED` to `0` in `board_config.h` if not populated.

## Hardware assumptions to check

All wiring constants live in `board_config.h`. The CAN SPI pins are set to SPI0 on GP2/3/4 with CS on GP5 and INT on GP6. Change these if your board uses different pins.

## Servo sweep test from QGroundControl / DroneCAN

A relay-powered servo test mode has been added. It is intentionally only accepted when the controller is disarmed, in `ENGINE_DISARMED`, and RPM is below `RPM_DETECT_THRESHOLD`.

Preferred trigger:

1. Open the DroneCAN node parameters in QGroundControl.
2. Set `SERVO_TEST` to `1`.
3. The parameter is momentary, so it reads back as `0` after the command is accepted.

Test sequence:

1. Relay turns on first.
2. Firmware waits `SERVO_TEST_RELAY_SETTLE_MS` so the servo power rail can settle.
3. Throttle and choke sweep together from their home positions to their far positions.
4. They hold briefly, then sweep back home.
5. Relay turns off and the controller returns to `ENGINE_DISARMED`.

Configured sweep ranges:

- Throttle: `SERVO_TEST_THROTTLE_A_US` to `SERVO_TEST_THROTTLE_B_US`, default `1900 us -> 1450 us`.
- Choke: `SERVO_TEST_CHOKE_A_US` to `SERVO_TEST_CHOKE_B_US`, default `1100 us -> 1900 us`.

Fallback trigger: a raw ESC command of `SERVO_TEST_RAWCOMMAND_VALUE` (`-8192`) on `DRONECAN_ESC_INDEX` also requests the same test. The engine state machine still rejects it unless it is safe.

Note: the parameter server code is guarded by `SERVO_TEST_ENABLE_CAN_PARAM && defined(UAVCAN_PROTOCOL_PARAM_GETSET_ID)`. If your generated `dronecan_msgs.h` does not include `uavcan.protocol.param.GetSet`, regenerate the DroneCAN headers with the parameter DSDL types included, or use the fallback raw-command trigger.

## LED / buzzer behavior

The LED and buzzer are driven from the engine state machine and FC link status:

- No recent FC link: slow LED heartbeat only; buzzer stays off.
- FC link present but disarmed/before arming: slow LED heartbeat with a synchronized short buzzer beep.
- `ENGINE_ARMED_WAIT_FOR_SPIN`: **armed and ready to be primed** indication; LED solid and buzzer continuous while waiting for RPM/spin detection.
- `ENGINE_PRIMING_AFTER_SPIN`: faster LED/buzzer pulse during the 1 second `1500 us` throttle prime.
- `ENGINE_IDLE_WAIT_FOR_ZERO`: primed/holding idle indication; LED solid and buzzer continuous while the controller waits for throttle stick/CAN command to return to zero.
- `ENGINE_RUNNING`: LED solid, buzzer off so normal running is not continuously noisy.
- `ENGINE_SERVO_TEST`: fast synchronized blink/beep while the relay-powered servo sweep is active.

The timing values are in `board_config.h` under `LED / buzzer status indication`.
