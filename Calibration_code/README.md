tio -b 115200 /dev/serial/by-id/usb-Raspberry_Pi_Pico_E465BC8247320722-if00

implement feed forward, and some kind of test command which opens and closes the trottle servo to test them

chat
new https://chatgpt.com/c/69e699af-2380-83ea-86ba-ba4ecfb77ec1
https://chatgpt.com/c/69e699af-2380-83ea-86ba-ba4ecfb77ec1

 # Engine node patch

This patch adds:
- `engine_control.c/.h` state machine
- `sensors.c/.h` for RPM / current / temperature / turn count
- `actuators.c/.h` for relay / starter / choke / throttle outputs
- `pid.c/.h` for the throttle loop
- updated `dronecan_node.c/.h` for RawCommand + ArmingStatus RX and NodeStatus/KeyValue TX
- updated `mcp2518fd.c/.h` with a basic TX FIFO 2 path
- updated `main.c` scheduler
- sample `board_config.h`

## Important assumptions
- MCP2518FD clock is 20 MHz
- CAN is classical 1 Mbit/s
- Hall sensor is on ADC pin 27
- Temperature NTC is on ADC pin 26
- INA219 current sensor is on I2C0 address 0x40
- Relay follows arming
- All chokes close when armed
- Each node starts only when its own RawCommand slot goes above `START_REQUEST_PCT`
- 0% command maps to 500 RPM, 100% maps to 6800 RPM

## PX4/QGC side
- `UAVCAN_ENABLE = 3`
- `UAVCAN_PUB_ARM = 1`
- assign the output index that matches `DRONECAN_ESC_INDEX`

## First bench test
- remove ignition/fuel or otherwise make the engine safe
- verify `armed` flips the relay and closes the choke
- verify commanding your output above ~1% starts the cranker only on the selected node
- verify the choke opens after 15 counted turns
- verify `rpm`, `temp_c`, `current_a`, `runtime_s`, and `state` KeyValue messages appear on the bus