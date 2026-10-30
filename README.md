# ESP32 motor controller example

The MCU side of the UGV UART protocol: the ROS driver `motor_control_node` sends motor commands, this firmware drives the motors. Full spec: [protocol_spec.md](../../docs/motor_control_node/protocol_spec.md).

## Integrating into your firmware

1. **Copy `components/ugv_protocol/`** as is. It is the same codec the ROS driver uses: plain C++20, no heap, no exceptions. Do not edit it. `just motor-fw-check` verifies the copy.
2. **Give the protocol its own UART**, 115200 8N1. Feed every received byte to `FrameParser::push()`. On `FrameReady`, handle `parser.frame()`.
3. **Handle four messages:**

   | Pi sends | You do | You reply |
   |---|---|---|
   | HELLO | nothing | INFO (version, channel count, limit, watchdog) |
   | ENABLE | clear faults, start watchdog → ENABLED | STATUS |
   | DISABLE | stop motors → DISABLED | STATUS |
   | SET | if ENABLED: validate, drive motors, restart watchdog | — |

4. **In your loop:**
   - **Watchdog:** in ENABLED with no valid SET for 300 ms → stop motors, FAILSAFE.
   - **STATUS:** send every 100 ms.

`src/main.cpp` does all of this in about 300 lines. Use it as the template.

## Rules that matter

- **Boot in DISABLED** with motors stopped. Never start moving on your own.
- **Only ENABLE leaves FAILSAFE.** Ignore SET until then.
- **Stop means now:** DISABLE and FAILSAFE set every output to zero, no ramp.
- **SET is all or nothing.** If any entry is invalid, reject the frame and do not restart the watchdog.
- **No ramp in ROS mode.** The ROS side already limits acceleration.

## What to adapt

| In `src/main.cpp` | What |
|---|---|
| `kUart`, `kUartTxPin`, `kUartRxPin` | UART and pins |
| `kPins`, `kMotorCount`, `drive()` | motor outputs, `duty = power × PWM_MAX / 10000` |
| `ros_is_active_source()` | return `false` while RC/ELRS/WEB drives; ENABLE is then refused |
| `send_status()` | `battery_mv`, if measured |

## Bench setup

LOLIN S2 mini + L298N, UART1 to Raspberry Pi 5:

| Pi 5 | S2 mini |
|---|---|
| pin 8 (TXD0) | GPIO18 (RX) |
| pin 10 (RXD0) | GPIO17 (TX) |
| pin 6 (GND) | GND |

Motors: GPIO3/5 → L298N IN1/IN2 (channel 0), GPIO7/9 → IN3/IN4 (channel 1).

```bash
pio run -t upload --upload-port /dev/cu.usbmodemXX   # bootloader first: hold 0, tap RST
pio device monitor --port /dev/cu.usbmodemYY         # port name changes after reset
```

A heartbeat every 2 s shows the state, frames received from the Pi, CRC errors and the last reset reason.

Pi side: `ros2 launch motor_control_node ugv.launch.py device:=/dev/ttyAMA0`.
