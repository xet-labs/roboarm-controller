# roboArm ESP32 controller (Arduino IDE)

Dumb driver tier. Owns servos/motor/sensors and a UART protocol. Owns
**zero** arm intelligence — no modes, no sequencing. That all lives in the RPi Go commander.

<p align="center">
  <video src="docs/demo.mp4" autoplay muted loop controls width="fit-content"></video>
</p>

## Architecture

![Architecture](docs/architecture.png)

This repo is the `controller` tier: it exposes a UART command API and
a separate USB debug console, and drives the servos directly. Every
tool (teach-and-replay, remote control, Xbox input, vision servoing)
lives upstream in `roboarm-commander`, either routed through it or
talking to this controller directly over UART.

## Sketch layout

Arduino IDE requires the sketch folder name to exactly match the
`.ino` file name. This folder is that sketch:

```
roboArm-controller/
  roboArm-controller.ino   <- open this in Arduino IDE
  config.h
  protocol.h  protocol.cpp
  joint.h
  claw.h
  state.h
  tasks.h  tasks.cpp
  debugcli.h  debugcli.cpp
```

Everything is flat on purpose — Arduino IDE compiles every `.cpp` in
the sketch folder as its own translation unit and adds the folder to
the include path automatically, so there's no build-system config to
get right, unlike the PlatformIO layout. Just open the `.ino` and hit
Verify/Upload.

## One-time setup

**1. ESP32 board support**: File → Preferences → Additional Boards Manager
URLs → add `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
→ Tools → Board → Boards Manager → search "esp32" → install.
Then Tools → Board → ESP32 Arduino → **ESP32 Dev Module**.

**2. Libraries** — Tools → Manage Libraries, install:
- **ESP32Servo** (by Kevin Harrington / madhephaestus)
- **Adafruit INA219**

**3. SCServo (SC15 driver)** — not reliably in the Library Manager.
Download Waveshare's SCServo library zip, then Sketch → Include
Library → Add .ZIP Library (or manually copy the `SCServo` folder into
`Documents/Arduino/libraries/`). This is the same library/step
Waveshare's own SC15 docs walk through.

## Firmware internals

Two FreeRTOS tasks, each owning a disjoint set of peripherals so
there's no cross-task bus contention to reason about:

- **`uartTask`** (core 0) — parses the commander UART, validates and
  clamps, pushes resolved commands onto a queue, replies (ACK/ERR/
  STATE/PONG). Touches nothing else.
- **`controlTask`** (core 1) — drains the queue, drives the SC15 bus,
  mg995 PWM, TB6612 claw, polls INA219, publishes `ArmState` (mutex-
  guarded) for `uartTask` to read back.

The USB debug console (`Serial`, 115200) and the commander link
(`Serial2`, pins 16/17, 115200) are **different UARTs**, keeping CLI
and control traffic on separate ports with no framing ambiguity.

## Wire protocol

```
[0xAA][0x55][LEN][CMD][PAYLOAD...][CHECKSUM]
LEN      = 1(cmd) + payload length
CHECKSUM = XOR of LEN, CMD, and all payload bytes
```

| CMD | Dir | Payload | Notes |
|---|---|---|---|
| `0x01` MOVE_JOINT | → | u8 jointId, i16 posDeg10, u16 timeMs | single joint, time-governed move |
| `0x02` MOVE_ALL | → | i16 pos[4]Deg10, u16 timeMs | synchronized 4-joint move |
| `0x03` CLAW_SET | → | u8 mode(0/1/2), u8 pwm | 0=stop 1=close 2=open |
| `0x04` STOP | → | — | jumps the queue, halts everything |
| `0x05` GET_STATE | → | — | triggers `0x81` reply |
| `0x06` SET_TORQUE | → | u8 jointId, u8 enable | SC15 only, stubbed (see below) |
| `0x07` PING | → | u32 token | echoed back, for RPi-side latency stats |
| `0x81` STATE | ← | i16 pos[4]Deg10, u16 clawCurrentMa, u8 clawMode, u32 uptimeMs | published every 50ms, sent on GET_STATE too |
| `0x82` PONG | ← | u32 token | |
| `0x83` ACK | ← | u8 cmdEcho | |
| `0x84` ERR | ← | u8 cmdEcho, u8 errCode | 1=badChecksum 2=badLength 3=unknownCmd 4=badJoint 5=queueFull |

Joint order everywhere: `0=Base 1=Shoulder 2=Elbow 3=Wrist`
(`JointId` enum in `config.h`). `posDeg10` = degrees × 10 (e.g. `900` = 90.0°).

## Verify before flashing

1. **SC15 raw-position range** (`SC15_RAW_MAX`/`SC15_RANGE_DEG` in
   `config.h`) — set to 1023/300° per the generic Feetech SCSCL memory
   map. Some units are 240°; wrong value only affects position
   scaling, not direction/safety.
2. **SC15 bus is true single-wire** — one GPIO (`BUS_UART_SIG_PIN`,
   default 18) does both TX and RX via the ESP32 UART peripheral's
   native `UART_MODE_RS485_HALF_DUPLEX` mode. No external mux/converter
   IC. Servo power comes from a separate 4.8–8.4V supply, not the ESP32.
3. **`SC15Joint::setTorque()` is a stub** — not needed for jog/record/
   replay, only for future hand-guided drag-teach. Left as a
   documented no-op since the exact torque-enable register call
   depends on the installed SCServo library version.
4. **TB6612FNG / INA219 pins** are placeholders in `config.h` — set to
   actual wiring before flashing.

## USB debug console

Separate from everything above — talks over plain USB `Serial`
(115200), not the framed protocol, and never touches the RPi link.
Open with the Arduino Serial Monitor, `minicom -D /dev/ttyUSB0 -b 115200`,
`screen /dev/ttyUSB0 115200`, or similar. A `[PING]` line for both SC15
servos on boot confirms the bus is wired correctly.

```
help                          list commands
demo on|off                   sweep all joints + claw between their
                                configured limits, back and forth,
                                until "demo off" or "stop"
state                         one-shot: joint positions (deg) + claw
                                current (mA) + claw mode
stream on [ms] | stream off   repeat 'state' automatically (default 500ms)
move <joint 0-3> <deg> <ms>   single joint move, plain degrees
                                (0=Base 1=Shoulder 2=Elbow 3=Wrist)
claw <mode 0-2> <pwm 0-255>   0=stop 1=close 2=open
stop                          immediate stop, also cancels demo mode
ping                          re-check both SC15 servos respond
```

`demo on` is a hardware-alive test — no RPi, no jog mapping, just
confirms every actuator moves and every sensor reads. Don't run it
while the RPi is actively sending commands; both land in the same
internal queue and will visibly fight each other.

## License

Apache 2.0 — see [LICENSE](LICENSE).