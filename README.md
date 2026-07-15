# roboArm ESP32 controller (Arduino IDE)

Dumb driver tier. Owns servos/motor/sensors and a UART protocol. Owns
**zero** arm intelligence — no modes, no sequencing, no "demo mode", no
teach/replay. That all lives in the RPi Go commander now.

## Sketch layout

Arduino IDE requires the sketch folder name to exactly match the
`.ino` file name. This folder is that sketch:

```
roboArm_controller/
  roboArm_controller.ino   <- open this in Arduino IDE
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

**1. ESP32 board support** (skip if already installed for the old
`proj-roboArm` sketch): File → Preferences → Additional Boards Manager
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

## Architecture

Two FreeRTOS tasks, each owning a disjoint set of peripherals so
there's no cross-task bus contention to reason about:

- **`uartTask`** (core 0) — parses the commander UART, validates and
  clamps, pushes resolved commands onto a queue, replies (ACK/ERR/
  STATE/PONG). Touches nothing else.
- **`controlTask`** (core 1) — drains the queue, drives the SC15 bus,
  mg995 PWM, TB6612 claw, polls INA219, publishes `ArmState` (mutex-
  guarded) for `uartTask` to read back.

The USB debug console (`Serial`, 115200, via Tools → Serial Monitor)
and the commander link (`Serial2`, pins 16/17, 115200) are **different
UARTs** — the old code multiplexed CLI and control traffic on one
port; splitting them removes a whole class of framing ambiguity.

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

## What changed vs v1 (`proj-roboArm`) — and why

- **Stepper base → mg995 servo base.** All `AccelStepper`/stepper code
  removed. Base is now `PwmJoint` like wrist.
- **Xbox/PS3 controller parsing moved off the ESP32 entirely.**
  `res/controller/*.py`, `other/python/*` (button maps, deadzones,
  packet framing) are retired — that's now the RPi Go app's job. The
  ESP32 never sees a raw controller frame, only resolved joint targets.
- **`demoMode` / 13-keyframe canned sequence deleted.** That's exactly
  the kind of sequencing logic that belongs in the commander (as a
  saved replay profile), not baked into firmware.
- **VL53L0X hole-sensing dropped.** It was already disabled and
  crashing in v1 (`ENABLE_VL53L0X 0`, `LoadProhibited`). Target
  detection is now the desktop AI's job via the Pi camera feed — say
  if this is still required for the report's original scope and I'll
  revisit.
- **`net::` (WiFi AP/STA/TCP/UDP/mDNS) not used.** Transport is
  UART-only. That code still exists in `libXetArduino` for other
  projects that want it.
- **`pins.motors.cpp`'s malloc + placement-new + manual-destructor
  motor registry is gone.** Replaced with plain static `IJoint*`
  instances sized at compile time. Your arm's actuator count never
  changes at runtime — the old system was solving a problem you don't
  have, at the cost of being a very plausible source of your
  `LoadProhibited` crash (raw pointer lifecycle bugs are exactly what
  that error means).
- **`lgc` core-switching kept in spirit, not in code.** The idea —
  runtime-swappable named logic blocks — was good. It's now expressed
  as *mode selection living entirely on the RPi*; the ESP32 has no
  mode concept to switch. If you want a bench-test CLI similar to the
  old `handle::cmd`/`fnv1a` dispatcher for poking servos over USB
  without the RPi attached, say so and I'll bring it back scoped to
  debug-only, gated off the command UART.

## Things to verify on hardware before trusting this blind

1. **SC15 raw-position range** (`SC15_RAW_MAX`/`SC15_RANGE_DEG` in
   `config.h`) — set to 1023/300° per the generic Feetech SCSCL memory
   map from Waveshare's docs. Some units are 240°. Wrong value only
   throws off position scaling, not direction/safety — but check it
   first thing, it's a two-constant fix if wrong.
2. **SC15 bus is true single-wire** — one GPIO (`BUS_UART_SIG_PIN`,
   default 18) does both TX and RX via the ESP32 UART peripheral's
   native `UART_MODE_RS485_HALF_DUPLEX` mode (set in `controlTask`,
   `tasks.cpp`). No external mux/converter IC. Physically: that one
   GPIO to the servo's SIG pin, common GND, servo power from a
   separate 4.8-8.4V supply — not from the ESP32. If servos don't
   respond, check the boot-time ping log first (next section) before
   suspecting the protocol layer.
3. **`SC15Joint::setTorque()` is a stub.** Not needed for jog/record/
   replay (Day 1 goal), only for future hand-guided drag-teach. Left
   as a documented no-op rather than guessed at, since the exact
   torque-enable register call depends on the exact SCServo library
   version you install.
4. **TB6612FNG / INA219 pins** are placeholders (`config.h`) — set to
   your actual wiring before flashing.

## USB debug console

Separate from everything above — this talks over plain USB `Serial`
(115200), not the framed protocol, and never touches the RPi link.
Open it with the Arduino Serial Monitor, `minicom -D /dev/ttyUSB0 -b 115200`,
`screen /dev/ttyUSB0 115200`, or similar. On boot you'll also see a
`[PING]` line for both SC15 servos — that alone tells you if the bus
is wired correctly before you type anything.

```
help                          list commands
demo on|off                   sweep all joints + claw between their
                                configured limits, back and forth,
                                until you type "demo off" or "stop"
state                         one-shot: joint positions (deg) + claw
                                current (mA) + claw mode
stream on [ms] | stream off   repeat 'state' automatically (default 500ms)
move <joint 0-3> <deg> <ms>   single joint move, plain degrees
                                (0=Base 1=Shoulder 2=Elbow 3=Wrist)
claw <mode 0-2> <pwm 0-255>   0=stop 1=close 2=open
stop                          immediate stop, also cancels demo mode
ping                          re-check both SC15 servos respond
```

`demo on` is the "is the hardware alive" test — no RPi, no jog
mapping, just confirms every actuator moves and every sensor reads.
Don't run it at the same time the RPi is actively sending commands —
both land in the same internal queue and will visibly fight each
other. It's a bench tool, not a second control channel.

## Suggested Day-1 bring-up order

1. Flash, confirm boot tone + heartbeat LED, confirm USB serial prints
   incl. the `[PING]` SC15 check (Tools → Serial Monitor, 115200).
2. `ping` again from the console if either SC15 didn't respond at
   boot — fix wiring/ID before going further.
3. `move 1 120 1000` (shoulder to 120°), then `state` — confirm sensed
   position moved. Repeat for joint 2 (elbow).
4. `move 0 ...` / `move 3 ...` for base/wrist — expect commanded, not
   sensed, position back (by design, no feedback on mg995).
5. `claw 1 150` against something soft, `stream on 200` to watch
   current climb and confirm the cutoff fires near
   `CLAW_CURRENT_LIMIT_MA` (mode drops back to 0 on its own).
6. `demo on` for a minute as an end-to-end soak test.
7. Only then wire up the RPi side and drop the USB-serial crutch.
