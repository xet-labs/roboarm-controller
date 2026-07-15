#pragma once
#include <Arduino.h>
#include "config.h"
#include "tasks.h"
#include "state.h"

// =====================================================================
// debugcli — USB `Serial` only. Completely separate from the RPi
// command link (`Serial2`/CmdSerial) and from the protocol.h framing.
// This exists purely so you can plug the ESP32 into a laptop, open
// minicom/screen/Arduino Serial Monitor at 115200, and sanity-check
// the hardware without the RPi/Go commander in the loop at all.
//
// Commands (newline-terminated):
//   help                        list commands
//   demo on|off                 sweep all joints between their limits,
//                                cycle the claw; runs until "demo off"
//                                or "stop"
//   state                       print one snapshot: joint positions,
//                                claw current/mode
//   stream on [ms]|off          repeat `state` every [ms] (default 500)
//   move <joint 0-3> <deg> <ms> single joint move, deg is plain degrees
//   claw <mode 0-2> <pwm 0-255> 0=stop 1=close 2=open
//   stop                        immediate stop, also cancels demo mode
//   ping                        Ping() both SC15 servos, report bus health
//
// NOTE: this is a bench tool. If the RPi is also connected and sending
// commands on Serial2 while you run `demo on` here, both streams land
// in the same g_cmdQueue and will visibly fight each other. That's
// expected — disconnect the RPi link (or just don't run demo mode)
// while the arm is under RPi control.
// =====================================================================

namespace debugcli {

void begin();
void poll();          // call every uartTask loop iteration; non-blocking
void demoTick(uint32_t now);   // call every uartTask loop iteration too

} // namespace debugcli
