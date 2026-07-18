#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "config.h"

// =====================================================================
// Two tasks total — matches what this hardware actually needs, no more:
//
//   uartTask    (core 0) — owns CMD_UART parsing + framing/replies,
//                AND polls the USB debug console (debugcli.h/.cpp).
//                Never touches servos/claw directly; only pushes
//                resolved commands onto g_cmdQueue and reads g_armState.
//                The debug console piggybacks here rather than getting
//                its own task because it does the exact same thing
//                uartTask already does — parse a command, validate,
//                enqueue — just from a different UART. Splitting it
//                into a third task would add scheduling complexity for
//                no isolation benefit, since neither one ever blocks
//                on a peripheral.
//
//   controlTask (core 1) — owns the SC15 bus, mg995 PWM, TB6612 claw,
//                and INA219. Drains g_cmdQueue, ticks joint
//                interpolation, polls claw current, refreshes
//                g_armState. This is the only task that ever talks to
//                a peripheral bus, so there's no cross-task bus
//                contention to reason about.
//
// This directly targets the crash pattern in the old firmware: a
// blocking bus operation (UART read, I2C poll) stalling a completely
// unrelated peripheral's timing.
// =====================================================================

enum CmdMsgType : uint8_t {
    MSG_MOVE_JOINT,
    MSG_MOVE_ALL,
    MSG_CLAW_SET,
    MSG_STOP,
    MSG_SET_TORQUE,
};

struct JointCmdMsg {
    CmdMsgType type;
    int16_t    pos[JOINT_COUNT];   // only entries covered by jointMask are meaningful
    uint8_t    jointMask;          // bit i set => pos[i] valid (MOVE_JOINT/MOVE_ALL)
    uint16_t   timeMs;
    uint8_t    clawMode;
    uint8_t    clawPwm;
    uint8_t    torqueJoint;
    bool       torqueEnable;
};

extern QueueHandle_t g_cmdQueue;

void uartTask(void *pv);
void controlTask(void *pv);

// Bench-only: Ping() both SC15 servos on the bus, prints result to USB
// Serial. Lives in tasks.cpp because that's where the bus/joint objects
// are instantiated; called from debugcli.cpp.
void debugPingSc15();
