#pragma once
#include <Arduino.h>

// =====================================================================
// UART 0 — RPi commander link (via USB or dedicated GPIO UART)
//   ESP32 <-> RPi Zero 2 W. This IS the "any UART master" interface.
// =====================================================================
#define CMD_UART_NUM      2          // Serial2
#define CMD_UART_BAUD     115200
#define CMD_UART_RX_PIN   16
#define CMD_UART_TX_PIN   17

// =====================================================================
// UART 1 — SC15 bus servo link, TRUE single-wire.
//   ESP32's UART peripheral runs in native RS485 half-duplex mode: one
//   GPIO does both TX and RX, direction switching handled in hardware
//   by the peripheral itself. No external mux/converter IC needed.
//   (See controlTask setup in tasks.cpp for the uart_set_mode() call
//   that turns this on — Serial1.begin() alone is NOT single-wire.)
// =====================================================================
#define BUS_UART_NUM         1               // Serial1
#define BUS_UART_DRIVER_NUM  UART_NUM_1      // must match BUS_UART_NUM
#define BUS_UART_BAUD        1000000
#define BUS_UART_SIG_PIN     19              // single wire: TX and RX both here

// =====================================================================
// mg995 PWM servos (plain hobby PWM, no feedback)
// =====================================================================
#define PIN_SERVO_BASE    5   // J0 — Base
#define PIN_SERVO_WRIST   18   // J3 — Wrist / end-effector

// =====================================================================
// SC15 bus servo IDs (smart, position-feedback capable)
//   IDs must be pre-programmed onto the servos once via Waveshare's
//   tool/demo before this firmware is any use — see README.
// =====================================================================
#define SC15_ID_SHOULDER  2    // J1
#define SC15_ID_ELBOW     3    // J2

// SC15 raw-position <-> degree scaling.
// ASSUMPTION: 0-1023 raw maps to a 300 deg mechanical range, per the
// generic Feetech SCSCL memory map. VERIFY against your specific SC15
// datasheet/firmware — some units are 240 deg. If wrong, only the
// scale factor here needs to change, nothing else in the codebase.
#define SC15_RAW_MAX      1023
#define SC15_RANGE_DEG    300

// =====================================================================
// TB6612FNG — N20 claw motor driver
// =====================================================================
#define PIN_CLAW_IN1      27
#define PIN_CLAW_IN2      26
#define PIN_CLAW_PWM      25
#define PIN_CLAW_STBY     13
#define CLAW_PWM_CHANNEL  0
#define CLAW_PWM_FREQ_HZ  20000
#define CLAW_PWM_RES_BITS 8

// =====================================================================
// INA219 — claw current sense (I2C)
// =====================================================================
#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22
#define CLAW_CURRENT_LIMIT_MA   1200   // cut power above this: stall/clamp protection

// =====================================================================
// Misc onboard IO
// =====================================================================
#define PIN_LED           2
#define PIN_BUZZER        4

// =====================================================================
// Joint indexing — single source of truth for joint order everywhere
// =====================================================================
enum JointId : uint8_t {
    JOINT_BASE     = 0,   // mg995, PWM
    JOINT_SHOULDER = 1,   // SC15, bus ID SC15_ID_SHOULDER
    JOINT_ELBOW    = 2,   // SC15, bus ID SC15_ID_ELBOW
    JOINT_WRIST    = 3,   // mg995, PWM
    JOINT_COUNT    = 4
};

struct JointLimits {
    int16_t minDeg10;   // tenths of a degree
    int16_t maxDeg10;
    int16_t homeDeg10;
};

// Conservative defaults — narrow these once you've bench-tested each
// joint's real safe range (avoid frame/cable collisions).
static const JointLimits kJointLimits[JOINT_COUNT] = {
    /* BASE     */ { 0, 1800, 900 },
    /* SHOULDER */ { 200, 1800, 900 },
    /* ELBOW    */ { 200, 1800, 900 },
    /* WRIST    */ { 0, 1800, 900 },
};

// Control loop cadence
#define CONTROL_TICK_MS     20    // joint interpolation + claw poll cadence
#define STATE_PUBLISH_MS    50    // how often shared ArmState is refreshed

// USB debug-CLI demo mode — bench test only, sweeps all joints between
// their configured min/max limits and cycles the claw. Never touches
// the RPi command link; purely a "is the hardware alive" sanity check.
#define DEMO_TRAVEL_MS       2000
#define DEMO_PAUSE_MS        300
