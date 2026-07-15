/*
 * SC15 Serial Bus Servo - basic sweep/read test (SCSCL protocol)
 * Library: SCServo (Waveshare/Feetech) - install from:
 *   https://github.com/waveshare/servo_serial_bus (or Arduino Library Manager: "SCServo")
 * Board: ESP32 Dev Module
 *
 * Wiring (single-wire half-duplex, no external converter):
 *   ESP32 GPIO18 -> SIG (servo data line, single wire)
 *   Servo power: 4.8-8.4V separate supply, common GND with ESP32
 *
 * Default servo ID from factory = 1. Default baud is typically 1,000,000
 * on SC15 - confirm with Waveshare's servo config tool if unsure.
 */

#include <SCServo.h>
#include "driver/uart.h"

SCSCL sc;

#define SIG_PIN 18          // single wire: same pin used for RX and TX
#define UART_NUM UART_NUM_1

// Per-servo motion profile: each servo can have its own range, speed, and timing.
struct ServoProfile {
  uint8_t id;
  int posA;        // first target position (0-1023)
  int posB;        // second target position (0-1023)
  int timeA_ms;     // move time to posA
  int timeB_ms;     // move time to posB
  int dwell_ms;     // pause after each move before reading position
  uint16_t speed;   // 0 = use time-based profile only, or set a speed cap
};

// Update IDs/ranges/speeds here - this is the only place you need to touch
// to give each servo different behavior.
ServoProfile profiles[] = {
  // id, posA, posB, timeA_ms, timeB_ms, dwell_ms, speed
  {1,  600,  200,  1000,  600,  1200, 0},   // servo 1: wide sweep, fast return
  {2,  750,  400,   700, 1500,   900, 0},   // servo 2: narrower sweep, slow return
};
const int NUM_SERVOS = sizeof(profiles) / sizeof(profiles[0]);

void setup() {
  Serial.begin(115200);   // USB monitor

  // Bring up Serial1 normally first (required before switching UART mode)
  Serial1.begin(1000000, SERIAL_8N1, SIG_PIN, SIG_PIN);

  // Switch the UART peripheral into RS485 half-duplex mode:
  // the driver now handles direction switching internally on SIG_PIN.
  uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX);

  sc.pSerial = &Serial1;
  delay(1000);

  Serial.println("SC15 test starting...");

  // Ping each servo - confirms it responds on the bus before we try moving it
  for (int i = 0; i < NUM_SERVOS; i++) {
    int id = sc.Ping(profiles[i].id);
    Serial.print("ID ");
    Serial.print(profiles[i].id);
    Serial.print(": ");
    if (id != -1) {
      Serial.println("responded OK");
    } else {
      Serial.println("NO RESPONSE - check wiring/ID/baud");
    }
  }
}

void loop() {
  // Sweep every servo through its OWN pattern, one at a time,
  // reporting position before/after.
  for (int i = 0; i < NUM_SERVOS; i++) {
    ServoProfile &p = profiles[i];
    int before = sc.ReadPos(p.id);

    // SCSCL WritePos(id, position, time_ms, speed)
    // position range is 0-1023 (10-bit), not 0-4095 like STS/SM servos
    sc.WritePos(p.id, p.posA, p.timeA_ms, p.speed);
    delay(p.timeA_ms + p.dwell_ms);
    int atA = sc.ReadPos(p.id);

    sc.WritePos(p.id, p.posB, p.timeB_ms, p.speed);
    delay(p.timeB_ms + p.dwell_ms);
    int atB = sc.ReadPos(p.id);

    Serial.print("ID "); Serial.print(p.id);
    Serial.print(" | before: "); Serial.print(before);
    Serial.print(" -> A("); Serial.print(p.posA); Serial.print("): "); Serial.print(atA);
    Serial.print(" -> B("); Serial.print(p.posB); Serial.print("): "); Serial.println(atB);

    // If a servo isn't actually moving, before/atA/atB will all read
    // roughly the same value (or -1 if it's not responding at all).
  }

  Serial.println("---");
  delay(500);
}