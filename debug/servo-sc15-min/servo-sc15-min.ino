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
 * This uses the ESP32 UART peripheral's native RS485 half-duplex mode,
 * which time-multiplexes TX/RX on ONE GPIO in hardware. You still need
 * a physical wire from that GPIO to the servo's SIG pin (unavoidable -
 * that's the definition of a one-wire bus), but no separate half-duplex
 * converter IC or resistor/diode combiner circuit is needed.
 *
 * Default servo ID from factory = 1. Default baud is typically 1,000,000
 * on SC15 - confirm with Waveshare's servo config tool if unsure.
 */

#include <SCServo.h>
#include "driver/uart.h"

SCSCL sc;

#define SIG_PIN 18          // single wire: same pin used for RX and TX
#define UART_NUM UART_NUM_1

// IDs of the two servos to test - update if you've reassigned them
const uint8_t SERVO_IDS[] = {2, 3};
const int NUM_SERVOS = sizeof(SERVO_IDS) / sizeof(SERVO_IDS[0]);

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
    int id = sc.Ping(SERVO_IDS[i]);
    Serial.print("ID ");
    Serial.print(SERVO_IDS[i]);
    Serial.print(": ");
    if (id != -1) {
      Serial.println("responded OK");
    } else {
      Serial.println("NO RESPONSE - check wiring/ID/baud");
    }
  }
}

void loop() {
  // Sweep every servo to max, then to mid, one at a time, reporting position before/after
  for (int i = 0; i < NUM_SERVOS; i++) {
    uint8_t id = SERVO_IDS[i];
    int before = sc.ReadPos(id);

    // SCSCL WritePos(id, position, time_ms, speed)
    // position range is 0-1023 (10-bit), not 0-4095 like STS/SM servos
    sc.WritePos(id, 0, 1000, 0);   // move to max over ~1s
    delay(1200);
    int atMax = sc.ReadPos(id);

    sc.WritePos(id, 450, 1000, 0);    // move to mid over ~1s
    delay(1200);
    int atMid = sc.ReadPos(id);

    Serial.print("ID "); Serial.print(id);
    Serial.print(" | before: "); Serial.print(before);
    Serial.print(" -> max: ");   Serial.print(atMax);
    Serial.print(" -> mid: ");   Serial.println(atMid);

    // If a servo isn't actually moving, before/atMax/atMid will all read
    // roughly the same value (or -1 if it's not responding at all).
  }

  Serial.println("---");
  delay(500);
}