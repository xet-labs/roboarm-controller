/*
 * SC15 Serial Bus Servo - auto-discover + sweep/read test (SCSCL protocol)
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
 * Startup: scans SCAN_ID_MIN..SCAN_ID_MAX, pings every ID, and builds a
 * list of whatever actually responds - no need to hardcode IDs up front.
 * Loop: sweeps every discovered servo between max/mid, printing raw
 * positions, with an explicit "NO RESPONSE" label instead of a bare -1.
 */

#include <SCServo.h>
#include "driver/uart.h"

SCSCL sc;

#define SIG_PIN 18          // single wire: same pin used for RX and TX
#define UART_NUM UART_NUM_1

// Full valid SCSCL ID range is 0-253. Narrow this once you know roughly
// what's in use - scanning the full range is slow.
#define SCAN_ID_MIN 0
#define SCAN_ID_MAX 4

#define MAX_SERVOS 16   // cap on how many discovered IDs we'll track/sweep

uint8_t foundIds[MAX_SERVOS];
int     numFound = 0;

// Wraps ReadPos so callers never have to special-case -1 themselves.
// Returns true and fills posOut on success; returns false (posOut
// untouched) if the servo didn't respond.
bool readPosSafe(uint8_t id, int &posOut) {
  int raw = sc.ReadPos(id);
  if (raw < 0) return false;
  posOut = raw;
  return true;
}

void printPos(const char *label, uint8_t id, bool ok, int pos) {
  Serial.print(label);
  Serial.print(": ");
  if (ok) Serial.print(pos);
  else    Serial.print("NO RESPONSE");
}

void scanForServos() {
  Serial.println("Scanning SC15 bus for servo IDs...");
  Serial.printf("Range: %d - %d\n\n", SCAN_ID_MIN, SCAN_ID_MAX);

  numFound = 0;
  for (int id = SCAN_ID_MIN; id <= SCAN_ID_MAX && numFound < MAX_SERVOS; id++) {
    int pingResult = sc.Ping(id);
    if (pingResult != -1) {
      int pos;
      bool ok = readPosSafe(id, pos);
      Serial.printf("  ID %3d -> responded  (current pos raw: ", id);
      if (ok) Serial.print(pos); else Serial.print("NO RESPONSE");
      Serial.println(")");

      foundIds[numFound++] = (uint8_t)id;
    }
  }

  Serial.println();
  Serial.printf("Scan complete. %d servo(s) found.\n", numFound);
  if (numFound == 0) {
    Serial.println("Check wiring, baud rate, and power to the servos.");
  }
}

void setup() {
  Serial.begin(115200);   // USB monitor

  // Bring up Serial1 normally first (required before switching UART mode)
  Serial1.begin(1000000, SERIAL_8N1, SIG_PIN, SIG_PIN);

  // Switch the UART peripheral into RS485 half-duplex mode:
  // the driver now handles direction switching internally on SIG_PIN.
  uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX);

  sc.pSerial = &Serial1;
  delay(1000);

  Serial.println("SC15 test starting...\n");
  scanForServos();
}

void loop() {
  if (numFound == 0) {
    Serial.println("No servos found - re-scanning...");
    scanForServos();
    delay(1000);
    return;
  }

  // Sweep every discovered servo to max, then to mid, one at a time,
  // reporting position before/after.
  for (int i = 0; i < numFound; i++) {
    uint8_t id = foundIds[i];

    int beforePos;
    bool beforeOk = readPosSafe(id, beforePos);

    // SCSCL WritePos(id, position, time_ms, speed)
    // position range is 0-1023 (10-bit), not 0-4095 like STS/SM servos
    sc.WritePos(id, 0, 1000, 0);   // move to max over ~1s
    delay(1200);
    int atMaxPos;
    bool atMaxOk = readPosSafe(id, atMaxPos);

    sc.WritePos(id, 460, 1000, 0);    // move to mid over ~1s
    delay(1200);
    int atMidPos;
    bool atMidOk = readPosSafe(id, atMidPos);

    Serial.printf("ID %d | ", id);
    printPos("before", id, beforeOk, beforePos);
    Serial.print(" -> ");
    printPos("max", id, atMaxOk, atMaxPos);
    Serial.print(" -> ");
    printPos("mid", id, atMidOk, atMidPos);
    Serial.println();

    // If a servo isn't actually moving, before/atMax/atMid will all read
    // roughly the same value. If it's stopped responding mid-test,
    // you'll now see "NO RESPONSE" instead of a misleading -1.
  }

  Serial.println("---");
  delay(500);
}