/*
 * SC15 - Force-set servo ID (no availability checks)
 *
 * Unconditionally unlocks OLD_ID, writes NEW_ID, locks it back up.
 * Does NOT check whether OLD_ID actually responds first, and does NOT
 * check whether NEW_ID is already taken by another servo - it just
 * performs the write. Use this only when you know exactly what's on
 * the bus (e.g. a single servo connected).
 *
 * After the write, scans the whole bus and lists every responding ID.
 */

#include <SCServo.h>
#include "driver/uart.h"

SCSCL sc;

#define SIG_PIN 18
#define UART_NUM UART_NUM_1

#define OLD_ID 3
#define NEW_ID 2

#define SCAN_ID_MIN 0
#define SCAN_ID_MAX 16

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial1.begin(1000000, SERIAL_8N1, SIG_PIN, SIG_PIN);
  uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
  sc.pSerial = &Serial1;


  Serial.println("Done. Scanning bus...\n");

  int found = 0;
  for (int id = SCAN_ID_MIN; id <= SCAN_ID_MAX; id++) {
    if (sc.Ping(id) != -1) {
      int pos = sc.ReadPos(id);
      Serial.printf("  ID %3d -> responded", id);
      if (pos >= 0) Serial.printf("  (pos raw: %d)\n", pos);
      else          Serial.println("  (pos read failed)");
      found++;
    }
  }
  Serial.printf("\nTotal found: %d\n", found);
}

void loop() {
  // nothing - runs once
}