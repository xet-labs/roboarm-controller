/*
 * SC15 - Continuous auto-renumber ID 1 to next free ID
 *
 * Every loop iteration:
 * 1. Scans the whole bus (SCAN_ID_MIN..SCAN_ID_MAX), prints every
 *    servo found.
 * 2. If a servo is sitting at ID 1 (e.g. factory default, or a
 *    freshly-added servo you haven't renamed yet):
 *      - walks candidate IDs starting at CANDIDATE_START (2, 3, 4...)
 *      - skips any candidate ID that's already taken by another servo
 *      - assigns the first free candidate ID to the servo at ID 1
 * 3. Re-scans and prints the final state.
 * 4. Waits LOOP_DELAY_MS, then repeats - so you can just plug in the
 *    next factory-fresh servo (always ID 1) and it gets renumbered
 *    automatically on the next pass, without resetting the board.
 *
 * IMPORTANT: only ever have ONE servo sitting at ID 1 on the bus at a
 * time. If two are simultaneously at ID 1, they'll both respond to
 * every command here and can't be told apart - connect one, let it
 * get renumbered, THEN connect the next.
 */

#include <SCServo.h>
#include "driver/uart.h"

SCSCL sc;

#define SIG_PIN 18
#define UART_NUM UART_NUM_1

#define SCAN_ID_MIN       0
#define SCAN_ID_MAX       16
#define MAX_SERVOS        16
#define RENUMBER_FROM_ID  1     // servo ID that triggers a renumber
#define CANDIDATE_START   3     // first ID to try assigning it to
#define LOOP_DELAY_MS     2000  // pause between scan passes

uint8_t foundIds[MAX_SERVOS];
int     numFound = 0;

// Scans SCAN_ID_MIN..SCAN_ID_MAX, fills foundIds[]/numFound, prints result.
void scanBus(const char *label) {
  Serial.printf("\n--- %s ---\n", label);
  numFound = 0;
  for (int id = SCAN_ID_MIN; id <= SCAN_ID_MAX && numFound < MAX_SERVOS; id++) {
    if (sc.Ping(id) != -1) {
      int pos = sc.ReadPos(id);
      Serial.printf("  ID %3d -> responded", id);
      if (pos >= 0) Serial.printf("  (pos raw: %d)\n", pos);
      else          Serial.println("  (pos read failed)");
      foundIds[numFound++] = (uint8_t)id;
    }
  }
  Serial.printf("Total found: %d\n", numFound);
}

bool isIdInFoundList(uint8_t id) {
  for (int i = 0; i < numFound; i++)
    if (foundIds[i] == id) return true;
  return false;
}

// Directly checks the bus (not just the cached list) for a given ID.
bool idRespondsOnBus(uint8_t id) {
  return sc.Ping(id) != -1;
}

// Runs one full check-and-renumber pass. Returns true if a renumber
// happened (so the caller can pause a bit longer if it wants).
bool renumberPass() {
  scanBus("Bus scan");

  if (!isIdInFoundList(RENUMBER_FROM_ID)) {
    Serial.printf("No servo at ID %d - nothing to renumber this pass.\n", RENUMBER_FROM_ID);
    return false;
  }

  Serial.printf("\nServo found at ID %d - searching for a free ID starting at %d...\n",
                RENUMBER_FROM_ID, CANDIDATE_START);

  int newId = -1;
  for (int candidate = CANDIDATE_START; candidate <= SCAN_ID_MAX; candidate++) {
    if (candidate == RENUMBER_FROM_ID) continue;
    if (idRespondsOnBus((uint8_t)candidate)) {
      Serial.printf("  ID %d already taken - skipping.\n", candidate);
      continue;
    }
    newId = candidate;
    Serial.printf("  ID %d is free - using this one.\n", candidate);
    break;
  }

  if (newId == -1) {
    Serial.println("No free ID found in range - skipping this pass.");
    return false;
  }

  Serial.printf("\nRenaming ID %d -> %d...\n", RENUMBER_FROM_ID, newId);
  sc.unLockEprom(RENUMBER_FROM_ID);
  sc.writeByte(RENUMBER_FROM_ID, SCSCL_ID, (uint8_t)newId);
  delay(50);
  sc.LockEprom((uint8_t)newId);
  delay(200);

  if (sc.Ping(newId) != -1) {
    Serial.printf("Success - servo now responds at ID %d.\n", newId);
  } else {
    Serial.println("Verification failed - new ID did not respond. Check wiring/power.");
  }

  if (sc.Ping(RENUMBER_FROM_ID) != -1) {
    Serial.printf("WARNING: ID %d still responds - another servo may still be on that ID.\n",
                  RENUMBER_FROM_ID);
  }

  scanBus("Post-renumber scan");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial1.begin(1000000, SERIAL_8N1, SIG_PIN, SIG_PIN);
  uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
  sc.pSerial = &Serial1;

  Serial.println("Continuous SC15 auto-renumber starting...");
  Serial.println("Plug in one factory-fresh servo (ID 1) at a time.");
}

void loop() {
  bool renamed = renumberPass();
  Serial.println("----------------------------------------");
  delay(renamed ? LOOP_DELAY_MS * 2 : LOOP_DELAY_MS);
}