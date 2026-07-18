// Arduino IDE sketch — this file must live in a folder named
// exactly "roboArm_controller" alongside the other .h/.cpp files.
// See README.md for board settings and library installation.

#include <Arduino.h>
#include "config.h"
#include "tasks.h"
#include "state.h" 

// roboArm ESP32 controller
//
// Deliberately dumb: this firmware has no sequencing, no replay logic,
// no mode concept. It exposes joint moves, claw control, and state
// readback over UART. All arm intelligence (teach/replay, jog mapping,
// vision-guided pick) lives in the RPi Go commander. Any UART master
// speaking the framed protocol in protocol.h can drive this arm.

void setup()
{
    Serial.begin(115200); // USB debug console only — NOT the command link
    delay(200);
    Serial.println("\n[BOOT] roboArm controller starting");

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    g_armState.begin();
    g_cmdQueue = xQueueCreate(16, sizeof(JointCmdMsg));
    if (!g_cmdQueue) {
        Serial.println("[BOOT] FATAL: could not create command queue");
        while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
    }

    // uartTask: command-plane, core 0
    xTaskCreatePinnedToCore(uartTask, "uartTask", 4096, nullptr, 2, nullptr, 0);
    // controlTask: peripheral-plane, core 1
    xTaskCreatePinnedToCore(controlTask, "controlTask", 4096, nullptr, 2, nullptr, 1);

    tone(PIN_BUZZER, 1000, 150); delay(200);
    tone(PIN_BUZZER, 1500, 150); delay(200);
    noTone(PIN_BUZZER);

    Serial.println("[BOOT] tasks running");
}

void loop()
{
    // Everything happens in uartTask / controlTask. Heartbeat only.
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(1000);
}
