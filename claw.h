#pragma once
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include "config.h"

// =====================================================================
// Claw — TB6612FNG-driven N20 gear motor, current-sensed via INA219 so
// closing force is limited by measured motor current rather than a
// fixed timer. mode: 0=stop, 1=close, 2=open.
// =====================================================================
class Claw {
public:
    void begin() {
        pinMode(PIN_CLAW_IN1, OUTPUT);
        pinMode(PIN_CLAW_IN2, OUTPUT);
        pinMode(PIN_CLAW_STBY, OUTPUT);
        digitalWrite(PIN_CLAW_STBY, HIGH); // take TB6612 out of standby

        ledcAttach(PIN_CLAW_PWM, CLAW_PWM_FREQ_HZ, CLAW_PWM_RES_BITS);

        if (!ina_.begin()) {
            Serial.println("[CLAW] WARN: INA219 not detected — current limiting disabled");
            inaOk_ = false;
        } else {
            inaOk_ = true;
        }

        setMode(0, 0);
    }

    void setMode(uint8_t mode, uint8_t pwmVal) {
        mode_ = mode;
        switch (mode) {
        case 1: // close
            digitalWrite(PIN_CLAW_IN1, HIGH);
            digitalWrite(PIN_CLAW_IN2, LOW);
            break;
        case 2: // open
            digitalWrite(PIN_CLAW_IN1, LOW);
            digitalWrite(PIN_CLAW_IN2, HIGH);
            break;
        default: // stop
            digitalWrite(PIN_CLAW_IN1, LOW);
            digitalWrite(PIN_CLAW_IN2, LOW);
            pwmVal = 0;
            mode_ = 0;
            break;
        }
        ledcWrite(PIN_CLAW_PWM, pwmVal);
    }

    // Call every CONTROL_TICK_MS. Polls current, enforces stall/clamp cutoff.
    void update() {
        if (!inaOk_) return;
        uint32_t now = millis();
        if (now - lastPollMs_ < 20) return;
        lastPollMs_ = now;

        float mA = ina_.getCurrent_mA();
        lastCurrentMa_ = (uint16_t)fabsf(mA);

        if (mode_ != 0 && lastCurrentMa_ >= CLAW_CURRENT_LIMIT_MA) {
            setMode(0, 0); // stall / target grip force reached — cut power
        }
    }

    uint16_t lastCurrentMa() const { return lastCurrentMa_; }
    uint8_t  mode() const { return mode_; }

private:
    Adafruit_INA219 ina_;
    bool     inaOk_ = false;
    uint8_t  mode_ = 0;
    uint16_t lastCurrentMa_ = 0;
    uint32_t lastPollMs_ = 0;
};
