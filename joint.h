#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include <SCServo.h>
#include "config.h"

// =====================================================================
// IJoint — common interface so the control task never needs to know
// whether it's talking to a smart bus servo or a dumb PWM servo.
//
// moveTo() starts a move; tick() must be called frequently (every
// CONTROL_TICK_MS) so PWM joints can software-interpolate toward their
// target over `timeMs` (SC15 does this in hardware, so its tick() is a
// no-op — but the caller doesn't need to care).
// =====================================================================

class IJoint {
public:
    virtual ~IJoint() = default;
    virtual void begin() = 0;
    virtual void moveTo(int16_t targetDeg10, uint16_t timeMs) = 0;
    virtual void tick() {}                       // advance in-progress software moves
    virtual int16_t readPos() = 0;                // deg10; sensed if hasFeedback(), else last-commanded
    virtual void setTorque(bool enable) {}        // no-op unless supported (SC15)
    virtual void stop() = 0;
    virtual bool hasFeedback() const = 0;
};

// ---------------------------------------------------------------------
// SC15 — Feetech/Waveshare SCS-protocol bus servo (shoulder, elbow).
// Real closed-loop position feedback. Shares one half-duplex UART bus
// across all SC15 joints — instantiate one SCSCL per bus, pass by ref.
// ---------------------------------------------------------------------
class Sc15Joint : public IJoint {
public:
    Sc15Joint(SCSCL &bus, uint8_t servoId) : bus_(bus), id_(servoId) {}

    void begin() override {
        lastCommandedDeg10_ = kJointLimits[0].homeDeg10; // caller sets real limits via moveTo on init
    }

    void moveTo(int16_t targetDeg10, uint16_t timeMs) override {
        int raw = deg10ToRaw(targetDeg10);
        bus_.WritePos(id_, raw, timeMs, 0); // time-governed move; speed=0 lets time dominate
        lastCommandedDeg10_ = targetDeg10;
    }

    int16_t readPos() override {
        int raw = bus_.ReadPos(id_);
        if (raw >= 0) lastReadDeg10_ = rawToDeg10(raw);
        return lastReadDeg10_;
    }

    // Bench-only: does the servo respond on the bus at all.
    bool ping() { return bus_.Ping(id_) != -1; }

    void setTorque(bool enable) override {
        // Feetech SCSCL exposes torque enable via a status register write;
        // left as a stub — wire up bus_.EnableTorque(id_, enable) once
        // confirmed against the exact SCServo fork's API, needed only for
        // future drag-teach, not required for Day-1 jog/record/replay.
    }

    void stop() override {
        // No dedicated "stop" on SCSCL; re-commanding current sensed
        // position with a short time is the standard way to halt.
        int16_t here = readPos();
        moveTo(here, 50);
    }

    bool hasFeedback() const override { return true; }

private:
    static int deg10ToRaw(int16_t deg10) {
        float deg = deg10 / 10.0f;
        int raw = (int)((deg * SC15_RAW_MAX) / SC15_RANGE_DEG + 0.5f);
        return constrain(raw, 0, SC15_RAW_MAX);
    }
    static int16_t rawToDeg10(int raw) {
        float deg = (float)raw * SC15_RANGE_DEG / SC15_RAW_MAX;
        return (int16_t)(deg * 10.0f);
    }

    SCSCL   &bus_;
    uint8_t  id_;
    int16_t  lastCommandedDeg10_ = 0;
    int16_t  lastReadDeg10_ = 0;
};

// ---------------------------------------------------------------------
// PwmJoint — plain hobby PWM servo (mg995, base + wrist). No feedback;
// readPos() reports last-commanded position, not sensed position. Does
// its own software time-interpolation so its API matches SC15's.
// ---------------------------------------------------------------------
class PwmJoint : public IJoint {
public:
    explicit PwmJoint(uint8_t pin) : pin_(pin) {}

    void begin() override {
        servo_.setPeriodHertz(50);
        servo_.attach(pin_, 500, 2500);
        posDeg10_ = 900; // assume centered on boot; first commanded move corrects it
        servo_.write(posDeg10_ / 10);
    }

    void moveTo(int16_t targetDeg10, uint16_t timeMs) override {
        // Hard safety bound, independent of whatever clamping the caller
        // did (or forgot to do). Servo::write() silently treats values
        // >=~200 as a raw microsecond pulse width instead of an angle —
        // clamping here to a valid degree range means a bad/unscaled
        // input (e.g. 1800 meant as 180.0deg) can never fall through
        // into that mode.
        targetDeg10 = constrain(targetDeg10, 0, 1800);

        startDeg10_   = posDeg10_;
        targetDeg10_  = targetDeg10;
        moveStartMs_  = millis();
        moveDurMs_    = timeMs == 0 ? 1 : timeMs;
        moving_       = true;
    }

    void tick() override {
        if (!moving_) return;
        uint32_t elapsed = millis() - moveStartMs_;
        if (elapsed >= moveDurMs_) {
            posDeg10_ = targetDeg10_;
            moving_ = false;
        } else {
            posDeg10_ = startDeg10_ + (int32_t)(targetDeg10_ - startDeg10_) * (int32_t)elapsed / (int32_t)moveDurMs_;
        }
        servo_.write(posDeg10_ / 10);
    }

    int16_t readPos() override { return posDeg10_; }

    void stop() override { moving_ = false; }

    bool hasFeedback() const override { return false; }

private:
    uint8_t  pin_;
    Servo    servo_;
    int16_t  posDeg10_ = 900;
    int16_t  startDeg10_ = 900;
    int16_t  targetDeg10_ = 900;
    uint32_t moveStartMs_ = 0;
    uint32_t moveDurMs_ = 1;
    bool     moving_ = false;
};
