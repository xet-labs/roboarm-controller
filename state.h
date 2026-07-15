#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

// =====================================================================
// ArmState — the one piece of data shared between controlTask (writer)
// and uartTask (reader, for RSP_STATE / GET_STATE). Mutex-guarded so
// neither task ever reads/writes a half-updated snapshot — this is the
// specific bug class the old malloc/placement-new motor registry was
// prone to; a plain mutexed struct avoids it entirely.
// =====================================================================

struct ArmStateSnapshot {
    int16_t  jointPosDeg10[JOINT_COUNT] = {0, 0, 0, 0};
    uint16_t clawCurrentMa = 0;
    uint8_t  clawMode = 0;
    uint32_t uptimeMs = 0;
};

class ArmState {
public:
    void begin() { mutex_ = xSemaphoreCreateMutex(); }

    void set(const ArmStateSnapshot &s) {
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            data_ = s;
            xSemaphoreGive(mutex_);
        }
    }

    ArmStateSnapshot get() {
        ArmStateSnapshot copy;
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            copy = data_;
            xSemaphoreGive(mutex_);
        }
        return copy;
    }

private:
    SemaphoreHandle_t mutex_ = nullptr;
    ArmStateSnapshot  data_;
};

extern ArmState g_armState;
