#include "tasks.h"
#include "protocol.h"
#include "joint.h"
#include "claw.h"
#include "state.h"
#include "debugcli.h"
#include "driver/uart.h"

QueueHandle_t g_cmdQueue = nullptr;
ArmState g_armState;

static HardwareSerial CmdSerial(CMD_UART_NUM);
static HardwareSerial BusSerial(BUS_UART_NUM);
static SCSCL scBus;

static PwmJoint  baseJoint(PIN_SERVO_BASE);
static PwmJoint  wristJoint(PIN_SERVO_WRIST);
static Sc15Joint shoulderJoint(scBus, SC15_ID_SHOULDER);
static Sc15Joint elbowJoint(scBus, SC15_ID_ELBOW);
static IJoint   *joints[JOINT_COUNT] = { &baseJoint, &shoulderJoint, &elbowJoint, &wristJoint };
static Claw      claw;

// clampToLimits() now lives in config.h — shared with debugcli.cpp so
// both entry points onto g_cmdQueue enforce the same per-joint limits.

// ---------------------------------------------------------------------
// uartTask — command-plane only. No peripheral access besides CmdSerial.
// (debugcli polls its own USB Serial separately, see below.)
// ---------------------------------------------------------------------
static void sendAck(uint8_t cmdEcho)
{
    uint8_t out[8];
    size_t n = proto::encode(proto::RSP_ACK, &cmdEcho, 1, out, sizeof(out));
    if (n) CmdSerial.write(out, n);
}

static void sendErr(uint8_t cmdEcho, uint8_t errCode)
{
    uint8_t payload[2] = { cmdEcho, errCode };
    uint8_t out[8];
    size_t n = proto::encode(proto::RSP_ERR, payload, 2, out, sizeof(out));
    if (n) CmdSerial.write(out, n);
}

static void sendState()
{
    ArmStateSnapshot s = g_armState.get();
    uint8_t payload[15];
    for (int i = 0; i < JOINT_COUNT; ++i) proto::putI16(&payload[i * 2], s.jointPosDeg10[i]);
    proto::putU16(&payload[8], s.clawCurrentMa);
    payload[10] = s.clawMode;
    proto::putU32(&payload[11], s.uptimeMs);

    uint8_t out[24];
    size_t n = proto::encode(proto::RSP_STATE, payload, sizeof(payload), out, sizeof(out));
    if (n) CmdSerial.write(out, n);
}

static void handleFrame(const proto::Frame &f)
{
    JointCmdMsg msg{};

    switch (f.cmd) {

    case proto::CMD_MOVE_JOINT: {
        if (f.len != 5) { sendErr(f.cmd, proto::ERR_BAD_LENGTH); return; }
        uint8_t jointId = f.payload[0];
        if (jointId >= JOINT_COUNT) { sendErr(f.cmd, proto::ERR_BAD_JOINT); return; }
        int16_t pos  = clampToLimits(jointId, proto::getI16(&f.payload[1]));
        uint16_t tMs = proto::getU16(&f.payload[3]);

        msg.type = MSG_MOVE_JOINT;
        msg.jointMask = 1 << jointId;
        msg.pos[jointId] = pos;
        msg.timeMs = tMs;
        break;
    }

    case proto::CMD_MOVE_ALL: {
        if (f.len != 10) { sendErr(f.cmd, proto::ERR_BAD_LENGTH); return; }
        msg.type = MSG_MOVE_ALL;
        msg.jointMask = 0x0F;
        for (int i = 0; i < JOINT_COUNT; ++i)
            msg.pos[i] = clampToLimits(i, proto::getI16(&f.payload[i * 2]));
        msg.timeMs = proto::getU16(&f.payload[8]);
        break;
    }

    case proto::CMD_CLAW_SET: {
        if (f.len != 2) { sendErr(f.cmd, proto::ERR_BAD_LENGTH); return; }
        msg.type = MSG_CLAW_SET;
        msg.clawMode = f.payload[0];
        msg.clawPwm  = f.payload[1];
        break;
    }

    case proto::CMD_STOP: {
        msg.type = MSG_STOP;
        if (g_cmdQueue) xQueueSendToFront(g_cmdQueue, &msg, 0); // jump the queue
        sendAck(f.cmd);
        return;
    }

    case proto::CMD_GET_STATE:
        sendState();
        return;

    case proto::CMD_SET_TORQUE: {
        if (f.len != 2) { sendErr(f.cmd, proto::ERR_BAD_LENGTH); return; }
        if (f.payload[0] >= JOINT_COUNT) { sendErr(f.cmd, proto::ERR_BAD_JOINT); return; }
        msg.type = MSG_SET_TORQUE;
        msg.torqueJoint = f.payload[0];
        msg.torqueEnable = f.payload[1] != 0;
        break;
    }

    case proto::CMD_PING: {
        uint8_t out[16];
        size_t n = proto::encode(proto::RSP_PONG, f.payload, f.len, out, sizeof(out));
        if (n) CmdSerial.write(out, n);
        return;
    }

    default:
        sendErr(f.cmd, proto::ERR_UNKNOWN_CMD);
        return;
    }

    if (g_cmdQueue && xQueueSend(g_cmdQueue, &msg, 0) == pdTRUE) {
        sendAck(f.cmd);
    } else {
        sendErr(f.cmd, proto::ERR_QUEUE_FULL);
    }
}

// Bench-only: Ping() both SC15 servos, prints result to USB Serial.
// Declared in tasks.h, called from main.cpp (boot) and debugcli.cpp
// ('ping' command) — lives here because this is where the bus/joint
// objects are actually instantiated.
void debugPingSc15()
{
    Serial.println("[PING] SC15 bus check:");
    Serial.printf("  Shoulder (id %d): %s\n", SC15_ID_SHOULDER, shoulderJoint.ping() ? "OK" : "NO RESPONSE");
    Serial.printf("  Elbow    (id %d): %s\n", SC15_ID_ELBOW, elbowJoint.ping() ? "OK" : "NO RESPONSE");
}

void uartTask(void *pv)
{
    (void)pv;
    CmdSerial.begin(CMD_UART_BAUD, SERIAL_8N1, CMD_UART_RX_PIN, CMD_UART_TX_PIN);
    debugcli::begin();

    proto::Parser parser;
    proto::Frame  frame;

    for (;;) {
        while (CmdSerial.available()) {
            uint8_t b = (uint8_t)CmdSerial.read();
            if (parser.feed(b, frame)) {
                handleFrame(frame);
            }
        }

        debugcli::poll();              // USB console: parse any typed command
        debugcli::demoTick(millis());  // USB console: advance sweep if "demo on"

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ---------------------------------------------------------------------
// controlTask — owns every peripheral bus. Single writer of g_armState.
// Has no concept of "demo mode" — it just drains whatever lands in the
// queue, whether that's from the RPi (CmdSerial) or the USB debug
// console (debugcli). Keeping this task ignorant of where a command
// came from is what keeps it simple.
// ---------------------------------------------------------------------
void controlTask(void *pv)
{
    (void)pv;

    // Bring the UART up normally first (required before switching mode),
    // same pin for RX and TX — then flip the peripheral into RS485
    // half-duplex so it handles TX/RX direction switching in hardware.
    BusSerial.begin(BUS_UART_BAUD, SERIAL_8N1, BUS_UART_SIG_PIN, BUS_UART_SIG_PIN);
    uart_set_mode(BUS_UART_DRIVER_NUM, UART_MODE_RS485_HALF_DUPLEX);
    scBus.pSerial = &BusSerial;

    debugPingSc15(); // bench sanity check on every boot, printed to USB Serial

    for (int i = 0; i < JOINT_COUNT; ++i) joints[i]->begin();
    claw.begin();

    uint32_t lastStateMs = 0;

    for (;;) {
        JointCmdMsg msg;
        if (xQueueReceive(g_cmdQueue, &msg, pdMS_TO_TICKS(CONTROL_TICK_MS)) == pdTRUE) {
            switch (msg.type) {
            case MSG_MOVE_JOINT:
            case MSG_MOVE_ALL:
                for (int i = 0; i < JOINT_COUNT; ++i)
                    if (msg.jointMask & (1 << i)) joints[i]->moveTo(msg.pos[i], msg.timeMs);
                break;

            case MSG_CLAW_SET:
                claw.setMode(msg.clawMode, msg.clawPwm);
                break;

            case MSG_STOP:
                for (int i = 0; i < JOINT_COUNT; ++i) joints[i]->stop();
                claw.setMode(0, 0);
                break;

            case MSG_SET_TORQUE:
                joints[msg.torqueJoint]->setTorque(msg.torqueEnable);
                break;
            }
        }

        for (int i = 0; i < JOINT_COUNT; ++i) joints[i]->tick();
        claw.update();

        uint32_t now = millis();
        if (now - lastStateMs >= STATE_PUBLISH_MS) {
            lastStateMs = now;
            ArmStateSnapshot s;
            for (int i = 0; i < JOINT_COUNT; ++i) s.jointPosDeg10[i] = joints[i]->readPos();
            s.clawCurrentMa = claw.lastCurrentMa();
            s.clawMode = claw.mode();
            s.uptimeMs = now;
            g_armState.set(s);
        }
    }
}
