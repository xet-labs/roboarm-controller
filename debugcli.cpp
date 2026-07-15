#include "debugcli.h"
#include "joint.h" // JointId

namespace debugcli {

static String  lineBuf;
static bool    demoOn = false;
static bool    demoToggle = false;
static uint32_t demoStateStart = 0;

static bool    streamOn = false;
static uint32_t streamPeriodMs = 500;
static uint32_t lastStreamMs = 0;

static void printHelp()
{
    Serial.println(F(
        "\n--- roboArm debug console ---\n"
        "  help                          this text\n"
        "  demo on|off                   sweep all joints + claw between limits\n"
        "  state                         print one state snapshot\n"
        "  stream on [ms] | stream off   repeat 'state' periodically (default 500ms)\n"
        "  move <joint 0-3> <deg> <ms>   single joint move (0=Base 1=Shoulder 2=Elbow 3=Wrist)\n"
        "  claw <mode 0-2> <pwm 0-255>   0=stop 1=close 2=open\n"
        "  stop                          immediate stop, cancels demo mode\n"
        "  ping                          check SC15 servos respond on the bus\n"
    ));
}

static void printState()
{
    ArmStateSnapshot s = g_armState.get();
    Serial.printf(
        "[STATE] Base:%6.1f  Shoulder:%6.1f  Elbow:%6.1f  Wrist:%6.1f deg | Claw: %umA mode=%u | up=%lums\n",
        s.jointPosDeg10[JOINT_BASE] / 10.0f,
        s.jointPosDeg10[JOINT_SHOULDER] / 10.0f,
        s.jointPosDeg10[JOINT_ELBOW] / 10.0f,
        s.jointPosDeg10[JOINT_WRIST] / 10.0f,
        s.clawCurrentMa, s.clawMode, (unsigned long)s.uptimeMs);
}

static void enqueueOrWarn(const JointCmdMsg &msg)
{
    if (!g_cmdQueue || xQueueSend(g_cmdQueue, &msg, 0) != pdTRUE) {
        Serial.println("[WARN] command queue full, dropped");
    }
}

static void cmdMove(int joint, float deg, int timeMs)
{
    if (joint < 0 || joint >= JOINT_COUNT) { Serial.println("[ERR] joint must be 0-3"); return; }
    JointCmdMsg msg{};
    msg.type = MSG_MOVE_JOINT;
    msg.jointMask = 1 << joint;
    msg.pos[joint] = (int16_t)(deg * 10.0f);
    msg.timeMs = (uint16_t)timeMs;
    enqueueOrWarn(msg);
    Serial.printf("[OK] joint %d -> %.1f deg over %dms\n", joint, deg, timeMs);
}

static void cmdClaw(int mode, int pwm)
{
    JointCmdMsg msg{};
    msg.type = MSG_CLAW_SET;
    msg.clawMode = (uint8_t)mode;
    msg.clawPwm = (uint8_t)pwm;
    enqueueOrWarn(msg);
    Serial.printf("[OK] claw mode=%d pwm=%d\n", mode, pwm);
}

static void cmdStop()
{
    demoOn = false;
    JointCmdMsg msg{};
    msg.type = MSG_STOP;
    if (g_cmdQueue) xQueueSendToFront(g_cmdQueue, &msg, 0);
    Serial.println("[OK] stopped, demo mode cancelled");
}

static void handleLine(String line)
{
    line.trim();
    if (line.length() == 0) return;

    // tokenize on spaces (max 4 tokens is all we need)
    String tok[4];
    int n = 0;
    int start = 0;
    for (int i = 0; i <= line.length() && n < 4; ++i) {
        if (i == line.length() || line[i] == ' ') {
            if (i > start) tok[n++] = line.substring(start, i);
            start = i + 1;
        }
    }
    if (n == 0) return;

    if (tok[0] == "help") {
        printHelp();
    } else if (tok[0] == "demo") {
        if (n >= 2 && tok[1] == "on") {
            demoOn = true; demoStateStart = 0;
            Serial.println("[OK] demo mode ON — sweeping joints/claw");
        } else if (n >= 2 && tok[1] == "off") {
            demoOn = false;
            Serial.println("[OK] demo mode OFF");
        } else {
            Serial.println("[ERR] usage: demo on|off");
        }
    } else if (tok[0] == "state") {
        printState();
    } else if (tok[0] == "stream") {
        if (n >= 2 && tok[1] == "on") {
            streamOn = true;
            streamPeriodMs = n >= 3 ? (uint32_t)tok[2].toInt() : 500;
            Serial.printf("[OK] streaming state every %lums\n", (unsigned long)streamPeriodMs);
        } else if (n >= 2 && tok[1] == "off") {
            streamOn = false;
            Serial.println("[OK] stream off");
        } else {
            Serial.println("[ERR] usage: stream on [ms] | stream off");
        }
    } else if (tok[0] == "move") {
        if (n != 4) { Serial.println("[ERR] usage: move <joint> <deg> <ms>"); return; }
        cmdMove(tok[1].toInt(), tok[2].toFloat(), tok[3].toInt());
    } else if (tok[0] == "claw") {
        if (n != 3) { Serial.println("[ERR] usage: claw <mode 0-2> <pwm 0-255>"); return; }
        cmdClaw(tok[1].toInt(), tok[2].toInt());
    } else if (tok[0] == "stop") {
        cmdStop();
    } else if (tok[0] == "ping") {
        debugPingSc15();
    } else {
        Serial.print("[ERR] unknown command: ");
        Serial.println(tok[0]);
        Serial.println("type 'help' for a list");
    }
}

void begin()
{
    lineBuf.reserve(64);
    Serial.println("\n[DEBUG CLI] ready — type 'help'");
}

void poll()
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            handleLine(lineBuf);
            lineBuf = "";
        } else {
            lineBuf += c;
        }
    }

    if (streamOn) {
        uint32_t now = millis();
        if (now - lastStreamMs >= streamPeriodMs) {
            lastStreamMs = now;
            printState();
        }
    }
}

void demoTick(uint32_t now)
{
    if (!demoOn) return;

    if (demoStateStart == 0 || now - demoStateStart >= (DEMO_TRAVEL_MS + DEMO_PAUSE_MS)) {
        demoStateStart = now;
        demoToggle = !demoToggle;

        JointCmdMsg armMsg{};
        armMsg.type = MSG_MOVE_ALL;
        armMsg.jointMask = 0x0F;
        armMsg.timeMs = DEMO_TRAVEL_MS;
        for (int i = 0; i < JOINT_COUNT; ++i)
            armMsg.pos[i] = demoToggle ? kJointLimits[i].maxDeg10 : kJointLimits[i].minDeg10;
        enqueueOrWarn(armMsg);

        JointCmdMsg clawMsg{};
        clawMsg.type = MSG_CLAW_SET;
        clawMsg.clawMode = demoToggle ? 1 : 2; // close / open, alternating
        clawMsg.clawPwm = 150;
        enqueueOrWarn(clawMsg);

        Serial.printf("[DEMO] sweep -> %s\n", demoToggle ? "MAX + close" : "MIN + open");
    }
}

} // namespace debugcli
