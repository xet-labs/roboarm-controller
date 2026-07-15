#pragma once
#include <Arduino.h>

// =====================================================================
// UART frame format (commander <-> controller)
//
//   [0xAA][0x55][LEN][CMD][PAYLOAD...][CHECKSUM]
//
//   LEN      = 1 (for CMD) + payload length   (i.e. bytes covered by checksum minus itself)
//   CHECKSUM = XOR of LEN, CMD, and every payload byte
//
// Deliberately dumb: this framing has no notion of joints/servos, so
// any UART master that respects header+checksum can drive the arm.
// All arm semantics live in the CMD payload definitions below, which
// the RPi commander (Go app) is the sole intended source of.
// =====================================================================

namespace proto {

constexpr uint8_t MAGIC0 = 0xAA;
constexpr uint8_t MAGIC1 = 0x55;
constexpr size_t  MAX_PAYLOAD = 32;

enum Cmd : uint8_t {
    // Host -> Controller
    CMD_MOVE_JOINT  = 0x01,  // u8 jointId, i16 posDeg10, u16 timeMs           (5B)
    CMD_MOVE_ALL    = 0x02,  // i16 pos[4]Deg10, u16 timeMs                    (10B)
    CMD_CLAW_SET    = 0x03,  // u8 mode(0=stop,1=close,2=open), u8 pwm(0-255)  (2B)
    CMD_STOP        = 0x04,  // (0B) — immediate stop, all actuators
    CMD_GET_STATE   = 0x05,  // (0B) — request RSP_STATE
    CMD_SET_TORQUE  = 0x06,  // u8 jointId, u8 enable                          (2B)
    CMD_PING        = 0x07,  // u32 echoToken                                  (4B)

    // Controller -> Host
    RSP_STATE       = 0x81,  // i16 pos[4]Deg10, u16 clawCurrentMa, u8 clawMode, u32 uptimeMs (15B)
    RSP_PONG        = 0x82,  // u32 echoToken                                  (4B)
    RSP_ACK         = 0x83,  // u8 cmdEcho                                     (1B)
    RSP_ERR         = 0x84,  // u8 cmdEcho, u8 errCode                         (2B)
};

enum ErrCode : uint8_t {
    ERR_NONE          = 0,
    ERR_BAD_CHECKSUM  = 1,
    ERR_BAD_LENGTH    = 2,
    ERR_UNKNOWN_CMD   = 3,
    ERR_BAD_JOINT     = 4,
    ERR_QUEUE_FULL    = 5,
};

struct Frame {
    uint8_t cmd;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t len;   // payload length (excludes cmd byte)
};

// Byte-at-a-time streaming parser. Feed it every received byte; when it
// returns true, `out` holds a validated (checksum-correct) frame.
class Parser {
public:
    bool feed(uint8_t b, Frame &out);

private:
    enum State { WAIT_M0, WAIT_M1, WAIT_LEN, WAIT_CMD, WAIT_PAYLOAD, WAIT_CHK };
    State   state_   = WAIT_M0;
    uint8_t len_     = 0;   // total = 1(cmd) + payloadLen
    uint8_t cmdByte_ = 0;
    uint8_t idx_     = 0;
    uint8_t buf_[MAX_PAYLOAD];
};

uint8_t checksum(uint8_t len, uint8_t cmd, const uint8_t *payload, uint8_t payloadLen);

// Encodes a full frame into outBuf. Returns bytes written, or 0 on overflow.
size_t encode(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen,
              uint8_t *outBuf, size_t outCap);

// ---- little-endian pack/unpack helpers (payloads are LE on the wire) ----
inline void putI16(uint8_t *p, int16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
inline void putU16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
inline void putU32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
inline int16_t  getI16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
inline uint16_t getU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t getU32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

} // namespace proto
