#include "protocol.h"

namespace proto {

uint8_t checksum(uint8_t len, uint8_t cmd, const uint8_t *payload, uint8_t payloadLen)
{
    uint8_t c = len ^ cmd;
    for (uint8_t i = 0; i < payloadLen; ++i) c ^= payload[i];
    return c;
}

size_t encode(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen,
              uint8_t *outBuf, size_t outCap)
{
    const size_t total = 2 /*magic*/ + 1 /*len*/ + 1 /*cmd*/ + payloadLen + 1 /*chk*/;
    if (payloadLen > MAX_PAYLOAD || total > outCap) return 0;

    const uint8_t len = 1 + payloadLen; // cmd + payload

    size_t i = 0;
    outBuf[i++] = MAGIC0;
    outBuf[i++] = MAGIC1;
    outBuf[i++] = len;
    outBuf[i++] = cmd;
    for (uint8_t k = 0; k < payloadLen; ++k) outBuf[i++] = payload[k];
    outBuf[i++] = checksum(len, cmd, payload, payloadLen);

    return i;
}

bool Parser::feed(uint8_t b, Frame &out)
{
    switch (state_) {
    case WAIT_M0:
        if (b == MAGIC0) state_ = WAIT_M1;
        break;

    case WAIT_M1:
        if (b == MAGIC1) state_ = WAIT_LEN;
        else if (b != MAGIC0) state_ = WAIT_M0; // stay in WAIT_M1 on repeated 0xAA (resync)
        break;

    case WAIT_LEN:
        len_ = b;
        if (len_ == 0 || len_ > (MAX_PAYLOAD + 1)) {
            state_ = WAIT_M0; // malformed length, drop and resync
        } else {
            state_ = WAIT_CMD;
        }
        break;

    case WAIT_CMD:
        cmdByte_ = b;
        idx_ = 0;
        state_ = (len_ - 1 == 0) ? WAIT_CHK : WAIT_PAYLOAD;
        break;

    case WAIT_PAYLOAD:
        buf_[idx_++] = b;
        if (idx_ >= (uint8_t)(len_ - 1)) state_ = WAIT_CHK;
        break;

    case WAIT_CHK: {
        uint8_t payloadLen = len_ - 1;
        uint8_t expected = checksum(len_, cmdByte_, buf_, payloadLen);
        state_ = WAIT_M0; // always resync after this byte
        if (b == expected) {
            out.cmd = cmdByte_;
            out.len = payloadLen;
            memcpy(out.payload, buf_, payloadLen);
            return true;
        }
        break;
    }
    }
    return false;
}

} // namespace proto
