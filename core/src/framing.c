#include "fc/framing.h"

uint8_t fc_crc8(const uint8_t *data, size_t len) {
    /* CRC-8/SMBUS-ish: poly 0x07, init 0x00 -- simple and dependency-free. */
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

size_t fc_frame_encode(uint8_t type, const uint8_t *payload, uint8_t payload_len, uint8_t *out,
                        size_t out_cap) {
    size_t total = (size_t)payload_len + 3; /* type + len + payload + crc */
    if (out_cap < total) {
        return 0;
    }
    out[0] = type;
    out[1] = payload_len;
    if (payload_len > 0) {
        for (uint8_t i = 0; i < payload_len; i++) {
            out[2 + i] = payload[i];
        }
    }
    out[2 + payload_len] = fc_crc8(out, (size_t)payload_len + 2);
    return total;
}

size_t fc_frame_decode(const uint8_t *buf, size_t len, fc_frame_t *out) {
    if (len < 3) {
        return 0;
    }
    uint8_t type = buf[0];
    uint8_t payload_len = buf[1];
    size_t total = (size_t)payload_len + 3;
    if (len < total) {
        return 0;
    }
    uint8_t expected_crc = fc_crc8(buf, (size_t)payload_len + 2);
    uint8_t actual_crc = buf[2 + payload_len];
    if (expected_crc != actual_crc) {
        return 0;
    }
    out->type = type;
    out->payload = (payload_len > 0) ? &buf[2] : NULL;
    out->payload_len = payload_len;
    return total;
}
