#include "lora_lr02.h"

#include <stdio.h>
#include <string.h>

static void to_hex(const uint8_t *data, size_t len, char *out) {
    static const char digits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < len; i++) {
        out[2 * i] = digits[data[i] >> 4];
        out[2 * i + 1] = digits[data[i] & 0x0F];
    }
    out[2 * len] = '\0';
}

bool LoraLr02::atCommand(const char *cmd, uint32_t timeout_ms) {
    serial_->print(cmd);
    serial_->print("\r\n");

    char resp[160];
    size_t resp_len = 0;
    uint32_t deadline = millis() + timeout_ms;

    while (millis() < deadline && resp_len < sizeof(resp) - 1) {
        if (serial_->available() > 0) {
            resp[resp_len++] = (char)serial_->read();
            resp[resp_len] = '\0';
            if (strstr(resp, "+OK") != nullptr) {
                return true;
            }
            if (strstr(resp, "+ERR") != nullptr) {
                return false;
            }
        }
    }
    return false;
}

bool LoraLr02::begin(HardwareSerial &serial, int rx_gpio, int tx_gpio, uint16_t local_address,
                      uint16_t target_address, uint16_t network_id, int baud) {
    serial_ = &serial;
    target_address_ = target_address;
    serial_->begin(baud, SERIAL_8N1, rx_gpio, tx_gpio);

    if (!atCommand("AT")) {
        return false;
    }

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%u", local_address);
    if (!atCommand(cmd)) return false;

    snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%u", network_id);
    if (!atCommand(cmd)) return false;

    return true;
}

bool LoraLr02::send(const uint8_t *payload, size_t len) {
    if (len > 64) {
        return false; /* larger than fits in the hex[] scratch buffer below */
    }
    char hex[2 * 64 + 1];
    to_hex(payload, len, hex);

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+SEND=%u,%u,%s", target_address_, (unsigned)(len * 2), hex);
    return atCommand(cmd);
}

/* Expects a line like "+RCV=<addr>,<hexlen>,<hexdata>,<rssi>,<snr>". Only
 * the hex payload is decoded; addr/rssi/snr are ignored (extend here if
 * the flight computer needs to react to uplink commands). */
size_t LoraLr02::pollReceive(uint8_t *out, size_t out_cap) {
    while (serial_->available() > 0 && rx_buf_len_ < sizeof(rx_buf_) - 1) {
        rx_buf_[rx_buf_len_++] = (char)serial_->read();
        rx_buf_[rx_buf_len_] = '\0';
    }
    if (rx_buf_len_ == 0) {
        return 0;
    }

    char *buf = rx_buf_;
    char *rcv = strstr(buf, "+RCV=");
    if (!rcv) {
        /* no marker found, buffer full: drop it so we don't wedge forever */
        if (rx_buf_len_ >= sizeof(rx_buf_) - 1) rx_buf_len_ = 0;
        return 0;
    }

    char *p = rcv + 5;
    char *comma1 = strchr(p, ',');
    char *comma2 = comma1 ? strchr(comma1 + 1, ',') : nullptr;
    if (!comma2) {
        return 0; /* incomplete line so far, wait for more bytes */
    }
    char *hex_start = comma2 + 1;
    char *comma3 = strchr(hex_start, ',');
    if (!comma3) {
        return 0; /* still incomplete */
    }
    size_t hex_len = (size_t)(comma3 - hex_start);

    size_t byte_len = hex_len / 2;
    if (byte_len > out_cap) {
        byte_len = out_cap;
    }
    for (size_t i = 0; i < byte_len; i++) {
        unsigned int b;
        if (sscanf(&hex_start[i * 2], "%2x", &b) != 1) {
            rx_buf_len_ = 0;
            return 0;
        }
        out[i] = (uint8_t)b;
    }
    rx_buf_len_ = 0; /* consumed */
    return byte_len;
}
