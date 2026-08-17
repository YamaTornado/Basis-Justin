#pragma once

#include <HardwareSerial.h>
#include <stddef.h>
#include <stdint.h>

/* LR-02 LoRa module, controlled via AT commands over UART.
 *
 * IMPORTANT: the exact AT command set for "LR-02" modules varies by vendor
 * -- there is no single standardized "LR-02". This driver implements the
 * command set used by REYAX RYLR-style modules (AT / AT+ADDRESS= /
 * AT+NETWORKID= / AT+SEND=<addr>,<len>,<data>), which is the most common
 * pattern for cheap AT-command LoRa modules. VERIFY THIS AGAINST YOUR
 * MODULE'S DATASHEET before relying on it -- this is the single biggest
 * unverified assumption in the firmware (see docs/ARCHITECTURE.md). */
class LoraLr02 {
   public:
    bool begin(HardwareSerial &serial, int rx_gpio, int tx_gpio, uint16_t local_address = 1,
               uint16_t target_address = 2, uint16_t network_id = 18, int baud = 115200);

    /* Sends payload as an AT+SEND=<addr>,<hexlen>,<hex> command. Payload is
     * hex-encoded before transmission -- AT command lines are text/line
     * oriented, and raw binary risks colliding with the module's line
     * parser (embedded \r, \n, or comma bytes). len must be <= 64 bytes
     * (telemetry packets are small, see fc/telemetry.h FC_TLM_MAX_FRAME). */
    bool send(const uint8_t *payload, size_t len);

    /* Non-blocking-ish receive: reads whatever's waiting on the UART and,
     * if a complete "+RCV=<addr>,<hexlen>,<hexdata>,..." line was seen,
     * decodes the payload into out (out_cap bytes) and returns the decoded
     * length. Returns 0 if nothing decodable arrived this call. */
    size_t pollReceive(uint8_t *out, size_t out_cap);

   private:
    bool atCommand(const char *cmd, uint32_t timeout_ms = 500);

    HardwareSerial *serial_ = nullptr;
    uint16_t target_address_ = 2;
    char rx_buf_[256] = {0};
    size_t rx_buf_len_ = 0;
};
