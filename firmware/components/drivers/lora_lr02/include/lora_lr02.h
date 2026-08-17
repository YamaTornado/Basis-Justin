#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LR-02 LoRa module, controlled via AT commands over UART.
 *
 * IMPORTANT: the exact AT command set for "LR-02" modules varies by vendor
 * -- there is no single standardized "LR-02". This driver implements the
 * command set used by REYAX RYLR-style modules (AT / AT+ADDRESS= /
 * AT+NETWORKID= / AT+SEND=<addr>,<len>,<data>), which is the most common
 * pattern for cheap AT-command LoRa modules. VERIFY THIS AGAINST YOUR
 * MODULE'S DATASHEET before relying on it -- this is the single biggest
 * unverified assumption in the firmware (see docs/ARCHITECTURE.md). */

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
    uint16_t local_address;
    uint16_t target_address;
    uint16_t network_id;
} lora_lr02_config_t;

typedef struct {
    uart_port_t uart_port;
    uint16_t target_address;
} lora_lr02_t;

void lora_lr02_config_default(lora_lr02_config_t *cfg, uart_port_t port, int tx_gpio,
                               int rx_gpio);

/* Configures the UART, then sends AT / AT+ADDRESS / AT+NETWORKID and checks
 * for "+OK" after each. Returns ESP_ERR_TIMEOUT if the module never
 * responds (wrong wiring/baud rate/command set). */
esp_err_t lora_lr02_init(lora_lr02_t *dev, const lora_lr02_config_t *cfg);

/* Sends payload as an AT+SEND=<addr>,<hexlen>,<hex> command. Payload is
 * hex-encoded before transmission -- AT command lines are text/line
 * oriented, and raw binary risks colliding with the module's line parser
 * (embedded \r, \n, or comma bytes). Costs 2x airtime bytes; telemetry
 * packets are small (see fc/telemetry.h, FC_TLM_MAX_FRAME) so this is
 * cheap. Returns ESP_OK once "+OK" (or a timeout) is seen. */
esp_err_t lora_lr02_send(lora_lr02_t *dev, const uint8_t *payload, size_t len);

/* Non-blocking-ish receive: reads whatever's waiting on UART (bounded by
 * timeout_ticks) and, if a complete "+RCV=<addr>,<len>,<hexdata>,..." line
 * was received, decodes the payload into out (out_cap bytes) and returns
 * the decoded length. Returns 0 if nothing decodable arrived this call. */
size_t lora_lr02_poll_receive(lora_lr02_t *dev, uint8_t *out, size_t out_cap,
                               TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif
