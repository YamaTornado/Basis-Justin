#include "lora_lr02.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "lora_lr02";

#define UART_RX_BUF_SIZE 512
#define AT_RESPONSE_TIMEOUT_MS 500

void lora_lr02_config_default(lora_lr02_config_t *cfg, uart_port_t port, int tx_gpio,
                               int rx_gpio) {
    cfg->uart_port = port;
    cfg->tx_gpio = tx_gpio;
    cfg->rx_gpio = rx_gpio;
    cfg->baud_rate = 115200;
    cfg->local_address = 1;
    cfg->target_address = 2; /* ground station */
    cfg->network_id = 18;
}

/* Sends a raw AT command line (without \r\n, added here) and waits up to
 * AT_RESPONSE_TIMEOUT_MS for "+OK" to appear in the response. Drains
 * whatever else the module sends. Returns true if "+OK" was seen. */
static bool at_command(uart_port_t port, const char *cmd) {
    char line[160];
    int n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_write_bytes(port, line, n);

    char resp[160];
    size_t resp_len = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(AT_RESPONSE_TIMEOUT_MS);

    while (xTaskGetTickCount() < deadline && resp_len < sizeof(resp) - 1) {
        int got = uart_read_bytes(port, (uint8_t *)&resp[resp_len], 1, pdMS_TO_TICKS(50));
        if (got == 1) {
            resp_len++;
            resp[resp_len] = '\0';
            if (strstr(resp, "+OK") != NULL) {
                return true;
            }
            if (strstr(resp, "+ERR") != NULL) {
                ESP_LOGW(TAG, "module returned error for '%s': %s", cmd, resp);
                return false;
            }
        }
    }
    ESP_LOGW(TAG, "no response to '%s' (got: %s)", cmd, resp_len ? resp : "<nothing>");
    return false;
}

esp_err_t lora_lr02_init(lora_lr02_t *dev, const lora_lr02_config_t *cfg) {
    dev->uart_port = cfg->uart_port;
    dev->target_address = cfg->target_address;

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(cfg->uart_port, &uart_cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(cfg->uart_port, cfg->tx_gpio, cfg->rx_gpio, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(cfg->uart_port, UART_RX_BUF_SIZE, UART_RX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) return err;

    if (!at_command(cfg->uart_port, "AT")) {
        return ESP_ERR_TIMEOUT;
    }

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%u", cfg->local_address);
    if (!at_command(cfg->uart_port, cmd)) return ESP_FAIL;

    snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%u", cfg->network_id);
    if (!at_command(cfg->uart_port, cmd)) return ESP_FAIL;

    return ESP_OK;
}

static void to_hex(const uint8_t *data, size_t len, char *out) {
    static const char digits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < len; i++) {
        out[2 * i] = digits[data[i] >> 4];
        out[2 * i + 1] = digits[data[i] & 0x0F];
    }
    out[2 * len] = '\0';
}

esp_err_t lora_lr02_send(lora_lr02_t *dev, const uint8_t *payload, size_t len) {
    if (len > 64) {
        /* larger than fits in the hex[] scratch buffer below, and well
         * beyond FC_TLM_MAX_FRAME anyway -- telemetry packets should never
         * be this big. */
        return ESP_ERR_INVALID_SIZE;
    }
    char hex[2 * 64 + 1];
    to_hex(payload, len, hex);

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+SEND=%u,%u,%s", dev->target_address, (unsigned)(len * 2), hex);

    if (!at_command(dev->uart_port, cmd)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Expects a line like "+RCV=<addr>,<hexlen>,<hexdata>,<rssi>,<snr>". Only
 * the hex payload is decoded; addr/rssi/snr are ignored (extend here if
 * the flight computer needs to react to uplink commands). */
size_t lora_lr02_poll_receive(lora_lr02_t *dev, uint8_t *out, size_t out_cap,
                               TickType_t timeout_ticks) {
    uint8_t chunk[256];
    int n = uart_read_bytes(dev->uart_port, chunk, sizeof(chunk) - 1, timeout_ticks);
    if (n <= 0) {
        return 0;
    }
    chunk[n] = '\0';

    char *rcv = strstr((char *)chunk, "+RCV=");
    if (!rcv) {
        return 0;
    }
    char *p = rcv + 5;
    char *comma1 = strchr(p, ',');
    if (!comma1) return 0;
    char *comma2 = strchr(comma1 + 1, ',');
    if (!comma2) return 0;
    char *hex_start = comma2 + 1;
    char *comma3 = strchr(hex_start, ',');
    size_t hex_len = comma3 ? (size_t)(comma3 - hex_start) : strlen(hex_start);

    size_t byte_len = hex_len / 2;
    if (byte_len > out_cap) {
        byte_len = out_cap;
    }
    for (size_t i = 0; i < byte_len; i++) {
        unsigned int b;
        if (sscanf(&hex_start[i * 2], "%2x", &b) != 1) {
            return 0;
        }
        out[i] = (uint8_t)b;
    }
    return byte_len;
}
