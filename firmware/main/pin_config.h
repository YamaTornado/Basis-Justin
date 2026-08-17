#pragma once

/* Pin assignments -- TODO: verify against actual wiring once the board is
 * assembled. Chosen to avoid ESP32-S3 strapping pins (0, 3, 45, 46) and the
 * native USB pins (19, 20); otherwise arbitrary general-purpose GPIOs. */

#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

#define PIN_GPS_UART_TX 17
#define PIN_GPS_UART_RX 18

#define PIN_LORA_UART_TX 15
#define PIN_LORA_UART_RX 16

#define PIN_PYRO_DROGUE 4
#define PIN_PYRO_MAIN 5
#define PIN_ARM_SWITCH 6
