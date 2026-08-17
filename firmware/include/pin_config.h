#pragma once

/* Pin assignments -- TODO: verify against actual wiring once the board is
 * assembled. Chosen to avoid ESP32-S3 strapping pins (0, 3, 45, 46) and the
 * native USB pins (19, 20); otherwise arbitrary general-purpose GPIOs.
 *
 * Listed in bring-up/test order: GPS first (own UART, easiest to verify
 * standalone via raw NMEA on the serial monitor), then the I2C bus (BMP280
 * + LSM9DS1 share it -- BMP280 typically easier to get responding first,
 * check its address 0x76/0x77 if init fails), then LoRa (biggest unverified
 * assumption, test with manual AT commands before trusting the driver),
 * then pyro/arm last (never wire live igniters during bring-up). */

#define PIN_GPS_UART_RX 18
#define PIN_GPS_UART_TX 17

/* Shared I2C bus: BMP280 (baro) and LSM9DS1 (IMU) both hang off these two
 * pins -- there's no separate "baro pin", the bus is what's shared. */
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

#define PIN_LORA_UART_RX 16
#define PIN_LORA_UART_TX 15

#define PIN_PYRO_DROGUE 4
#define PIN_PYRO_MAIN 5
#define PIN_ARM_SWITCH 6
