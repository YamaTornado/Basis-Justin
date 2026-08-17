#pragma once

/* Pin assignments -- board is an ESP32-S3 SuperMini with only the outer
 * header populated/accessible: GPIO 1-13 available, nothing else. GPIO 3 is
 * still avoided (strapping pin, selects JTAG signal source at boot) even
 * though it's in range, to leave one fewer thing to debug if boot gets
 * weird. GPIO 11-13 are left free for later use (e.g. a battery ADC
 * reading, see the TODO in telemetry_task.cpp).
 *
 * Listed in bring-up/test order: GPS first (own UART, easiest to verify
 * standalone via raw NMEA on the serial monitor), then the I2C bus (BMP280
 * + LSM9DS1 share it -- BMP280 typically easier to get responding first,
 * check its address 0x76/0x77 if init fails), then LoRa (biggest unverified
 * assumption, test with manual AT commands before trusting the driver),
 * then pyro/arm last (never wire live igniters during bring-up). */

#define PIN_GPS_UART_RX 1
#define PIN_GPS_UART_TX 2

/* Shared I2C bus: BMP280 (baro) and LSM9DS1 (IMU) both hang off these two
 * pins -- there's no separate "baro pin", the bus is what's shared. */
#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

#define PIN_LORA_UART_RX 6
#define PIN_LORA_UART_TX 7

#define PIN_PYRO_DROGUE 8
#define PIN_PYRO_MAIN 9
#define PIN_ARM_SWITCH 10
