#pragma once

/* Pin assignments -- board is an ESP32-S3 SuperMini with only the outer
 * header populated/accessible: GPIO 1-13 available, nothing else. GPIO 3 is
 * still avoided (strapping pin, selects JTAG signal source at boot) even
 * though it's in range, to leave one fewer thing to debug if boot gets
 * weird. GPIO 13 is left free for later use (e.g. a battery ADC reading,
 * see the TODO in telemetry_task.cpp).
 *
 * Listed in bring-up/test order: GPS first (own UART, easiest to verify
 * standalone via raw NMEA on the serial monitor), then BMP280 on the first
 * I2C bus (check its address 0x76/0x77 if init fails), then LSM9DS1 on its
 * own, second I2C bus (ESP32-S3 has two hardware I2C controllers -- Wire
 * and Wire1 -- so BMP280 and LSM9DS1 don't have to share one), then LoRa
 * (biggest unverified assumption, test with manual AT commands before
 * trusting the driver), then pyro/arm last (never wire live igniters
 * during bring-up). */

#define PIN_GPS_UART_RX 1
#define PIN_GPS_UART_TX 2

/* BMP280 (baro) -- own I2C bus (Wire). */
#define PIN_BARO_I2C_SDA 4
#define PIN_BARO_I2C_SCL 5

#define PIN_LORA_UART_RX 6
#define PIN_LORA_UART_TX 7

#define PIN_PYRO_DROGUE 8
#define PIN_PYRO_MAIN 9
#define PIN_ARM_SWITCH 10

/* LSM9DS1 (IMU: accel+gyro+mag) -- separate I2C bus (Wire1), kept off the
 * BMP280 bus at the user's request. */
#define PIN_IMU_I2C_SDA 11
#define PIN_IMU_I2C_SCL 12
