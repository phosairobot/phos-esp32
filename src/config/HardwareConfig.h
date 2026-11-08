#pragma once

#include <Arduino.h>

namespace HardwareConfig {
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
// Names avoid collisions with address macros exported by Adafruit libraries.
constexpr uint8_t BMP280_I2C_ADDRESS = 0x76;
constexpr uint8_t CCS811_I2C_ADDRESS = 0x5A;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68;

constexpr uint8_t LED_PIN = 19;
constexpr uint16_t LED_COUNT = 12;
// Legacy board default; LED output now uses PHOS_LED_DEFAULT_BRIGHTNESS from
// platformio.ini so the requested protocol brightness remains a float.
constexpr float LED_DEFAULT_BRIGHTNESS = 0.15F;
// Ring calibration: physical LED 6 is top; incrementing physical indexes move
// clockwise. Physical validation found LED 0 at the bottom. Animation code
// uses logical positions only, so these are the only board-specific values to
// change if the wiring orientation differs.
constexpr uint16_t LED_TOP_PHYSICAL_INDEX = 6;
constexpr int8_t LED_CLOCKWISE_PHYSICAL_STEP = 1;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr size_t MAX_FRAME_BYTES = 8192;
constexpr size_t SERIAL_TX_QUEUE_BYTES = 8192;
constexpr uint32_t SERIAL_TX_STALL_DROP_MS = 2000;
constexpr uint16_t I2C_TIMEOUT_MS = 50;
}
