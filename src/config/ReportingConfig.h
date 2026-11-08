#pragma once

#include <Arduino.h>

namespace ReportingConfig {
constexpr uint32_t SENSOR_SAMPLE_INTERVAL_MS = 1000;
// The Raspberry Pi declares controller-backed environmental values stale at
// 30 seconds, so refresh every valid independently-tracked value within 15 s.
constexpr uint32_t ENVIRONMENT_REFRESH_INTERVAL_MS = 15000;
constexpr uint32_t AIR_QUALITY_REFRESH_INTERVAL_MS = 15000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
constexpr float TEMPERATURE_DELTA_C = 0.2F;
constexpr float PRESSURE_DELTA_HPA = 0.5F;
constexpr uint16_t ECO2_DELTA_PPM = 50;
constexpr uint16_t TVOC_DELTA_PPB = 10;

// MPU6050 work stays local at 50 Hz. Only bounded semantic/orientation data
// reaches serial so command and LED handling remains responsive.
constexpr uint32_t IMU_SAMPLE_INTERVAL_MS = 20;
constexpr uint32_t ORIENTATION_MIN_PUBLISH_INTERVAL_MS = 200;
constexpr uint32_t ORIENTATION_REFRESH_INTERVAL_MS = 5000;
constexpr float ORIENTATION_ACCELERATION_DELTA_M_S2 = 0.3F;
constexpr float ORIENTATION_GYRO_DELTA_DPS = 3.0F;
constexpr uint32_t MOTION_SETTLE_INTERVAL_MS = 400;
constexpr uint32_t TRANSIENT_MOTION_INTERVAL_MS = 250;

// Thresholds use SI acceleration and degrees/second. Tilt uses the low-pass
// gravity vector: X is left/right and Y is forward/backward in board axes.
constexpr float GRAVITY_FILTER_ALPHA = 0.05F;
constexpr float TILT_ENTER_M_S2 = 3.0F;
constexpr float TILT_EXIT_M_S2 = 2.0F;
constexpr float MOVING_ENTER_DYNAMIC_M_S2 = 1.2F;
constexpr float MOVING_EXIT_DYNAMIC_M_S2 = 0.6F;
constexpr float MOVING_ENTER_GYRO_DPS = 45.0F;
constexpr float MOVING_EXIT_GYRO_DPS = 20.0F;
constexpr float SHAKE_DYNAMIC_M_S2 = 4.0F;
constexpr float SHAKE_GYRO_DPS = 160.0F;
constexpr uint8_t SHAKE_PEAK_COUNT = 3;
constexpr uint32_t SHAKE_WINDOW_MS = 400;
constexpr float IMPACT_ACCELERATION_M_S2 = 18.0F;
constexpr float IMPACT_DYNAMIC_M_S2 = 12.0F;

constexpr float INTENSITY_MEDIUM_DYNAMIC_M_S2 = 2.5F;
constexpr float INTENSITY_HIGH_DYNAMIC_M_S2 = 6.0F;
constexpr float INTENSITY_MEDIUM_GYRO_DPS = 90.0F;
constexpr float INTENSITY_HIGH_GYRO_DPS = 180.0F;
}
