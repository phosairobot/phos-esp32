#include "SensorManager.h"

#include "../config/HardwareConfig.h"
#include "../config/ReportingConfig.h"

#include <Wire.h>
#include <cmath>
#include <math.h>
#include <string.h>

#ifndef PHOS_IMU_DIAGNOSTICS
#define PHOS_IMU_DIAGNOSTICS 0
#endif

void SensorManager::begin() {
  Wire.begin(HardwareConfig::I2C_SDA_PIN, HardwareConfig::I2C_SCL_PIN);
  Wire.setTimeOut(HardwareConfig::I2C_TIMEOUT_MS);
  bmpOk_ = bmp_.begin(HardwareConfig::BMP280_I2C_ADDRESS) || bmp_.begin(0x77);
  ccsOk_ = ccs_.begin(HardwareConfig::CCS811_I2C_ADDRESS) || ccs_.begin(0x5B);
  mpuOk_ = mpu_.begin(HardwareConfig::MPU6050_I2C_ADDRESS) || mpu_.begin(0x69);
}

void SensorManager::update(uint32_t now, SensorReporter& reporter) {
  if (now - lastSampleAt_ >= ReportingConfig::SENSOR_SAMPLE_INTERVAL_MS) {
    lastSampleAt_ = now;
    if (bmpOk_) {
      temperature_ = bmp_.readTemperature();
      pressure_ = bmp_.readPressure() / 100.0F;
      temperatureValid_ = std::isfinite(temperature_);
      pressureValid_ = std::isfinite(pressure_);
    }
    ++diagnostics_.environmentalSamples;
    if (ccsOk_ && ccs_.available() && ccs_.readData() == 0) {
      eco2_ = ccs_.geteCO2();
      tvoc_ = ccs_.getTVOC();
      eco2Valid_ = true; tvocValid_ = true;
    } else if (ccsOk_) eco2Valid_ = tvocValid_ = false;
    reportIfChanged(reporter, now);
  }
  updateImu(now, reporter);
}

void SensorManager::reportIfChanged(SensorReporter& reporter, uint32_t now) {
  // A recovered sensor must publish its next valid sample even if it happens
  // to equal the value sent before the failed read.
  if (!temperatureValid_) hasTemperature_ = false;
  if (!pressureValid_) hasPressure_ = false;
  if (!eco2Valid_) hasEco2_ = false;
  if (!tvocValid_) hasTvoc_ = false;
  const bool environmentRefreshDue = now - lastTemperatureSentAt_ >= ReportingConfig::ENVIRONMENT_REFRESH_INTERVAL_MS;
  const bool pressureRefreshDue = now - lastPressureSentAt_ >= ReportingConfig::ENVIRONMENT_REFRESH_INTERVAL_MS;
  const bool airRefreshDue = now - lastEco2SentAt_ >= ReportingConfig::AIR_QUALITY_REFRESH_INTERVAL_MS;
  const bool tvocRefreshDue = now - lastTvocSentAt_ >= ReportingConfig::AIR_QUALITY_REFRESH_INTERVAL_MS;
  const bool temperatureChanged = hasTemperature_ && fabsf(temperature_ - reportedTemperature_) >= ReportingConfig::TEMPERATURE_DELTA_C;
  if (temperatureValid_ && (!hasTemperature_ || environmentRefreshDue || temperatureChanged)) {
    if (reporter.sensorUpdate("temperature", temperature_, "c")) {
      reportedTemperature_ = temperature_; lastTemperatureSentAt_ = now; hasTemperature_ = true;
      ++diagnostics_.updatesTransmitted; if (environmentRefreshDue && !temperatureChanged) ++diagnostics_.updatesRefreshed;
    } else ++diagnostics_.transmissionFailures;
  } else if (temperatureValid_) ++diagnostics_.updatesSuppressed;
  const bool pressureChanged = hasPressure_ && fabsf(pressure_ - reportedPressure_) >= ReportingConfig::PRESSURE_DELTA_HPA;
  if (pressureValid_ && (!hasPressure_ || pressureRefreshDue || pressureChanged)) {
    if (reporter.sensorUpdate("pressure", pressure_, "hpa")) {
      reportedPressure_ = pressure_; lastPressureSentAt_ = now; hasPressure_ = true;
      ++diagnostics_.updatesTransmitted; if (pressureRefreshDue && !pressureChanged) ++diagnostics_.updatesRefreshed;
    } else ++diagnostics_.transmissionFailures;
  } else if (pressureValid_) ++diagnostics_.updatesSuppressed;
  if (eco2Valid_ && (!hasEco2_ || airRefreshDue || abs(static_cast<int>(eco2_) - static_cast<int>(reportedEco2_)) >= ReportingConfig::ECO2_DELTA_PPM)) {
    if (reporter.sensorUpdate("eco2", eco2_, "ppm")) {
      reportedEco2_ = eco2_; lastEco2SentAt_ = now; hasEco2_ = true;
      ++diagnostics_.updatesTransmitted; if (airRefreshDue) ++diagnostics_.updatesRefreshed;
    } else ++diagnostics_.transmissionFailures;
  } else if (eco2Valid_) ++diagnostics_.updatesSuppressed;
  if (tvocValid_ && (!hasTvoc_ || tvocRefreshDue || abs(static_cast<int>(tvoc_) - static_cast<int>(reportedTvoc_)) >= ReportingConfig::TVOC_DELTA_PPB)) {
    if (reporter.sensorUpdate("tvoc", tvoc_, "ppb")) {
      reportedTvoc_ = tvoc_; lastTvocSentAt_ = now; hasTvoc_ = true;
      ++diagnostics_.updatesTransmitted; if (tvocRefreshDue) ++diagnostics_.updatesRefreshed;
    } else ++diagnostics_.transmissionFailures;
  } else if (tvocValid_) ++diagnostics_.updatesSuppressed;
}

void SensorManager::updateImu(uint32_t now, SensorReporter& reporter) {
  if (!mpuOk_ || now - lastImuSampleAt_ < ReportingConfig::IMU_SAMPLE_INTERVAL_MS) return;
  lastImuSampleAt_ = now;
  ++diagnostics_.imuSamples;
  sensors_event_t acceleration, gyro, temperature;
  mpu_.getEvent(&acceleration, &gyro, &temperature);
  accelerationX_ = acceleration.acceleration.x;
  accelerationY_ = acceleration.acceleration.y;
  accelerationZ_ = acceleration.acceleration.z;
  // Adafruit reports MPU6050 gyro events in radians/second; phos-hw requires dps.
  gyroX_ = gyro.gyro.x * RAD_TO_DEG;
  gyroY_ = gyro.gyro.y * RAD_TO_DEG;
  gyroZ_ = gyro.gyro.z * RAD_TO_DEG;
  if (!std::isfinite(accelerationX_) || !std::isfinite(accelerationY_) || !std::isfinite(accelerationZ_) ||
      !std::isfinite(gyroX_) || !std::isfinite(gyroY_) || !std::isfinite(gyroZ_)) {
    hasOrientation_ = false;
    return;
  }
  accelerationMagnitude_ = sqrtf(accelerationX_ * accelerationX_ + accelerationY_ * accelerationY_ + accelerationZ_ * accelerationZ_);
  gyroMagnitudeDps_ = sqrtf(gyroX_ * gyroX_ + gyroY_ * gyroY_ + gyroZ_ * gyroZ_);

  if (!gravityInitialized_) {
    gravityX_ = accelerationX_; gravityY_ = accelerationY_; gravityZ_ = accelerationZ_;
    gravityInitialized_ = true;
  } else {
    const float alpha = ReportingConfig::GRAVITY_FILTER_ALPHA;
    gravityX_ += alpha * (accelerationX_ - gravityX_);
    gravityY_ += alpha * (accelerationY_ - gravityY_);
    gravityZ_ += alpha * (accelerationZ_ - gravityZ_);
  }
  const float dx = accelerationX_ - gravityX_;
  const float dy = accelerationY_ - gravityY_;
  const float dz = accelerationZ_ - gravityZ_;
  dynamicAcceleration_ = sqrtf(dx * dx + dy * dy + dz * dz);

  const float adx = accelerationX_ - reportedAccelerationX_;
  const float ady = accelerationY_ - reportedAccelerationY_;
  const float adz = accelerationZ_ - reportedAccelerationZ_;
  const float gdx = gyroX_ - reportedGyroX_;
  const float gdy = gyroY_ - reportedGyroY_;
  const float gdz = gyroZ_ - reportedGyroZ_;
  const float accelerationDelta = hasOrientation_ ? sqrtf(adx * adx + ady * ady + adz * adz) : 0;
  const float gyroDelta = hasOrientation_ ? sqrtf(gdx * gdx + gdy * gdy + gdz * gdz) : 0;
  const bool orientationChanged = hasOrientation_ &&
      (accelerationDelta >= ReportingConfig::ORIENTATION_ACCELERATION_DELTA_M_S2 || gyroDelta >= ReportingConfig::ORIENTATION_GYRO_DELTA_DPS);
  const bool orientationRefreshDue = hasOrientation_ && now - lastOrientationAt_ >= ReportingConfig::ORIENTATION_REFRESH_INTERVAL_MS;
  const bool minimumIntervalElapsed = !hasOrientation_ || now - lastOrientationAt_ >= ReportingConfig::ORIENTATION_MIN_PUBLISH_INTERVAL_MS;
  if ((!hasOrientation_ || orientationRefreshDue || orientationChanged) && minimumIntervalElapsed) {
    if (reporter.orientationUpdate(accelerationX_, accelerationY_, accelerationZ_, gyroX_, gyroY_, gyroZ_)) {
      reportedAccelerationX_ = accelerationX_; reportedAccelerationY_ = accelerationY_; reportedAccelerationZ_ = accelerationZ_;
      reportedGyroX_ = gyroX_; reportedGyroY_ = gyroY_; reportedGyroZ_ = gyroZ_;
      lastOrientationAt_ = now; hasOrientation_ = true; ++diagnostics_.updatesTransmitted;
      if (orientationRefreshDue && !orientationChanged) ++diagnostics_.updatesRefreshed;
    } else ++diagnostics_.transmissionFailures;
#if PHOS_IMU_DIAGNOSTICS
    Serial.printf("IMU ax=%.2f ay=%.2f az=%.2f gx=%.1f gy=%.1f gz=%.1f state=%s intensity=%s\n",
                  accelerationX_, accelerationY_, accelerationZ_, gyroX_, gyroY_, gyroZ_,
                  persistentMotionState(), motionIntensity());
#endif
  } else if (hasOrientation_) ++diagnostics_.updatesSuppressed;

  const char* intensity = motionIntensity();
  if (accelerationMagnitude_ >= ReportingConfig::IMPACT_ACCELERATION_M_S2 ||
      dynamicAcceleration_ >= ReportingConfig::IMPACT_DYNAMIC_M_S2) {
    transientUntil_ = now + ReportingConfig::TRANSIENT_MOTION_INTERVAL_MS;
    reportMotion(reporter, "impact", intensity);
    return;
  }
  const bool shakePeak = dynamicAcceleration_ >= ReportingConfig::SHAKE_DYNAMIC_M_S2 ||
                         gyroMagnitudeDps_ >= ReportingConfig::SHAKE_GYRO_DPS;
  if (now - shakeWindowAt_ > ReportingConfig::SHAKE_WINDOW_MS) { shakeWindowAt_ = now; shakePeaks_ = 0; }
  if (shakePeak && now - lastShakePeakAt_ >= 80) { lastShakePeakAt_ = now; ++shakePeaks_; }
  if (shakePeaks_ >= ReportingConfig::SHAKE_PEAK_COUNT) {
    shakePeaks_ = 0; shakeWindowAt_ = now; transientUntil_ = now + ReportingConfig::TRANSIENT_MOTION_INTERVAL_MS;
    reportMotion(reporter, "shake", intensity);
    return;
  }
  if (now < transientUntil_) return;

  const bool motionAboveEnter = dynamicAcceleration_ >= ReportingConfig::MOVING_ENTER_DYNAMIC_M_S2 ||
                                gyroMagnitudeDps_ >= ReportingConfig::MOVING_ENTER_GYRO_DPS;
  const bool motionBelowExit = dynamicAcceleration_ <= ReportingConfig::MOVING_EXIT_DYNAMIC_M_S2 &&
                               gyroMagnitudeDps_ <= ReportingConfig::MOVING_EXIT_GYRO_DPS;
  const bool wantedMoving = moving_ ? !motionBelowExit : motionAboveEnter;
  if (wantedMoving != moving_) {
    if (motionCandidateAt_ == 0) motionCandidateAt_ = now;
    else if (now - motionCandidateAt_ >= ReportingConfig::MOTION_SETTLE_INTERVAL_MS) {
      moving_ = wantedMoving; motionCandidateAt_ = 0;
    }
  } else motionCandidateAt_ = 0;
  reportMotion(reporter, persistentMotionState(), intensity);
}

const char* SensorManager::persistentMotionState() const {
  // Priority after transient impact/shake: dominant stable tilt, moving, stationary.
  const bool retainingTilt = reportedMotionState_ &&
      (!strcmp(reportedMotionState_, "tilt_left") || !strcmp(reportedMotionState_, "tilt_right") ||
       !strcmp(reportedMotionState_, "tilt_forward") || !strcmp(reportedMotionState_, "tilt_backward"));
  const float threshold = retainingTilt ? ReportingConfig::TILT_EXIT_M_S2 : ReportingConfig::TILT_ENTER_M_S2;
  if (fabsf(gravityX_) >= threshold || fabsf(gravityY_) >= threshold) {
    if (fabsf(gravityX_) >= fabsf(gravityY_)) return gravityX_ >= 0 ? "tilt_left" : "tilt_right";
    return gravityY_ >= 0 ? "tilt_forward" : "tilt_backward";
  }
  return moving_ ? "moving" : "stationary";
}

const char* SensorManager::motionIntensity() const {
  if (dynamicAcceleration_ >= ReportingConfig::INTENSITY_HIGH_DYNAMIC_M_S2 ||
      gyroMagnitudeDps_ >= ReportingConfig::INTENSITY_HIGH_GYRO_DPS ||
      accelerationMagnitude_ >= ReportingConfig::IMPACT_ACCELERATION_M_S2) return "high";
  if (dynamicAcceleration_ >= ReportingConfig::INTENSITY_MEDIUM_DYNAMIC_M_S2 ||
      gyroMagnitudeDps_ >= ReportingConfig::INTENSITY_MEDIUM_GYRO_DPS) return "medium";
  return "low";
}

void SensorManager::reportMotion(SensorReporter& reporter, const char* state, const char* intensity) {
  if (!hasMotionState_ || strcmp(state, reportedMotionState_) || strcmp(intensity, reportedMotionIntensity_)) {
    if (reporter.motionUpdate(state, intensity)) {
      reportedMotionState_ = state; reportedMotionIntensity_ = intensity; hasMotionState_ = true;
      ++diagnostics_.updatesTransmitted;
    } else ++diagnostics_.transmissionFailures;
  }
}
