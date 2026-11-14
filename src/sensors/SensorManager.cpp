#include "SensorManager.h"

#include "../config/HardwareConfig.h"
#include "../config/ReportingConfig.h"

#include <Wire.h>
#include <ScioSense_ENS16x.h>
#include <cmath>
#include <math.h>
#include <string.h>

#ifndef PHOS_IMU_DIAGNOSTICS
#define PHOS_IMU_DIAGNOSTICS 0
#endif

#ifndef PHOS_CONTROLLER_DIAGNOSTICS
#define PHOS_CONTROLLER_DIAGNOSTICS 0
#endif

#ifndef PHOS_MPU_DISABLE_MOTION_REPORTING
#define PHOS_MPU_DISABLE_MOTION_REPORTING 0
#endif

namespace {
ENS160 primaryEns160;
ENS160 alternateEns160;
}

void SensorManager::begin() {
  Wire.begin(HardwareConfig::I2C_SDA_PIN, HardwareConfig::I2C_SCL_PIN);
  Wire.setTimeOut(HardwareConfig::I2C_TIMEOUT_MS);
  bmpOk_ = bmp_.begin(HardwareConfig::BMP280_I2C_ADDRESS) || bmp_.begin(0x77);
  ensPrimary_ = &primaryEns160;
  ensPrimary_->begin(&Wire, HardwareConfig::ENS160_I2C_ADDRESS);
  if (ensPrimary_->init()) {
    ens_ = ensPrimary_;
    ensAddress_ = HardwareConfig::ENS160_I2C_ADDRESS;
  } else {
    ensAlternate_ = &alternateEns160;
    ensAlternate_->begin(&Wire, HardwareConfig::ENS160_ALTERNATE_I2C_ADDRESS);
    if (ensAlternate_->init()) {
      ens_ = ensAlternate_;
      ensAddress_ = HardwareConfig::ENS160_ALTERNATE_I2C_ADDRESS;
    }
  }
  ensOk_ = ens_ && ens_->startStandardMeasure() == RESULT_OK;
#if PHOS_CONTROLLER_DIAGNOSTICS
  diagnostics_.ensDetected = ensOk_;
  diagnostics_.ensAddress = ensOk_ ? ensAddress_ : 0;
  if (ensOk_) {
    uint8_t operatingMode = 0;
    if (readEnsRegister(ENS16X_REGISTER_ADDRESS_OPMODE, operatingMode))
      diagnostics_.ensOperatingMode = static_cast<int8_t>(operatingMode);
  }
#endif
  ahtOk_ = aht_.begin(&Wire, 0, HardwareConfig::AHT21_I2C_ADDRESS);
  mpuOk_ = mpu_.begin(HardwareConfig::MPU6050_I2C_ADDRESS);
  if (mpuOk_) mpuAddress_ = HardwareConfig::MPU6050_I2C_ADDRESS;
  else {
    mpuOk_ = mpu_.begin(0x69);
    if (mpuOk_) mpuAddress_ = 0x69;
  }
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
    if (ahtOk_) {
      sensors_event_t humidityEvent, temperatureEvent;
      aht_.getEvent(&humidityEvent, &temperatureEvent);
      humidity_ = humidityEvent.relative_humidity;
      ahtTemperature_ = temperatureEvent.temperature;
      humidityValid_ = std::isfinite(humidity_) && humidity_ >= 0 && humidity_ <= 100;
      ahtTemperatureValid_ = std::isfinite(ahtTemperature_);
#if PHOS_CONTROLLER_DIAGNOSTICS
      diagnostics_.ahtHumidityValid = humidityValid_;
      diagnostics_.ahtTemperatureValid = ahtTemperatureValid_;
#endif
    }
    // This is the proven reference path. update() is a single status read;
    // it additionally reads DATA_x for NEWDAT and GPR_READ for NEWGPR, with
    // no internal wait. Its cached status avoids a competing DATA_STATUS read.
    const Result ensUpdate = ensOk_ ? ens_->update() : RESULT_IO_ERROR;
    const uint8_t ensStatus = ensOk_ ? ens_->getDeviceStatus() : 0;
    const bool ensStatusRead = ensOk_ && ensUpdate != RESULT_IO_ERROR;
    ensValidity_ = ensStatusRead ? ((ensStatus >> 2) & 0x03) : 3;
    diagnostics_.ensValidity = ensValidity_;
    const bool ensNewData = ensOk_ && ens_->hasNewData();
    bool compensationWritten = false;
    if (ensUpdate == RESULT_OK && humidityValid_ && ahtTemperatureValid_) {
      const uint16_t temperature = static_cast<uint16_t>((ahtTemperature_ + 273.15F) * 64.0F);
      const uint16_t humidity = static_cast<uint16_t>(humidity_ * 512.0F);
      compensationWritten = ens_->writeCompensation(temperature, humidity) == RESULT_OK;
    }
    diagnostics_.ensCompensationActive = compensationWritten;
#if PHOS_CONTROLLER_DIAGNOSTICS
    readEnsDiagnostics(now);
    diagnostics_.ensNewData = ensNewData;
    diagnostics_.ensMeasurementAttempted = ensOk_;
    diagnostics_.ensMeasurementRead = ensNewData;
    diagnostics_.ensMeasurementUsable = false;
    if (ensUpdate == RESULT_OK && humidityValid_ && ahtTemperatureValid_) {
      ++diagnostics_.ensCompensationAttempts;
      diagnostics_.ensTempCompensationWriteOk = compensationWritten;
      diagnostics_.ensRhCompensationWriteOk = compensationWritten;
      diagnostics_.ensLastCompensationAt = now;
      if (!compensationWritten) ++diagnostics_.ensCompensationFailures;
    }
#endif
    if (ensNewData) {
      // ENS160 DATA_STATUS validity is bits 3:2: 0 normal, 1 warm-up,
      // 2 initial start-up, 3 no valid output.
      aqi_ = static_cast<uint8_t>(ens_->getAirQualityIndex_UBA());
      eco2_ = ens_->getEco2();
      tvoc_ = ens_->getTvoc();
      // An acquired sample is telemetry even while ENS160 is warming up.
      // sensor_status supplies its contemporaneous validity; the Pi records
      // such values but cannot use them for environmental decisions.
      eco2SamplePending_ = true;
      tvocSamplePending_ = true;
      aqiSamplePending_ = aqi_ >= 1 && aqi_ <= 5;
      aqiValid_ = ensValidity_ == 0 && aqi_ >= 1 && aqi_ <= 5;
      eco2Valid_ = tvocValid_ = aqiValid_;
#if PHOS_CONTROLLER_DIAGNOSTICS
      ++diagnostics_.ensDataReadySamples;
      diagnostics_.ensHasRawMeasurement = true;
      diagnostics_.ensLastMeasurementAt = now;
      diagnostics_.ensRawAqi = aqi_;
      diagnostics_.ensRawEco2 = eco2_;
      diagnostics_.ensRawTvoc = tvoc_;
      if (ensValidity_ != 0) {
        ++diagnostics_.ensRejectedValidity;
        diagnostics_.ensLastRejection = 2;
        diagnostics_.ensAvailabilityReason = ensValidity_ == 1 ? "warmup" :
            ensValidity_ == 2 ? "initial_startup" : "invalid_output";
      } else if (!aqiValid_) {
        ++diagnostics_.ensRejectedAqi;
        diagnostics_.ensLastRejection = 3;
        diagnostics_.ensAvailabilityReason = "aqi_out_of_range";
      } else {
        ++diagnostics_.ensAcceptedSamples;
        diagnostics_.ensLastAcceptedAt = now;
        diagnostics_.ensLastRejection = 0;
        diagnostics_.ensMeasurementUsable = true;
        diagnostics_.ensAvailabilityReason = "usable";
      }
#endif
    } else if (ensOk_) eco2Valid_ = tvocValid_ = aqiValid_ = false;
#if PHOS_CONTROLLER_DIAGNOSTICS
    if (ensOk_ && !ensNewData) {
      ++diagnostics_.ensRejectedNoNewData;
      diagnostics_.ensLastRejection = 1;
      diagnostics_.ensAvailabilityReason = ensUpdate == RESULT_IO_ERROR ? "i2c_error" : "no_new_data";
    } else if (!ensOk_) {
      diagnostics_.ensAvailabilityReason = "sensor_unavailable";
    }
#endif
    const bool compensationActive = diagnostics_.ensCompensationActive;
    if (ensValidity_ != reportedEnsValidity_ || compensationActive != reportedCompensationActive_ ||
        now - lastEnsStatusSentAt_ >= ReportingConfig::AIR_QUALITY_REFRESH_INTERVAL_MS) {
      if (reporter.sensorStatus(ensValidity_, compensationActive)) {
        reportedEnsValidity_ = ensValidity_; reportedCompensationActive_ = compensationActive; lastEnsStatusSentAt_ = now;
      }
    }
    reportIfChanged(reporter, now);
  }
  updateImu(now, reporter);
}

void SensorManager::reportIfChanged(SensorReporter& reporter, uint32_t now) {
  // A recovered sensor must publish its next valid sample even if it happens
  // to equal the value sent before the failed read.
  if (!temperatureValid_) hasTemperature_ = false;
  if (!pressureValid_) hasPressure_ = false;
  if (!humidityValid_) hasHumidity_ = false;
  if (!ahtTemperatureValid_) hasAhtTemperature_ = false;
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
  if (eco2SamplePending_ && (!hasEco2_ || airRefreshDue || abs(static_cast<int>(eco2_) - static_cast<int>(reportedEco2_)) >= ReportingConfig::ECO2_DELTA_PPM)) {
    if (reporter.sensorUpdate("eco2", eco2_, "ppm")) {
      reportedEco2_ = eco2_; lastEco2SentAt_ = now; hasEco2_ = true;
      eco2SamplePending_ = false;
      ++diagnostics_.updatesTransmitted; if (airRefreshDue) ++diagnostics_.updatesRefreshed;
#if PHOS_CONTROLLER_DIAGNOSTICS
      ++diagnostics_.ensReportFrames; diagnostics_.ensLastReportAt = now;
#endif
    } else ++diagnostics_.transmissionFailures;
  } else if (eco2SamplePending_) ++diagnostics_.updatesSuppressed;
  if (tvocSamplePending_ && (!hasTvoc_ || tvocRefreshDue || abs(static_cast<int>(tvoc_) - static_cast<int>(reportedTvoc_)) >= ReportingConfig::TVOC_DELTA_PPB)) {
    if (reporter.sensorUpdate("tvoc", tvoc_, "ppb")) {
      reportedTvoc_ = tvoc_; lastTvocSentAt_ = now; hasTvoc_ = true;
      tvocSamplePending_ = false;
      ++diagnostics_.updatesTransmitted; if (tvocRefreshDue) ++diagnostics_.updatesRefreshed;
#if PHOS_CONTROLLER_DIAGNOSTICS
      ++diagnostics_.ensReportFrames; diagnostics_.ensLastReportAt = now;
#endif
    } else ++diagnostics_.transmissionFailures;
  } else if (tvocSamplePending_) ++diagnostics_.updatesSuppressed;
  const bool aqiRefreshDue = now - lastAqiSentAt_ >= ReportingConfig::AIR_QUALITY_REFRESH_INTERVAL_MS;
  if (aqiSamplePending_ && (!hasAqi_ || aqiRefreshDue || aqi_ != reportedAqi_)) {
    if (reporter.sensorUpdate("aqi", aqi_, "index")) {
      reportedAqi_ = aqi_; lastAqiSentAt_ = now; hasAqi_ = true; ++diagnostics_.updatesTransmitted;
      aqiSamplePending_ = false;
#if PHOS_CONTROLLER_DIAGNOSTICS
      ++diagnostics_.ensReportFrames; diagnostics_.ensLastReportAt = now;
#endif
    }
    else ++diagnostics_.transmissionFailures;
  }
  const bool humidityRefreshDue = now - lastHumiditySentAt_ >= ReportingConfig::ENVIRONMENT_REFRESH_INTERVAL_MS;
  if (humidityValid_ && (!hasHumidity_ || humidityRefreshDue || fabsf(humidity_ - reportedHumidity_) >= .5F)) {
    if (reporter.sensorUpdate("humidity", humidity_, "%rh")) { reportedHumidity_ = humidity_; lastHumiditySentAt_ = now; hasHumidity_ = true; ++diagnostics_.updatesTransmitted; }
    else ++diagnostics_.transmissionFailures;
  }
  const bool ahtTemperatureRefreshDue = now - lastAhtTemperatureSentAt_ >= ReportingConfig::ENVIRONMENT_REFRESH_INTERVAL_MS;
  if (ahtTemperatureValid_ && (!hasAhtTemperature_ || ahtTemperatureRefreshDue || fabsf(ahtTemperature_ - reportedAhtTemperature_) >= ReportingConfig::TEMPERATURE_DELTA_C)) {
    if (reporter.sensorUpdate("aht21_temperature", ahtTemperature_, "c")) { reportedAhtTemperature_ = ahtTemperature_; lastAhtTemperatureSentAt_ = now; hasAhtTemperature_ = true; ++diagnostics_.updatesTransmitted; }
    else ++diagnostics_.transmissionFailures;
  }
}

bool SensorManager::readEnsRegister(uint8_t reg, uint8_t& value) {
  return ens_ && ens_->read(static_cast<Ens16x_RegisterAddress>(reg), &value, 1) == RESULT_OK;
}

void SensorManager::readEnsDiagnostics(uint32_t now) {
#if PHOS_CONTROLLER_DIAGNOSTICS
  (void)now;
  if (!ensOk_) return;
  // update() already read and cached DEVICE_STATUS. Do not issue another
  // status transaction in diagnostics, so the reference acquisition order is
  // exactly preserved.
  const uint8_t rawStatus = ens_->getDeviceStatus();
  diagnostics_.ensRawStatusRead = true;
  diagnostics_.ensRawStatus = rawStatus;
  diagnostics_.ensNewdat = ens_->hasNewData();
  uint8_t operatingMode = 0;
  if (readEnsRegister(ENS16X_REGISTER_ADDRESS_OPMODE, operatingMode))
    diagnostics_.ensOperatingMode = static_cast<int8_t>(operatingMode);
#else
  (void)now;
#endif
}

void SensorManager::updateImu(uint32_t now, SensorReporter& reporter) {
  if (!mpuOk_ || now - lastImuSampleAt_ < ReportingConfig::IMU_SAMPLE_INTERVAL_MS) return;
  lastImuSampleAt_ = now;
  ++diagnostics_.imuSamples;
#if PHOS_CONTROLLER_DIAGNOSTICS
  // Adafruit_MPU6050::getEvent() does not expose Wire status. Probe the same
  // address first and time both operations so an I2C fault is observable
  // without changing motion thresholds or normal reporting.
  const uint32_t probeStartedAt = micros();
  Wire.beginTransmission(mpuAddress_);
  const uint8_t i2cStatus = Wire.endTransmission(true);
  const uint32_t probeElapsedUs = micros() - probeStartedAt;
  if (i2cStatus != 0) ++diagnostics_.imuI2cErrors;
  if (probeElapsedUs >= static_cast<uint32_t>(HardwareConfig::I2C_TIMEOUT_MS) * 1000UL)
    ++diagnostics_.imuI2cTimeouts;
  if (i2cStatus != 0) return;
  const uint32_t readStartedAt = micros();
#endif
  sensors_event_t acceleration, gyro, temperature;
  mpu_.getEvent(&acceleration, &gyro, &temperature);
#if PHOS_CONTROLLER_DIAGNOSTICS
  const uint32_t readElapsedUs = micros() - readStartedAt;
  if (readElapsedUs >= static_cast<uint32_t>(HardwareConfig::I2C_TIMEOUT_MS) * 1000UL)
    ++diagnostics_.imuI2cTimeouts;
  else if (readElapsedUs >= ReportingConfig::IMU_SAMPLE_INTERVAL_MS * 1000UL)
    ++diagnostics_.imuSlowReads;
#endif
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
#if PHOS_CONTROLLER_DIAGNOSTICS
    ++diagnostics_.motionEventsGenerated;
#endif
#if PHOS_MPU_DISABLE_MOTION_REPORTING
    // Keep MPU sampling and state calculation active while isolating serial
    // traffic. State is still latched to avoid counting the same event at 50Hz.
    reportedMotionState_ = state; reportedMotionIntensity_ = intensity; hasMotionState_ = true;
#if PHOS_CONTROLLER_DIAGNOSTICS
    ++diagnostics_.motionEventsDisabled;
#endif
    return;
#else
    if (reporter.motionUpdate(state, intensity)) {
      reportedMotionState_ = state; reportedMotionIntensity_ = intensity; hasMotionState_ = true;
      ++diagnostics_.updatesTransmitted;
#if PHOS_CONTROLLER_DIAGNOSTICS
      ++diagnostics_.motionEventsTransmitted;
#endif
    } else ++diagnostics_.transmissionFailures;
#endif
  }
}
