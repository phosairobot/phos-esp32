#pragma once

#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_MPU6050.h>
#include <Arduino.h>

#include "../config/HardwareConfig.h"

class ENS160;

class SensorReporter {
 public:
  // True means the frame was accepted by the current serial transport.
  virtual bool sensorUpdate(const char* sensor, float value, const char* unit) = 0;
  virtual bool sensorStatus(uint8_t validity, bool compensationActive) = 0;
  virtual bool motionUpdate(const char* state, const char* intensity) = 0;
  virtual bool orientationUpdate(float ax, float ay, float az, float gx, float gy, float gz) = 0;
};

class SensorManager {
 public:
  struct Diagnostics {
    uint32_t environmentalSamples = 0;
    uint32_t imuSamples = 0;
    uint32_t updatesTransmitted = 0;
    uint32_t updatesSuppressed = 0;
    uint32_t updatesRefreshed = 0;
    uint32_t transmissionFailures = 0;
    uint32_t motionEventsGenerated = 0;
    uint32_t motionEventsTransmitted = 0;
    uint32_t motionEventsDisabled = 0;
    uint32_t imuI2cErrors = 0;
    uint32_t imuI2cTimeouts = 0;
    uint32_t imuSlowReads = 0;
    uint8_t ensValidity = 3;
    bool ensCompensationActive = false;
    bool ensDetected = false;
    uint8_t ensAddress = 0;
    int8_t ensOperatingMode = -1;
    bool ensRawStatusRead = false;
    uint8_t ensRawStatus = 0;
    bool ensNewdat = false;
    bool ensNewData = false;
    bool ensMeasurementAttempted = false;
    bool ensMeasurementRead = false;
    bool ensHasRawMeasurement = false;
    bool ensMeasurementUsable = false;
    uint32_t ensLastMeasurementAt = 0;
    const char* ensAvailabilityReason = "sensor_unavailable";
    uint8_t ensRawAqi = 0;
    uint16_t ensRawEco2 = 0;
    uint16_t ensRawTvoc = 0;
    uint32_t ensDataReadySamples = 0;
    uint32_t ensAcceptedSamples = 0;
    uint32_t ensRejectedNoNewData = 0;
    uint32_t ensRejectedValidity = 0;
    uint32_t ensRejectedAqi = 0;
    uint32_t ensReportFrames = 0;
    uint8_t ensLastRejection = 0;
    uint32_t ensLastAcceptedAt = 0;
    uint32_t ensLastReportAt = 0;
    bool ahtHumidityValid = false;
    bool ahtTemperatureValid = false;
    uint32_t ensCompensationAttempts = 0;
    uint32_t ensCompensationFailures = 0;
    bool ensTempCompensationWriteOk = false;
    bool ensRhCompensationWriteOk = false;
    uint32_t ensLastCompensationAt = 0;
  };
  void begin();
  void update(uint32_t now, SensorReporter& reporter);
  bool bmpAvailable() const { return bmpOk_; }
  bool ensAvailable() const { return ensOk_; }
  bool ahtAvailable() const { return ahtOk_; }
  bool mpuAvailable() const { return mpuOk_; }

 private:
  Adafruit_BMP280 bmp_;
  // ENS16x binds its I2C address in begin(); keep one instance for each
  // permitted strap state and select the instance that initializes.
  ENS160* ensPrimary_ = nullptr;
  ENS160* ensAlternate_ = nullptr;
  ENS160* ens_ = nullptr;
  Adafruit_AHTX0 aht_;
  Adafruit_MPU6050 mpu_;
  bool bmpOk_ = false, ensOk_ = false, ahtOk_ = false, mpuOk_ = false;
  uint8_t ensAddress_ = 0;
  uint8_t mpuAddress_ = 0;
  bool hasTemperature_ = false, hasPressure_ = false, hasEco2_ = false, hasTvoc_ = false, hasAqi_ = false, hasHumidity_ = false, hasAhtTemperature_ = false;
  bool eco2SamplePending_ = false, tvocSamplePending_ = false, aqiSamplePending_ = false;
  bool temperatureValid_ = false, pressureValid_ = false, eco2Valid_ = false, tvocValid_ = false, aqiValid_ = false, humidityValid_ = false, ahtTemperatureValid_ = false;
  bool hasMotionState_ = false, gravityInitialized_ = false, moving_ = false;
  bool hasOrientation_ = false;
  float temperature_ = 0, pressure_ = 0, humidity_ = 0, ahtTemperature_ = 0;
  float accelerationX_ = 0, accelerationY_ = 0, accelerationZ_ = 0;
  float gyroX_ = 0, gyroY_ = 0, gyroZ_ = 0;
  float gravityX_ = 0, gravityY_ = 0, gravityZ_ = 0;
  float dynamicAcceleration_ = 0, accelerationMagnitude_ = 0, gyroMagnitudeDps_ = 0;
  float reportedTemperature_ = 0, reportedPressure_ = 0, reportedHumidity_ = 0, reportedAhtTemperature_ = 0;
  float reportedAccelerationX_ = 0, reportedAccelerationY_ = 0, reportedAccelerationZ_ = 0;
  float reportedGyroX_ = 0, reportedGyroY_ = 0, reportedGyroZ_ = 0;
  uint16_t eco2_ = 0, tvoc_ = 0, reportedEco2_ = 0, reportedTvoc_ = 0;
  uint8_t aqi_ = 0, reportedAqi_ = 0, ensValidity_ = 3;
  uint8_t reportedEnsValidity_ = 255;
  bool reportedCompensationActive_ = false;
  uint32_t lastEnsStatusSentAt_ = 0;
  uint32_t lastSampleAt_ = 0, lastImuSampleAt_ = 0, lastOrientationAt_ = 0;
  uint32_t lastTemperatureSentAt_ = 0, lastPressureSentAt_ = 0, lastEco2SentAt_ = 0, lastTvocSentAt_ = 0, lastAqiSentAt_ = 0, lastHumiditySentAt_ = 0, lastAhtTemperatureSentAt_ = 0;
  uint32_t motionCandidateAt_ = 0, transientUntil_ = 0, shakeWindowAt_ = 0, lastShakePeakAt_ = 0;
  uint8_t shakePeaks_ = 0;
  const char* reportedMotionState_ = nullptr;
  const char* reportedMotionIntensity_ = nullptr;
  Diagnostics diagnostics_;
 public:
  // Snapshot-only diagnostics: callers may emit this through a separate debug
  // channel without contaminating the phos-hw JSONL serial stream.
  Diagnostics diagnostics() const { return diagnostics_; }
 private:
  void reportIfChanged(SensorReporter& reporter, uint32_t now);
  bool readEnsRegister(uint8_t reg, uint8_t& value);
  void readEnsDiagnostics(uint32_t now);
  void updateImu(uint32_t now, SensorReporter& reporter);
  const char* persistentMotionState() const;
  const char* motionIntensity() const;
  void reportMotion(SensorReporter& reporter, const char* state, const char* intensity);
};
