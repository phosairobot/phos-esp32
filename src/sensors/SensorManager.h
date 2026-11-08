#pragma once

#include <Adafruit_BMP280.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_MPU6050.h>
#include <Arduino.h>

class SensorReporter {
 public:
  // True means the frame was accepted by the current serial transport.
  virtual bool sensorUpdate(const char* sensor, float value, const char* unit) = 0;
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
  };
  void begin();
  void update(uint32_t now, SensorReporter& reporter);
  bool bmpAvailable() const { return bmpOk_; }
  bool ccsAvailable() const { return ccsOk_; }
  bool mpuAvailable() const { return mpuOk_; }

 private:
  Adafruit_BMP280 bmp_;
  Adafruit_CCS811 ccs_;
  Adafruit_MPU6050 mpu_;
  bool bmpOk_ = false, ccsOk_ = false, mpuOk_ = false;
  bool hasTemperature_ = false, hasPressure_ = false, hasEco2_ = false, hasTvoc_ = false;
  bool temperatureValid_ = false, pressureValid_ = false, eco2Valid_ = false, tvocValid_ = false;
  bool hasMotionState_ = false, gravityInitialized_ = false, moving_ = false;
  bool hasOrientation_ = false;
  float temperature_ = 0, pressure_ = 0;
  float accelerationX_ = 0, accelerationY_ = 0, accelerationZ_ = 0;
  float gyroX_ = 0, gyroY_ = 0, gyroZ_ = 0;
  float gravityX_ = 0, gravityY_ = 0, gravityZ_ = 0;
  float dynamicAcceleration_ = 0, accelerationMagnitude_ = 0, gyroMagnitudeDps_ = 0;
  float reportedTemperature_ = 0, reportedPressure_ = 0;
  float reportedAccelerationX_ = 0, reportedAccelerationY_ = 0, reportedAccelerationZ_ = 0;
  float reportedGyroX_ = 0, reportedGyroY_ = 0, reportedGyroZ_ = 0;
  uint16_t eco2_ = 0, tvoc_ = 0, reportedEco2_ = 0, reportedTvoc_ = 0;
  uint32_t lastSampleAt_ = 0, lastImuSampleAt_ = 0, lastOrientationAt_ = 0;
  uint32_t lastTemperatureSentAt_ = 0, lastPressureSentAt_ = 0, lastEco2SentAt_ = 0, lastTvocSentAt_ = 0;
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
  void updateImu(uint32_t now, SensorReporter& reporter);
  const char* persistentMotionState() const;
  const char* motionIntensity() const;
  void reportMotion(SensorReporter& reporter, const char* state, const char* intensity);
};
