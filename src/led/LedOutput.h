#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "../config/HardwareConfig.h"

class LedOutput {
 public:
  void begin();
  void update(uint32_t now);
  void clear();
  bool setBrightness(float brightness);
  bool configure(const char* animation, const char* color, float brightness,
                 uint32_t periodMs, uint32_t durationMs, float minIntensity,
                 float maxIntensity, const char* direction = "",
                 const char* mode = "", const char* origin = "");

 private:
  Adafruit_NeoPixel pixels_{HardwareConfig::LED_COUNT, HardwareConfig::LED_PIN,
                            NEO_GRB + NEO_KHZ800};
  String animation_ = "off";
  uint32_t color_ = 0;
  uint32_t startedAt_ = 0;
  uint32_t periodMs_ = 1000;
  uint32_t durationMs_ = 0;
  float minIntensity_ = 0.0F;
  float maxIntensity_ = 1.0F;
  float brightness_ = static_cast<float>(HardwareConfig::LED_DEFAULT_BRIGHTNESS) / 255.0F;
  bool initialized_ = false;
  bool outputIsClear_ = true;
  bool frameDirty_ = false;
  String direction_ = "clockwise";
  String mode_ = "circular";
  String origin_ = "top";

  static bool oneOf(const char* value, const char* const* values, size_t count);
  uint16_t physicalIndex(uint16_t logicalIndex) const;
  uint16_t originIndex() const;
  uint16_t circularIndex(uint16_t progress) const;
  void setLogicalPixel(uint16_t logicalIndex, uint32_t color);
  void showCircularWipe(uint16_t progress);
  void showSymmetricChase(uint16_t progress);
  void showSymmetricWipe(uint16_t progress);
  void showSolid(uint32_t color, float intensity = 1.0F);
};
