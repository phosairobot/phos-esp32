#include "LedOutput.h"
#include "LedColor.h"

#include "../config/HardwareConfig.h"

#include <math.h>
#include <string.h>

#ifndef PHOS_LED_STANDALONE_SOLID_AB
#define PHOS_LED_STANDALONE_SOLID_AB 0
#endif

#ifndef PHOS_LED_STANDALONE_BRIGHTNESS
#define PHOS_LED_STANDALONE_BRIGHTNESS 40
#endif

#ifndef PHOS_LED_MAX_BRIGHTNESS
#define PHOS_LED_MAX_BRIGHTNESS 0.25F
#endif

#ifndef PHOS_LED_DEFAULT_BRIGHTNESS
#define PHOS_LED_DEFAULT_BRIGHTNESS 0.15F
#endif

void LedOutput::begin() {
  pinMode(HardwareConfig::LED_PIN, OUTPUT);
  digitalWrite(HardwareConfig::LED_PIN, LOW);
  delay(10);
  pixels_.begin();
  // Normal firmware rendering always keeps RGB input and brightness separate;
  // NeoPixel's setter rescales its existing RAM and is therefore not used
  // after initialization. The optional A/B mode deliberately matches the
  // standalone sketch for solid commands.
#if PHOS_LED_STANDALONE_SOLID_AB
  pixels_.setBrightness(PHOS_LED_STANDALONE_BRIGHTNESS);
  brightness_ = static_cast<float>(PHOS_LED_STANDALONE_BRIGHTNESS) / 255.0F;
#else
  pixels_.setBrightness(255);
#endif
  initialized_ = true;
  outputIsClear_ = false;
  clear();
}

void LedOutput::clear() {
  if (outputIsClear_) return;
  pixels_.clear();
  pixels_.show();
  outputIsClear_ = true;
  frameDirty_ = false;
}

bool LedOutput::setBrightness(float brightness) {
  if (!initialized_ || !std::isfinite(brightness)) return false;
#if PHOS_LED_STANDALONE_SOLID_AB
  // The A/B experiment intentionally holds solid output at the standalone
  // test level while JSON parsing and ACK handling remain unchanged.
  if (animation_ == "solid") brightness_ = static_cast<float>(PHOS_LED_STANDALONE_BRIGHTNESS) / 255.0F;
  else brightness_ = constrain(brightness, 0.0F, 1.0F);
#else
  brightness = constrain(brightness, 0.0F, 1.0F);
  brightness_ = brightness;
#endif
  frameDirty_ = true;
  update(millis());
  return true;
}

bool LedOutput::configure(const char* animation, const char* color, float brightness,
                          uint32_t periodMs, uint32_t durationMs, float minIntensity,
                          float maxIntensity, const char* direction, const char* mode,
                          const char* origin) {
  if (!initialized_ || animation == nullptr || periodMs == 0 || !std::isfinite(brightness) ||
      !std::isfinite(minIntensity) || !std::isfinite(maxIntensity)) return false;
  const char* supported[] = {"off", "solid", "blink", "pulse", "breathe", "chase", "spin", "wipe"};
  bool valid = oneOf(animation, supported, sizeof(supported) / sizeof(supported[0]));
  if (!valid) return false;
  direction = direction ? direction : "";
  mode = mode ? mode : "";
  origin = origin ? origin : "";
  const bool hasDirection = direction[0] != '\0', hasMode = mode[0] != '\0', hasOrigin = origin[0] != '\0';
  const char* directional[] = {"clockwise", "counterclockwise", "outward", "inward"};
  const char* modes[] = {"circular", "symmetric"};
  const char* origins[] = {"top", "bottom", "left", "right"};
  if ((hasDirection && !oneOf(direction, directional, 4)) || (hasMode && !oneOf(mode, modes, 2)) ||
      (hasOrigin && !oneOf(origin, origins, 4))) return false;
  if (!strcmp(animation, "off") || !strcmp(animation, "solid") || !strcmp(animation, "blink") ||
      !strcmp(animation, "pulse") || !strcmp(animation, "breathe")) {
    if (hasDirection || hasMode || hasOrigin) return false;
  } else if (!strcmp(animation, "spin")) {
    if ((hasMode && strcmp(mode, "circular")) || (hasDirection && strcmp(direction, "clockwise") &&
        strcmp(direction, "counterclockwise")) || hasOrigin) return false;
  } else {
    const char* actualMode = hasMode ? mode : "circular";
    if (!strcmp(actualMode, "circular")) {
      if (hasDirection && strcmp(direction, "clockwise") && strcmp(direction, "counterclockwise")) return false;
      if (!strcmp(animation, "chase") && hasOrigin) return false;
    } else if (!hasOrigin || !hasDirection || (strcmp(direction, "outward") && strcmp(direction, "inward"))) {
      return false;
    }
  }
  uint32_t parsedColor = 0;
  if (!LedColor::parse(color, parsedColor)) return false;
  animation_ = animation;
  direction_ = hasDirection ? direction : "clockwise";
  mode_ = hasMode ? mode : "circular";
  // Circular wipe is the only animation that accepts an optional origin.
  origin_ = hasOrigin ? origin : "top";
  color_ = parsedColor;
  periodMs_ = periodMs;
  durationMs_ = durationMs;
  minIntensity_ = constrain(minIntensity, 0.0F, 1.0F);
  maxIntensity_ = constrain(maxIntensity, minIntensity_, 1.0F);
  startedAt_ = millis();
  frameDirty_ = true;
  // Render and submit the new state now, before the command ACK is queued.
  // This is also required for a stable duration_ms=0 solid frame.
  return setBrightness(brightness);
}

void LedOutput::update(uint32_t now) {
  if (durationMs_ && now - startedAt_ >= durationMs_) animation_ = "off";
  if (animation_ == "off") return clear();
  // A static frame must be sent only after a command or brightness change.
  // Repeating the WS2812 transfer in every main-loop iteration can expose
  // marginal wiring/power setups as visible flicker without changing pixels.
  if (animation_ == "solid") {
    if (!frameDirty_) return;
    showSolid(color_);
    frameDirty_ = false;
    return;
  }

  const uint32_t elapsed = now - startedAt_;
  const float phase = static_cast<float>(elapsed % periodMs_) / periodMs_;
  if (animation_ == "blink") return showSolid(color_, phase < 0.5F ? maxIntensity_ : minIntensity_);
  if (animation_ == "pulse" || animation_ == "breathe") {
    const float wave = (sinf(phase * 2.0F * PI - PI / 2.0F) + 1.0F) * 0.5F;
    return showSolid(color_, minIntensity_ + wave * (maxIntensity_ - minIntensity_));
  }

  const uint16_t position = static_cast<uint16_t>(phase * HardwareConfig::LED_COUNT) % HardwareConfig::LED_COUNT;
  pixels_.clear();
  if (mode_ == "symmetric") {
    if (animation_ == "wipe") showSymmetricWipe(position);
    else showSymmetricChase(position);  // chase and spin are never symmetric by contract.
  } else if (animation_ == "wipe") {
    showCircularWipe(position);
  } else {
    setLogicalPixel(circularIndex(position), color_);  // spin and circular chase.
  }
  pixels_.show();
  outputIsClear_ = false;
}

bool LedOutput::oneOf(const char* value, const char* const* values, size_t count) {
  for (size_t i = 0; i < count; ++i) if (!strcmp(value, values[i])) return true;
  return false;
}

uint16_t LedOutput::physicalIndex(uint16_t logicalIndex) const {
  const int32_t count = HardwareConfig::LED_COUNT;
  int32_t physical = HardwareConfig::LED_TOP_PHYSICAL_INDEX +
      HardwareConfig::LED_CLOCKWISE_PHYSICAL_STEP * static_cast<int32_t>(logicalIndex % HardwareConfig::LED_COUNT);
  physical %= count;
  if (physical < 0) physical += count;
  return static_cast<uint16_t>(physical);
}

uint16_t LedOutput::originIndex() const {
  if (origin_ == "right") return HardwareConfig::LED_COUNT / 4;
  if (origin_ == "bottom") return HardwareConfig::LED_COUNT / 2;
  if (origin_ == "left") return (HardwareConfig::LED_COUNT * 3) / 4;
  return 0;  // top
}

uint16_t LedOutput::circularIndex(uint16_t progress) const {
  const uint16_t base = originIndex();
  if (direction_ == "counterclockwise")
    return (base + HardwareConfig::LED_COUNT - (progress % HardwareConfig::LED_COUNT)) % HardwareConfig::LED_COUNT;
  return (base + progress) % HardwareConfig::LED_COUNT;
}

void LedOutput::setLogicalPixel(uint16_t logicalIndex, uint32_t color) {
  const float brightness = effectiveBrightness();
  const uint8_t red = LedColor::scaleChannel((color >> 16) & 0xFF, brightness);
  const uint8_t green = LedColor::scaleChannel((color >> 8) & 0xFF, brightness);
  const uint8_t blue = LedColor::scaleChannel(color & 0xFF, brightness);
  pixels_.setPixelColor(physicalIndex(logicalIndex), red, green, blue);
}

void LedOutput::showCircularWipe(uint16_t progress) {
  for (uint16_t step = 0; step <= progress; ++step) setLogicalPixel(circularIndex(step), color_);
}

void LedOutput::showSymmetricChase(uint16_t progress) {
  const uint16_t half = HardwareConfig::LED_COUNT / 2;
  const uint16_t phase = progress % (half + 1);
  // Outward moves away from the origin; inward starts at the opposite point and converges to it.
  const uint16_t distance = direction_ == "inward" ? half - phase : phase;
  const uint16_t origin = originIndex();
  setLogicalPixel((origin + distance) % HardwareConfig::LED_COUNT, color_);
  setLogicalPixel((origin + HardwareConfig::LED_COUNT - distance) % HardwareConfig::LED_COUNT, color_);
}

void LedOutput::showSymmetricWipe(uint16_t progress) {
  const uint16_t half = HardwareConfig::LED_COUNT / 2;
  const uint16_t phase = progress % (half + 1);
  const uint16_t origin = originIndex();
  // For even counts, +half and -half are the same opposite LED and setting it twice is harmless.
  for (uint16_t distance = 0; distance <= half; ++distance) {
    const bool active = direction_ == "inward" ? distance >= half - phase : distance <= phase;
    if (!active) continue;
    setLogicalPixel((origin + distance) % HardwareConfig::LED_COUNT, color_);
    setLogicalPixel((origin + HardwareConfig::LED_COUNT - distance) % HardwareConfig::LED_COUNT, color_);
  }
}

void LedOutput::showSolid(uint32_t color, float intensity) {
#if PHOS_LED_STANDALONE_SOLID_AB
  if (animation_ == "solid") {
    showRawSolid(color);
    return;
  }
#endif
  const float combinedIntensity = effectiveBrightness(intensity);
  const uint8_t red = LedColor::scaleChannel((color >> 16) & 0xFF, combinedIntensity);
  const uint8_t green = LedColor::scaleChannel((color >> 8) & 0xFF, combinedIntensity);
  const uint8_t blue = LedColor::scaleChannel(color & 0xFF, combinedIntensity);
  for (uint16_t i = 0; i < HardwareConfig::LED_COUNT; ++i) pixels_.setPixelColor(i, red, green, blue);
  pixels_.show();
  outputIsClear_ = false;
}

float LedOutput::effectiveBrightness(float intensity) const {
  return LedColor::cappedBrightness(brightness_, PHOS_LED_MAX_BRIGHTNESS) *
      constrain(intensity, 0.0F, 1.0F);
}

void LedOutput::showRawSolid(uint32_t color) {
  uint8_t red = (color >> 16) & 0xFF;
  uint8_t green = (color >> 8) & 0xFF;
  uint8_t blue = color & 0xFF;
  for (uint16_t i = 0; i < HardwareConfig::LED_COUNT; ++i) pixels_.setPixelColor(i, red, green, blue);
  pixels_.show();
  outputIsClear_ = false;
}
