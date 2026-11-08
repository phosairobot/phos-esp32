#pragma once

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace LedColor {

inline bool parse(const char* value, uint32_t& color) {
  if (!value || strlen(value) != 7 || value[0] != '#') return false;
  for (uint8_t i = 1; i < 7; ++i) if (!isxdigit(static_cast<unsigned char>(value[i]))) return false;
  color = static_cast<uint32_t>(strtoul(value + 1, nullptr, 16));
  return true;
}

// Round after scaling in floating point.  The constrained factor guarantees
// that an 8-bit channel can never overflow at full brightness.
inline uint8_t scaleChannel(uint8_t channel, float intensity) {
  intensity = constrain(intensity, 0.0F, 1.0F);
  return static_cast<uint8_t>(channel * intensity + 0.5F);
}

inline uint8_t brightnessByte(float brightness) {
  brightness = constrain(brightness, 0.0F, 1.0F);
  return static_cast<uint8_t>(brightness * 255.0F + 0.5F);
}

// Keep the requested protocol value separate from the brightness actually
// rendered on the LEDs. Both operands are bounded so a malformed build flag
// cannot increase output above full scale.
inline float cappedBrightness(float requested, float maximum) {
  return constrain(constrain(requested, 0.0F, 1.0F), 0.0F, constrain(maximum, 0.0F, 1.0F));
}

}  // namespace LedColor
