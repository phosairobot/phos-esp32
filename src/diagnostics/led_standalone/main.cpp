// Standalone WS2812B ring diagnostic. This deliberately does not include PHOS
// headers, protocol handling, sensors, queues, or animations.
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Configuration defaults can be overridden in [env:esp32_led_test] build_flags.
#ifndef LED_TEST_PIN
#define LED_TEST_PIN 19
#endif

#ifndef LED_TEST_COUNT
#define LED_TEST_COUNT 12
#endif

#ifndef LED_TEST_BRIGHTNESS
#define LED_TEST_BRIGHTNESS 40
#endif

#ifndef LED_TEST_COLOR_ORDER
#define LED_TEST_COLOR_ORDER NEO_GRB
#endif

#ifndef LED_TEST_SIGNAL_FREQUENCY
#define LED_TEST_SIGNAL_FREQUENCY NEO_KHZ800
#endif

#ifndef LED_TEST_NORMAL_HOLD_MS
#define LED_TEST_NORMAL_HOLD_MS 2000UL
#endif

// Run the extended hardware checks after this many normal colour cycles.
#ifndef LED_TEST_NORMAL_CYCLES_BEFORE_DIAGNOSTICS
#define LED_TEST_NORMAL_CYCLES_BEFORE_DIAGNOSTICS 3
#endif

// Set to 0 to run only the normal eight-colour sequence continuously.
#ifndef LED_TEST_DIAGNOSTICS
#define LED_TEST_DIAGNOSTICS 1
#endif

constexpr neoPixelType LED_TEST_PIXEL_TYPE = LED_TEST_COLOR_ORDER + LED_TEST_SIGNAL_FREQUENCY;
constexpr uint32_t DIAGNOSTIC_HOLD_MS = 300;
constexpr uint32_t FAST_HOLD_MS = 150;
constexpr uint32_t OFF_VERIFY_HOLD_MS = 2000;

Adafruit_NeoPixel strip(LED_TEST_COUNT, LED_TEST_PIN, LED_TEST_PIXEL_TYPE);

struct Step {
  const char* name;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint32_t holdMs;
};

const Step normalSteps[] = {
    {"red", 255, 0, 0, LED_TEST_NORMAL_HOLD_MS},
    {"green", 0, 255, 0, LED_TEST_NORMAL_HOLD_MS},
    {"blue", 0, 0, 255, LED_TEST_NORMAL_HOLD_MS},
    {"white", 255, 255, 255, LED_TEST_NORMAL_HOLD_MS},
    {"yellow", 255, 255, 0, LED_TEST_NORMAL_HOLD_MS},
    {"purple", 255, 0, 255, LED_TEST_NORMAL_HOLD_MS},
    {"cyan", 0, 255, 255, LED_TEST_NORMAL_HOLD_MS},
    {"off", 0, 0, 0, LED_TEST_NORMAL_HOLD_MS},
};

// Each group exercises a distinct electrical/data-path transition. These are
// intentionally finite checks, not PHOS runtime animations.
const Step diagnosticSteps[] = {
    {"diag power on", 255, 255, 255, DIAGNOSTIC_HOLD_MS},
    {"diag power off", 0, 0, 0, DIAGNOSTIC_HOLD_MS},
    {"diag power on", 255, 255, 255, DIAGNOSTIC_HOLD_MS},
    {"diag power off", 0, 0, 0, DIAGNOSTIC_HOLD_MS},
    {"diag fast red", 255, 0, 0, FAST_HOLD_MS},
    {"diag fast green", 0, 255, 0, FAST_HOLD_MS},
    {"diag fast blue", 0, 0, 255, FAST_HOLD_MS},
    {"diag primary red", 255, 0, 0, DIAGNOSTIC_HOLD_MS},
    {"diag mixed yellow", 255, 255, 0, DIAGNOSTIC_HOLD_MS},
    {"diag primary green", 0, 255, 0, DIAGNOSTIC_HOLD_MS},
    {"diag mixed cyan", 0, 255, 255, DIAGNOSTIC_HOLD_MS},
    {"diag primary blue", 0, 0, 255, DIAGNOSTIC_HOLD_MS},
    {"diag mixed purple", 255, 0, 255, DIAGNOSTIC_HOLD_MS},
    {"diag PHOS #CCCF29", 204, 207, 41, DIAGNOSTIC_HOLD_MS},
    {"diag PHOS blue", 0, 0, 255, DIAGNOSTIC_HOLD_MS},
    {"diag PHOS green", 0, 255, 0, DIAGNOSTIC_HOLD_MS},
    {"diag PHOS black", 0, 0, 0, DIAGNOSTIC_HOLD_MS},
    {"diag off verify white", 255, 255, 255, DIAGNOSTIC_HOLD_MS},
    {"diag off command", 0, 0, 0, OFF_VERIFY_HOLD_MS},
};

uint32_t normalCycle = 0;
uint32_t diagnosticCycle = 0;
uint8_t shownRed = 0;
uint8_t shownGreen = 0;
uint8_t shownBlue = 0;

void showStep(const Step& step, const char* phase, uint32_t cycle) {
  const bool colorChanged = step.red != shownRed || step.green != shownGreen || step.blue != shownBlue;
  if (colorChanged) {
    for (uint16_t pixel = 0; pixel < LED_TEST_COUNT; ++pixel)
      strip.setPixelColor(pixel, step.red, step.green, step.blue);
    // show() is called once for a colour transition, never continuously.
    strip.show();
    shownRed = step.red;
    shownGreen = step.green;
    shownBlue = step.blue;
  }
  Serial.print(phase);
  Serial.print(" cycle ");
  Serial.print(cycle);
  Serial.print(": ");
  Serial.println(step.name);
  delay(step.holdMs);
}

void runSteps(const Step* steps, size_t count, const char* phase, uint32_t cycle) {
  for (size_t index = 0; index < count; ++index) showStep(steps[index], phase, cycle);
}

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(LED_TEST_BRIGHTNESS);  // Fixed once; never adjusted in loop().
  strip.clear();
  strip.show();
  Serial.println("WS2812B standalone diagnostic started");
}

void loop() {
  ++normalCycle;
  runSteps(normalSteps, sizeof(normalSteps) / sizeof(normalSteps[0]), "normal", normalCycle);

#if LED_TEST_DIAGNOSTICS
  if (normalCycle >= LED_TEST_NORMAL_CYCLES_BEFORE_DIAGNOSTICS) {
    ++diagnosticCycle;
    runSteps(diagnosticSteps, sizeof(diagnosticSteps) / sizeof(diagnosticSteps[0]),
             "diagnostic", diagnosticCycle);
    normalCycle = 0;
  }
#endif
}
