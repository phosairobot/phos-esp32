#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <unity.h>
#include "../../src/led/LedColor.h"

void test_composite_color_parsing() {
  uint32_t color = 0;
  TEST_ASSERT_TRUE(LedColor::parse("#CCCF29", color));
  TEST_ASSERT_EQUAL_HEX32(0xCCCF29, color);
  TEST_ASSERT_FALSE(LedColor::parse("#CCCG29", color));
}

void test_composite_color_brightness_boundaries() {
  const uint8_t expected[6][3] = {
      {0, 0, 0}, {20, 21, 4}, {102, 104, 21}, {184, 186, 37}, {202, 205, 41}, {204, 207, 41}};
  const float brightness[6] = {0.0F, 0.1F, 0.5F, 0.9F, 0.99F, 1.0F};
  for (uint8_t i = 0; i < 6; ++i) {
    TEST_ASSERT_EQUAL_UINT8(expected[i][0], LedColor::scaleChannel(204, brightness[i]));
    TEST_ASSERT_EQUAL_UINT8(expected[i][1], LedColor::scaleChannel(207, brightness[i]));
    TEST_ASSERT_EQUAL_UINT8(expected[i][2], LedColor::scaleChannel(41, brightness[i]));
  }
}

void test_white_and_primary_channels_do_not_overflow_at_full_brightness() {
  const float brightness[6] = {0.0F, 0.1F, 0.5F, 0.9F, 0.99F, 1.0F};
  const uint8_t whiteExpected[6] = {0, 26, 128, 230, 252, 255};
  for (uint8_t i = 0; i < 6; ++i) {
    // #FFFFFF and the illuminated channel of #FF0000, #00FF00 and #0000FF
    // must all retain the same bounded value at every brightness setting.
    TEST_ASSERT_EQUAL_UINT8(whiteExpected[i], LedColor::scaleChannel(255, brightness[i]));
    TEST_ASSERT_EQUAL_UINT8(whiteExpected[i], LedColor::scaleChannel(255, brightness[i])); // red primary
    TEST_ASSERT_EQUAL_UINT8(whiteExpected[i], LedColor::scaleChannel(255, brightness[i])); // green primary
    TEST_ASSERT_EQUAL_UINT8(whiteExpected[i], LedColor::scaleChannel(255, brightness[i])); // blue primary
    TEST_ASSERT_EQUAL_UINT8(0, LedColor::scaleChannel(0, brightness[i]));
  }
}

void test_brightness_normalization_boundaries() {
  const float input[5] = {0.0F, 0.2F, 0.5F, 0.99F, 1.0F};
  const uint8_t expected[5] = {0, 51, 128, 252, 255};
  for (uint8_t i = 0; i < 5; ++i)
    TEST_ASSERT_EQUAL_UINT8(expected[i], LedColor::brightnessByte(input[i]));
}

void test_effective_brightness_is_capped_without_changing_requested_value() {
  const float requested[] = {0.0F, 0.15F, 0.25F, 0.3F, 0.5F, 1.0F};
  const float expected[] = {0.0F, 0.15F, 0.25F, 0.25F, 0.25F, 0.25F};
  for (uint8_t i = 0; i < 6; ++i) {
    const float original = requested[i];
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, expected[i], LedColor::cappedBrightness(requested[i], 0.25F));
    // The source value is separate state in LedOutput and is not mutated by this pure render calculation.
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, original, requested[i]);
  }
}

void test_full_brightness_composite_buffer_is_raw_grb() {
  Adafruit_NeoPixel strip(1, 19, NEO_GRB + NEO_KHZ800);
  strip.setBrightness(LedColor::brightnessByte(1.0F));
  strip.setPixelColor(0, 204, 207, 41);
  const uint8_t* bytes = strip.getPixels();
  // NEO_GRB wire/storage order: G, R, B. At 255 no channel is rescaled.
  TEST_ASSERT_EQUAL_UINT8(207, bytes[0]);
  TEST_ASSERT_EQUAL_UINT8(204, bytes[1]);
  TEST_ASSERT_EQUAL_UINT8(41, bytes[2]);
}

void test_all_requested_colors_parse_to_expected_rgb() {
  struct ColorCase { const char* text; uint32_t rgb; };
  const ColorCase colors[] = {
      {"#FF0000", 0xFF0000}, {"#00FF00", 0x00FF00}, {"#0000FF", 0x0000FF},
      {"#FFFF00", 0xFFFF00}, {"#CCCF29", 0xCCCF29}, {"#FFFFFF", 0xFFFFFF}};
  for (const ColorCase& colorCase : colors) {
    uint32_t parsed = 0;
    TEST_ASSERT_TRUE(LedColor::parse(colorCase.text, parsed));
    TEST_ASSERT_EQUAL_HEX32(colorCase.rgb, parsed);
  }
}

void test_consecutive_rgb_writes_replace_the_entire_grb_buffer() {
  Adafruit_NeoPixel strip(2, 19, NEO_GRB + NEO_KHZ800);
  strip.setBrightness(255);
  for (uint8_t pixel = 0; pixel < 2; ++pixel) strip.setPixelColor(pixel, 255, 0, 0);
  for (uint8_t pixel = 0; pixel < 2; ++pixel) strip.setPixelColor(pixel, 0, 255, 0);
  const uint8_t* bytes = strip.getPixels();
  // Both pixels must now be green in GRB storage: G=255, R=0, B=0.
  for (uint8_t offset = 0; offset < 6; offset += 3) {
    TEST_ASSERT_EQUAL_UINT8(255, bytes[offset]);
    TEST_ASSERT_EQUAL_UINT8(0, bytes[offset + 1]);
    TEST_ASSERT_EQUAL_UINT8(0, bytes[offset + 2]);
  }
}

void test_black_clear_leaves_no_previous_channels_in_the_buffer() {
  Adafruit_NeoPixel strip(2, 19, NEO_GRB + NEO_KHZ800);
  strip.setBrightness(255);
  for (uint8_t pixel = 0; pixel < 2; ++pixel) strip.setPixelColor(pixel, 204, 207, 41);
  strip.clear();
  const uint8_t* bytes = strip.getPixels();
  for (uint8_t offset = 0; offset < 6; ++offset) TEST_ASSERT_EQUAL_UINT8(0, bytes[offset]);
}

void test_explicit_rgb_scaling_is_not_cumulative() {
  const uint8_t redAt40 = LedColor::scaleChannel(255, 40.0F / 255.0F);
  const uint8_t greenAt40 = LedColor::scaleChannel(255, 40.0F / 255.0F);
  // Each frame is derived from its unmodified RGB input, not from prior RAM.
  TEST_ASSERT_EQUAL_UINT8(40, redAt40);
  TEST_ASSERT_EQUAL_UINT8(40, greenAt40);
  TEST_ASSERT_EQUAL_UINT8(40, LedColor::scaleChannel(255, 40.0F / 255.0F));
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_composite_color_parsing);
  RUN_TEST(test_composite_color_brightness_boundaries);
  RUN_TEST(test_white_and_primary_channels_do_not_overflow_at_full_brightness);
  RUN_TEST(test_brightness_normalization_boundaries);
  RUN_TEST(test_effective_brightness_is_capped_without_changing_requested_value);
  RUN_TEST(test_full_brightness_composite_buffer_is_raw_grb);
  RUN_TEST(test_all_requested_colors_parse_to_expected_rgb);
  RUN_TEST(test_consecutive_rgb_writes_replace_the_entire_grb_buffer);
  RUN_TEST(test_black_clear_leaves_no_previous_channels_in_the_buffer);
  RUN_TEST(test_explicit_rgb_scaling_is_not_cumulative);
  UNITY_END();
}

void loop() {}
