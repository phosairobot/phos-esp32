#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "config/HardwareConfig.h"
#include "config/ReportingConfig.h"
#include "led/LedOutput.h"
#include "sensors/SensorManager.h"

namespace {
constexpr char PROTOCOL[] = "phos-hw";
constexpr uint8_t VERSION = 1;
LedOutput leds;
SensorManager sensors;
uint32_t lastHeartbeat = 0;
char rxBuffer[HardwareConfig::MAX_FRAME_BYTES + 1];
size_t rxLength = 0;
bool discardingOversizeFrame = false;

// Serial.write can wait indefinitely if a USB-UART peer stops draining while
// the host closes/reopens. Keep the main loop independent of that backpressure.
char txQueue[HardwareConfig::SERIAL_TX_QUEUE_BYTES];
char txEncodeBuffer[HardwareConfig::MAX_FRAME_BYTES + 1];
size_t txHead = 0, txTail = 0, txCount = 0;
uint32_t lastTxProgressAt = 0;

bool queueTx(const char* data, size_t length) {
  if (length > HardwareConfig::SERIAL_TX_QUEUE_BYTES - txCount) return false;
  for (size_t i = 0; i < length; ++i) txQueue[(txHead + i) % HardwareConfig::SERIAL_TX_QUEUE_BYTES] = data[i];
  txHead = (txHead + length) % HardwareConfig::SERIAL_TX_QUEUE_BYTES;
  txCount += length;
  if (txCount == length) lastTxProgressAt = millis();
  return true;
}

void serviceTx(uint32_t now) {
  if (!txCount) return;
  if (now - lastTxProgressAt >= HardwareConfig::SERIAL_TX_STALL_DROP_MS) {
    // Stale telemetry must not prevent replies after a host restart. Frames
    // are discarded only after no serial-progress for the bounded interval.
    txHead = txTail = txCount = 0;
    return;
  }
  const int writable = Serial.availableForWrite();
  if (writable <= 0) return;
  const size_t length = min(txCount, min(static_cast<size_t>(writable), HardwareConfig::SERIAL_TX_QUEUE_BYTES - txTail));
  const size_t written = Serial.write(reinterpret_cast<const uint8_t*>(txQueue + txTail), length);
  if (!written) return;
  txTail = (txTail + written) % HardwareConfig::SERIAL_TX_QUEUE_BYTES;
  txCount -= written;
  lastTxProgressAt = now;
}

bool send(const char* type, JsonDocument& payload) {
  // phos-hw v1 requires every outbound payload to be a JSON object.  In
  // particular, an empty JsonDocument would otherwise serialize as null.
  JsonObject payloadObject = payload.as<JsonObject>();
  if (payloadObject.isNull()) payloadObject = payload.to<JsonObject>();
  JsonDocument frame;
  frame["protocol"] = PROTOCOL; frame["version"] = VERSION; frame["type"] = type;
  frame["timestamp_ms"] = millis(); frame["payload"] = payloadObject;
  const size_t encoded = serializeJson(frame, txEncodeBuffer, sizeof(txEncodeBuffer));
  if (!encoded || encoded >= sizeof(txEncodeBuffer)) return false;
  txEncodeBuffer[encoded] = '\n';
  return queueTx(txEncodeBuffer, encoded + 1);
}

void ack(const char* command, bool ok, const char* error = nullptr) {
  JsonDocument payload;
  payload["command"] = command; payload["ok"] = ok;
  if (error && !ok) payload["error"] = error;
  send(ok ? "ack" : "error", payload);
}

class Reporter : public SensorReporter {
 public:
  bool sensorUpdate(const char* sensor, float value, const char* unit) override {
    JsonDocument p; p["sensor"] = sensor; p["value"] = value; p["unit"] = unit; return send("sensor_update", p);
  }
  bool motionUpdate(const char* state, const char* intensity) override {
    JsonDocument p; p["state"] = state; p["intensity"] = intensity; return send("motion_update", p);
  }
  bool orientationUpdate(float ax, float ay, float az, float gx, float gy, float gz) override {
    JsonDocument p;
    p["acceleration_m_s2"]["x"] = ax; p["acceleration_m_s2"]["y"] = ay; p["acceleration_m_s2"]["z"] = az;
    p["angular_velocity_dps"]["x"] = gx; p["angular_velocity_dps"]["y"] = gy; p["angular_velocity_dps"]["z"] = gz;
    return send("orientation_update", p);
  }
} reporter;

void capabilities() {
  JsonDocument p; JsonArray list = p["capabilities"].to<JsonArray>();
  if (sensors.bmpAvailable()) { list.add("temperature"); list.add("pressure"); }
  if (sensors.ccsAvailable()) list.add("air_quality");
  if (sensors.mpuAvailable()) list.add("motion");
  list.add("led"); p["led"]["count"] = HardwareConfig::LED_COUNT;
  send("capabilities", p);
}

void command(const char* input) {
  JsonDocument frame;
  if (deserializeJson(frame, input)) return ack("unknown", false, "malformed_frame");
  const char* protocol = frame["protocol"].as<const char*>();
  const uint8_t version = frame["version"] | 0;
  if (!protocol || strcmp(protocol, PROTOCOL) != 0 || version != VERSION) return ack("unknown", false, "invalid_envelope");
  const char* type = frame["type"] | "";
  JsonObject p = frame["payload"].as<JsonObject>();
  if (!p) return ack(type, false, "invalid_payload");
  if (!strcmp(type, "request_capabilities")) { capabilities(); return ack(type, true); }
  if (!strcmp(type, "ping")) { JsonDocument reply; send("heartbeat", reply); return; }
  if (!strcmp(type, "clear_led") || !strcmp(type, "stop_led_animation"))
    return ack(type, leds.configure("off", "#000000", 0, 1000, 0, 0, 1), "led_unavailable");
  if (!strcmp(type, "set_led_brightness")) {
    if (!p["brightness"].is<float>()) return ack(type, false, "invalid_parameter");
    return ack(type, leds.setBrightness(p["brightness"].as<float>()), "led_unavailable");
  }
  if (!strcmp(type, "set_led_animation")) {
    const bool ok = leds.configure(p["animation"] | "", p["color"] | "#FFFFFF", p["brightness"] | 1.0F,
      p["period_ms"] | 1000UL, p["duration_ms"] | 0UL, p["min_intensity"] | 0.0F, p["max_intensity"] | 1.0F,
      p["direction"] | "", p["mode"] | "", p["origin"] | "");
    return ack(type, ok, "invalid_parameter");
  }
  if (!strcmp(type, "configure_sensor_reporting")) return ack(type, true);
  ack(type, false, "unknown_message_type");
}

void serialPoll() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (!discardingOversizeFrame && rxLength) {
        rxBuffer[rxLength] = '\0';
        command(rxBuffer);
      }
      rxLength = 0;
      discardingOversizeFrame = false;
    } else if (c != '\r' && !discardingOversizeFrame) {
      if (rxLength >= HardwareConfig::MAX_FRAME_BYTES) {
        rxLength = 0;
        discardingOversizeFrame = true;
        ack("unknown", false, "frame_too_large");
      } else rxBuffer[rxLength++] = c;
    }
  }
}
}

void setup() {
  Serial.begin(HardwareConfig::SERIAL_BAUD);
  leds.begin(); sensors.begin();
  JsonDocument hello; hello["controller"] = "esp32"; send("hello", hello);
}

void loop() {
  const uint32_t now = millis();
  serviceTx(now); serialPoll(); leds.update(now); sensors.update(now, reporter); serviceTx(millis());
  if (now - lastHeartbeat >= ReportingConfig::HEARTBEAT_INTERVAL_MS) { JsonDocument p; send("heartbeat", p); lastHeartbeat = now; }
}
