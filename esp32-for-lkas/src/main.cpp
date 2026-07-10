#include <Arduino.h>

// GPIO48 WS2812 RGB LED at full brightness, solid white - confirmed correct
// pin for this exact board (YD-ESP32-23 / YD-ESP32-S3) per its official repo.

constexpr uint8_t kLedPin = 48;

void setup() {
  Serial.begin(115200);
  Serial.println("Blink test: GPIO48 WS2812, FULL brightness white on/off every 800ms");
}

void loop() {
  rgbLedWrite(kLedPin, 255, 255, 255);  // max brightness white
  Serial.println("LED ON (white, full brightness)");
  delay(800);

  rgbLedWrite(kLedPin, 0, 0, 0);
  Serial.println("LED OFF");
  delay(800);
}
