#include <Arduino.h>
#include "config.h"

// Stage 1 only: verify a non-blocking ESP32 blink in Wokwi.

bool ledState = false;
unsigned long lastBlinkAt = 0;

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println("HRC2026 starter: ESP32 blink demo");
}

void loop() {
  const unsigned long now = millis();
  if (now - lastBlinkAt >= STATUS_LED_INTERVAL_MS) {
    lastBlinkAt = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
    Serial.println(ledState ? "LED ON" : "LED OFF");
  }
}
