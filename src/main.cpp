#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "sensors.h"
#include "modes.h"

namespace {
unsigned long lastLedAt = 0;
bool ledOn = false;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    motorInit();
    sensorsInit();
    modesInit(millis());
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
    const unsigned long now = millis();
    modesUpdate(now);
    if (now - lastLedAt >= STATUS_LED_INTERVAL_MS) {
        lastLedAt = now;
        ledOn = !ledOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
    }
}
