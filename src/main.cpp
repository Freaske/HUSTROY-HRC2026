#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "sensors.h"
#include "modes.h"
#include "test_link.h"

namespace {
unsigned long ledStartedAt = 0;
bool ledOn = false;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.printf("FW=%s | LED GPIO%u: double blink\n", FIRMWARE_ID,
                  static_cast<unsigned int>(STATUS_LED_PIN));
    motorInit();
    sensorsInit();
    modesInit(millis());
    testLinkInit();
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    ledStartedAt = millis();
}

void loop() {
    const unsigned long now = millis();
    modesUpdate(now);
    testLinkUpdate(millis());
    const unsigned long phase = (now - ledStartedAt) % STATUS_LED_CYCLE_MS;
    const bool nextLedOn = phase < STATUS_LED_PULSE_MS ||
                          (phase >= 2 * STATUS_LED_PULSE_MS && phase < 3 * STATUS_LED_PULSE_MS);
    if (nextLedOn != ledOn) {
        ledOn = nextLedOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
    }
}
