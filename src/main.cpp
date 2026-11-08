
#include <Arduino.h>
#include "config.h"
#include "motor.h"

void setup() {
    Serial.begin(115200);
    motorInit();
}

void loop() {
    motorForward(BASE_SPEED);
}
