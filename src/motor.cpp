
#include <Arduino.h>
#include "motor.h"
#include "config.h"

void motorInit() {
    Serial.println("Motor module initialized");
}

void motorForward(int speed) {
    Serial.print("Motor speed: ");
    Serial.println(speed);
}

void motorStop() {
    Serial.println("Motor stopped");
}
