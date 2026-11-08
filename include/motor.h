
#pragma once

void motorInit();

// Signed 8-bit duty command, clamped to +/-MAX_SPEED. NOT an RPM target.
// A zero-speed wheel coasts. All functions are safe before init/when disabled.
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void motorForward(int speed);  // Magnitude of speed, both wheels forward.
void motorStop();

// True only after validated pins and successful PWM initialization.
bool motorIsReady();
